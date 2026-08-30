#ifndef __SVPWM_H_
#define __SVPWM_H_

#include "stm32f10x.h"
#include "math.h"
#include "stm32f10x_gpio.h"

/* 挡板电机(FOC/SVPWM)驱动器引脚: SD使能=PB4, 故障输入=PB5 */
#define AXIS2_SD_PORT    GPIOB
#define AXIS2_SD_PIN     GPIO_Pin_4
#define AXIS2_BKIN_PORT  GPIOB
#define AXIS2_BKIN_PIN   GPIO_Pin_5

#define MAX_SPEED  1500
#define PWM_PERIOD (72000000 / 10000)

#define CCW           (1)
#define CW            (2)
#define HALL_ERROR    (0xF0)

#define MOTOR_STOP    (0)
#define MOTOR_ALIGN   (1)
#define MOTOR_RUN     (2)
#define MOTOR_FAULT   (3)

#define I_MAX         3.0f
#define POS_LIMIT     100000

/* 电流标定: ADC码 -> 安培 (按实际硬件修改采样电阻/运放增益) */
#define CURR_SENSE_R    0.5f
#define CURR_AMP_GAIN   10.0f
#define ADC_TO_AMP      (3.3f / (4096.0f * CURR_SENSE_R * CURR_AMP_GAIN))

/* 挡板堵转保护阈值 */
#define STALL_ERR_COUNTS   1000    /* 位置误差超过该值且不动则计时 */
#define STALL_TIME_MS      2000    /* 持续不动超过2s判定堵转 */

/* 过流监测阈值 */
#define OC_LEVEL_AMP       3.5f    /* 实测q轴电流超过该值(安培)计过流 */
#define OC_TIME_MS         500     /* 持续超过500ms报过流故障 */

/* 前馈增益(速度环加速度前馈, 位置环速度前馈) */
#define FF_ACC_KT          0.005f  /* 加速度前馈系数: iq_ff = FF_ACC_KT * a */
#define FF_VEL_KP          1.0f    /* 速度前馈系数: sp_ff = FF_VEL_KP * v_ref */

/* ---------- 故障码体系 ---------- */
#define FAULT_NONE         0x00
#define FAULT_STALL        0x01    /* 挡板堵转 */
#define FAULT_OC           0x02    /* 持续过流 */
#define FAULT_HALL_LOST    0x10    /* 传送带Hall丢失(挡板轴视角) */
#define FAULT_DRV          0x20    /* 驱动器故障输入 */

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
    float limit;
} PID_t;

typedef struct {
    TIM_TypeDef *tim_pwm;
    TIM_TypeDef *tim_enc;
    uint8_t adc_ch_a;
    uint8_t adc_ch_b;

    float theta_elec;
    float theta_mech;
    int32_t enc_raw;
    int32_t pos;
    int16_t speed_rpm;

    float ia, ib;
    float id, iq;
    float id_ref, iq_ref;
    float ud_ref, uq_ref;

    int32_t target_pos;
    int16_t speed_ref;

    PID_t pid_c_d;
    PID_t pid_c_q;
    PID_t pid_speed;
    PID_t pid_pos;

    uint8_t state;
    uint8_t dir;
    uint16_t align_ms;    /* 对齐阶段计时, 超时后自动进入运行态 */

    int32_t theta_offset;     /* 电角度零点标定: 转子d轴对应的编码器计数 */
    float i_offset_a;         /* A相电流采样零点偏置(ADC原始码) */
    float i_offset_b;         /* B相电流采样零点偏置(ADC原始码) */
    uint16_t stall_ms;        /* 堵转计时 */
    int32_t stall_last_pos;   /* 堵转检测: 上次位置快照 */

    /* 前馈与过流 */
    float v_ff;               /* 轨迹瞬时速度(counts/s), 供速度前馈 */
    float a_ff;               /* 轨迹瞬时加速度(counts/s^2), 供加速度前馈 */
    uint16_t oc_ms;           /* 过流持续时间 */
    uint8_t  enc_dir;         /* 编码器方向: +1或-1(自整定), 修正测速符号 */
    uint8_t  fault_code;      /* 当前故障码(FAULT_xxx) */
} Motor_t;

typedef struct {
    uint8_t status;
    int32_t pos;
    int16_t speed;
    uint8_t dir;
    uint8_t state;
} Motor_Status_t;

typedef enum {
    SECTOR_1 = 1,
    SECTOR_2,
    SECTOR_3,
    SECTOR_4,
    SECTOR_5,
    SECTOR_6
} Sector_t;

extern Motor_t motor[2];
extern volatile uint8_t tim6_flag;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float limit);
float PID_Calc(PID_t *pid, float ref, float fb, float dt);

void Clarke(float ia, float ib, float *ialpha, float *ibeta);
void Park(float ialpha, float ibeta, float theta, float *id, float *iq);
void InvPark(float vd, float vq, float theta, float *valpha, float *vbeta);

void Motor_Init(Motor_t *m, TIM_TypeDef *tim_pwm, TIM_TypeDef *tim_enc, uint8_t ch_a, uint8_t ch_b);
void Motor_SetFault(Motor_t *m, uint8_t code);   /* 置故障码并切断(联动+过流+堵转共用) */
void Motor_CurrentLoop(Motor_t *m);
void Motor_SpeedLoop(Motor_t *m, float dt);
void Motor_PositionLoop(Motor_t *m);
void Motor_SetTarget(Motor_t *m, int32_t pos);
void Motor_Stop(Motor_t *m);
void Motor_Fault(Motor_t *m);

void SVPWM_Update_Duty(Motor_t *m, float ualpha, float ubeta);

void svpwm_gpio2_init(void);

uint8_t SVPWM_Sector_Calc(float Ualpha, float Ubeta);

#endif
