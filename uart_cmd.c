/* uart_cmd.c - 双电机统一命令接口(USART2, PA2=TX PA3=RX, 115200)
 *
 * 挡板电机命令(FOC/SVPWM位置控制, '#'前缀, 轴号固定为1):
 *   "#1 1000 300"     目标位置+轨迹速度, 梯形轨迹执行
 *   "#1 V 3"          视觉标签(1~8)->挡板目标角度, 梯形轨迹执行
 *   "#1 PID 0 0.2 0.001"  在线修改环路增益(0:位置 1:速度 2:d电流 3:q电流)
 *   "#1 TEST 5"       固定负载下重复相同位置命令5次, 输出响应曲线遥测
 *   "#1 LOG 1"        手动开/关遥测流
 *   "#1 HOME"         转子对齐
 *   "#1 STOP"         停止
 *   "#1 ?"            查询状态
 *
 * 传送带电机命令(六步换相+PI速度环, 无前缀):
 *   "SPD <rpm>"       设置基准带速(堆积度=0时)
 *   "ACC <0-100>"     设置下游堆积度, 自动按堆积度折算速度目标
 *   "PID <kp> <ki> <kd>"  在线修改速度环增益(调参对比用)
 *   "TEST <n>"        固定负载下重复相同速度命令n次, 输出响应曲线遥测
 *   "LOG <0|1>"       手动开/关遥测流
 *   "RUN" / "STOP"    电机启停
 *   "?"               查询状态
 */
#include "uart_cmd.h"
#include "svpwm.h"
#include "trajectory.h"
#include "sorter.h"
#include "bldc.h"
#include "conveyor.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include "sys.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

/*==================== 轻量格式化助手 ====================*/

char *fmt_chr(char *p, char c)
{
    *p++ = c;
    return p;
}

char *fmt_int(char *p, int32_t v)
{
    char tmp[11];
    int i = 0;
    if (v < 0) {
        *p++ = '-';
        v = -v;
    }
    do {
        tmp[i++] = (char)('0' + (v % 10));
        v /= 10;
    } while (v);
    while (i) *p++ = tmp[--i];
    return p;
}

char *fmt_uint(char *p, uint32_t v)
{
    char tmp[11];
    int i = 0;
    do {
        tmp[i++] = (char)('0' + (v % 10));
        v /= 10;
    } while (v);
    while (i) *p++ = tmp[--i];
    return p;
}

static volatile uint8_t rx_buf[UART_BUF_SIZE];
static volatile uint16_t rx_idx = 0;

/* 发送环形缓冲: 遥测流经中断异步发送, 不阻塞1ms控制主循环 */
#define TX_BUF_SIZE 512
static volatile char tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_head = 0, tx_tail = 0;

extern Trajectory_t traj[2];

void UART_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    USART_InitTypeDef usart;
    usart.USART_BaudRate = UART_BAUD;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &usart);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE)) {
        uint8_t c = USART_ReceiveData(USART2);
        if (rx_idx < UART_BUF_SIZE - 1) {
            rx_buf[rx_idx++] = c;
            if (c == '\r') {
                rx_buf[rx_idx] = '\0';
            }
        }
    }
    if (USART_GetITStatus(USART2, USART_IT_TXE)) {
        if (tx_tail == tx_head) {
            USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
        } else {
            USART_SendData(USART2, (uint16_t)tx_buf[tx_tail]);
            tx_tail = (uint16_t)((tx_tail + 1) % TX_BUF_SIZE);
        }
    }
}

void UART_SendString(char *str)
{
    while (*str) {
        uint16_t next = (uint16_t)((tx_head + 1) % TX_BUF_SIZE);
        if (next == tx_tail) break;   /* 缓冲满丢弃, 保证控制环不被阻塞 */
        tx_buf[tx_head] = *str++;
        tx_head = next;
    }
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}

/*==================== 挡板电机命令(FOC/SVPWM) ====================*/

static void gate_send_reply(void)
{
    char buf[64];
    sprintf(buf, "#1 P=%d S=%d ST=%d T=%d\r\n", motor[1].pos,
            motor[1].speed_rpm, motor[1].state, motor[1].target_pos);
    UART_SendString(buf);
}

static void gate_parse(char *p)
{
    if (strstr(p, "STOP")) {
        Motor_Stop(&motor[1]);
        /* 故障恢复: 堵转保护会关断TIM8与驱动器, STOP后重新使能(占空比已为零) */
        TIM_Cmd(TIM8, ENABLE);
        TIM_CtrlPWMOutputs(TIM8, ENABLE);
        GPIO_WriteBit(AXIS2_SD_PORT, AXIS2_SD_PIN, Bit_SET);
        gate_send_reply();
    } else if (strstr(p, "CAL") || strstr(p, "HOME")) {
        /* 转子对齐, 对齐完成后自动捕获电角度零点 */
        motor[1].state = MOTOR_ALIGN;
        motor[1].align_ms = 0;
        gate_send_reply();
    } else if (strstr(p, "?")) {
        gate_send_reply();
    } else {
        /* 应用层命令按指令首字母分派: V=视觉标签 P=PID T=TEST L=LOG */
        char cmd = (rx_idx > 3) ? p[3] : 0;

        if (cmd == 'V' || cmd == 'v') {
            /* "#1 V 3": 视觉标签->挡板目标角度, 梯形轨迹执行 */
            int label = 0;
            if (sscanf(p + 4, "%d", &label) == 1 &&
                gate_move_to_label(1, (uint8_t)label)) {
                gate_send_reply();
            } else {
                UART_SendString("ERR:label\r\n");
            }
        } else if (cmd == 'P') {
            /* "#1 PID 0 0.2 0.001 [kd]": 在线修改环路增益(kd可选) */
            int loop = -1;
            float kp = 0.0f, ki = 0.0f, kd = 0.0f;
            int n = sscanf(p + 3, "PID %d %f %f %f", &loop, &kp, &ki, &kd);
            if (n >= 3 && loop >= 0 && loop <= 3 && kp >= 0.0f && ki >= 0.0f && kd >= 0.0f) {
                PID_t *loops[4] = {
                    &motor[1].pid_pos, &motor[1].pid_speed,
                    &motor[1].pid_c_d, &motor[1].pid_c_q
                };
                loops[loop]->kp = kp;
                loops[loop]->ki = ki;
                if (n >= 4) loops[loop]->kd = kd;
                loops[loop]->integral = 0.0f;
                gate_send_reply();
            } else {
                UART_SendString("ERR:pid\r\n");
            }
        } else if (cmd == 'T') {
            /* "#1 TEST 5": 固定负载下重复相同位置命令5次 */
            int runs = 0;
            if (sscanf(p + 3, "TEST %d", &runs) == 1 &&
                runs >= 1 && runs <= 50) {
                sorter_test_start(1, (uint8_t)runs);
            } else {
                UART_SendString("ERR:test\r\n");
            }
        } else if (cmd == 'L') {
            /* "#1 LOG 1": 手动开/关遥测流 */
            int on = 0;
            if (sscanf(p + 3, "LOG %d", &on) == 1) {
                sorter_log_ctrl(1, (uint8_t)on);
                gate_send_reply();
            } else {
                UART_SendString("ERR:log\r\n");
            }
        } else {
            /* "#1 1000 300": 目标位置 + 轨迹速度, 梯形轨迹执行 */
            int32_t pos = 0;
            float speed = 200.0f;
            int n = sscanf(p + 3, "%d %f", &pos, &speed);
            if (n >= 1) {
                if (abs(pos) > POS_LIMIT) {
                    UART_SendString("ERR:limit\r\n");
                    return;
                }
                Motor_SetTarget(&motor[1], pos);
                traj_plan(&traj[1], (float)pos, (float)motor[1].pos, speed, 500.0f);
                gate_send_reply();
            } else {
                UART_SendString("ERR:fmt\r\n");
            }
        }
    }
}

/*==================== 传送带电机命令(六步换相) ====================*/

static void conv_send_reply(void)
{
    char buf[80];
    sprintf(buf, "#S=%d T=%d B=%d D=%u A=%d R=%d\r\n",
            motor1.speed,
            (int)pid_target,
            conveyor_base_speed(),
            motor1.pwm_duty,
            conveyor_accumulation(),
            motor1.run_flag);
    UART_SendString(buf);
}

static void conv_parse(char *p)
{
    if (p[0] == '?') {
        conv_send_reply();
        return;
    }

    if (strstr(p, "RUN")) {
        if (motor1.run_flag != RUN) {
            motor1.run_flag = RUN;
            start_motor1();
            UVW_6_Step_Ponoff();
            TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
        }
        conv_send_reply();
    } else if (strstr(p, "STOP")) {
        /* 保留速度目标, RUN时按原目标恢复; 停机由驱动关闭实现 */
        stop_motor1();
        motor1.run_flag = STOP;
        conv_send_reply();
    } else if (strncmp(p, "SAVE", 4) == 0) {
        /* 保存标定值与整定增益到Flash(掉电不丢失) */
        ParamBlock_t pb;
        pb.theta_offset = (float)motor[1].theta_offset;
        pb.i_offset_a = motor[1].i_offset_a;
        pb.i_offset_b = motor[1].i_offset_b;
        pb.conv_kp = Kp;
        pb.conv_ki = Ki;
        pb.conv_kd = Kd;
        UART_SendString(param_save(&pb) ? "OK:save\r\n" : "ERR:save\r\n");
    } else if (p[0] == 'S') {
        int rpm = 0;
        if (sscanf(p, "SPD %d", &rpm) == 1 && rpm >= 0 && rpm <= 3000) {
            conveyor_set_base_speed((int16_t)rpm);
            conv_send_reply();
        } else {
            UART_SendString("ERR:spd\r\n");
        }
    } else if (p[0] == 'A') {
        int lv = 0;
        if (sscanf(p, "ACC %d", &lv) == 1 && lv >= 0 && lv <= 100) {
            conveyor_set_accumulation((uint8_t)lv);
            conv_send_reply();
        } else {
            UART_SendString("ERR:acc\r\n");
        }
    } else if (p[0] == 'P') {
        float kp = 0.0f, ki = 0.0f, kd = 0.0f;
        if (sscanf(p, "PID %f %f %f", &kp, &ki, &kd) == 3 &&
            kp >= 0.0f && ki >= 0.0f && kd >= 0.0f) {
            Kp = kp;
            Ki = ki;
            Kd = kd;
            conv_send_reply();
        } else {
            UART_SendString("ERR:pid\r\n");
        }
    } else if (p[0] == 'T') {
        int runs = 0;
        if (sscanf(p, "TEST %d", &runs) == 1 && runs >= 1 && runs <= 50) {
            conveyor_test_start((uint8_t)runs);
        } else {
            UART_SendString("ERR:test\r\n");
        }
    } else if (p[0] == 'L') {
        int on = 0;
        if (sscanf(p, "LOG %d", &on) == 1) {
            conveyor_set_log((uint8_t)on);
            conv_send_reply();
        } else {
            UART_SendString("ERR:log\r\n");
        }
    } else {
        UART_SendString("ERR:cmd\r\n");
    }
}

static void parse_and_execute(void)
{
    char *p = (char *)rx_buf;

    if (p[0] == '#') {
        uint8_t axis = p[1] - '0';
        if (axis != 1) { UART_SendString("ERR:axis\r\n"); return; }
        gate_parse(p);
    } else {
        conv_parse(p);
    }
}

void uart_poll(void)
{
    if (rx_idx > 0 && rx_buf[rx_idx - 1] == '\r') {
        /* 命令长度校验: 过滤噪声/残帧(有效命令长度1~40字符) */
        uint16_t len = rx_idx - 1;
        if (len < 1 || len > 40) {
            UART_SendString("ERR:len\r\n");
        } else {
            parse_and_execute();
        }
        rx_idx = 0;
        memset((void *)rx_buf, 0, UART_BUF_SIZE);
    }
    if (rx_idx >= UART_BUF_SIZE - 1) {
        rx_idx = 0;
        memset((void *)rx_buf, 0, UART_BUF_SIZE);
    }
}
