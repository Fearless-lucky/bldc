/* sys.c - 系统级工具函数(中断控制/低功耗/复位/时钟配置/Flash参数存储) */
#include "sys.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_flash.h"
#include "string.h"

/*====================================================================
 * Flash参数存储: 使用512KB Flash的末页(0x0807F800)保存标定值与增益
 *====================================================================*/
#define PARAM_FLASH_ADDR   0x0807F800UL
#define PARAM_WORDS        (sizeof(ParamBlock_t) / 4)

static uint32_t param_checksum(const ParamBlock_t *pb)
{
    const uint32_t *w = (const uint32_t *)pb;
    uint32_t sum = 0, i;
    for (i = 0; i < PARAM_WORDS - 1; i++) {   /* sum字段自身不参与 */
        sum += w[i];
    }
    return sum;
}

uint8_t param_save(const ParamBlock_t *pb)
{
    ParamBlock_t b;
    memcpy(&b, pb, sizeof(b));
    b.magic = PARAM_MAGIC;
    b.sum = param_checksum(&b);

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    if (FLASH_ErasePage(PARAM_FLASH_ADDR) != FLASH_COMPLETE) {
        FLASH_Lock();
        return 0;
    }

    const uint32_t *w = (const uint32_t *)&b;
    uint32_t i;
    for (i = 0; i < PARAM_WORDS; i++) {
        if (FLASH_ProgramWord(PARAM_FLASH_ADDR + i * 4, w[i]) != FLASH_COMPLETE) {
            FLASH_Lock();
            return 0;
        }
    }
    FLASH_Lock();

    /* 回读校验 */
    return memcmp((const void *)PARAM_FLASH_ADDR, &b, sizeof(b)) == 0;
}

uint8_t param_load(ParamBlock_t *pb)
{
    memcpy(pb, (const void *)PARAM_FLASH_ADDR, sizeof(ParamBlock_t));
    if (pb->magic != PARAM_MAGIC) return 0;
    if (pb->sum != param_checksum(pb)) return 0;
    return 1;
}

/* 设置NVIC中断向量表偏移地址 */
void sys_nvic_set_vector_table(uint32_t baseaddr, uint32_t offset)
{
    /* 设置NVIC向量表偏移寄存器, VTOR低9位保留 */
    SCB->VTOR = baseaddr | (offset & (uint32_t)0xFFFFFE00);
}

/* 执行WFI指令(进入低功耗状态, 等待中断唤醒) */
void sys_wfi_set(void)
{
    __ASM volatile("wfi");
}

/* 关闭所有中断(不包括fault和NMI中断) */
void sys_intx_disable(void)
{
    __ASM volatile("cpsid i");
}

/* 开启所有中断 */
void sys_intx_enable(void)
{
    __ASM volatile("cpsie i");
}

/* 设置栈顶地址 */
void sys_msr_msp(uint32_t addr)
{
    __set_MSP(addr);
}

/* 进入待机模式 */
void sys_standby(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    SET_BIT(PWR->CR, PWR_CR_PDDS);
}

/* 系统软复位 */
void sys_soft_reset(void)
{
    NVIC_SystemReset();
}

/* 系统时钟初始化(HSE+PLL, pll为PLL倍频系数2~9) */
void Rcc_InitOnFuncLib(uint32_t pll)
{
    ErrorStatus HSEStartUpStatus;
    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_ON);
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    if (HSEStartUpStatus == SUCCESS)
    {
        RCC_HCLKConfig(RCC_SYSCLK_Div1);              /* AHB时钟=系统时钟 */
        RCC_PCLK2Config(RCC_HCLK_Div1);               /* APB2时钟=HCLK */
        RCC_PCLK1Config(RCC_HCLK_Div2);               /* APB1时钟=HCLK/2 */

        FLASH_SetLatency(FLASH_ACR_LATENCY_2);        /* 2个等待周期 */
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

        switch (pll)
        {
            case 2: RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_2); break;
            case 3: RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_3); break;
            case 4: RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_4); break;
            case 5: RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_5); break;
            case 6: RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_6); break;
            case 7: RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_7); break;
            case 8: RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_8); break;
            case 9: RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); break;
            default: RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_2); break;
        }
        RCC_PLLCmd(ENABLE);

        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) { }
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        while (RCC_GetSYSCLKSource() != 0x08) { }    /* 0x08: PLL作为系统时钟 */
    }
}
