#include "delay.h"

static volatile uint32_t sysTickCount = 0;

//初始化DWT
void Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;			//使能DWT
    DWT->CYCCNT = 0;										//计数器清零
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;					//启动计数
}

//毫秒级延时
void delay_ms(uint32_t ms)
{
    while(ms--)
	{
		delay_us(1000);										//延时1000us
	}
}

//微秒级延时
void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;							//计数器起始计数值
    uint32_t ticks = us * (SystemCoreClock / 1000000);		//1us需要的时钟周期
    while ((DWT->CYCCNT - start) < ticks);					//等1us
}
