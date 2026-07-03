#include "battery_task.h"
#include "state.h"
#include "stm32f4xx.h"

#define BAT_ADC_GPIO_PORT       GPIOB										//ADC端口
#define BAT_ADC_GPIO_PIN        GPIO_Pin_0									//ADC引脚
#define BAT_ADC_GPIO_CLK        RCC_AHB1Periph_GPIOB						//ADC端口时钟外设
#define BAT_ADC_CHANNEL         ADC_Channel_8								//ADC引脚对应的通道

#define BAT_SAMPLE_COUNT        10											//采集10次求平均
#define BAT_DIVIDER_GAIN        11u											//分压电阻放大倍数
#define BAT_FULL_MV             25200u										//电池满电电压
#define BAT_EMPTY_MV            19800u										//电池空电电压
#define BAT_LOW_MV              20400u										//电池低电带电压

#define BAT_STOP_ENABLE         0											//低电锁车使能标志

//单次ADC采集
static uint16_t read_battery_adc(void)
{
    uint32_t timeout = 100000u;												//超时计数

	//配置ADC通道
    ADC_RegularChannelConfig(ADC1,
                             BAT_ADC_CHANNEL,
                             1,
                             ADC_SampleTime_480Cycles);

	//模数转换
    ADC_SoftwareStartConv(ADC1);

	//等待转换
    while ((ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) && (timeout > 0u))
    {
        timeout--;
    }

	//超时
    if (timeout == 0u)
    {
        return 0;
    }

	//返回读取结果
    return ADC_GetConversionValue(ADC1);
}

//10次采集求平均
static uint16_t read_battery_adc_average(void)
{
    uint8_t i;
    uint32_t sum = 0;

	//采集10次
    for (i = 0; i < BAT_SAMPLE_COUNT; i++)
    {
        sum += read_battery_adc();
    }

	//返回平均数
    return (uint16_t)(sum / BAT_SAMPLE_COUNT);
}

//电压转化百分比
static uint8_t calc_percent(uint16_t mv)
{
    uint32_t percent;

    if (mv <= BAT_EMPTY_MV)
    {
        return 0;
    }

    if (mv >= BAT_FULL_MV)
    {
        return 100;
    }

	//线性估算
    percent = ((uint32_t)(mv - BAT_EMPTY_MV) * 100u) /
              (BAT_FULL_MV - BAT_EMPTY_MV);

    return (uint8_t)percent;
}

//ADC引脚初始化
void Battery_Init(void)
{
    GPIO_InitTypeDef gpio;
    ADC_CommonInitTypeDef adc_common;
    ADC_InitTypeDef adc;

    RCC_AHB1PeriphClockCmd(BAT_ADC_GPIO_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    gpio.GPIO_Pin = BAT_ADC_GPIO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AN;										//模拟模式
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(BAT_ADC_GPIO_PORT, &gpio);

    ADC_CommonStructInit(&adc_common);
    adc_common.ADC_Mode = ADC_Mode_Independent;
    adc_common.ADC_Prescaler = ADC_Prescaler_Div4;
    adc_common.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    adc_common.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&adc_common);

    ADC_StructInit(&adc);
    adc.ADC_Resolution = ADC_Resolution_12b;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &adc);

    ADC_Cmd(ADC1, ENABLE);
}

//电池电压采集任务主函数
void Battery_TaskUpdate(void)
{
    static uint32_t filtered_mv = 0;									//存放上一次的mv值
    uint16_t adc;
    uint32_t mv;

    adc = read_battery_adc_average();

    mv = ((uint32_t)adc * 3300u * BAT_DIVIDER_GAIN) / 4096u;

    if (filtered_mv == 0)
    {
        filtered_mv = mv;
    }
    else
    {
        filtered_mv = (filtered_mv * 7u + mv) / 8u;						//低通滤波，电压平滑跳变不抖动
    }

	//写全局电池状态
    g.battery.voltage_mv = (uint16_t)filtered_mv;
    g.battery.current_ma = 0;
    g.battery.percent = calc_percent((uint16_t)filtered_mv);

    if (filtered_mv <= BAT_LOW_MV)
    {
        g.battery.flags |= 0x0001;										//低电标志置1

#if BAT_STOP_ENABLE														//判断低电锁车使能
        g.low_battery = 1;
#else
        g.low_battery = 0;
#endif
    }
    else
    {
        g.battery.flags &= (uint16_t)(~0x0001);							//低电标志清零
        g.low_battery = 0;
    }
}
