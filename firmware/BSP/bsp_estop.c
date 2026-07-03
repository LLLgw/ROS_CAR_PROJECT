#include "bsp_estop.h"

//软件急停引脚初始化
void EStop_Init(void)
{
    GPIO_InitTypeDef GPIO_InitTypeStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_InitTypeStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitTypeStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitTypeStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitTypeStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitTypeStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitTypeStructure);
}

//判断软件急停按键是否按下
uint8_t EStop_IsPressed(void)
{
    return GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4) == Bit_RESET;
}
