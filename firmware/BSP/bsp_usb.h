#ifndef __BSP_USB_H
#define __BSP_USB_H

#include "stm32f4xx.h"
#include <stdint.h>

//手柄按键定义
#define USBPAD_BTN_UP       (1u << 0)
#define USBPAD_BTN_DOWN     (1u << 1)
#define USBPAD_BTN_LEFT     (1u << 2)
#define USBPAD_BTN_RIGHT    (1u << 3)
#define USBPAD_BTN_START    (1u << 4)
#define USBPAD_BTN_SELECT   (1u << 5)
#define USBPAD_BTN_LJOY     (1u << 6)
#define USBPAD_BTN_RJOY     (1u << 7)
#define USBPAD_BTN_LB       (1u << 8)
#define USBPAD_BTN_RB       (1u << 9)
#define USBPAD_BTN_HOME     (1u << 10)
#define USBPAD_BTN_A        (1u << 12)
#define USBPAD_BTN_B        (1u << 13)
#define USBPAD_BTN_X        (1u << 14)
#define USBPAD_BTN_Y        (1u << 15)

typedef struct
{
    uint8_t connected;
    int16_t lx;
    int16_t ly;
    int16_t rx;
    int16_t ry;
    uint8_t lt;
    uint8_t rt;
    uint16_t buttons;                       //按键位图
} UsbPadState_t;

void Usb_Init(void);
void Usb_Update(void);
uint8_t Usb_GetState(UsbPadState_t *state);

#endif
