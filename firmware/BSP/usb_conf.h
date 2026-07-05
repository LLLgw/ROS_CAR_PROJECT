#ifndef __USB_CONF__H__
#define __USB_CONF__H__

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include <stdint.h>

#define USE_USB_OTG_FS                  //OTG-FS
#define USB_OTG_FS_CORE                 //编译FS-CORE

#define USE_HOST_MODE                   //主机模式

//FIFO缓冲区
#define RX_FIFO_FS_SIZE      128        //RX缓冲区总大小
#define TX0_FIFO_FS_SIZE      64        //TX端点0缓冲区大小
#define TX1_FIFO_FS_SIZE     128        //TX端点1缓冲区大小
#define TX2_FIFO_FS_SIZE       0        //TX端点2缓冲区大小
#define TX3_FIFO_FS_SIZE       0        //TX端点3缓冲区大小

#define TXH_NP_FS_FIFOSIZ    96         //非周期性发送缓冲区大小
#define TXH_P_FS_FIFOSIZ     96         //周期性发送缓冲区大小

//不支持低功耗
#define USB_OTG_FS_LOW_PWR_MGMT_SUPPORT
#undef USB_OTG_FS_LOW_PWR_MGMT_SUPPORT

#undef USB_OTG_FS_SOF_OUTPUT_ENABLED    //不发送帧起始信号
#undef VBUS_SENSING_ENABLED             //不做VBUS检测

#endif
