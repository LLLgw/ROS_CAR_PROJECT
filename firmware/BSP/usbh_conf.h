#ifndef __USBH_CONF__H__
#define __USBH_CONF__H__

#include "stm32f4xx.h"
#include <stdint.h>

#define USBH_MAX_NUM_ENDPOINTS     4            //最大支持端点数
#define USBH_MAX_NUM_INTERFACES    4            //最大支持接口数
#define USBH_MAX_DATA_BUFFER       512          //单次最大传输量

#ifdef USE_USB_OTG_FS
#define USBH_MSC_MPS_SIZE          0x40         //OTG-FS最大包大小64
#else
#define USBH_MSC_MPS_SIZE          0x200
#endif

#if defined(__CC_ARM)
#define __ALIGN_BEGIN              __align(4)   //4字节对齐
#define __ALIGN_END
#else
#define __ALIGN_BEGIN
#define __ALIGN_END
#endif

#endif
