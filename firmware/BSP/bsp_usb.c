#include "bsp_usb.h"
#include "usb_bsp.h"
#include "usb_hcd_int.h"
#include "usbh_core.h"
#include "usbh_ioreq.h"
#include "usbh_hcs.h"
#include "usb_hcd.h"
#include "delay.h"
#include <string.h>

#define USB_RX_BUF_SIZE			64				//接收缓冲大小
#define USB_RX_RETRY_LIMIT      50				//重试等待次数
#define USB_PAD_SILENT_LIMIT    300				//手柄掉线时间间隔
#define USB_PIPE_REOPEN_LIMIT   1500			//usb通道掉线时间间隔

USB_OTG_CORE_HANDLE USB_OTG_Core;				//硬件结构体对象
USBH_HOST USB_Host;								//协议结构体对象

static UsbPadState_t usb_pad;					//手柄状态结构体对象
static volatile uint8_t usb_enum_done;			//USB枚举完成标志

static uint8_t usb_in_ch = 0xFF;				//USB数据通道
static uint8_t usb_in_mps;						//最大包大小
static uint8_t usb_rx_busy;						//接收标志
static uint8_t usb_rx_buf[USB_RX_BUF_SIZE];		//接收缓冲
static uint16_t usb_rx_wait_ms;					//重试等待次数
static uint16_t usb_pad_silent_ms;				//手柄掉线时间间隔
static uint16_t usb_pipe_silent_ms;				//usb通道掉线时间间隔

//手柄设备初始化
static USBH_Status gamepad_class_init(USB_OTG_CORE_HANDLE *pdev, void *phost)
{
    USBH_HOST *host = (USBH_HOST *)phost;								//协议对象类型转换
    uint8_t i;
    uint8_t ep_in = 0;

	//找IN端点
    for (i = 0; i < host->device_prop.Itf_Desc[0].bNumEndpoints; i++)
    {
        USBH_EpDesc_TypeDef *ep = &host->device_prop.Ep_Desc[0][i];

		//判断是否为IN端点
        if (ep->bEndpointAddress & 0x80)
        {
            ep_in = ep->bEndpointAddress;
            usb_in_mps = (uint8_t)ep->wMaxPacketSize;					//端点最大传输字节数
            break;
        }
    }

	//没找到IN端点
    if (ep_in == 0)
    {
        return USBH_FAIL;
    }

	//修正包大小
    if ((usb_in_mps == 0) || (usb_in_mps > USB_RX_BUF_SIZE))
    {
        usb_in_mps = USB_RX_BUF_SIZE;
    }

    usb_in_ch = USBH_Alloc_Channel(pdev, ep_in);						//分配通道

    //配置通道
	USBH_Open_Channel(pdev,
                      usb_in_ch,
                      host->device_prop.address,
                      host->device_prop.speed,
                      EP_TYPE_INTR,
                      usb_in_mps);

	//标志初始化
    usb_rx_busy = 0;
	usb_rx_wait_ms = 0;
	usb_pad_silent_ms = USB_PAD_SILENT_LIMIT;
	usb_pad.connected = 0;
	usb_pipe_silent_ms = 0;

    return USBH_OK;
}

//通道清除
static void gamepad_class_deinit(USB_OTG_CORE_HANDLE *pdev, void *phost)
{
    (void)phost;														//忽略参数

	//释放通道
    if (usb_in_ch != 0xFF)
    {
        USBH_Free_Channel(pdev, usb_in_ch);
        usb_in_ch = 0xFF;
    }

    usb_rx_busy = 0;													//接收标志清零
    memset(&usb_pad, 0, sizeof(usb_pad));								//手柄数据清零
}

//类请求（空实现，只收不发）
static USBH_Status gamepad_class_requests(USB_OTG_CORE_HANDLE *pdev, void *phost)
{
    (void)pdev;
    (void)phost;

    return USBH_OK;
}

//解析手柄数据
static void gamepad_decode(const uint8_t *buf, uint8_t len)
{
	//xbox游戏手柄14字节数据
    if (len < 14)
    {
        return;
    }

    usb_pad.buttons = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);		//按键

    usb_pad.lt = buf[4];												//左扳机
    usb_pad.rt = buf[5];												//右扳机

    usb_pad.lx = (int8_t)buf[9];										//左摇杆x
    usb_pad.ly = (int8_t)buf[7];										//左摇杆y
    usb_pad.rx = (int8_t)buf[13];										//右摇杆x
    usb_pad.ry = (int8_t)buf[11];										//右摇杆y

    usb_pad.connected = 1;												//连接到手柄收到数据
	usb_pad_silent_ms = 0;
	usb_pipe_silent_ms = 0;
}

//重建通道
static void gamepad_reopen_in_pipe(USB_OTG_CORE_HANDLE *pdev, void *phost)
{
    USBH_HOST *host = (USBH_HOST *)phost;
    uint8_t i;
    uint8_t ep_in = 0;

	//释放旧通道
    if (usb_in_ch != 0xFF)
    {
        USBH_Free_Channel(pdev, usb_in_ch);
        usb_in_ch = 0xFF;
    }

	//找IN端点
    for (i = 0; i < host->device_prop.Itf_Desc[0].bNumEndpoints; i++)
    {
        USBH_EpDesc_TypeDef *ep = &host->device_prop.Ep_Desc[0][i];

        if ((ep->bEndpointAddress & 0x80) != 0)
        {
            ep_in = ep->bEndpointAddress;
            usb_in_mps = (uint8_t)ep->wMaxPacketSize;
            break;
        }
    }

	//没找到IN端点
    if (ep_in == 0)
    {
        usb_rx_busy = 0;
        return;
    }

	//修正包大小
    if ((usb_in_mps == 0) || (usb_in_mps > USB_RX_BUF_SIZE))
    {
        usb_in_mps = USB_RX_BUF_SIZE;
    }

	//分配通道
    usb_in_ch = USBH_Alloc_Channel(pdev, ep_in);

	//通道配置
    USBH_Open_Channel(pdev,
                      usb_in_ch,
                      host->device_prop.address,
                      host->device_prop.speed,
                      EP_TYPE_INTR,
                      usb_in_mps);

    usb_rx_busy = 0;
    usb_rx_wait_ms = 0;
    usb_pipe_silent_ms = 0;
}

//接收数据状态机
static USBH_Status gamepad_class_machine(USB_OTG_CORE_HANDLE *pdev, void *phost)
{
    URB_STATE urb;
    HC_STATUS hc;
    uint8_t len;

	//通道已经配置好
    if (usb_in_ch == 0xFF)
    {
        return USBH_OK;
    }

	//通道空闲
    if (!usb_rx_busy)
	{
		USBH_InterruptReceiveData(pdev, usb_rx_buf, usb_in_mps, usb_in_ch);	//数据请求中断，收数据
		usb_rx_busy = 1;													//通道忙要收数据
		usb_rx_wait_ms = 0;													//重试等待计数清零
		return USBH_OK;
	}

    urb = HCD_GetURB_State(pdev, usb_in_ch);								//数据请求状态
    hc = HCD_GetHCState(pdev, usb_in_ch);									//通道状态

	//数据已经收完
    if (urb == URB_DONE)
    {
        len = (uint8_t)HCD_GetXferCnt(pdev, usb_in_ch);
        gamepad_decode(usb_rx_buf, len);									//解析数据
        usb_rx_busy = 0;													//通道空闲
    }
	//通道没数据或数据没准备好或数据传输错误或数据阻塞
    else if ((hc == HC_NAK) ||
             (urb == URB_NOTREADY) ||
             (urb == URB_ERROR) ||
             (urb == URB_STALL))
    {
        usb_rx_busy = 0;													//通道空闲，等重试
    }
	else
	{
		//等待数据
		if (usb_rx_wait_ms < USB_RX_RETRY_LIMIT)
		{
			usb_rx_wait_ms++;
		}
		//数据请求超时重置标志和计数
		else
		{
			usb_rx_busy = 0;
			usb_rx_wait_ms = 0;
		}
	}

	//手柄超时掉线没发数据
	if (usb_pad.connected == 0)
	{
		//等待通道数据
		if (usb_pipe_silent_ms < USB_PIPE_REOPEN_LIMIT)
		{
			usb_pipe_silent_ms++;
		}
		//通道超时掉线
		else
		{
			gamepad_reopen_in_pipe(pdev, phost);							//重建通道
		}
	}

    return USBH_OK;
}

//USB回调结构体
static USBH_Class_cb_TypeDef gamepad_class_cb =
{
    gamepad_class_init,
    gamepad_class_deinit,
    gamepad_class_requests,
    gamepad_class_machine
};

static void usb_user_init(void)
{
}

//usb库关闭重置标志
static void usb_user_deinit(void)
{
    usb_enum_done = 0;														//未枚举			
    memset(&usb_pad, 0, sizeof(usb_pad));									//手柄数据清零
}

static void usb_user_device_attached(void)
{
}

static void usb_user_reset_device(void)
{
}

//usb拔出重置标志
static void usb_user_device_disconnected(void)
{
    usb_enum_done = 0;														//未枚举
    memset(&usb_pad, 0, sizeof(usb_pad));									//手柄数据清零
}

static void usb_user_over_current(void)
{
}

static void usb_user_speed_detected(uint8_t speed)
{
    (void)speed;
}

static void usb_user_device_desc_available(void *desc)
{
    (void)desc;
}

static void usb_user_address_assigned(void)
{
}

//枚举读取设备描述符
static void usb_user_cfg_desc_available(USBH_CfgDesc_TypeDef *cfg,
                                        USBH_InterfaceDesc_TypeDef *itf,
                                        USBH_EpDesc_TypeDef *ep)
{
    (void)cfg;
    (void)itf;
    (void)ep;
}

static void usb_user_manufacturer_string(void *str)
{
    (void)str;
}

static void usb_user_product_string(void *str)
{
    (void)str;
}

static void usb_user_serial_string(void *str)
{
    (void)str;
}

//设置枚举完成标志
static void usb_user_enum_done(void)
{
    usb_enum_done = 1;
}

static USBH_USR_Status usb_user_input(void)
{
    return USBH_USR_RESP_OK;
}

static int usb_user_application(void)
{
    return 0;
}

static void usb_user_not_supported(void)
{
}

static void usb_user_unrecovered_error(void)
{
}
//用户事件回调结构体
static USBH_Usr_cb_TypeDef usb_user_cb =
{
    usb_user_init,
    usb_user_deinit,
    usb_user_device_attached,
    usb_user_reset_device,
    usb_user_device_disconnected,
    usb_user_over_current,
    usb_user_speed_detected,
    usb_user_device_desc_available,
    usb_user_address_assigned,
    usb_user_cfg_desc_available,
    usb_user_manufacturer_string,
    usb_user_product_string,
    usb_user_serial_string,
    usb_user_enum_done,
    usb_user_input,
    usb_user_application,
    usb_user_not_supported,
    usb_user_unrecovered_error
};

//外部调用，USB初始化
void Usb_Init(void)
{
    memset(&usb_pad, 0, sizeof(usb_pad));								//手柄数据清零
    usb_enum_done = 0;
    usb_in_ch = 0xFF;													//通道未分配
    usb_in_mps = 0;
    usb_rx_busy = 0;

	//手柄设备初始化
    USBH_Init(&USB_OTG_Core,
              USB_OTG_FS_CORE_ID,
              &USB_Host,
              &gamepad_class_cb,
              &usb_user_cb);
}

//外部调用，轮询
void Usb_Update(void)
{
    USBH_Process(&USB_OTG_Core, &USB_Host);							//运行USB库

	//未枚举
    if (!usb_enum_done)
    {
        usb_pad.connected = 0;
        usb_pad_silent_ms = USB_PAD_SILENT_LIMIT;
        return;
    }

	//等待手柄超时
    if (usb_pad_silent_ms < USB_PAD_SILENT_LIMIT)
    {
        usb_pad_silent_ms++;
    }
	//手柄超时重置标志
    else
    {
        usb_pad.connected = 0;
    }
}

//外部调用，获取手柄数据
uint8_t Usb_GetState(UsbPadState_t *state)
{
    if (state != 0)
    {
        *state = usb_pad;
    }

    return usb_pad.connected;
}

//USB引脚初始化
void USB_OTG_BSP_Init(USB_OTG_CORE_HANDLE *pdev)
{
    GPIO_InitTypeDef gpio;

    (void)pdev;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_OTG_FS);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_OTG_FS);

    gpio.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio);
}

//USB中断初始化
void USB_OTG_BSP_EnableInterrupt(USB_OTG_CORE_HANDLE *pdev)
{
    NVIC_InitTypeDef nvic;

    (void)pdev;

    nvic.NVIC_IRQChannel = OTG_FS_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 6;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

void USB_OTG_BSP_ConfigVBUS(USB_OTG_CORE_HANDLE *pdev)
{
    (void)pdev;
}

void USB_OTG_BSP_DriveVBUS(USB_OTG_CORE_HANDLE *pdev, uint8_t state)
{
    (void)pdev;
    (void)state;
}

void USB_OTG_BSP_uDelay(const uint32_t usec)
{
    delay_us(usec);
}

void USB_OTG_BSP_mDelay(const uint32_t msec)
{
    delay_ms(msec);
}

void USB_OTG_BSP_TimerIRQ(void)
{
}

//USB中断函数
void OTG_FS_IRQHandler(void)
{
    USBH_OTG_ISR_Handler(&USB_OTG_Core);
}
