#include "MDR32F9Qx_config.h"
#include "MDR32F9Qx_rst_clk.h"
#include "MDR32F9Qx_port.h"

#include "MDR32F9Qx_usb.h"
#include "MDR32F9Qx_usb_handlers.h"
#include "MDR32F9Qx_usb_device.h"


#define	CPU_C1(n)			((uint16_t)(n)  & 0x03)
#define CPU_C2(n)			(((uint16_t)(n) & 0x03) << 2)
#define	CPU_C3(n)			(((uint16_t)(n) & 0x0F) << 4)
#define HCLK_SEL(n)		(((uint16_t)(n) & 0x03) << 8)

#define BUFFER_LENGTH                    100

USB_Clock_TypeDef USB_Clock_InitStruct;
USB_DeviceBUSParam_TypeDef USB_DeviceBUSParam;

volatile uint32_t PendingDataLength = 2;
volatile uint8_t cnt;

static uint8_t Buffer[20];
static uint8_t USB_Buf[BUFFER_LENGTH];
static USB_CDC_LineCoding_TypeDef LineCoding;
