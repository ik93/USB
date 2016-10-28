#include "MDR32F9Qx_config.h"
#include "MDR32F9Qx_rst_clk.h"
#include "MDR32F9Qx_port.h"
#include "MDR32F9Qx_timer.h"
#include "MDR32F9Qx_adc.h"
#include "stdint.h"

#include "MDR32F9Qx_usb.h"
#include "MDR32F9Qx_usb_handlers.h"
#include "MDR32F9Qx_usb_device.h"

#define BUFFER_LENGTH                    100

#define NUMBER_OF_INTERVALS 250
#define INITIAL_INTERVAL_VALUE 0.0f
#define FINAL_INTERVAL_VALUE 3.3f

USB_Clock_TypeDef USB_Clock_InitStruct;
USB_DeviceBUSParam_TypeDef USB_DeviceBUSParam;


typedef struct Interval
{
    float left;
    float right;
    uint16_t numberOfEntities;
} Interval;




volatile uint32_t PendingDataLength = 2;
volatile uint8_t cnt;

static uint8_t Buffer[20];
static uint8_t USB_Buf[BUFFER_LENGTH];
static USB_CDC_LineCoding_TypeDef LineCoding;

int indexOfValueInterval(float value, Interval *intervals, int left, int right);
int put(float value, Interval *intervals);


