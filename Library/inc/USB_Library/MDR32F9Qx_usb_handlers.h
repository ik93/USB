/*******************************************************************************
  * @file    MDR32F9Qx_usb_handlers.h
  * @author  Milandr Application Team
  * @version V1.0.0
  * @date    18/03/2011
  * @brief   USB Library user-defined handlers definition file.
  ******************************************************************************/


#ifndef __MDR32F9Qx_USB_HANDLERS_H
#define __MDR32F9Qx_USB_HANDLERS_H


#include <MDR32F9Qx_usb_default_handlers.h>

#undef USB_CDC_HANDLE_DATA_RECEIVE
#define USB_CDC_HANDLE_DATA_RECEIVE(BUFFER, LENGTH)   USB_CDC_RecieveData(BUFFER, LENGTH)


#ifdef USB_CDC_CONTROL_LINE_STATE_SUPPORTED
#undef USB_CDC_HANDLE_CONTROL_LINE_STATE
#define USB_CDC_HANDLE_CONTROL_LINE_STATE(wVALUE, wINDEX)       USB_SUCCESS
#endif /* USB_CDC_CONTROL_LINE_STATE_SUPPORTED */

USB_Result USB_CDC_RecieveData(uint8_t* Buffer, uint32_t Length);

#endif /* __MDR32F9Qx_USB_HANDLERS_H */
