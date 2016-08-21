#include <MDR32F9x.h>
#include "main.h"
#include "delays.h"

PORT_InitTypeDef PORT_InitStructure;

//==========================Эксперименты===============================

uint8_t RxUSB_Buffer[10];	  //Буфер приема сообщений от ПК
uint8_t task;                 //Текущая команда от ПК: начать набор, остановить, сброс

#define START_SPECTR 0x04	//Набор спектра
#define STOP_SPECTR 0x01	//Останов
#define RESET_SPECTR 0x02	//Сброс

#define LED1            PORT_Pin_10
#define LED2            PORT_Pin_11
#define LED3            PORT_Pin_12

void Command_Handler (void)
{
	LEDOff();
	task = RxUSB_Buffer[8];

	switch (task)
	{
		case START_SPECTR:
		LEDOn(LED1);
		break;

		case STOP_SPECTR:
		LEDOn(LED2);
		break;

		case RESET_SPECTR:
		LEDOn(LED3);
		break;
	}

	LEDOn(LED2);
}


void PortD_Init ()
{
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTD, ENABLE); // вкл тактирование от HSI

  // Настраиваем пины 10-12 порта D на выход для мигания
  PORT_InitStructure.PORT_Pin   = (PORT_Pin_10 | PORT_Pin_11 | PORT_Pin_12);
  PORT_InitStructure.PORT_OE    = PORT_OE_OUT;
  PORT_InitStructure.PORT_FUNC  = PORT_FUNC_PORT;
  PORT_InitStructure.PORT_MODE  = PORT_MODE_DIGITAL;
  PORT_InitStructure.PORT_SPEED = PORT_SPEED_SLOW;
  PORT_InitStructure.PORT_PULL_UP = PORT_PULL_UP_ON;
  PORT_InitStructure.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
  PORT_InitStructure.PORT_PD = PORT_PD_DRIVER;

  PORT_Init(MDR_PORTD, &PORT_InitStructure); // инициализация порта D
}

void LEDOn(uint32_t LED_Num)
{
  PORT_SetBits(MDR_PORTD, LED_Num);
}


void LEDOff(uint32_t LED_Num)
{
  PORT_ResetBits(MDR_PORTD, LED_Num);
}

//=====================================================================

static void VCom_Configuration(void)
{
	LineCoding.dwDTERate = 9600;
	LineCoding.bCharFormat = 0;
	LineCoding.bParityType = 0;
	LineCoding.bDataBits = 8;
}

void Setup_USB(void)
{
	//подать тактирование на USB
	RST_CLK_PCLKcmd(RST_CLK_PCLK_USB, ENABLE);

	//инициализация режима работы USB
	USB_Clock_InitStruct.USB_USBC1_Source = USB_C1HSEdiv2;
	USB_Clock_InitStruct.USB_PLLUSBMUL = USB_PLLUSBMUL12;

	USB_DeviceBUSParam.MODE = USB_SC_SCFSP_Full;
	USB_DeviceBUSParam.SPEED = USB_SC_SCFSR_12Mb;
	USB_DeviceBUSParam.PULL = USB_HSCR_DP_PULLUP_Set;

	// инициализация как Device
	USB_DeviceInit(&USB_Clock_InitStruct, &USB_DeviceBUSParam);

	//разрешить все USB прерывания
	USB_SetSIM(USB_SIS_Msk);

	USB_DevicePowerOn();

	NVIC_EnableIRQ(USB_IRQn);

	USB_DEVICE_HANDLE_RESET;
}

void PLL_init(void)
{
	MDR_RST_CLK->HS_CONTROL |= RST_CLK_HSE_ON;
	while (!(MDR_RST_CLK->CLOCK_STATUS & 0x04))
	{
	}
	MDR_RST_CLK->PLL_CONTROL |= 9 << 8;
	MDR_RST_CLK->PLL_CONTROL |= 0x04;
	while (!(MDR_RST_CLK->CLOCK_STATUS & 0x02))
	{
	}
	MDR_RST_CLK->CPU_CLOCK = CPU_C1(2) | CPU_C2(1) | HCLK_SEL(1);
}

void Port_init(void)
{

	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTB, ENABLE);


	PORT_InitStructure.PORT_Pin = PORT_Pin_6;
	PORT_InitStructure.PORT_OE = PORT_OE_OUT;
	PORT_InitStructure.PORT_FUNC = PORT_FUNC_PORT;
	PORT_InitStructure.PORT_MODE = PORT_MODE_DIGITAL;
	PORT_InitStructure.PORT_SPEED = PORT_SPEED_FAST;
	PORT_Init(MDR_PORTB, &PORT_InitStructure);
}

int main(void)
{

	PLL_init();
	Port_init();
	PortD_Init();
	Setup_USB();
	USB_CDC_Init(RxUSB_Buffer, 1, SET);
	USB_CDC_Reset();
	VCom_Configuration();

	while (1)
	{
		/*
		MDR_PORTB->RXTX |=(1<<6);
		delay_ms(100);
		MDR_PORTB->RXTX &=~(1<<6);
		USB_CDC_SendData("1234", 4);
		MDR_PORTB->RXTX &=~(1<<6);
		delay_ms(50);
		*/
	}
}

// 7Е 81 ADR1  /ADR1 ADR2  Q CMD CONTC CHSL CHSH F0





USB_Result USB_CDC_RecieveData(uint8_t* Buffer, uint32_t Length)
{
  //USB_Result result;
	USB_Buf[cnt++] = Buffer[0];

	Command_Handler();   //=========================относится к экспериментам

  USB_Result result = USB_CDC_SendData(Buffer, Length);

  //return result;

}

/* USB_CDC_HANDLE_GET_LINE_CODING implementation example */
USB_Result USB_CDC_GetLineCoding(uint16_t wINDEX, USB_CDC_LineCoding_TypeDef* DATA)
{
  assert_param(DATA);
  if (wINDEX != 0)
  {
    /* Invalid interface */
    return USB_ERR_INV_REQ;
  }

  /* Just store received settings */
  *DATA = LineCoding;
  return USB_SUCCESS;
}

/* USB_CDC_HANDLE_SET_LINE_CODING implementation example */
USB_Result USB_CDC_SetLineCoding(uint16_t wINDEX, const USB_CDC_LineCoding_TypeDef* DATA)
{
  assert_param(DATA);
  if (wINDEX != 0)
  {
    /* Invalid interface */
    return USB_ERR_INV_REQ;
  }

  /* Just send back settings stored earlier */
  LineCoding = *DATA;
  return USB_SUCCESS;
}
