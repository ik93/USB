#include <MDR32F9x.h>
#include "main.h"
#include "delays.h"
#include "MDR32F9Qx_adc.h"
#include "MDR32F9Qx_it.h"
#include "MDR32F9Qx_timer.h"

PORT_InitTypeDef PORT_InitStructure;

//==========================Эксперименты===============================================================================

typedef struct
{

	uint8_t StartByte1;    		// 7E 81 Обрамление начала сообщения
	uint8_t StartByte2;    		// 7E 81 Обрамление начала сообщения
	uint8_t Address;   			// ADR1 or ADR2 Receiver address
	uint8_t InvAddress;			// /ADR1 or /ADR2
	uint8_t SenderAddress;		//
	uint8_t DataLenght;			// Q
	uint8_t Command;			// CMD
	uint8_t CommandData1;		// CONTC
	uint8_t CommandData2;		// CONTC
	uint8_t CheckSumLow;		// CHSL
	uint8_t CheckSumHigh;		// CHSH
	uint8_t EndByte;			// F0

} ParseRxMessage;

uint8_t RxUSB_Buffer[11];	  		  //Буфер приема сообщений от ПК
ParseRxMessage task;                  //Текущая команда от ПК: начать набор, остановить, сброс
ParseRxMessage previous_task;		  //Предыдущая команда от ПК

#define START_SPECTR 0x04	//Набор спектра
#define STOP_SPECTR 0x01	//Останов
#define RESET_SPECTR 0x02	//Сброс

#define LED1            PORT_Pin_10
#define LED2            PORT_Pin_11
#define LED3            PORT_Pin_12

ParseRxMessage RxParser(uint8_t *RxBuffer)
{

	ParseRxMessage RxMessage;

	RxMessage.StartByte1 = RxBuffer[0];
	RxMessage.StartByte2 = RxBuffer[1];
	RxMessage.Address = RxBuffer[2];
	RxMessage.InvAddress = RxBuffer[3];
	RxMessage.SenderAddress = RxBuffer[4];
	RxMessage.DataLenght = RxBuffer[5];

	if (RxMessage.DataLenght == 0x01)				// Команды очистка, пуск , стоп
	{
		RxMessage.Command = RxBuffer[6];
		RxMessage.CommandData1 = RxBuffer[7];
		RxMessage.CheckSumLow = RxBuffer[8];
		RxMessage.CheckSumHigh = RxBuffer[9];
		RxMessage.EndByte = RxBuffer[10];
	}

	else if (RxMessage.DataLenght == 0x02)			// Команда "Результат"
	{
		RxMessage.Command = RxBuffer[6];
		RxMessage.CommandData1 = RxBuffer[7];
		RxMessage.CommandData2 = RxBuffer[8];
		RxMessage.CheckSumLow = RxBuffer[9];
		RxMessage.CheckSumHigh = RxBuffer[10];
		RxMessage.EndByte = RxBuffer[11];
	}

	return RxMessage;
}

void TxUSB_Constructor(uint8_t *outBuffer, ParseRxMessage RxMessage, ParseRxMessage previous_task)
{
	outBuffer[0] = RxMessage.StartByte1;
	outBuffer[1] = RxMessage.StartByte2;
	outBuffer[2] = RxMessage.SenderAddress;
	outBuffer[3] = RxMessage.SenderAddress ^ 0xFF;
	outBuffer[4] = RxMessage.Address;
	outBuffer[5] = 0x09;
	outBuffer[6] = 0x89;
	outBuffer[7] = previous_task.CommandData1 != RxMessage.CommandData1;
	uint16_t checksum =  0;
	for (int i = 0; i < 8; ++i) {
		checksum += outBuffer[i];
	}
	uint8_t low = checksum & 0xFF;
	uint8_t high = (checksum >> 8) & 0xFF;
	outBuffer[8] = low;
	outBuffer[9] = high;
	outBuffer[10] = RxMessage.EndByte;
}

void Command_Handler(uint8_t *outBuffer, ParseRxMessage RxMessage)
{

	previous_task = task;
	task = RxMessage;

	TxUSB_Constructor(outBuffer, task, previous_task);
}

//===============================================================Инициализация таймера для прерывания по стробу================================================
void PortB_Init()
{
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTB, ENABLE); // вкл тактирование от HSI

		/* Reset PORTB settings */
			  PORT_DeInit(MDR_PORTB);

			  // инициализируем PB5 как вход таймера _____________________________ ВОЗМОЖНО В ИНИЦИАЛИЗАЦИИ ЛИШНИЕ ПАРАМЕТРЫ !!!
		  	PORT_InitStructure.PORT_Pin = PORT_Pin_5;
		 	PORT_InitStructure.PORT_FUNC = PORT_FUNC_OVERRID;
		  	PORT_InitStructure.PORT_GFEN = PORT_GFEN_OFF;			// -?
		  	PORT_InitStructure.PORT_PULL_UP = PORT_PULL_UP_OFF;		// -возможно подтяжки по умолчанию откл?
		  	PORT_InitStructure.PORT_MODE = PORT_MODE_DIGITAL;
		  	PORT_InitStructure.PORT_SPEED = PORT_SPEED_MAXFAST;
		  	PORT_InitStructure.PORT_OE = PORT_OE_IN;
		  	PORT_Init(MDR_PORTB, &PORT_InitStructure);
}


void Timer_init(void)
{
		RST_CLK_PCLKcmd(RST_CLK_PCLK_TIMER3, ENABLE);

		TIMER_CntInitTypeDef Timer_Cnt_InitStructure;
		TIMER_ChnInitTypeDef Timer_Chn_InitStructure;

		TIMER_DeInit(MDR_TIMER3);

		TIMER_BRGInit(MDR_TIMER3, TIMER_HCLKdiv1);			// Initializes the TIMER3 peripheral Clock

		  /* Initializes the TIMERx Counter -------------------------------------*/
		TIMER_CntStructInit(&Timer_Cnt_InitStructure);

		Timer_Cnt_InitStructure.TIMER_Period = 0xFFFF; 			// 0xFFFF - кроме этого все значения дефолтные, можно использовать TIMER_CntStructInit()

		TIMER_CntInit (MDR_TIMER3, &Timer_Cnt_InitStructure);

		TIMER_ITConfig (MDR_TIMER3, TIMER_STATUS_CCR_CAP_CH3 | TIMER_STATUS_ETR_FALLING_EDGE, ENABLE);

		/* Initializes the TIMER3 Channel3 -------------------------------------*/
		TIMER_ChnStructInit(&Timer_Chn_InitStructure);
		/** Режим работы каналов - захват */
		Timer_Chn_InitStructure.TIMER_CH_Number              = TIMER_CHANNEL3;
		Timer_Chn_InitStructure.TIMER_CH_Mode                = TIMER_CH_MODE_CAPTURE; /** Режим работы таймера - захват */
		Timer_Chn_InitStructure.TIMER_CH_EventSource         = TIMER_CH_EvSrc_NE; /** Отрицательный фронт для события для CCR */

		  TIMER_ChnInit(MDR_TIMER3, &Timer_Chn_InitStructure);

		  NVIC_EnableIRQ(Timer3_IRQn);

}

		 // прерывание таймера
	void Timer3_IRQHandler(void)
	{

	   if(TIMER_GetFlagStatus(MDR_TIMER3, TIMER_STATUS_CCR_CAP_CH3 )) // прерывание по спаду
	   {
		   ADC2_Start();												// Включили АЦП
		  TIMER_ClearFlag(MDR_TIMER3, TIMER_STATUS_CCR_CAP_CH3);
	   }
	}

//======================================================================================================================================

//Инициализация PD3, PD2 для работы c АЦП
void PortD_Init ()
{
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTD, ENABLE); // вкл тактирование от HSI

	/* Reset PORTD settings */
		  PORT_DeInit(MDR_PORTD);

		  /* Configure ADC pin: ADC2, ADC3 */
		  /* Configure PORTD pin 2, 3 */
		  PORT_InitStructure.PORT_Pin   = PORT_Pin_3 | PORT_Pin_2;
		  PORT_InitStructure.PORT_OE    = PORT_OE_IN;
		  PORT_InitStructure.PORT_MODE  = PORT_MODE_ANALOG;
		  PORT_Init(MDR_PORTD, &PORT_InitStructure);
}


//============================================================================================= Инициализация АЦП ===================================================================================
// Иниц АЦП для Термодатчика
void ADC1_Configuration(void)
{
	ADC_InitTypeDef sADC;
	ADCx_InitTypeDef sADCx;

		//подать тактирование на АЦП
		RST_CLK_PCLKcmd(RST_CLK_PCLK_ADC, ENABLE);
		RST_CLK_ADCclkSelection(RST_CLK_ADCclkCPU_C1); 					// Берем частоту с CPU_C1 = 16 МГц
		RST_CLK_ADCclkPrescaler(RST_CLK_ADCclkDIV32); 					// Предделитель в блоке ADC_C3 ( 16 МГц / 32 = 500 КГц )
		RST_CLK_ADCclkEnable(ENABLE);


	/* Init NVIC */
	  SCB->AIRCR = 0x05FA0000 | ((uint32_t)0x500);
	  SCB->VTOR = 0x08000000;
	  /* Disable all interrupt */
	  NVIC->ICPR[0] = 0xFFFFFFFF;
	  NVIC->ICER[0] = 0xFFFFFFFF;

	  /* Enable ADC interrupt  */
	  NVIC->ISER[0] = (1<<ADC_IRQn);

	  /* ADC Configuration */
	  /* Reset all ADC settings */
	  ADC_DeInit();
	  ADC_StructInit(&sADC);
	  sADC.ADC_SynchronousMode      = ADC_SyncMode_Independent;
	  sADC.ADC_StartDelay           = 0;
	  sADC.ADC_TempSensor           = ADC_TEMP_SENSOR_Disable;
	  sADC.ADC_TempSensorAmplifier  = ADC_TEMP_SENSOR_AMPLIFIER_Disable;
	  sADC.ADC_TempSensorConversion = ADC_TEMP_SENSOR_CONVERSION_Disable;
	  sADC.ADC_IntVRefConversion    = ADC_VREF_CONVERSION_Disable;			// Запретить преобразования для канала VREF (внутренней опоры)
	  sADC.ADC_IntVRefTrimming      = 0;
	  ADC_Init (&sADC);

	  ADCx_StructInit (&sADCx);
	  sADCx.ADC_ClockSource      = ADC_CLOCK_SOURCE_CPU;
	  sADCx.ADC_SamplingMode     = ADC_SAMPLING_MODE_CICLIC_CONV;			// Режим циклического преобразования (несколько раз подряд)
	  sADCx.ADC_ChannelSwitching = ADC_CH_SWITCHING_Disable;
	  sADCx.ADC_ChannelNumber    = ADC_CH_ADC3;
	  sADCx.ADC_Channels         = 0;
	  sADCx.ADC_LevelControl     = ADC_LEVEL_CONTROL_Disable;				// Контроль уровня входнрого напряжения (отключено)
	  sADCx.ADC_LowLevel         = 0;
	  sADCx.ADC_HighLevel        = 0;
	  sADCx.ADC_VRefSource       = ADC_VREF_SOURCE_INTERNAL;				// Вид источника опроного напряжения (внутренний)
	  sADCx.ADC_IntVRefSource    = ADC_INT_VREF_SOURCE_INEXACT;				// Вид внутреннего источника опроного напряжения (не точный)
	  sADCx.ADC_Prescaler        = ADC_CLK_div_32768;						// Предделитель частоты тактирования АЦП
	  sADCx.ADC_DelayGo          = 0xF;										// Задержка между запусками АЦП (максимальная)
	  ADC1_Init (&sADCx);

	  /* Enable ADC1 EOCIF and AWOIFEN interrupts */
	  ADC1_ITConfig((ADCx_IT_END_OF_CONVERSION  | ADCx_IT_OUT_OF_RANGE), ENABLE);

	  /* ADC1 enable */
	  ADC1_Cmd (ENABLE);

}

// Иниц АЦП для приема импульсов
void ADC2_Configuration(void)
{
	ADC_InitTypeDef sADC;
	ADCx_InitTypeDef sADCx;

		//подать тактирование на АЦП
		RST_CLK_PCLKcmd(RST_CLK_PCLK_ADC, ENABLE);
		RST_CLK_ADCclkSelection(RST_CLK_ADCclkCPU_C1); 					// Берем частоту с CPU_C1 = 16 МГц
		RST_CLK_ADCclkPrescaler(RST_CLK_ADCclkDIV32); 					// Предделитель в блоке ADC_C3 ( 16 МГц / 32 = 500 КГц )
		RST_CLK_ADCclkEnable(ENABLE);


	/* Init NVIC */
	  SCB->AIRCR = 0x05FA0000 | ((uint32_t)0x500);
	  SCB->VTOR = 0x08000000;
	  /* Disable all interrupt */
	  NVIC->ICPR[0] = 0xFFFFFFFF;
	  NVIC->ICER[0] = 0xFFFFFFFF;

	  /* Enable ADC interrupt  */
	  NVIC->ISER[0] = (1<<ADC_IRQn);

	  /* ADC Configuration */
	  /* Reset all ADC settings */
	  ADC_DeInit();
	  ADC_StructInit(&sADC);
	  sADC.ADC_SynchronousMode      = ADC_SyncMode_Independent;
	  sADC.ADC_StartDelay           = 0;
	  sADC.ADC_TempSensor           = ADC_TEMP_SENSOR_Disable;
	  sADC.ADC_TempSensorAmplifier  = ADC_TEMP_SENSOR_AMPLIFIER_Disable;
	  sADC.ADC_TempSensorConversion = ADC_TEMP_SENSOR_CONVERSION_Disable;
	  sADC.ADC_IntVRefConversion    = ADC_VREF_CONVERSION_Disable;			// Запретить преобразования для канала VREF (внутренней опоры)
	  sADC.ADC_IntVRefTrimming      = 0;
	  ADC_Init (&sADC);

	  ADCx_StructInit (&sADCx);
	  sADCx.ADC_ClockSource      = ADC_CLOCK_SOURCE_CPU;
	  sADCx.ADC_SamplingMode     = ADC_SAMPLING_MODE_CICLIC_CONV;			// Режим циклического преобразования (несколько раз подряд)
	  sADCx.ADC_ChannelSwitching = ADC_CH_SWITCHING_Disable;
	  sADCx.ADC_ChannelNumber    = ADC_CH_ADC2;
	  sADCx.ADC_Channels         = 0;
	  sADCx.ADC_LevelControl     = ADC_LEVEL_CONTROL_Disable;				// Контроль уровня входнрого напряжения (отключено)
	  sADCx.ADC_LowLevel         = 0;
	  sADCx.ADC_HighLevel        = 0;
	  sADCx.ADC_VRefSource       = ADC_VREF_SOURCE_INTERNAL;				// Вид источника опроного напряжения (внутренний)
	  sADCx.ADC_IntVRefSource    = ADC_INT_VREF_SOURCE_INEXACT;				// Вид внутреннего источника опроного напряжения (не точный)
	  sADCx.ADC_Prescaler        = ADC_CLK_div_32768;						// Предделитель частоты тактирования АЦП
	  sADCx.ADC_DelayGo          = 0xF;										// Задержка между запусками АЦП (максимальная)
	  ADC1_Init (&sADCx);

	  /* Enable ADC1 EOCIF and AWOIFEN interrupts */
	  ADC2_ITConfig((ADCx_IT_END_OF_CONVERSION  | ADCx_IT_OUT_OF_RANGE), ENABLE);

	  /* ADC1 enable */
	  ADC2_Cmd (ENABLE);

}


void ADC_IRQHandler()
{
	int temperature_C = 0;
	uint32_t Vtemp;
	uint32_t Vspec;

	if (ADC1_GetFlagStatus(ADC1_FLAG_END_OF_CONVERSION) == SET)				// АЦП термодатчика. флаг сам обнулится после чтения из буф
	{
		Vtemp = ADC1_GetResult();
	}

	else if (ADC2_GetFlagStatus(ADC2_FLAG_END_OF_CONVERSION) == SET)		// АЦП детектора
	{
		Vspec = ADC2_GetResult();
	}

}

//===============================================================================================================================================================================

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
	MDR_RST_CLK->HS_CONTROL |= RST_CLK_HSE_ON;				// Включение HSE
	while (!(MDR_RST_CLK->CLOCK_STATUS & 0x04))				// !(HSE RDY)
	{
	}
	MDR_RST_CLK->PLL_CONTROL |= 9 << 8;						// коэф умножения = 9
	MDR_RST_CLK->PLL_CONTROL |= 0x04;						// PLL_CPU_ON
	while (!(MDR_RST_CLK->CLOCK_STATUS & 0x02))				// !(PLL запущена и стабильна)
	{
	}
	MDR_RST_CLK->CPU_CLOCK = CPU_C1(2) | CPU_C2(1) | HCLK_SEL(1);			// Источник для CPU_C1 - HSE, PLL_CPU0, CPU_C3
}


int main(void)
{

	PLL_init();
	PortD_Init();
	ADC1_Configuration();
	Setup_USB();
	USB_CDC_Init(RxUSB_Buffer, 1, SET);
	USB_CDC_Reset();
	VCom_Configuration();

	ADC1_Start();

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
	//USB_Buf[cnt++] = Buffer[0];

	ParseRxMessage rxMessage = RxParser(RxUSB_Buffer);
	uint8_t sendBuffer[11] = {0};
	Command_Handler(&sendBuffer[0], rxMessage);  // -&

	USB_Result result = USB_CDC_SendData(sendBuffer, 11);

	return result;
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
