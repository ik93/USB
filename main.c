#include <MDR32F9x.h>
#include "main.h"
#include "delays.h"
#include "MDR32F9Qx_adc.h"
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

	double temperature_C = 0;
	uint32_t Vtemp;
	uint32_t Vspec;
	double  U, V;							//измеренное напряжение
	uint8_t Flag_EOCIF;
	uint32_t Flag_ADC;

#define START_SPECTR 0x04	//Набор спектра
#define STOP_SPECTR 0x01	//Останов
#define RESET_SPECTR 0x02	//Сброс

#define V_Ref 3.3			//Опорное напряжение АЦП
#define ADC_resolution 4096	//Разрешение АЦП
#define Scale_Factor 0.010 	//Температурный коэффициент термодатчика [мВ/град.С]
#define Offset 0.500			//Смещение термодатчика [мВ]

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
		  PORT_StructInit(&PORT_InitStructure);
		  /* Configure ADC pin: ADC2, ADC3 */
		  /* Configure PORTD pin 2, 3 */
		  PORT_InitStructure.PORT_Pin   = PORT_Pin_3 | PORT_Pin_2;
		  PORT_InitStructure.PORT_OE    = PORT_OE_IN;
		  PORT_InitStructure.PORT_MODE  = PORT_MODE_ANALOG;
		  PORT_Init(MDR_PORTD, &PORT_InitStructure);
}


//============================================================================================= Инициализация АЦП ===================================================================================
		/* Общая для 2ух АЦП инициализация */
void ADC_Configuration()
{
	ADC_InitTypeDef ADC_InitStructure;

	//подать тактирование на АЦП
			RST_CLK_PCLKcmd(RST_CLK_PCLK_ADC, ENABLE);
			RST_CLK_ADCclkSelection(RST_CLK_ADCclkCPU_C1); 					// Берем частоту с CPU_C1 = 16 МГц
			RST_CLK_ADCclkPrescaler(RST_CLK_ADCclkDIV32); 					// Предделитель в блоке ADC_C3 ( 16 МГц / 32 = 500 КГц )
			RST_CLK_ADCclkEnable(ENABLE);


		/* Init NVIC */
		 /* SCB->AIRCR = 0x05FA0000 | ((uint32_t)0x500);
		  SCB->VTOR = 0x08000000;
		  	  */

		  /* ADC Configuration */
		  /* Reset all ADC settings */
		  ADC_DeInit();
		  ADC_StructInit(&ADC_InitStructure);
		  ADC_InitStructure.ADC_SynchronousMode      = ADC_SyncMode_Independent;
		  ADC_InitStructure.ADC_StartDelay           = 0;
		  ADC_InitStructure.ADC_TempSensor           = ADC_TEMP_SENSOR_Disable;
		  ADC_InitStructure.ADC_TempSensorAmplifier  = ADC_TEMP_SENSOR_AMPLIFIER_Disable;
		  ADC_InitStructure.ADC_TempSensorConversion = ADC_TEMP_SENSOR_CONVERSION_Disable;
		  ADC_InitStructure.ADC_IntVRefConversion    = ADC_VREF_CONVERSION_Disable;			// Запретить преобразования для канала VREF (внутренней опоры)
		  ADC_InitStructure.ADC_IntVRefTrimming      = 1;									// !!! Изменила с 0 на 1.  Подстройка источника напряжения VREF
		  ADC_Init (&ADC_InitStructure);
}

// Иниц АЦП для Термодатчика
void ADC1_Configuration(void)
{
		ADCx_InitTypeDef ADC1_InitStructure;


	  ADCx_StructInit (&ADC1_InitStructure);
	  ADC1_InitStructure.ADC_ClockSource      = ADC_CLOCK_SOURCE_ADC;						// !! Тоже изменила
	  ADC1_InitStructure.ADC_SamplingMode     = ADC_SAMPLING_MODE_SINGLE_CONV;				// Режим циклического преобразования (несколько раз подряд)
	  ADC1_InitStructure.ADC_ChannelSwitching = ADC_CH_SWITCHING_Disable;
	  ADC1_InitStructure.ADC_ChannelNumber    = ADC_CH_ADC3;
	  ADC1_InitStructure.ADC_Channels         = 0;
	  ADC1_InitStructure.ADC_LevelControl     = ADC_LEVEL_CONTROL_Disable;					// Контроль уровня входнрого напряжения (отключено)
	  ADC1_InitStructure.ADC_LowLevel         = 0;
	  ADC1_InitStructure.ADC_HighLevel        = 0;
	  ADC1_InitStructure.ADC_VRefSource       = ADC_VREF_SOURCE_INTERNAL;					// Вид источника опроного напряжения (внутренний)
	  ADC1_InitStructure.ADC_IntVRefSource    = ADC_INT_VREF_SOURCE_INEXACT;				// Вид внутреннего источника опроного напряжения (не точный)
	  ADC1_InitStructure.ADC_Prescaler        = ADC_CLK_div_None;							// Предделитель частоты тактирования АЦП
	  ADC1_InitStructure.ADC_DelayGo          = 0xf;										// Задержка между запусками АЦП (максимальная)
	  ADC1_Init (&ADC1_InitStructure);

	  /* Enable ADC1 EOCIF and AWOIFEN interrupts */
	  ADC1_ITConfig(ADC1_IT_END_OF_CONVERSION, ENABLE);

	  /* ADC1 enable */
	  ADC1_Cmd (ENABLE);
	  /* Enable ADC interrupt  */
	//  NVIC_EnableIRQ(ADC_IRQn);

}

// Иниц АЦП для приема импульсов
void ADC2_Configuration(void)
{
		ADCx_InitTypeDef ADC2_InitStructure;

	  ADCx_StructInit (&ADC2_InitStructure);
	  ADC2_InitStructure.ADC_ClockSource      = ADC_CLOCK_SOURCE_ADC;
	  ADC2_InitStructure.ADC_SamplingMode     = ADC_SAMPLING_MODE_SINGLE_CONV;			// Режим циклического преобразования (несколько раз подряд)
	  ADC2_InitStructure.ADC_ChannelSwitching = ADC_CH_SWITCHING_Disable;
	  ADC2_InitStructure.ADC_ChannelNumber    = ADC_CH_ADC2;
	  ADC2_InitStructure.ADC_Channels         = 0;
	  ADC2_InitStructure.ADC_LevelControl     = ADC_LEVEL_CONTROL_Disable;				// Контроль уровня входнрого напряжения (отключено)
	  ADC2_InitStructure.ADC_LowLevel         = 0;
	  ADC2_InitStructure.ADC_HighLevel        = 0;
	  ADC2_InitStructure.ADC_VRefSource       = ADC_VREF_SOURCE_INTERNAL;				// Вид источника опроного напряжения (внутренний)
	  ADC2_InitStructure.ADC_IntVRefSource    = ADC_INT_VREF_SOURCE_INEXACT;			// Вид внутреннего источника опроного напряжения (не точный)
	  ADC2_InitStructure.ADC_Prescaler        = ADC_CLK_div_None;						// Предделитель частоты тактирования АЦП
	  ADC2_InitStructure.ADC_DelayGo          = 0xf;									// Задержка между запусками АЦП (максимальная)
	  ADC1_Init (&ADC2_InitStructure);

	  /* Enable ADC1 EOCIF and AWOIFEN interrupts */
	  ADC2_ITConfig(ADC2_IT_END_OF_CONVERSION, ENABLE);

	  /* ADC2 enable */
	  ADC2_Cmd (ENABLE);
	  /* Enable ADC interrupt  */
	  NVIC_EnableIRQ(ADC_IRQn);

}

double Get_Temperature_C(uint32_t Vtemp)
{
	double Temp_C;
	U = (V_Ref / ADC_resolution) * (Vtemp & 0x00000FFF);
	Temp_C = (U - Offset) / Scale_Factor;

	return Temp_C;
}

void ADC_IRQHandler()
{
	Flag_ADC = ADC1_GetStatus();
	Flag_EOCIF = ADC1_GetFlagStatus(ADC1_FLAG_END_OF_CONVERSION);

	if (ADC1_GetFlagStatus(ADC1_FLAG_END_OF_CONVERSION) == SET)				// АЦП термодатчика. флаг сам обнулится после чтения из буф
	{
		Vtemp = ADC1_GetResult();

		Flag_ADC = ADC1_GetStatus();
		Flag_EOCIF = ADC1_GetFlagStatus(ADC1_FLAG_END_OF_CONVERSION);
	}

	if (ADC2_GetFlagStatus(ADC2_FLAG_END_OF_CONVERSION) == SET)		// АЦП детектора
	{
		Vspec = ADC2_GetResult();
	}

	temperature_C = Get_Temperature_C(Vtemp);			//убрать! вставлено для отладки
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
	ADC_Configuration();
	ADC1_Configuration();
	ADC2_Configuration();
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
