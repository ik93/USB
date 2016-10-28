#include "main.h"

// =========== МАКРОСЫ =========================================================
#define	CPU_C1(n)			((uint16_t)(n)  & 0x03)
#define CPU_C2(n)			(((uint16_t)(n) & 0x03) << 2)
#define	CPU_C3(n)			(((uint16_t)(n) & 0x0F) << 4)
#define HCLK_SEL(n)			(((uint16_t)(n) & 0x03) << 8)
#define _BV(bit) 			(1<<(bit))


// ========== ПЕРЕМЕННЫЕ =======================================================

PORT_InitTypeDef PORT_InitStructure;
ADC_InitTypeDef ADC2_InitStructure;
uint32_t Flag_Timer3;
uint32_t Flag_Timer2;
double T;

uint8_t i = 0;						//счетчик для принятых частиц 0...255

// Переменные для отладки

uint32_t ADC_Spec_Value[256];		//массив принятых амплитуд в формате значения с АЦП
float Particle_Voltage[3072];		//массив амплитуд в формате "напряжение"


//------------------------------------------------------------------------------------
uint32_t number1;
MDR_TIMER_TypeDef *number3;
MDR_RST_CLK_TypeDef * number2;
MDR_PORT_TypeDef *number4;
MDR_ADC_TypeDef *number5;


// =========== НАСТРОЙКА ПЕРЕФЕРИИ =============================================

//============ Инициализация таймера для прерывания по стробу ==================

//Настройка PB5 на нём TIMER3_CH3
void PortB_Init()
{
	PORT_InitTypeDef TimerPortB_InitSrtucrure;

	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTB, ENABLE); // вкл тактирование

	PORT_DeInit(MDR_PORTB);
	PORT_StructInit(&TimerPortB_InitSrtucrure);
	// инициализируем PB5 как вход таймера
	TimerPortB_InitSrtucrure.PORT_Pin = PORT_Pin_5;
	TimerPortB_InitSrtucrure.PORT_PULL_UP = PORT_PULL_UP_OFF;
	TimerPortB_InitSrtucrure.PORT_PULL_DOWN = PORT_PULL_DOWN_ON;
	TimerPortB_InitSrtucrure.PORT_FUNC = PORT_FUNC_OVERRID;
	TimerPortB_InitSrtucrure.PORT_MODE = PORT_MODE_DIGITAL;
	TimerPortB_InitSrtucrure.PORT_SPEED = PORT_SPEED_MAXFAST;
	TimerPortB_InitSrtucrure.PORT_GFEN = PORT_GFEN_OFF; 		// фильтр выкл. (фильтрация импульсов до 10 нс)

	PORT_Init(MDR_PORTB, &TimerPortB_InitSrtucrure);
}

void Timer3_Init()
{
	TIMER_CntInitTypeDef	Timer_Count;
	TIMER_ChnInitTypeDef	Timer_Channel;

	RST_CLK_PCLKcmd(RST_CLK_PCLK_TIMER3, ENABLE);						// Разрешение тактовой частоты таймера 3
	TIMER_BRGInit(MDR_TIMER3, TIMER_HCLKdiv1);							// Управление тактовой частотой таймера 3

	TIMER_DeInit(MDR_TIMER3);

	TIMER_CntStructInit(&Timer_Count);
	TIMER_ChnStructInit(&Timer_Channel);

	Timer_Count.TIMER_CounterDirection = TIMER_CntDir_Up;
	Timer_Count.TIMER_ARR_UpdateMode = TIMER_ARR_Update_On_CNT_Overflow;
	Timer_Count.TIMER_CounterMode = TIMER_CntMode_ClkFixedDir;
	Timer_Count.TIMER_EventSource = TIMER_EvSrc_None;
	Timer_Count.TIMER_Prescaler = 0x00000000;			//частота процессора/1
	Timer_Count.TIMER_Period = 0x0000FFFF;
	Timer_Count.TIMER_IniCounter = 0;

	TIMER_CntInit(MDR_TIMER3, &Timer_Count);

	Timer_Channel.TIMER_CH_Number = TIMER_CHANNEL3;
	Timer_Channel.TIMER_CH_Mode = TIMER_CH_MODE_CAPTURE;
	Timer_Channel.TIMER_CH_EventSource = TIMER_CH_EvSrc_NE; 			//Захват по Отрицательному фронту !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	Timer_Channel.TIMER_CH_FilterConf = TIMER_Filter_8FF_at_TIMER_CLK;
	//Timer_Channel.TIMER_CH_CCR1_Ena = DISABLE;

	TIMER_ChnInit(MDR_TIMER3, &Timer_Channel);

	//  NVIC_EnableIRQ(Timer3_IRQn);

	MDR_TIMER3->IE = 0x00000080;

	MDR_TIMER3->STATUS = 0;
	Flag_Timer3 = TIMER_GetStatus(MDR_TIMER3);

	// Пуск таймера 3
	//TIMER_Cmd(MDR_TIMER3, ENABLE);

	NVIC->ISER[0] = (1<<Timer3_IRQn); //разрешить прерывание Timer3

}


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
	uint8_t CommandData;		// CONTC
	uint8_t NumberOfSegment;	// CONTC
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
	uint8_t Flag_EOCIF, Flag_EOCIF2;
	uint32_t Flag_ADC, Flag_ADC2 ;
	uint32_t Flag_Timer3;
	uint16_t Reg_Timer3, Reg1_Timer3, Timer3InitStruct, flag_T3CH3, Reg_ARR, Reg_CNT, Reg_CNT1, Reg_CCR, Reg_CCR1;					// Состояния регистра конфигурации
	uint32_t Flag_Timer2;
	uint32_t Reg_Timer2, Reg1_Timer2, Timer2InitStruct, flag_T2_ETR;
	uint8_t Command_from_USB;
	float Sample_Voltage = 0;
	uint8_t Flag_New_Sample = 0;

#define START 0x04		//Набор спектра
#define STOP 0x01		//Останов
#define RESET 0x02		//Сброс
#define RESULT 0xFF     //Результат

#define V_Ref 3.3f			//Опорное напряжение АЦП
#define ADC_resolution 4096	//Разрешение АЦП
#define Scale_Factor 0.010f 	//Температурный коэффициент термодатчика [мВ/град.С]
#define Offset 0.500f			//Смещение термодатчика [мВ]

ParseRxMessage RxParser(uint8_t *RxBuffer)
{

	ParseRxMessage RxMessage;

	RxMessage.StartByte1 = RxBuffer[0];
	RxMessage.StartByte2 = RxBuffer[1];
	RxMessage.Address = RxBuffer[2];
	RxMessage.InvAddress = RxBuffer[3];
	RxMessage.SenderAddress = RxBuffer[4];
	RxMessage.DataLenght = RxBuffer[5];

	if (RxMessage.DataLenght == 0x01)				// Команды Очистка, Пуск , Стоп
	{
		RxMessage.Command = RxBuffer[6];
		RxMessage.CommandData = RxBuffer[7];
		RxMessage.CheckSumLow = RxBuffer[8];
		RxMessage.CheckSumHigh = RxBuffer[9];
		RxMessage.EndByte = RxBuffer[10];
	}

	else if (RxMessage.DataLenght == 0x02)			// Команда "Результат"
	{
		RxMessage.Command = RxBuffer[6];
		RxMessage.CommandData = RxBuffer[7];
		RxMessage.NumberOfSegment = RxBuffer[8];  // номер Сегмента
		RxMessage.CheckSumLow = RxBuffer[9];
		RxMessage.CheckSumHigh = RxBuffer[10];
		RxMessage.EndByte = RxBuffer[11];
	}

	Command_from_USB = RxMessage.CommandData;
	return RxMessage;
}

// Ответ на команды ОЧИСТКА ПУСК СТОП длина 11 симв.
void TxUSB_Constructor(uint8_t *outBuffer, ParseRxMessage RxMessage, ParseRxMessage previous_task)
{
	uint8_t i;

	outBuffer[0] = RxMessage.StartByte1;
	outBuffer[1] = RxMessage.StartByte2;
	outBuffer[2] = RxMessage.SenderAddress;
	outBuffer[3] = RxMessage.SenderAddress ^ 0xFF;
	outBuffer[4] = RxMessage.Address;
	outBuffer[5] = 0x01;
	outBuffer[6] = 0x89;
	outBuffer[7] = previous_task.CommandData != RxMessage.CommandData;
	uint16_t checksum =  0;
	for (i = 0; i < 8; ++i)
		{
			checksum += outBuffer[i];
		}
	uint8_t low = checksum & 0xFF;
	uint8_t high = (checksum >> 8) & 0xFF;
	outBuffer[8] = low;
	outBuffer[9] = high;
	outBuffer[10] = RxMessage.EndByte;
}

void TxUSB_Result_Constructor(uint8_t *outBuffer, ParseRxMessage RxMessage, ParseRxMessage previous_task, Interval *intervals)
{
	uint8_t i;

	outBuffer[0] = RxMessage.StartByte1;
	outBuffer[1] = RxMessage.StartByte2;
	outBuffer[2] = RxMessage.SenderAddress;
	outBuffer[3] = RxMessage.SenderAddress ^ 0xFF;
	outBuffer[4] = RxMessage.Address;
	outBuffer[5] = 0x82;			//130 байт
	outBuffer[6] = 0x81;
	uint8_t num_segment_part = 0;
	if (RxMessage.NumberOfSegment == 0x00)
		{
			num_segment_part = 0;
			for (i = 7; i < 137; ++i)
				{
					outBuffer[i] = (intervals[num_segment_part].numberOfEntities >> 8) & 0xFF;
					outBuffer[++i] = intervals[num_segment_part++].numberOfEntities & 0xFF;
				}
		}
	else if (RxMessage.NumberOfSegment == 0x01)
		{
			num_segment_part = 65;
			for (i = 7; i < 137; ++i)
				{
					outBuffer[i] = (intervals[num_segment_part].numberOfEntities >> 8) & 0xFF;
					outBuffer[++i] = intervals[num_segment_part++].numberOfEntities & 0xFF;
				}
		}
	else if (RxMessage.NumberOfSegment == 0x02)
		{
			num_segment_part = 130;
			for (i = 7; i < 137; ++i)
				{
					outBuffer[i] = (intervals[num_segment_part].numberOfEntities >> 8) & 0xFF;
					outBuffer[++i] = intervals[num_segment_part++].numberOfEntities & 0xFF;
				}
		}
	else if (RxMessage.NumberOfSegment == 0x03)
		{
			num_segment_part = 195;
			for (i = 7; i < 117; ++i)
				{
					outBuffer[i] = (intervals[num_segment_part].numberOfEntities >> 8) & 0xFF;
					outBuffer[++i] = intervals[num_segment_part++].numberOfEntities & 0xFF;
				}
			for (i = 117; i < 137; ++i)
				{
					outBuffer[i] = 0;
				}
		}


	uint16_t checksum =  0;
	for (i = 0; i < 137; ++i) {
		checksum += outBuffer[i];
	}
	uint8_t low = checksum & 0xFF;
	uint8_t high = (checksum >> 8) & 0xFF;
	outBuffer[137] = low;
	outBuffer[138] = high;
	outBuffer[139] = RxMessage.EndByte;
}

void Command_Handler(uint8_t *outBuffer, ParseRxMessage RxMessage, Interval *intervals)
{

	previous_task = task;
	task = RxMessage;

	if (Command_from_USB == START || Command_from_USB == STOP || Command_from_USB == RESET )
		{
			TxUSB_Constructor(outBuffer, task, previous_task);
		}
	if (Command_from_USB == RESULT)
		{
			TxUSB_Result_Constructor(outBuffer, task, previous_task, intervals);
		}
}


//============================================================================================= Инициализация АЦП ===================================================================================
//======================================================================================================================================

//Инициализация PD3, PD2 для работы c АЦП
void PortD_ADC_Init ()
{
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTD, ENABLE); // вкл тактирование от HSI

	/* Reset PORTD settings */
		  PORT_DeInit(MDR_PORTD);
		  PORT_StructInit(&PORT_InitStructure);
		  /* Configure ADC pin: ADC2, ADC3 */
		  /* Configure PORTD pin 2, 3 */
		  PORT_InitStructure.PORT_Pin   = PORT_Pin_6; // для ОТЛАДОЧНОЙ
		  //PORT_InitStructure.PORT_Pin   = PORT_Pin_3 | PORT_Pin_2 ; //для МАКЕТА
		  PORT_InitStructure.PORT_OE    = PORT_OE_IN;
		  PORT_InitStructure.PORT_MODE  = PORT_MODE_ANALOG;
		  PORT_Init(MDR_PORTD, &PORT_InitStructure);
}


/* Общая для 2ух АЦП инициализация */
void ADC_Configuration()
{
	ADC_InitTypeDef ADC_InitStructure;

	//для тактирования АЦП используется ADC_CLK
			RST_CLK_PCLKcmd(RST_CLK_PCLK_ADC, ENABLE);
			RST_CLK_ADCclkSelection(RST_CLK_ADCclkCPU_C1); 					// Берем частоту с CPU_C1 = 8 МГц
			RST_CLK_ADCclkPrescaler(RST_CLK_ADCclkDIV1); 					// Предделитель в блоке ADC_C3 ( 8 МГц / 64 = 125 КГц )
			RST_CLK_ADCclkEnable(ENABLE);

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
		  ADC_InitStructure.ADC_IntVRefTrimming      = 1;									//  Подстройка источника напряжения VREF
		  ADC_Init (&ADC_InitStructure);
}

// Индивидуальная Иниц АЦП для Термодатчика
void ADC_Thermo_Configuration(void)
{
	 ADCx_InitTypeDef ADC_Thermo_InitStructure;

	  /** инициализация АПЦ первого канала - термодатчик */
	    ADCx_StructInit(&ADC_Thermo_InitStructure);
	    ADC_Thermo_InitStructure.ADC_ClockSource = ADC_CLOCK_SOURCE_ADC;
	    ADC_Thermo_InitStructure.ADC_SamplingMode = ADC_SAMPLING_MODE_SINGLE_CONV;				// Режим циклического преобразования отключен
	    ADC_Thermo_InitStructure.ADC_ChannelNumber = ADC_CH_ADC3;
	    ADC_Thermo_InitStructure.ADC_IntVRefSource = ADC_INT_VREF_SOURCE_INEXACT;
	    ADC_Thermo_InitStructure.ADC_Prescaler = ADC_CLK_div_None;								// Предделитель частоты преобразования
	    ADC_Thermo_InitStructure.ADC_DelayGo = 0xF; 											// максимальная Задержка перед преобразованием , чтобы зарядилась емкость АЦП
	    ADC1_Init(&ADC_Thermo_InitStructure);
	    ADC1_Cmd(ENABLE);

	  /* Enable ADC1 EOCIF and AWOIFEN interrupts */
	  ADC1_ITConfig(ADC1_IT_END_OF_CONVERSION, ENABLE);

	  /* Enable ADC interrupt  */
	 // NVIC_EnableIRQ(ADC_IRQn);
}


void ADC_Detector_Configuration(void)
{
	 ADCx_InitTypeDef ADC_Detector_InitStructure;

	 	ADCx_StructInit(&ADC_Detector_InitStructure);
	    ADC_Detector_InitStructure.ADC_ClockSource = ADC_CLOCK_SOURCE_ADC;
	    ADC_Detector_InitStructure.ADC_SamplingMode = ADC_SAMPLING_MODE_SINGLE_CONV;				// Режим циклического преобразования отключен
	    ADC_Detector_InitStructure.ADC_ChannelNumber = ADC_CH_ADC6;									// канал 6 для ОТЛАДОЧНОЙ платы
	    ADC_Detector_InitStructure.ADC_IntVRefSource = ADC_INT_VREF_SOURCE_INEXACT;
	    ADC_Detector_InitStructure.ADC_Prescaler = ADC_CLK_div_None;								// Предделитель частоты преобразования
	    ADC_Detector_InitStructure.ADC_DelayGo = 0xF; 												// максимальная Задержка перед преобразованием , чтобы зарядилась емкость АЦП
	    ADC1_Init(&ADC_Detector_InitStructure);
	    ADC1_Cmd(ENABLE);

	  /* Enable ADC2 EOCIF and AWOIFEN interrupts */
	    ADC1_ITConfig(ADC1_IT_END_OF_CONVERSION, ENABLE);

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


float Get_Particle_Energy(uint32_t Vspec)
{
	U = (V_Ref / ADC_resolution) * (Vspec & 0x00000FFF);
}

//====Функция=====
//преобразуем массив с АЦП ADC_Spec_Value[256]
//в массив значений в вольтах Particle_Voltage[3072]
void Get_Particle_Voltage()
{
	uint8_t circle_num = 0;
	uint16_t j = 0;				//счетчик для значений в вольтах 0...65.535
	uint8_t sample_num = 0;
	while (circle_num <= 11)
	{
		if ( ADC_Spec_Value[sample_num] != 0)
		{
		Particle_Voltage[j] = (V_Ref / ADC_resolution) * (float)(ADC_Spec_Value[sample_num] & 0x00000FFF);
		++j;
		++sample_num;
		if (sample_num == 0)
		{
			++circle_num;
		}
		}
	}
}

//===========================================================ОБРАБОТКА ИНТЕРВАЛОВ============================================================

// Puts some Voltage value to one of the intervals
// Return number of entities in found interval Or -1 if not found interval for the Value
int put(float value, Interval *intervals)
{
    int index = indexOfValueInterval(value, intervals, 0, NUMBER_OF_INTERVALS);
    //if (index == -1)
    //{
      //  return -1;
    //}
    ++intervals[index].numberOfEntities;
}

// Return index of interval that value is belong to Or -1 if not found interval for the Value
int indexOfValueInterval(float value, Interval *intervals, int left, int right)
{
    if (value < intervals[0].left || value > intervals[NUMBER_OF_INTERVALS - 1].right)
    {
        return -1;
    }
    int median = left + (right - left) / 2;
    Interval interval = intervals[median];
    if (value >= interval.left && value <= interval.right)
    {
        return median;
    }
    if (value > interval.right)
    {
        left = median;
        return indexOfValueInterval(value, intervals, left, right);
    }
    if (value < interval.left)
    {
        right = median;
        return indexOfValueInterval(value, intervals, left, right);
    }
    // wtf case
    return -1;
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
	// Работа от внешнего генератора
	RST_CLK_HSEconfig(RST_CLK_HSE_ON);

	RST_CLK_CPU_PLLuse(ENABLE);
	RST_CLK_CPU_PLLcmd(ENABLE);
	RST_CLK_CPUclkPrescaler(RST_CLK_CPUclkDIV1);
	RST_CLK_CPUclkSelection(RST_CLK_CPUclkCPU_C3);

	// Задаем частоту ядра 80МГц
	RST_CLK_CPU_PLLconfig(RST_CLK_CPU_PLLsrcHSEdiv1, 9);

/*	MDR_RST_CLK->HS_CONTROL |= RST_CLK_HSE_ON;				// Включение HSE. 8 Mhz
	MDR_RST_CLK->PLL_CONTROL |= 9 << 8;						// коэф умножения = 9. Получаем 8*10 = 80 МГц
	MDR_RST_CLK->PLL_CONTROL |= 0x04;						// PLL_CPU_ON
	MDR_RST_CLK->CPU_CLOCK = CPU_C1(2) | CPU_C2(1) | HCLK_SEL(1);			// Источник для CPU_C1 - HSE, PLL_CPU0, CPU_C3
*/
}



int main(void)
{

 	PLL_init();
	PortD_ADC_Init();			// Здесь висят АЦП
	PortB_Init();				// Сюда приходит строб
	ADC_Configuration();
	ADC_Detector_Configuration();
	//Setup_USB();
	//USB_CDC_Init(RxUSB_Buffer, 1, SET);					// SET обозначает, что устройство примет вх данные сразу после вызова USB_CDC_Reset в первый раз
	//USB_CDC_Reset();
	//VCom_Configuration();
	Timer3_Init();

	float interval_step = (FINAL_INTERVAL_VALUE - INITIAL_INTERVAL_VALUE) / NUMBER_OF_INTERVALS;
	    Interval *intervals = (Interval *)malloc(sizeof(Interval) * NUMBER_OF_INTERVALS);
	    for (int i = 0; i < NUMBER_OF_INTERVALS; ++i)
	    {
	        Interval *interval = (Interval *)malloc(sizeof(Interval));
	        interval->numberOfEntities = 0;
	        interval->left = i * interval_step;
	        interval->right = (i + 1) * interval_step;
	        intervals[i] = *interval;
	    }

	while (1)

	{
		switch (Command_from_USB)
		{
			case RESET:

			break;

			case START:
				if (Flag_New_Sample != 0)
					{
						Sample_Voltage = (V_Ref / ADC_resolution) * (float)(Vspec & 0x00000FFF);
						put(Sample_Voltage, intervals);
						Flag_New_Sample = 0;
					}
			break;

			case STOP:

			break;

			case RESULT:

			break;
		}


	}
}




USB_Result USB_CDC_RecieveData(uint8_t* Buffer, uint32_t Length)
{
	ParseRxMessage rxMessage = RxParser(RxUSB_Buffer);

	uint8_t sendBuffer[11] = {0};
	Command_Handler(&sendBuffer[0], rxMessage, Interval *intervals);  // -&

	USB_Result result = USB_CDC_SendData(sendBuffer, 11);

	switch (rxMessage.CommandData)
	{
		case START:
			// Пуск таймера 3
			TIMER_Cmd(MDR_TIMER3, ENABLE);
		break;

		case STOP:
			// Останов таймера 3
			TIMER_Cmd(MDR_TIMER3, DISABLE);
		break;

		default:
		break;
	}


	return result;
}

// USB_CDC_HANDLE_GET_LINE_CODING implementation example
USB_Result USB_CDC_GetLineCoding(uint16_t wINDEX, USB_CDC_LineCoding_TypeDef* DATA)
{
  assert_param(DATA);
  if (wINDEX != 0)
  {
    // Invalid interface
    return USB_ERR_INV_REQ;
  }

  //Just store received settings
  *DATA = LineCoding;
  return USB_SUCCESS;
}

// USB_CDC_HANDLE_SET_LINE_CODING implementation example
USB_Result USB_CDC_SetLineCoding(uint16_t wINDEX, const USB_CDC_LineCoding_TypeDef* DATA)
{
  assert_param(DATA);
  if (wINDEX != 0)
  {
    // Invalid interface
    return USB_ERR_INV_REQ;
  }

  // Just send back settings stored earlier
  LineCoding = *DATA;
  return USB_SUCCESS;
}

//==========================================Interrupts===========================================================

// прерывание таймера
	void Timer3_IRQHandler()
	{
		//Reg_CNT = MDR_TIMER3->CNT;
		//Reg_CCR = MDR_TIMER3->CCR3;
		//Flag_Timer3 = TIMER_GetStatus(MDR_TIMER3);
	   if(TIMER_GetITStatus(MDR_TIMER3, TIMER_STATUS_CCR_CAP_CH3 ) == SET) // прерывание по спаду
	   {
		  MDR_ADC->ADC1_CFG |= 0x00000002;		 // Включили АЦП
		  //Ждём окончания преобразования АЦП
		 // while (!ADC1_GetFlagStatus(ADC1_FLAG_END_OF_CONVERSION))
		   while (!(MDR_ADC->ADC1_STATUS & 0x00000004))
		  {
		  }
		  Vspec = MDR_ADC->ADC1_RESULT;
		  Flag_New_Sample = 1;
		 // Reg_CNT1 = MDR_TIMER3->CNT;
		 // Reg_CCR = MDR_TIMER3->CCR3;
		  //ADC_Spec_Value[i] = Vspec;
		  //i = i + 1;
		  MDR_TIMER3->STATUS = ~(MDR_TIMER3->STATUS & MDR_TIMER3->IE);		//вариант затирания флагов
		  TIMER_ClearFlag(MDR_TIMER3, (TIMER_STATUS_CCR_CAP_CH3 | TIMER_STATUS_CCR_CAP1_CH3 | TIMER_STATUS_CNT_ARR | TIMER_STATUS_CNT_ZERO));
		  Flag_Timer3 = TIMER_GetStatus(MDR_TIMER3);

	   }

	   MDR_TIMER3->STATUS = 0;
	   Flag_Timer3 = TIMER_GetStatus(MDR_TIMER3);
	}

	void ADC_IRQHandler()
	{
		Flag_ADC = ADC1_GetStatus();
		Flag_ADC2 = ADC2_GetStatus();

		Flag_EOCIF = ADC1_GetFlagStatus(ADC1_FLAG_END_OF_CONVERSION);
		Flag_EOCIF2 = ADC2_GetFlagStatus(ADC2_FLAG_END_OF_CONVERSION);

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

		//========================КОСТЫЛИ=======================
		ADC1_ClearOutOfRangeFlag();
		ADC1_ClearOverwriteFlag();
		ADC2_ClearOutOfRangeFlag();
		ADC2_ClearOverwriteFlag();
		Flag_ADC = ADC1_GetStatus();
		Flag_ADC2 = ADC2_GetStatus();

	}
