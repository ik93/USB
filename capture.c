#include "capture.h"
#include <MDR32Fx.h>
#include "MDR32F9Qx_timer.h"
#include "MDR32f9Qx_port.h"
#include "MDR32f9Qx_rst_clk.h"



uint32_t Flag_Timer3;
uint32_t Flag_Timer2;

//===============================================================Инициализация таймера для прерывания по стробу================================================
void PortB_Init()
{
	PORT_InitTypeDef TimerPortB_InitSrtucrure;

	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTB, ENABLE); // вкл тактирование от HSI

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
	RST_CLK_PCLKcmd(RST_CLK_PCLK_TIMER3, ENABLE);						// Разрешение тактовой частоты таймера 3
	TIMER_BRGInit(MDR_TIMER3, TIMER_HCLKdiv1);							// Управление тактовой частотой таймера 3

	TIMER_CntInitTypeDef	Timer_Count;
	TIMER_ChnInitTypeDef	Timer_Channel;


	TIMER_DeInit(MDR_TIMER3);

	TIMER_CntStructInit(&Timer_Count);
	TIMER_ChnStructInit(&Timer_Channel);

	Timer_Count.TIMER_CounterDirection = TIMER_CntDir_Up;
	Timer_Count.TIMER_ARR_UpdateMode = TIMER_ARR_Update_On_CNT_Overflow;
	Timer_Count.TIMER_CounterMode = TIMER_CntMode_ClkFixedDir;
	Timer_Count.TIMER_EventSource = TIMER_EvSrc_None;
	Timer_Count.TIMER_Prescaler = 0x00000001;			//частота процессора/2 (80 МГц/2)
	Timer_Count.TIMER_Period = 0x0000FFFF;
	Timer_Count.TIMER_IniCounter = 0;

	TIMER_CntInit(MDR_TIMER3, &Timer_Count);

	Timer_Channel.TIMER_CH_Number = TIMER_CHANNEL3;
	Timer_Channel.TIMER_CH_Mode = TIMER_CH_MODE_CAPTURE;
	Timer_Channel.TIMER_CH_EventSource = TIMER_CH_EvSrc_PE;
	//Timer_Channel.TIMER_CH_CCR1_Ena = DISABLE;

	TIMER_ChnInit(MDR_TIMER3, &Timer_Channel);

	NVIC_EnableIRQ(TIMER3_IRQn);


	MDR_TIMER3->IE = 0x00000400;
	MDR_TIMER3->CH3_CNTRL = 0x00008000;


	TIMER_Cmd(MDR_TIMER3, ENABLE);
	MDR_TIMER3->STATUS = 0;
	Flag_Timer3 = TIMER_GetStatus(MDR_TIMER3);
}



//Настройка ножки PD5 для работы с TIMER2_ETR
void PortD_ETR_Init()
{
	PORT_InitTypeDef TimerPortD_InitStructure;

	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTD, ENABLE); // вкл тактирование от HSI

			  PORT_DeInit(MDR_PORTD);

			  PORT_StructInit(&TimerPortD_InitStructure);
			   // инициализируем PB5 как вход таймера
			  TimerPortD_InitStructure.PORT_Pin = PORT_Pin_5;
			  TimerPortD_InitStructure.PORT_PULL_UP = PORT_PULL_UP_OFF;
			  TimerPortD_InitStructure.PORT_PULL_DOWN = PORT_PULL_DOWN_ON;
			  TimerPortD_InitStructure.PORT_FUNC = PORT_FUNC_OVERRID;
			  TimerPortD_InitStructure.PORT_MODE = PORT_MODE_DIGITAL;
			  TimerPortD_InitStructure.PORT_SPEED = PORT_SPEED_MAXFAST;
			  TimerPortD_InitStructure.PORT_GFEN = PORT_GFEN_OFF; 		// фильтр выкл. (фильтрация импульсов до 10 нс)

		  	PORT_Init(MDR_PORTD, &TimerPortD_InitStructure);
}


void TimerETR_init()
{
	RST_CLK_PCLKcmd(RST_CLK_PCLK_TIMER2, ENABLE);						// Разрешение тактовой частоты таймера 2

	/*
	   TIMER_DeInit(MDR_TIMER2);

	  TIMER_BRGInit(MDR_TIMER2,TIMER_HCLKdiv1);

	  TIMER_CntInitTypeDef			Timer2_Count;


	  Timer2_Count.TIMER_IniCounter          		= 0;
	  Timer2_Count.TIMER_Prescaler                   = 0;
	  Timer2_Count.TIMER_Period                       = 0x00000001;
	  Timer2_Count.TIMER_CounterMode              = TIMER_CntMode_EvtFixedDir;
	  Timer2_Count.TIMER_CounterDirection         = TIMER_CntDir_Up;
	  Timer2_Count.TIMER_EventSource              = TIMER_EvSrc_ETR;
	  Timer2_Count.TIMER_FilterSampling           = TIMER_FDTS_TIMER_CLK_div_1;
	  Timer2_Count.TIMER_ARR_UpdateMode           = TIMER_ARR_Update_Immediately;
	  Timer2_Count.TIMER_ETR_FilterConf           = TIMER_Filter_1FF_at_TIMER_CLK;
	  Timer2_Count.TIMER_ETR_Prescaler            = TIMER_ETR_Prescaler_None;
	  Timer2_Count.TIMER_ETR_Polarity             = TIMER_ETRPolarity_NonInverted;
	  Timer2_Count.TIMER_BRK_Polarity             = TIMER_BRKPolarity_NonInverted;
	  TIMER_CntInit (MDR_TIMER2, &Timer2_Count);


	  TIMER_Cmd(MDR_TIMER2,ENABLE);
	  MDR_TIMER2->IE = 0x00000002;			//прерывание по совпадению CNT и ARR

*/
	MDR_TIMER3->CNTRL = 0;
	MDR_RST_CLK->TIM_CLOCK = ((0 << RST_CLK_TIM_CLOCK_TIM2_BRG_Pos) // делитель тактовой частоты Таймера 2
	               | (1 << RST_CLK_TIM_CLOCK_TIM2_CLK_EN_Pos)); // разрешение тактовой частоты Таймера 2

	      MDR_TIMER2->BRKETR_CNTRL |= (0<<4|0<<2);  // не делить внешнюю частоту
	      MDR_TIMER2->ARR = 0x0000FFFF;
	      MDR_TIMER2->PSG = 0;
	      MDR_TIMER2->CNT = 0;

	      MDR_TIMER2->IE = 0x00000004;				//запретили все прерывания, кроме.переднего фронта по ETR


	  MDR_TIMER2->CNTRL = 0x00000886; 			//событие на ETR режим2
	  NVIC_ClearPendingIRQ(TIMER2_IRQn);
	  MDR_TIMER2->STATUS = 0;
	  Flag_Timer2 = MDR_TIMER2->STATUS;

	  NVIC_EnableIRQ(TIMER2_IRQn);
}






void Timer3_Start(void)
{
	//Пуск таймера
	TIMER_Cmd(MDR_TIMER3, ENABLE);

}


