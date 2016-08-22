#include <MDR32F9x.h>
#include "main.h"
#include "delays.h"

PORT_InitTypeDef PORT_InitStructure;

//==========================������������===============================================================================

typedef struct
{

	uint8_t StartByte1;    		// 7E 81 ���������� ������ ���������
	uint8_t StartByte2;    		// 7E 81 ���������� ������ ���������
	uint8_t Address;   			// ADR1 or ADR2 Receiver address
	uint8_t InvAddress;			// /ADR1 or /ADR2
	uint8_t SenderAddress;		//
	uint8_t DataLenght;			// Q
	uint8_t Command;			// CMD
	uint8_t CommandData;		// CONTC
	uint8_t CheckSumLow;		// CHSL
	uint8_t CheckSumHigh;		// CHSH
	uint8_t EndByte;			// F0

} ParseRxMessage;

uint8_t RxUSB_Buffer[10];	  //����� ������ ��������� �� ��
ParseRxMessage task;                  //������� ������� �� ��: ������ �����, ����������, �����
ParseRxMessage previous_task;		  //���������� ������� �� ��

#define START_SPECTR 0x04	//����� �������
#define STOP_SPECTR 0x01	//�������
#define RESET_SPECTR 0x02	//�����

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
	RxMessage.Command = RxBuffer[6];
	RxMessage.CommandData = RxBuffer[7];
	RxMessage.CheckSumLow = RxBuffer[8];
	RxMessage.CheckSumHigh = RxBuffer[9];
	RxMessage.EndByte = RxBuffer[10];

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
	outBuffer[7] = previous_task.CommandData != RxMessage.CommandData;
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

	if (task.CommandData == previous_task.CommandData) {
		return;
	}

	TxUSB_Constructor(outBuffer, task, previous_task);

}


void PortD_Init ()
{
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTD, ENABLE); // ��� ������������ �� HSI

  // ����������� ���� 10-12 ����� D �� ����� ��� �������
  PORT_InitStructure.PORT_Pin   = (PORT_Pin_10 | PORT_Pin_11 | PORT_Pin_12);
  PORT_InitStructure.PORT_OE    = PORT_OE_OUT;
  PORT_InitStructure.PORT_FUNC  = PORT_FUNC_PORT;
  PORT_InitStructure.PORT_MODE  = PORT_MODE_DIGITAL;
  PORT_InitStructure.PORT_SPEED = PORT_SPEED_SLOW;
  PORT_InitStructure.PORT_PULL_UP = PORT_PULL_UP_ON;
  PORT_InitStructure.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
  PORT_InitStructure.PORT_PD = PORT_PD_DRIVER;

  PORT_Init(MDR_PORTD, &PORT_InitStructure); // ������������� ����� D
}

void LEDOn(uint32_t LED_Num)
{
  PORT_SetBits(MDR_PORTD, LED_Num);
}


void LEDOff(uint32_t LED_Num)
{
  PORT_ResetBits(MDR_PORTD, LED_Num);
}


//static void ADC_Configuration(void)
//{
//	// ��������� ��� ������������� ���
//	  ADC_InitTypeDef ADC_InitStructure;
//		ADCx_InitTypeDef ADCx_InitStructure;
//
//	  // ��������� ������������ ���
//	  RST_CLK_PCLKcmd (RST_CLK_PCLK_ADC, ENABLE);
//
//	  ADC_DeInit();
//	  ADC_StructInit(&ADC_InitStructure);
//
//	  // ������������ ��� �����
//	  ADC_InitStructure.ADC_SynchronousMode      = ADC_SyncMode_Independent;						// ����������� ������ ���
//	  ADC_InitStructure.ADC_StartDelay           = 0;																		// �������� ����� ��������� ���1 � ���2
//	  ADC_InitStructure.ADC_TempSensor           = ADC_TEMP_SENSOR_Enable;              // ��������� ������ �������������� �������
//	  ADC_InitStructure.ADC_TempSensorAmplifier  = ADC_TEMP_SENSOR_AMPLIFIER_Enable;    // ��������� ������ ��������� ��� �������������� �������
//	  ADC_InitStructure.ADC_TempSensorConversion = ADC_TEMP_SENSOR_CONVERSION_Enable;   // ��������� �������������� ��� ������ �������������� �������
//	  ADC_InitStructure.ADC_IntVRefConversion    = ADC_VREF_CONVERSION_Disable;         // ��������� �������������� ��� ������ VREF (���������� �����)
//	  ADC_InitStructure.ADC_IntVRefTrimming      = 1;                                   // ���������� ��������� ���������� VREF
//	  ADC_Init (&ADC_InitStructure);
//
//	  // ������������ ���1
//	  ADCx_StructInit (&ADCx_InitStructure);
//	  ADCx_InitStructure.ADC_ClockSource      = ADC_CLOCK_SOURCE_CPU;						// ����������� ��� ��� �� ��������, ��� � ���� ��
//	  ADCx_InitStructure.ADC_SamplingMode     = ADC_SAMPLING_MODE_CICLIC_CONV;	// ����� ������������ �������������� (��������� ��� ������)
//	  ADCx_InitStructure.ADC_ChannelSwitching = ADC_CH_SWITCHING_Disable;				// ����� ������������ ������� (���������)
//	  ADCx_InitStructure.ADC_ChannelNumber    = ADC_CH_TEMP_SENSOR;							// ��������� ����� ��� (������������� ������)
//	  ADCx_InitStructure.ADC_Channels         = 0;															// ��������� ������ ��� � ���������������� ������� (�� �������)
//	  ADCx_InitStructure.ADC_LevelControl     = ADC_LEVEL_CONTROL_Disable;			// �������� ������ ��������� ���������� (���������)
//	  ADCx_InitStructure.ADC_LowLevel         = 0;															// ������ �������
//	  ADCx_InitStructure.ADC_HighLevel        = 0;															// ������� �������
//	  ADCx_InitStructure.ADC_VRefSource       = ADC_VREF_SOURCE_INTERNAL;				// ��� ��������� �������� ���������� (����������)
//	  ADCx_InitStructure.ADC_IntVRefSource    = ADC_INT_VREF_SOURCE_INEXACT;	  // ��� ����������� ��������� �������� ���������� (�� ������)
//	  ADCx_InitStructure.ADC_Prescaler        = ADC_CLK_div_512;								// ������������ ������� ������������ ��� (512)
//	  ADCx_InitStructure.ADC_DelayGo          = 7;															// �������� ����� ��������� ��� (������������)
//	  ADC1_Init (&ADCx_InitStructure);
//
//	  // ��������� ������ ���1
//	  ADC1_Cmd (ENABLE);
//
//	  // ��������� ���������� �� DMA
//	  NVIC_EnableIRQ (DMA_IRQn);
//}


//======================================================================================================================================================

static void VCom_Configuration(void)
{
	LineCoding.dwDTERate = 9600;
	LineCoding.bCharFormat = 0;
	LineCoding.bParityType = 0;
	LineCoding.bDataBits = 8;
}

void Setup_USB(void)
{
	//������ ������������ �� USB
	RST_CLK_PCLKcmd(RST_CLK_PCLK_USB, ENABLE);

	//������������� ������ ������ USB
	USB_Clock_InitStruct.USB_USBC1_Source = USB_C1HSEdiv2;
	USB_Clock_InitStruct.USB_PLLUSBMUL = USB_PLLUSBMUL12;

	USB_DeviceBUSParam.MODE = USB_SC_SCFSP_Full;
	USB_DeviceBUSParam.SPEED = USB_SC_SCFSR_12Mb;
	USB_DeviceBUSParam.PULL = USB_HSCR_DP_PULLUP_Set;

	// ������������� ��� Device
	USB_DeviceInit(&USB_Clock_InitStruct, &USB_DeviceBUSParam);

	//��������� ��� USB ����������
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

// 7� 81 ADR1  /ADR1 ADR2  Q CMD CONTC CHSL CHSH F0





USB_Result USB_CDC_RecieveData(uint8_t* Buffer, uint32_t Length)
{
	//USB_Buf[cnt++] = Buffer[0];

	ParseRxMessage rxMessage = RxParser(Buffer);
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
