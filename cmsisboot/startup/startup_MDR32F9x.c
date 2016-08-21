/**
 ******************************************************************************
 * @file      startup_MDR32F9x.c
 * @author    Dmitry
 * @version   V1.0
 * @date      27/10/2013
 * @brief     Cortex M3 MDR32F9x Startup code.
 *            This module performs:
 *                - Set the initial SP
 *                - Set the vector table entries with the exceptions ISR address
 *                - Initialize data and bss
 *                - Call the application's entry point.
 *            After Reset the Cortex-M3 processor is in Thread mode,
 *            priority is Privileged, and the Stack is set to Main.
 *******************************************************************************
 */
 

/*----------Stack Configuration-----------------------------------------------*/  
#define STACK_SIZE       0x00000800      /*!< The Stack size suggest using even number   */
__attribute__ ((section(".co_stack")))
unsigned long pulStack[STACK_SIZE];      


/*----------Macro definition--------------------------------------------------*/  
#define WEAK __attribute__ ((weak))           


/*----------Declaration of the default fault handlers-------------------------*/  
/* System exception vector handler */
__attribute__ ((used))
/*----------Cortex-M3 vector handlers-----------------------------------------*/
void WEAK  Reset_Handler(void);   
void WEAK  NMI_Handler(void);       
void WEAK  HardFault_Handler(void); 
void WEAK  MemManage_Handler(void); 
void WEAK  BusFault_Handler(void);  
void WEAK  UsageFault_Handler(void);
void WEAK  SVC_Handler(void);       
void WEAK  DebugMon_Handler(void);  
void WEAK  PendSV_Handler(void);    
void WEAK  SysTick_Handler(void);   
/*----------Milandr vector handlers-------------------------------------------*/
void WEAK  CAN1_IRQHandler(void);
void WEAK  CAN2_IRQHandler(void);
void WEAK  USB_IRQHandler(void);
void WEAK  DMA_IRQHandler(void);
void WEAK  UART1_IRQHandler(void);
void WEAK  UART2_IRQHandler(void);
void WEAK  SSP1_IRQHandler(void);
void WEAK  I2C_IRQHandler(void);
void WEAK  POWER_IRQHandler(void);
void WEAK  WWDG_IRQHandler(void);
void WEAK  Timer1_IRQHandler(void);
void WEAK  Timer2_IRQHandler(void);
void WEAK  Timer3_IRQHandler(void);
void WEAK  ADC_IRQHandler(void);
void WEAK  COMPARATOR_IRQHandler(void);
void WEAK  SSP2_IRQHandler(void);
void WEAK  BACKUP_IRQHandler(void);
void WEAK  EXT_INT1_IRQHandler(void);
void WEAK  EXT_INT2_IRQHandler(void);
void WEAK  EXT_INT3_IRQHandler(void);
void WEAK  EXT_INT4_IRQHandler(void);

/*----------Symbols defined in linker script----------------------------------*/  
extern unsigned long _sidata;    /*!< Start address for the initialization 
                                      values of the .data section.            */
extern unsigned long _sdata;     /*!< Start address for the .data section     */    
extern unsigned long _edata;     /*!< End address for the .data section       */    
extern unsigned long _sbss;      /*!< Start address for the .bss section      */
extern unsigned long _ebss;      /*!< End address for the .bss section        */      
extern void _eram;               /*!< End address for ram                     */


/*----------Function prototypes-----------------------------------------------*/  
extern int main(void);           /*!< The entry point for the application.    */
extern void SystemInit(void);    /*!< Setup the microcontroller system(CMSIS) */
void Default_Reset_Handler(void);   /*!< Default reset handler                */
static void Default_Handler(void);  /*!< Default exception handler            */


/**
  *@brief The minimal vector table for a Cortex M3.  Note that the proper constructs
  *       must be placed on this to ensure that it ends up at physical address
  *       0x00000000.  
  */
__attribute__ ((used,section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
{	
  /*----------Core Exceptions------------------------------------------------ */
  (void *)&pulStack[STACK_SIZE],     /*!< The initial stack pointer         */
  Reset_Handler,             /*!< Reset Handler                               */
  NMI_Handler,               /*!< NMI Handler                                 */
  HardFault_Handler,         /*!< Hard Fault Handler                          */
  MemManage_Handler,         /*!< MPU Fault Handler                           */
  BusFault_Handler,          /*!< Bus Fault Handler                           */
  UsageFault_Handler,        /*!< Usage Fault Handler                         */
  0,0,0,0,                   /*!< Reserved                                    */
  SVC_Handler,               /*!< SVCall Handler                              */
  DebugMon_Handler,          /*!< Debug Monitor Handler                       */
  0,                         /*!< Reserved                                    */
  PendSV_Handler,            /*!< PendSV Handler                              */
  SysTick_Handler,           /*!< SysTick Handler                             */
  
  /*----------External Exceptions---------------------------------------------*/
  CAN1_IRQHandler,            //; IRQ0
  CAN2_IRQHandler,            //; IRQ1
  USB_IRQHandler,             //; IRQ2
  0,                          //; IRQ3  reserved
  0,                          //; IRQ4  reserved
  DMA_IRQHandler,             //; IRQ5
  UART1_IRQHandler,           //; IRQ6
  UART2_IRQHandler,           //; IRQ7
  SSP1_IRQHandler,            //; IRQ8
  0,                          //; IRQ9  reserved
  I2C_IRQHandler,             //; IRQ10
  POWER_IRQHandler,           //; IRQ11
  WWDG_IRQHandler,            //; IRQ12
  0,                          //; IRQ13 reserved
  Timer1_IRQHandler,          //; IRQ14
  Timer2_IRQHandler,          //; IRQ15
  Timer3_IRQHandler,          //; IRQ16
  ADC_IRQHandler,             //; IRQ17
  0,                          //; IRQ18 reserved
  COMPARATOR_IRQHandler,      //; IRQ19
  SSP2_IRQHandler,            //; IRQ20
  0,                          //; IRQ21 reserved
  0,                          //; IRQ22 reserved
  0,                          //; IRQ23 reserved
  0,                          //; IRQ24 reserved
  0,                          //; IRQ25 reserved
  0,                          //; IRQ26 reserved
  BACKUP_IRQHandler,          //; IRQ27
  EXT_INT1_IRQHandler,        //; IRQ28
  EXT_INT2_IRQHandler,        //; IRQ29
  EXT_INT3_IRQHandler,        //; IRQ30
  EXT_INT4_IRQHandler,        //; IRQ31
};


/**
  * @brief  This is the code that gets called when the processor first
  *         starts execution following a reset event. Only the absolutely
  *         necessary set is performed, after which the application
  *         supplied main() routine is called. 
  * @param  None
  * @retval None
  */
void Default_Reset_Handler(void)
{
  /* Initialize data and bss */
  unsigned long *pulSrc, *pulDest;

  /* Copy the data segment initializers from flash to SRAM */
  pulSrc = &_sidata;

  for(pulDest = &_sdata; pulDest < &_edata; )
  {
    *(pulDest++) = *(pulSrc++);
  }
  
  /* Zero fill the bss segment.  This is done with inline assembly since this
     will clear the value of pulDest if it is not kept in a register. */
  __asm("  ldr     r0, =_sbss\n"
        "  ldr     r1, =_ebss\n"
        "  mov     r2, #0\n"
        "  .thumb_func\n"
        "zero_loop:\n"
        "    cmp     r0, r1\n"
        "    it      lt\n"
        "    strlt   r2, [r0], #4\n"
        "    blt     zero_loop");

  /* Setup the microcontroller system. */
  SystemInit();

  /* Call the application's entry point.*/
  main();
}


/**
  *@brief Provide weak aliases for each Exception handler to the Default_Handler. 
  *       As they are weak aliases, any function with the same name will override 
  *       this definition.
  */
#pragma weak Reset_Handler = Default_Reset_Handler
#pragma weak NMI_Handler = Default_Handler     
#pragma weak HardFault_Handler = Default_Handler     
#pragma weak MemManage_Handler = Default_Handler     
#pragma weak BusFault_Handler = Default_Handler      
#pragma weak UsageFault_Handler = Default_Handler    
#pragma weak SVC_Handler = Default_Handler           
#pragma weak DebugMon_Handler = Default_Handler      
#pragma weak PendSV_Handler = Default_Handler        
#pragma weak SysTick_Handler = Default_Handler

#pragma weak CAN1_IRQHandler = Default_Handler
#pragma weak CAN2_IRQHandler = Default_Handler
#pragma weak USB_IRQHandler = Default_Handler
#pragma weak DMA_IRQHandler = Default_Handler
#pragma weak UART1_IRQHandler = Default_Handler
#pragma weak UART2_IRQHandler = Default_Handler
#pragma weak SSP1_IRQHandler = Default_Handler
#pragma weak I2C_IRQHandler = Default_Handler
#pragma weak POWER_IRQHandler = Default_Handler
#pragma weak WWDG_IRQHandler = Default_Handler
#pragma weak Timer1_IRQHandler = Default_Handler
#pragma weak Timer2_IRQHandler = Default_Handler
#pragma weak Timer3_IRQHandler = Default_Handler
#pragma weak ADC_IRQHandler = Default_Handler
#pragma weak COMPARATOR_IRQHandler = Default_Handler
#pragma weak SSP2_IRQHandler = Default_Handler
#pragma weak BACKUP_IRQHandler = Default_Handler
#pragma weak EXT_INT1_IRQHandler = Default_Handler
#pragma weak EXT_INT2_IRQHandler = Default_Handler
#pragma weak EXT_INT3_IRQHandler = Default_Handler
#pragma weak EXT_INT4_IRQHandler = Default_Handler

/**
  * @brief  This is the code that gets called when the processor receives an 
  *         unexpected interrupt.  This simply enters an infinite loop, 
  *         preserving the system state for examination by a debugger.
  * @param  None
  * @retval None  
  */
static void Default_Handler(void) 
{
	/* Go into an infinite loop. */
	while (1) 
	{
	}
}

/*********************** (C) COPYRIGHT 2013 Coocox ************END OF FILE*****/
