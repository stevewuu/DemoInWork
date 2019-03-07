/* ###################################################################
**     Filename    : main.c
**     Processor   : S32K14x
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 01.00
** @brief
**         Main module.
**         This module contains user's application code.
*/
/*!
**  @addtogroup main_module main module documentation
**  @{
*/
/* MODULE main */


/* Including necessary module. Cpu.h contains other modules needed for compiling.*/
#include "Cpu.h"

  volatile int exit_code = 0;
  uint32_t bytesRemaining;
  rtc_timedate_t tempTime;
  uint32_t print_flag;
/* User includes (#include below this line is not maintained by Processor Expert) */

/*!
  \brief The main function for the project.
  \details The startup initialization sequence is the following:
 * - startup asm routine
 * - main()
*/
#define welcomeMsg "This example is an simple echo using LPUART\r\n\
\nNow you can begin typing:\r\n"
#define PBRIDGE_RTC_SLOT    29
#define EVB

#ifdef EVB
    #define ALARM           15U
    #define SECOND          16U
    #define LED_GPIO        PTD
    #define BTN_GPIO        PTC
    #define BTN_PIN         13U
    #define BTN_PORT        PORTC
    #define BTN_PORT_IRQn   PORTC_IRQn
#else
    #define ALARM           0U
    #define SECOND          1U
    #define LED_GPIO        PTC
    #define BTN_GPIO        PTC
    #define BTN_PIN         13U
    #define BTN_PORT        PORTC
    #define BTN_PORT_IRQn   PORTC_IRQn
#endif

/* Switch to supervisor mode */
void swSupervisorMode(void)
{
    __asm(" MOVS R0, #0x0 ");
    __asm(" MSR CONTROL, R0 ");
    __asm("DSB");
    __asm("ISB");
}

/* Switch to user mode */
void swUserMode(void)
{
    __asm(" MOVS R0, #0x1 ");
    __asm(" MSR CONTROL, R0 ");
    __asm("DSB");
    __asm("ISB");
}

/* SVC Exception interrupt */
void SVC_Handler(void)
{
    /* Switch to supervisor mode need to be done through an exception handler*/
    swSupervisorMode();
}

/* Time Seconds interrupt handler */
void secondsISR(void)
{
    /* Toggle the seconds led */
    PINS_DRV_TogglePins(LED_GPIO, (1 << SECOND));
    print_flag = 1;
}
int main(void)
{
  /* Write your local variable definition here */

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of Processor Expert internal initialization.                    ***/
    uint8_t regIdx = PBRIDGE_RTC_SLOT/8;
  /* Write your code here */
  /* For example: for(;;) { } */
    CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT,
                        g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);

    /* Initialize pins
     *  -   See PinSettings component for more info
     */
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

    /* Setup alarm pin and seconds pin as output */
    PINS_DRV_SetPinsDirection(LED_GPIO, (1 << ALARM) | (1 << SECOND));
    PINS_DRV_SetPins(LED_GPIO, (1 << ALARM) | (1 << SECOND));
	LPUART_DRV_Init(INST_LPUART1, &lpuart1_State, &lpuart1_InitConfig0);
	/* Send a welcome message */
	LPUART_DRV_SendData(INST_LPUART1, (uint8_t *) welcomeMsg,
			strlen(welcomeMsg));
	/* Wait for transmission to be complete */
	while (LPUART_DRV_GetTransmitStatus(INST_LPUART1, &bytesRemaining)
			!= STATUS_SUCCESS)
		;
    /* Initialize RTC instance
     *  - See RTC configuration component for options
     */
    RTC_DRV_Init(RTCTIMER1, &rtcTimer1_Config0);
    /* Configure RTC Time Seconds Interrupt */
    RTC_DRV_ConfigureSecondsInt(RTCTIMER1, &rtcTimer1_SecIntConfig0);

    /* Set APIS to allow usermode access to RTC Memory Space */
    /* Clear bit position to grant usermode access level */
    AIPS->OPACR[regIdx] &= ~AIPS_OPACR_SP5_MASK;
    /* Switch to user mode */
    swUserMode();

    /* Set the time and date */
    RTC_DRV_SetTimeDate(RTCTIMER1, &rtcTimer1_StartTime0);

    /* Start the RTC counter */
    RTC_DRV_StartCounter(RTCTIMER1);

    /* Switch to supervisor mode by calling exception SVC_Handler */
    __asm("svc #0x32");
	while (1)
	{
		if(print_flag)
		{
			RTC_DRV_GetCurrentTimeDate(RTCTIMER1, &tempTime);
			printf("%d-%d-%d\r\n", tempTime.hour, tempTime.minutes,
					tempTime.seconds);
			print_flag = 0;
		}
	}
  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;) {
    if(exit_code != 0) {
      break;
    }
  }
  return exit_code;
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.1 [05.21]
**     for the Freescale S32K series of microcontrollers.
**
** ###################################################################
*/
