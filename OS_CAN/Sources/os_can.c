/*
 * os_can.c
 *
 *  Created on: 2019Äê3ÔÂ4ÈÕ
 *      Author: 4337
 */

#include <stdint.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
//#include "queue.h"
//#include "timers.h"

#include "Cpu.h"
#include "pins_driver.h"
/* This file defines the specific ports and pins for Evaluation Board and
 * Validation Board. To use with Validation Board uncomment the following
 * line
 */

#define EVB

#ifdef EVB
    #define LED1            15U
    #define LED2            16U
    #define LED_GPIO        PTD
    #define LED_PORT        PORTD
    #define LED_PORT_PCC    PCC_PORTD_CLOCK
    #define BTN_GPIO        PTC
    #define BTN_PIN         13U
    #define BTN_PORT        PORTC
    #define BTN_PORT_PCC    PCC_PORTC_CLOCK
    #define BTN_PORT_IRQn   PORTC_IRQn
#else
    #define LED1            0U
    #define LED2            1U
    #define LED_GPIO        PTC
    #define LED_PORT        PORTC
    #define LED_PORT_PCC    PCC_PORTC_CLOCK
    #define BTN_GPIO        PTC
    #define BTN_PIN         13U
    #define BTN_PORT        PORTC
    #define BTN_PORT_PCC    PCC_PORTC_CLOCK
    #define BTN_PORT_IRQn   PORTC_IRQn
#endif

#define MB_IDX 				0
#define MSG_ID				0x123
#define MB_LEN				3

#define DEFAULT_TASK_PRIO	2

flexcan_msgbuff_t recv_buff[MB_LEN];
void boardSetup(void)
{
    /* Configure ports */
    PINS_DRV_SetMuxModeSel(LED_PORT, LED1,      PORT_MUX_AS_GPIO);
    PINS_DRV_SetMuxModeSel(LED_PORT, LED2,      PORT_MUX_AS_GPIO);
    PINS_DRV_SetMuxModeSel(BTN_PORT, BTN_PIN,   PORT_MUX_AS_GPIO);
#ifdef EVB
    PINS_DRV_SetPinIntSel(BTN_PORT, BTN_PIN, PORT_INT_RISING_EDGE);
#else
    PINS_DRV_SetPinIntSel(BTN_PORT, BTN_PIN, PORT_INT_FALLING_EDGE);
#endif
}
static void prvSetupHardware( void )
{

    /* Initialize and configure clocks
     *  -   Setup system clocks, dividers
     *  -   see clock manager component for more details
     */
    CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT,
                   g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);

    boardSetup();

	/* Change LED1, LED2 to outputs. */
	PINS_DRV_SetPinsDirection(LED_GPIO,  (1 << LED1) | (1 << LED2));

	/* Change BTN1 to input */
	PINS_DRV_SetPinsDirection(BTN_GPIO, ~(1 << BTN_PIN));

	/* Start with LEDs off. */
	PINS_DRV_SetPins(LED_GPIO, (1 << LED1) | (1 << LED2));

	/* Install Button interrupt handler */
    //INT_SYS_InstallHandler(BTN_PORT_IRQn, vPort_C_ISRHandler, (isr_t *)NULL);
    /* Enable Button interrupt handler */
    //INT_SYS_EnableIRQ(BTN_PORT_IRQn);

    /* The interrupt calls an interrupt safe API function - so its priority must
    be equal to or lower than configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY. */
    //INT_SYS_SetPriority( BTN_PORT_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );

}
void flexcan_callback(uint8_t instance, flexcan_event_type_t eventType,
					  uint32_t buffIdx, flexcan_state_t *flexcanState)
{
	if (FLEXCAN_EVENT_RX_COMPLETE == eventType)
	{
		switch(instance)
		{
			case 0 :
				FLEXCAN_DRV_Receive(INST_CANCOM1, 1, &recv_buff[0]);
			break;

			default:
			break;

		}
	}
}
void CanRecvTask(void *pvParameters)
{
	flexcan_data_info_t rx_info =
	{
		.data_length = 8U,
		.msg_id_type = FLEXCAN_MSG_ID_STD,
		.enable_brs = false,
		.fd_enable = false,
		.fd_padding = 0U
	};

	FLEXCAN_DRV_Init(INST_CANCOM1, &canCom1_State, &canCom1_InitConfig0);
	FLEXCAN_DRV_ConfigRxMb(INST_CANCOM1, MB_IDX, &rx_info, MSG_ID);
	FLEXCAN_DRV_InstallEventCallback(INST_CANCOM1, flexcan_callback, ((void *)0));
	FLEXCAN_DRV_Receive(INST_CANCOM1, 1, &recv_buff[0]);
	for(;;)
	{
		PINS_DRV_TogglePins(LED_GPIO, 1 << LED2);
		OSIF_TimeDelay(500);
	}
}
void os_can(void)
{
	/* Configure the NVIC, LED outputs and button inputs. */
	prvSetupHardware();
	xTaskCreate( CanRecvTask, "CAN_RX", configMINIMAL_STACK_SIZE, NULL, DEFAULT_TASK_PRIO, NULL );
	vTaskStartScheduler();

	for(;;);
}
#if 0
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
    volatile size_t xFreeHeapSpace;

	/* This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeHeapSpace = xPortGetFreeHeapSize();

	if( xFreeHeapSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}

}
/*-----------------------------------------------------------*/

/* The Blinky build configuration does not include run time stats gathering,
however, the Full and Blinky build configurations share a FreeRTOSConfig.h
file.  Therefore, dummy run time stats functions need to be defined to keep the
linker happy. */
void vMainConfigureTimerForRunTimeStats( void ) {}
unsigned long ulMainGetRunTimeCounterValue( void ) { return 0UL; }

/* A tick hook is used by the "Full" build configuration.  The Full and blinky
build configurations share a FreeRTOSConfig.h header file, so this simple build
configuration also has to define a tick hook - even though it does not actually
use it for anything. */
void vApplicationTickHook( void ) {}

/*-----------------------------------------------------------*/
#endif
