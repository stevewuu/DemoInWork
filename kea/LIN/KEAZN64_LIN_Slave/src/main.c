/*
 * main.c
 *
 *  Created on: Sep 26, 2017
 *      Author: 4337
 */


#include "derivative.h" /* include peripheral declarations */
#include "lin.h"
#include "ics.h"
#include "gpio.h"
/**********************************************************************************************
* Constants and macros
**********************************************************************************************/
#define MASTER 0

#define LED0_TOGGLE		OUTPUT_TOGGLE(PTC,PTC0)
#define LED1_TOGGLE		OUTPUT_TOGGLE(PTC,PTC1)
#define LED2_TOGGLE		OUTPUT_TOGGLE(PTC,PTC2)
#define LED3_TOGGLE		OUTPUT_TOGGLE(PTC,PTC3)
                                               
#define LED0_OFF		OUTPUT_CLEAR(PTC,PTC0);
#define LED1_OFF		OUTPUT_CLEAR(PTC,PTC1);
#define LED2_OFF		OUTPUT_CLEAR(PTC,PTC2);
#define LED3_OFF		OUTPUT_CLEAR(PTC,PTC3);
                                               
#define LED0_ON			OUTPUT_SET(PTC,PTC0);  
#define LED1_ON			OUTPUT_SET(PTC,PTC1);  
#define LED2_ON			OUTPUT_SET(PTC,PTC2);  
#define LED3_ON			OUTPUT_SET(PTC,PTC3);  

#if 0
#define OUTPUT  1
#define INPUT	0

#define PORT_A  A
#define PORT_B  A
#define PORT_C  A
#define PORT_D  A

#define PORT_E	B
#define PORT_F	B
#define PORT_G	B
#define PORT_H	B

#define CONFIG_PIN_AS_GPIO(port,register_number,mode)    XCONFIG_PIN_AS_GPIO(port,register_number,mode)
#define XCONFIG_PIN_AS_GPIO(port,register_number,mode)   (mode == 0) ? (GPIO##port##_PDDR |= 0 << register_number) : (GPIO##port##_PDDR |= 1 << register_number)

#define ENABLE_INPUT(port,pin_number) 					XENABLE_INPUT(port,pin_number)
#define XENABLE_INPUT(port,pin_number)					GPIO##port##_PIDR ^= 1<<pin_number

#define ENABLE_PULLUP(port,pin_number) 					XENABLE_PULLUP(port,pin_number)
#define XENABLE_PULLUP(port,pin_number) 				PORT_PUE0 |= PORT_PUE0_PT##port##PE##pin_number##_MASK

#define OUTPUT_SET(port,register_num)				XOUTPUT_SET(port,register_num)
#define XOUTPUT_SET(port,register_num)				GPIO##port##_PSOR |=1<<register_num

#define OUTPUT_CLEAR(port,register_num)				XOUTPUT_CLEAR(port,register_num)
#define XOUTPUT_CLEAR(port,register_num)			GPIO##port##_PCOR |=1<<register_num

#define OUTPUT_TOGGLE(port,register_num)			XOUTPUT_TOGGLE(port,register_num)
#define XOUTPUT_TOGGLE(port,register_num)			GPIO##port##_PTOR |=1<<register_num
#endif

/**********************************************************************************************
* Global variables
**********************************************************************************************/
l_u8 LIN_counter =0, LED_counter = 0;

/***********************************************************************************************
*
* @brief    CLK_Init - Initialize the clocks to run at 20 MHz from the 10Mhz external XTAL
* @param    none
* @return   none
*
************************************************************************************************/
void Clk_Init()
{
	/* Perform processor initialization */
	ICS_ConfigType ICS_set;					/* Declaration of ICS_setup structure */

	ICS_set.u8ClkMode=ICS_CLK_MODE_FEI;
	ICS_set.bdiv=1;
	ICS_Init(&ICS_set);						/*Initialization of Core clock at 32.768 MHz, bus clock at 16.384 MHz*/

}
/***********************************************************************************************
*
* @brief    GPIO_Init - Initialize the pins for input/output
* @param    none
* @return   none
*
************************************************************************************************/
void GPIO_Init()
{
	#if 0
	CONFIG_PIN_AS_GPIO(PORT_C,16,OUTPUT); /* Configure LED 0 (PTC0) as an output */
	CONFIG_PIN_AS_GPIO(PORT_C,17,OUTPUT); /* Configure LED 1 (PTC1) as an output */

#if defined(MCU_SKEAZ1284)

	CONFIG_PIN_AS_GPIO(PORT_E,4,INPUT); /* Configure BTN0 (PTD0) as an input */
	CONFIG_PIN_AS_GPIO(PORT_E,5,INPUT); /* Configure BTN1 (PTD1) as an input */
	ENABLE_INPUT(PORT_E,4);			 /* Enable input SW1*/
	ENABLE_INPUT(PORT_E,5);			/*  Enable input SW2*/
#elif defined(MCU_SKEAZN642)
	CONFIG_PIN_AS_GPIO(PORT_D, 24, INPUT);
	CONFIG_PIN_AS_GPIO(PORT_D, 25, INPUT);
	ENABLE_INPUT(PORT_D, 24);
	ENABLE_INPUT(PORT_D, 25);
#else
	/* Configure the push buttons of your device. */
#endif

#endif
	CONFIG_PIN_AS_GPIO(PTC,PTC0,OUTPUT);
	CONFIG_PIN_AS_GPIO(PTC,PTC1,OUTPUT);
	CONFIG_PIN_AS_GPIO(PTC,PTC2,OUTPUT);
										
	CONFIG_PIN_AS_GPIO(PTD,PTD0,INPUT); 
	CONFIG_PIN_AS_GPIO(PTD,PTD1,INPUT); 
	ENABLE_INPUT(PTD,PTD0);			 	
	ENABLE_INPUT(PTD,PTD1);				
										
	LED0_OFF;							/* Turn off LED0 */
	LED1_OFF;							/* Turn off LED1 */
	LED2_OFF;							/* Turn off LED2 */


}
/***********************************************************************************************
*
* @brief   lin_application_timer_FTM0 - Initialize the timer for LIN application
* @param    none
* @return   none
*
************************************************************************************************/
#if MASTER
void lin_application_timer_FTM0()
{
	SIM_SCGC |= SIM_SCGC_FTM0_MASK; /* Enable Clock for FTM0 */
	FTM0_SC |= FTM_SC_PS(7);	/* Select Preescaler in this case 128. 20 Mhz /128 =156.25 Khz. */
									/* Counter increase by one every 6.4 us */
		/* Enable Channle 0*/
	FTM0_C0SC |= FTM_CnSC_CHIE_MASK; /* Enable channel 0 interrupt */
	FTM0_C0SC |= FTM_CnSC_MSA_MASK;  /* Channel as Output compare mode */
		/*Select interrupt frequency*/
	FTM0_C0V = FTM_CnV_VAL(189) ;	 	/* Interrupt every 2.5ms */

	FTM0_SC |= FTM_SC_CLKS(1); /*FTM0 use system clock*/

	/* Set the ICPR and ISER registers accordingly */
	NVIC_EnableIRQ(FTM0_IRQn);
}
#endif
int main(void)
{
#if MASTER
    l_u8 ret;
    l_u32 i;
    l_u16 LIN_resp;
    Clk_Init();
    GPIO_Init();

    l_sys_init();
	l_ifc_init(BootProtocol);
	lin_application_timer_FTM0();
	l_sch_set(BootProtocol, BootProtocol_NormalTable, 0);

	NVIC_EnableIRQ(UART0_IRQn);
	for(;;) {

			/* When pressed SW1*/
		   	if(GPIOA_PDIR & (1 << 24))
		   	{
		   		l_u8_wr_BootProtocol_BootCMD(0x01);
		   		GPIOA_PSOR |= (1 << 16);

		   	}
		   	else
		   	{
		   		l_u8_wr_BootProtocol_BootCMD(0x00);
		   		GPIOA_PCOR |= (1 << 16);
		   	}

		}
#else

	Clk_Init();
	GPIO_Init();
	
	l_sys_init();
	l_ifc_init(BootProtocol);
	
	NVIC_ClearPendingIRQ(UART0_IRQn);
	NVIC_EnableIRQ(UART0_IRQn);
	
	l_u16_wr_BootProtocol_BootStatus(0x1);
	for(;;)
	{
		if(0x1 == l_u8_rd_BootProtocol_BootCMD())
		{
			LED0_ON;
		}
		else
		{
			LED0_OFF;
		}

	}
#endif
}

#if MASTER
void FTM0_IRQHandler()
  {
    if (1==((FTM0_C0SC & FTM_CnSC_CHF_MASK)>>FTM_CnSC_CHF_SHIFT) )  /* If the CHF of the channel is equal to 0 */
  	{
  		(void)FTM0_C0SC;  							/* Read to clear flag */
  		FTM0_C0SC ^= FTM_CnSC_CHF_MASK;  			/* Clear flag */
  		FTM0_C0V = FTM0_C0V + 189 ; /* Refresh interrupt period */

  		if (LIN_counter>=6){
  		    /* Activate LIN frame transfer for every 15ms */
  		    l_sch_tick(BootProtocol);
  		    /* Reset counter */
  		    LIN_counter = 0;
  		  }

	  LIN_counter++;
  	}
  }
#endif
