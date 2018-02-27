#include "derivative.h" /* include peripheral declarations */
#include "lin.h"
/**********************************************************************************************
* Constants and macros
**********************************************************************************************/
#define MASTER 1

#define LED0_TOGGLE		OUTPUT_TOGGLE(PORT_C,16)
#define LED1_TOGGLE		OUTPUT_TOGGLE(PORT_C,17)
#define LED2_TOGGLE		OUTPUT_TOGGLE(PORT_C,18)
#define LED3_TOGGLE		OUTPUT_TOGGLE(PORT_C,19)

#define LED0_OFF		OUTPUT_CLEAR(PORT_C,16);
#define LED1_OFF		OUTPUT_CLEAR(PORT_C,17);
#define LED2_OFF		OUTPUT_CLEAR(PORT_C,18);
#define LED3_OFF		OUTPUT_CLEAR(PORT_C,19);

#define LED0_ON			OUTPUT_SET(PORT_C,16);
#define LED1_ON			OUTPUT_SET(PORT_C,17);
#define LED2_ON			OUTPUT_SET(PORT_C,18);
#define LED3_ON			OUTPUT_SET(PORT_C,19);

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
//#define GPIOB_PDIR                               GPIO_PDIR_REG(GPIOB)
l_u8 LIN_counter =0;//, LED_counter = 0;
/***********************************************************************************************
*
* @brief    CLK_Init - Initialize the clocks to run at 20 MHz from the 10Mhz external XTAL
* @param    none
* @return   none
*
************************************************************************************************/
void Clk_Init()
{
	ICS_C1|=ICS_C1_IRCLKEN_MASK; 		/* Enable the internal reference clock*/
	ICS_C3= 0x58;						/* Reference clock frequency = 39.0625 KHz*/
	while(!(ICS_S & ICS_S_LOCK_MASK));   /* Wait for PLL lock, now running at 40 MHz (1024 * 39.0625Khz) */
	ICS_C2|=ICS_C2_BDIV(1)  ; 			/*BDIV=2, Bus clock = 20 MHz*/
	ICS_S |= ICS_S_LOCK_MASK ; 			/* Clear Loss of lock sticky bit */


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
	CONFIG_PIN_AS_GPIO(PORT_D,24,INPUT); /* Configure BTN0 (PTD0) as an input */
	CONFIG_PIN_AS_GPIO(PORT_D,25,INPUT); /* Configure BTN1 (PTD1) as an input */

	CONFIG_PIN_AS_GPIO(PORT_C,16,OUTPUT);
	CONFIG_PIN_AS_GPIO(PORT_C,17,OUTPUT);

	GPIOA_PIDR &= ~(1 << 24);
	GPIOA_PIDR &= ~(1 << 25);

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
	FTM0_C0V = FTM_CnV_VAL(391 * 4) ;	 	/* Interrupt every 2.5ms */

	FTM0_SC |= FTM_SC_CLKS(1); /*FTM0 use system clock*/

	/* Set the ICPR and ISER registers accordingly */
	NVIC_ClearPendingIRQ(FTM0_IRQn);
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

	NVIC_ClearPendingIRQ(UART0_IRQn);
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
	return 0;
}
#if MASTER
void FTM0_IRQHandler()
  {
    if (1==((FTM0_C0SC & FTM_CnSC_CHF_MASK)>>FTM_CnSC_CHF_SHIFT) )  /* If the CHF of the channel is equal to 0 */
  	{
  		(void)FTM0_C0SC;  							/* Read to clear flag */
  		FTM0_C0SC ^= FTM_CnSC_CHF_MASK;  			/* Clear flag */
  		FTM0_C0V = FTM0_C0V + 391 * 4 ; /* Refresh interrupt period */

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
