/*******************************************************************************
 * NXP Semiconductors
 * (c) Copyright 2018 NXP Semiconductors
 * ALL RIGHTS RESERVED.
 ********************************************************************************
Services performed by NXP in this matter are performed AS IS and without any
warranty. CUSTOMER retains the final decision relative to the total design
and functionality of the end product. NXP neither guarantees nor will be held
liable by CUSTOMER for the success of this project.
NXP DISCLAIMS ALL WARRANTIES, EXPRESSED, IMPLIED OR STATUTORY INCLUDING,
BUT NOT LIMITED TO, IMPLIED WARRANTY OF MERCHANTABILITY OR FITNESS FOR
A PARTICULAR PURPOSE ON ANY HARDWARE, SOFTWARE ORE ADVISE SUPPLIED
TO THE PROJECT BY NXP, AND OR NAY PRODUCT RESULTING FROM NXP SERVICES.
IN NO EVENT SHALL NXP BE LIABLE FOR INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING
OUT OF THIS AGREEMENT.
CUSTOMER agrees to hold NXP harmless against any and all claims demands
or actions by anyone on account of any damage, or injury, whether commercial,
contractual, or tortuous, rising directly or indirectly as a result
of the advise or assistance supplied CUSTOMER in connection with product,
services or goods supplied under this Agreement.
********************************************************************************
 * File:             main.c
 * Owner:            Daniel Martynek
 * Version:          1.0
 * Date:             Sep-28-2018
 * Classification:   General Business Information
 * Brief:            RTC in VLPS
********************************************************************************
********************************************************************************
 * Detailed Description:
 * This example shows use of RTC in VLPS mode.
 * The MCU is put into the VLPS mode (Sleep-On-Exit), RTC alarm interrupt brings it to VLPR every 3s and toggles BLUE LED (PTD0).
 * Since it works in the Sleep-On-Exit mode, after the ISR, the MCU goes to VLPS again without calling the WFI instruction.
 * When BTN0 (S32K144 EVB) is pressed, the power mode switch from VLPS to VLPR and other way round.
 * Interrupt is triggered on rising edge (PTC12), filtered by digital filter (clocked from LPO).
 * In VLPR, RTC seconds interrupt is enabled as well and toggles RED LED (PTD15) in the ISR.
 * RTC_CLKOUT (1Hz) and CLKOUT (bus_clk) can be monitored at PTD13 and PTD14 respectively.
 * CLKOUT is not available in VLPS.
 * The MCU needs to be power-cycled run stand-alone.
 * ------------------------------------------------------------------------------
 * Test HW:         S32K144EVB-Q100
 * MCU:             S32K 0N57U
 * Debugger:        S32DSR1
 * Target:          internal_FLASH
********************************************************************************
Revision History:
1.0     Sep-28-2018     Daniel Martynek    Initial Version
*******************************************************************************/

/*******************************************************************************
 * Includes
*******************************************************************************/
#include "S32K144.h" /* include peripheral declarations S32K144 */

#include "peripherlas.h"

/*******************************************************************************
 * Macros and constants
*******************************************************************************/
#define RED 15
#define BLUE 0

/*******************************************************************************
 * Global variables
*******************************************************************************/
uint32_t volatile vlps = 1;

/*******************************************************************************
 * Local functions prototypes
*******************************************************************************/
void RTC_IRQHandler(void);
void RTC_Seconds_IRQHandler(void);
void PORTC_IRQHandler(void);
void delay(uint32_t d);
void toggle_LED(const uint32_t led);

/*******************************************************************************
 * Main function
*******************************************************************************/
int main(void)
{

	init_PORT();
	init_RTC();
	clkout_bus();
	init_NVIC();
	enter_VLPR();

	while(1)
	{
		if(vlps) enter_VLPS();
	}
}

/*******************************************************************************
Function: RTC_IRQHandler
*******************************************************************************/
void RTC_IRQHandler(void)
{
	toggle_LED(BLUE);

	RTC->TAR += 3; // Writing to the TAR clears the SR[TAF] Time Alarm Flag.
	// Next alarm in 3s
}

/*******************************************************************************
Function: RTC_IRQHandler
*******************************************************************************/
void RTC_Seconds_IRQHandler(void)
{
	toggle_LED(RED);
}

/*******************************************************************************
Function: RTC_IRQHandler
*******************************************************************************/
void PORTC_IRQHandler(void)
{
	PORTC->PCR[12] |= (1 << 24);
	// [24] ISF = 1 Clear the interrupt flag

	if(vlps)
	{
		vlps = 0; // VLPR mode

		RTC->IER |= (1 << 4);
		// [4] TSIE = 1 Time Seconds Interrupt Enabled

		S32_SCB->SCR = 0x04;
		// [4] SEVONPEND = 0 only enabled interrupts can wakeup
		// [2] SLEEPDEEP = 1 deep sleep
		// [1] SLEEPONEXIT = 0 do not enter deep sleep on return from an ISR

		(void)S32_SCB->SCR; // make sure it is written before it leaves the ISR
	}
	else
	{
		vlps = 1; // VLPS mode

		RTC->IER &= ~(1 << 4);
		// [4] TSIE = 1 Time Seconds Interrupt Disabled
	}
}
/*******************************************************************************
Function: delay
*******************************************************************************/
void delay(uint32_t d)
{
	while(d--);
}
/*******************************************************************************
Function: toggle_LED
*******************************************************************************/
void toggle_LED(uint32_t led)
{
	PTD->PCOR |= (1 << led); // Blue LED on
	delay(5000);
	PTD->PSOR |= (1 << led); // Blue LED off
}



