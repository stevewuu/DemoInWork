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
 * File:             peripherals.c
 * Owner:            Daniel Martynek
 * Version:          1.0
 * Date:             Sep-28-2018
 * Classification:   General Business Information
 * Brief:            RTC in VLPS
********************************************************************************/

/*******************************************************************************
Revision History:
1.0     Sep-28-2018     Daniel Martynek    Initial Version
*******************************************************************************/

/*******************************************************************************
 * Includes
*******************************************************************************/
#include "S32K144.h" /* include peripheral declarations S32K144 */
#include "s32_core_cm4.h"
#include "peripherlas.h"

/*******************************************************************************
Function: init_RTC
Notes   : 32kHz RTC clock derived from 128kHz internal LPO
        : Alarm in 3s, interrupt enabled
        : RTC_CLKOUT output 1Hz CLK
*******************************************************************************/
void init_RTC(void)
{
    // Power-on cycle the MCU before writing to this register
    SIM->LPOCLKS = 0x0000001B;
    // [5-4] RTCCLKSEL = 0b01 32 kHz LPO_CLK
    // [3-2] LPOCLKSEL = 0b10 32 kHz LPO_CLK which is derived from the 128 kHz LPO_CLK
    // [1] LPO32KCLKEN = 1 Enable 32 kHz LPO_CLK output
    // [0] LPO1KCLKEN = 1 Enable 1 kHz LPO_CLK output

    // Peripheral Clock Control for RTC
    PCC->PCCn[PCC_RTC_INDEX] = PCC_PCCn_CGC_MASK; // Enable clock to RTC module - BUS_CLK

    RTC->IER = 0x00000004;
    // [18-16] TSIC = 0x000 1 Hz second interrupt
    // [4] TSIE = 0 Time Seconds Interrupt Disable
    // [2] TAIE = 1 Time alarm flag does generate an interrupt
    // [1] TOIE = 0 Time overflow flag does not generate an interrupt

    // Writing to RTC_TSR when the time counter is disabled will clear Time Invalid Flag
    RTC->TSR = 0x00000000;

    // RTC_TAR Alarm Register
    RTC->TAR = 3;    // Alarm in 3s

    // The prescaler output clock is output on RTC_CLKOUT pin
    RTC->CR = 0x01000000;
    // [24] CPE = 1 RTC_CLKOUT is enabled
    // [9] CLKO = 0 The 32 kHz clock is not output to other peripherals.
    // [7] LPOS = 0 RTC prescaler increments using 32kHz
    // [5] CPS = 0 The prescaler output clock (as configured by TSIC) is output on RTC_CLKOUT

    RTC->SR = 0x00000010;
    // [4] TCE = 1 (Time Counter Enable)

    RTC->LR = 0x00; // lock SR, CR, LR, TCR registers
}

/*******************************************************************************
Function: init_PORT
Notes   : PTD0 BLUE LED, PTD15 RED LED, PTD13 RTC_CLKOUT, PTD14 CLKOUT,
        : PTC12 BTN0, digital filter 1ms (LPO)
*******************************************************************************/
void init_PORT(void)
{
    PCC-> PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK; // Enable clock to PortD - BUS_CLK
    PCC-> PCCn[PCC_PORTC_INDEX] = PCC_PCCn_CGC_MASK; // Enable clock to PortC - BUS_CLK

    // PTD0, BLUE LED, S32K144 EVB
    PTD->PDDR |= (1 << 0);             // Data Direction (output)
    PORTD->PCR[0] = 0x00008100;
    // [15] LK = 1 Pin Control Register fields [15:0] are locked
    // [10-8] MUX = 0b001 GPIO
    PTD-> PSOR |= (1 << 0);            // Clear Output on port D0 (LED off)

    // PTD15, RED LED, S32K144 EVB
    PTD->PDDR |= (1 << 15);            // Data Direction (output)
    PORTD->PCR[15] = 0x00008100;
    // [15] LK = 1 Pin Control Register fields [15:0] are locked
    // [10-8] MUX = 0b001 GPIO
    PTD-> PSOR |= (1 << 15);           // Clear Output on port D15 (LED off)

    // PTD13, RTC_CLKOUT
    PORTD->PCR[13] = 0x00008700;
    // [15] LK = 1 Pin Control Register fields [15:0] are locked
    // [10-8] MUX = 0b111 RTC_CLKOUT

    // PTD14, CLKOUT
    PORTD->PCR[14] = 0x00008700;
    // [15] LK = 1 Pin Control Register fields [15:0] are locked
    // [10-8] MUX = 0b111 CLKOUT

    // PTC12, BTN0 S32K144 EVB
    PTC->PDDR &= ~(1 << 12);             // Data Direction (input)
    PORTC->PCR[12] = 0x00098100;
    // [19�16] IRQC = 0b1001 ISF flag and Interrupt on rising-edge
    // [15] LK = 1 Pin Control Register fields [15:0] are locked
    // [10-8] MUX = 0b001 GPIO

    PORTC->DFCR = 0x00000001;
    // [0] CS = 1 Digital filters are clocked by the LPO clock

    PORTC->DFWR = 0x0000001F;
    // [4-0] FILT = 32  (1/32kHz) * 32 = 1ms

    PORTC->DFER |= (1 << 12);
    // Digital filter is enabled on the PTC12 pin,
}

/*******************************************************************************
Function: clkout_bus
*******************************************************************************/
void clkout_bus(void)
{
    SIM->CHIPCTL &= ~(1 << 11); //Disable CLKOUT
    SIM->CHIPCTL |= (9 << 4);   //BUS_CLK
    SIM->CHIPCTL |= (1 << 11);  //Enable CLKOUT
    // [11] CLKOUTEN = 1 CLKOUT enable
    // [7-4] CLKOUTSEL = 0b1001 BUS_CLK
}

/*******************************************************************************
Function: init_SIRC
Notes   : SIRC 8MHz, SIRCDIV1 1MHz, SIRCDIV2 1MHz
        : SIRC disabled in VLPS
*******************************************************************************/
void init_SIRC(void)
{
    SCG->SIRCCSR &= ~ (1 << 24);
    // [24] LK = 0 Unlock Control Status Register

    SCG->SIRCCSR |= 0x01;
    // [2] SIRCLPEN = 0 Slow IRC is disabled in VLP modes
    // [1] SIRCSTEN = 0 Slow IRC is disabled in Stop modes
    // [0] SIRCEN = 1 Slow IRC is enabled

    SCG->SIRCDIV |= 0x0404;
    // [10-8] SIRCDIV2 0b100 Divide by 8 (1MHz)
    // [2-0]  SIRCDIV1 0b100 Divide by 8 (1MHz)

    while((SCG->SIRCCSR & (1 << 24)) == 0); // wait until clock is valid
    // [24] SIRCVLD = 1 Slow IRC is enabled and output clock is valid

    SCG->SIRCCSR |= (1 << 24);
    // [24] LK = 1 lock Control Status Register

}

/*******************************************************************************
Function: init_VLPR
Notes   : Max frequencies in VLPR: Core/System 4MHz, Bus: 2MHz, Flash: 1MHz
        : See e11063 mask 0N57U
*******************************************************************************/
void init_VLPR(void)
{
    SCG->VCCR = 0x02010013;
    // [27-24] SCS = 2     Slow IRC (SIRC_CLK 8MHZ)
    // [19-16] DIVCORE = 1 Divide by 2  (4 MHz)
    // [7-4] DIVBUS = 1    Divide core by 2  (2 MHz)
    // [3-1] DIVSLOW = 3   Divide core by 4  (1 MHz)
}

/*******************************************************************************
Function: switch_to_SIRC_in_RUN
*******************************************************************************/
void switch_to_SIRC_in_RUN(void)
{
    uint32_t srie = RCM->SRIE;
    RCM->SRIE = 0x0000; // configure all reset sources to be 慠eset' (not as Interrupt)
    RCM->SRIE = 0xFFFF; // Program each reset source as Interrupt via RCM_SRIE
    // for a minimum delay time of 10 LPO.

    SCG->RCCR = 0x02010013;
    // [27-24] SCS = 2  Slow IRC (SIRC_CLK 8MHZ)
    // [19-16] DIVCORE = 1 Divide by 2  (4 MHz)
    // [7-4] DIVBUS = 1     Divide core by 2  (2 MHz)
    // [3-1] DIVSLOW = 3   Divide core by 4  (1 MHz)

    while(!((SCG->CSR & (0x0F000000)) & 0x02000000));
    // [27-24] SCS = 0b0010 Slow IRC (SIRC_CLK)
    // or
    while((SCG->SIRCCSR & (1 << 25)) == 0);
    // [25] SIRCCSR = 1 Until SIRC is the system clock source

    RCM->SRIE = srie;
}

/*******************************************************************************
Function: disable_FIRC_in_RUN
*******************************************************************************/
void disable_FIRC_in_RUN(void)
{
    // When entering VLPR/VLPS mode, the system clock should be SIRC. The FIRC, SOSC,
    // and SPLL must be disabled by software in RUN mode before making any mode
    // transition.

    if(!(SCG->FIRCCSR & (1 << 25)))
    {   // [25] FIRCSEL, if FIRC is not the system clock source
        SCG->FIRCCSR &= ~(1 << 0);
        // [0] FIRCEN = 0 FIRC disabled
        while(SCG->FIRCCSR & (1 << 24));
        // [24] FIRCVLD = 0 Fast IRC is not enabled or clock is not valid
    }
}

/*******************************************************************************
Function: enter_VLPR
*******************************************************************************/
void enter_VLPR(void)
{
    init_SIRC();
    switch_to_SIRC_in_RUN();
    disable_FIRC_in_RUN();
    init_VLPR();

    SMC->PMPROT |= (1 << 5);
    // [5] AVLP = 1 VLPS allowed

    PMC->REGSC |= (1 << 0);
    // [0] BIASEN = 1 Biasing enabled

    if(SMC->PMSTAT == 0x01) // [7-0] PMSTAT: 0x01 from RUN
    {
        SMC->PMCTRL |= 0x00000040; // enter VLPR
        // [6-5] RUNM = 0b10 Very-Low-Power Run mode (VLPR)

        while(!((SMC->PMSTAT & 0xFF) & 0x4))
        {   // [7:0] PMSTAT = 0x04 VLPR
            // wait until the current power mode is VLPR
        }
    }
}

/*******************************************************************************
Function: enter_VLPS
Notes   : VLPS in Sleep-On-Exit mode
        : Should VLPS transition failed, reset the MCU
*******************************************************************************/
void enter_VLPS(void)
{
    if(SMC->PMSTAT == 0x04)
    { // [7-0] PMSTAT: 0x04 from VLPR

        S32_SCB->SCR |= 0x06;
        // [4] SEVONPEND = 0 only enabled interrupts can wakeup
        // [2] SLEEPDEEP = 1 deep sleep
        // [1] SLEEPONEXIT = 1 enter sleep, or deep sleep, on return from an ISR

        PMC->REGSC |= (1 << 1);
        // [1] CLKBIASDIS = 1 In VLPS mode, the bias current for SIRC, FIRC, PLL is disabled

        SMC->PMCTRL |= 0x00000002;
        // [2-0] STOPM = Very-Low-Power Stop (VLPS)

        SMC->PMPROT |= (1 << 5);
        // [5] AVLP = 1 VLPS allowed

        (void)SMC->PMPROT; // Read-After-Write to ensure the register is written

        STANDBY();  // Move to Stop mode

        if (SMC->PMCTRL & 0x8)
        { // [3] VLPSA The previous stop mode entry was aborted

            // If aborted
            STANDBY();  // Move to Stop mode

            if (SMC->PMCTRL & 0x8)
            { // [3] VLPSA The previous stop mode entry was aborted

                // If aborted, reset the MCU.
                S32_SCB->AIRCR = 0x05FA0004;
                // [31:16] VECTKEY = 05FA Write 0x5FA to VECTKEY, otherwise the write is ignored
                // [2] SYSRESETREQ = 1 Asserts a signal to the outer system that requests a reset
            }
        }
    }
}

/*******************************************************************************
Function: init_NVIC
*******************************************************************************/
void init_NVIC(void)
{
    // RTC_Interrupt (alarm)
    S32_NVIC->ICPR[1] = (1 << (46 % 32));
    S32_NVIC->ISER[1] = (1 << (46 % 32));
    S32_NVIC->IP[46] = 0x00;  // Priority level 0

    // PORTC_interrupt
    S32_NVIC->ICPR[1] = (1 << (61 % 32));
    S32_NVIC->ISER[1] = (1 << (61 % 32));
    S32_NVIC->IP[61] = 0x10;  // Priority level 1

    // RTC_Seconds_Interrupt
    S32_NVIC->ICPR[1] = (1 << (47 % 32));
    S32_NVIC->ISER[1] = (1 << (47 % 32));
    S32_NVIC->IP[47] = 0x20;  // Priority level 2
}
