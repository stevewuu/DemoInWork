/*
 * config.c
 *
 *  Created on: 2017Äê7ÔÂ19ÈÕ
 *      Author: 4337
 */
#include "config.h"
#include "device_registers.h"


#define BL_PCC_CAN       PCC_PORTB_INDEX
#define BL_PCC_LED       PCC_PORTE_INDEX
#define BL_PCC_BTN       PCC_PORTC_INDEX
#define BL_PCC_UART      PCC_PORTA_INDEX

#define PORT_CAN         PORTB
#define PORT_LED         PORTE
#define PORT_BTN         PORTC
#define PORT_UART        PORTA

#define GPIO_LED         PTE
#define GPIO_BTN         PTC

#define LED0             21       /* RED LED -- E21 */
#define LED1             22       /* Green LED -- E22 */
#define BTN              13       /* BTN index -- C13 */
/**
 *  \brief WDOG_disable
 *  
 *  \return None
 *  
 *  \details Disable watchdog
 */
void WDOG_disable (void)
{
    WDOG->CNT=0xD928C520; 	        /* Unlock watchdog */
    WDOG->TOVAL=0x0000FFFF;	        /* Maximum timeout value */
    WDOG->CS = 0x00002100;          /* Disable watchdog */
}
/**
 *  \brief PORT_init
 *  
 *  \return None
 *  
 *  \details 
 */
void PORT_init (void)
{
//=============================== CAN PORT ====================================
    PCC->PCCn[BL_PCC_CAN] |= PCC_PCCn_CGC_MASK;   /* Enable clock for PORTB */
    PORT_CAN->PCR[0] |= PORT_PCR_MUX(5);          /* Port B: MUX = ALT5, CAN0_RX */
    PORT_CAN->PCR[1] |= PORT_PCR_MUX(5);          /* Port B: MUX = ALT5, CAN0_TX */
//=============================== LED PORT ====================================
    PCC->PCCn[BL_PCC_LED ]|=PCC_PCCn_CGC_MASK;    /* Enable clock for PORTE */
    PORT_LED->PCR[LED1] =  PORT_PCR_MUX(1);       /* Port E22: MUX = GPIO (to green LED) */
    GPIO_LED->PDDR |= 1<< LED1;                   /* Port E22: Data direction = output */
    GPIO_LED->PSOR |= 1<< LED1;
//=============================== BTN PORT ====================================
    PCC->PCCn[BL_PCC_BTN ]|=PCC_PCCn_CGC_MASK;    /* Enable clock for PORTC */
    PORT_BTN->PCR[BTN] =  PORT_PCR_MUX(1);        /* Port C13: MUX = GPIO (to btn0) */
    GPIO_BTN->PDDR &= ~(1<<BTN);                  /* Port C13: Data direction = output */
//=============================== UART PORT ===================================
    PCC->PCCn[BL_PCC_UART] |= PCC_PCCn_CGC_MASK;
    PORT_UART->PCR[27] |= PORT_PCR_MUX(4);
    PORT_UART->PCR[28] |= PORT_PCR_MUX(4);
}
/**
 *  \brief is_enter_triggered
 *  
 *  \return 1 : triggered if happend
 *          0 : Not triggered
 *  
 *  \details check boot signal is triggered
 */
unsigned char is_enter_triggered(void)
{
    uint16_t i;
    for(i = 0; i < 0xFFF; i++);
    if(GPIO_BTN->PDIR & (1<<BTN))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
