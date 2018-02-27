/******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2008-2015 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
******************************************************************************/
/**************************************************************************//**
* @addtogroup UART_group
* @{
******************************************************************************/
/**************************************************************************//**
*
* @file      bsp/UART/lin_isr.c
*
* @author    FPT Software
*
* @brief     Interrupt service routine used in LIN driver
*
******************************************************************************/

/******************************************************************************
 *
 * History:
 *
 * 20140422     v1.0    First version for KEA family support
 *
 *****************************************************************************/

#include "lin_hw_cfg.h"
#include "lin_lld_timesrv.h"
#include "lin_lld_uart.h"


/**********************************************************************************************
* Local functions
**********************************************************************************************/
#if _UART0_ == 1
void UART0_IRQHandler ();
/***********************************************************************************************
*
* @brief    UART0_SCI0_IRQHandler - Uart0 reception interrupt, calls the user defined callback function
* @param    none
* @return   none
*
************************************************************************************************/
void UART0_IRQHandler ()
{
#if (LIN_MODE == _MASTER_MODE_)
    lin_lld_uart_rx_isr(UART0);
#else
    lin_lld_uart_rx_isr();
#endif /* (LIN_MODE == _MASTER_MODE_) */

}
#endif /* end _UART0_ == 1 */

#if _UART1_ == 1
void UART1_SCI1_IRQHandler ();
/***********************************************************************************************
*
* @brief    UART1_SCI1_IRQHandler - Uart1 reception interrupt, calls the user defined callback function
* @param    none
* @return   none
*
************************************************************************************************/
void UART1_SCI1_IRQHandler ()
{
#if (LIN_MODE == _MASTER_MODE_)
    lin_lld_uart_rx_isr(UART1);
#else
    lin_lld_uart_rx_isr();
#endif /* (LIN_MODE == _MASTER_MODE_) */

}
#endif /* end _UART1_ == 1 */

#if _UART2_ == 1
void UART2_SCI2_IRQHandler ();
/***********************************************************************************************
*
* @brief    UART2_SCI2_IRQHandler - Uart2 reception interrupt, calls the user defined callback function
* @param    none
* @return   none
*
************************************************************************************************/
void UART2_SCI2_IRQHandler ()
{
#if (LIN_MODE == _MASTER_MODE_)
    lin_lld_uart_rx_isr(UART2);
#else
    lin_lld_uart_rx_isr();
#endif /* (LIN_MODE == _MASTER_MODE_) */

}
#endif /* end _UART2_ == 1 */

/***********************************************************************************************
*
* @brief    FTM2_IRQHandler - Interrupt for FTM channels, clear flag, set new tiemout and execute callback
* @param    none
* @return   none
*
************************************************************************************************/
#if !defined(MCU_SKEAZN84) /* Not support for KEA8 */
void FTM1_IRQHandler();
void FTM1_IRQHandler()
{
    if (1 == ((FTM1_C0SC & FTM_CnSC_CHF_MASK) >> FTM_CnSC_CHF_SHIFT))  /* If the CHF of the channel is equal to 0 */
    {
        (void)FTM1_C0SC;                            /* Read to clear flag */
        /* Clear flag */
        FTM1_C0SC ^= FTM_CnSC_CHF_MASK;
        /* Update Value of Channel Value register to generate next interrupt after TIME_BASE_PERIOD (us) */
        FTM1_C0V += ((MCU_BUS_FREQ  / 1000000.0) * (TIME_BASE_PERIOD / 128.0));
        /* Call Timer interrupt handler function */
        lin_lld_timer_isr();
    }
}
#endif /*!defined(MCU_SKEAZN84) */
