/*
 * boot_uart.c
 *
 *  Created on: 2018Äê1ÔÂ31ÈÕ
 *      Author: zlg-b4337
 */

#include "device_registers.h"
#include "boot_uart.h"

void boot_uart_init()
{
	PCC->PCCn[PCC_LPUART0_INDEX] &= ~PCC_PCCn_CGC_MASK;
	PCC->PCCn[PCC_LPUART0_INDEX] |= PCC_PCCn_PCS(1)
	                             |  PCC_PCCn_CGC_MASK;

	LPUART0->BAUD = 0x1F000034;
	/* Initialize for 9600 baud, 1 stop: */
	/* SBR=52 (0x34): baud divisor = 8M/9600/16 = ~52 */
	/* OSR=15: Over sampling ratio = 15+1=16 */
	/* SBNS=0: One stop bit */
	/* BOTHEDGE=0: receiver samples only on rising edge */
	/* M10=0: Rx and Tx use 7 to 9 bit data characters */
	/* RESYNCDIS=0: Resync during rec'd data word supported */
	/* LBKDIE, RXEDGIE=0: interrupts disable */
	/* TDMAE, RDMAE, TDMAE=0: DMA requests disabled */
	/* MAEN1, MAEN2,  MATCFG=0: Match disabled */

	LPUART0->CTRL = LPUART_CTRL_RE_MASK |
					LPUART_CTRL_TE_MASK |
					LPUART_CTRL_RIE_MASK;
	/* Enable transmitter & receiver, no parity, 8 bit char: */
	/* RE=1: Receiver enabled */
	/* TE=1: Transmitter enabled */
	/* PE,PT=0: No hw parity generation or checking */
	/* M7,M,R8T9,R9T8=0: 8-bit data characters*/
	/* DOZEEN=0: LPUART enabled in Doze mode */
	/* RIE = 1 */
	/* TxDIR=0: TxD pin is input if in single-wire mode */
	/* TXINV=0: TRansmit data not inverted */
	/* RWU,WAKE=0: normal operation; rcvr not in statndby */
	/* IDLCFG=0: one idle character */
	/* ILT=0: Idle char bit count starts after start bit */
	/* SBK=0: Normal transmitter operation - no break char */
	/* LOOPS,RSRC=0: no loop back */
	S32_NVIC->ISER[LPUART0_RxTx_IRQn >> 5U] |= (1 << ((uint32_t)(LPUART0_RxTx_IRQn) & (uint32_t)0x1FU));

}
/*
 * lpuart_put_char : Function to Transmit single Char
 */
void lpuart_put_char(LPUART_Type * pLPUART, char send)
{
	/* Wait for transmit buffer to be empty */
	while((pLPUART->STAT & LPUART_STAT_TDRE_MASK)>>LPUART_STAT_TDRE_SHIFT==0);
	pLPUART->DATA=send;              /* Send data */
}
/*
 * lpuart_get_char : Function to Receive single Char
 */
void lpuart_get_char(LPUART_Type * pLPUART, char * data)
{
	char recieve;
	/* Wait for received buffer to be full */
	while((pLPUART->STAT & LPUART_STAT_RDRF_MASK)>>LPUART_STAT_RDRF_SHIFT==0);
	recieve= pLPUART->DATA;            /* Read received data*/
	*data = recieve;
}
#if 0
/*
 * LPUART0_RxTx_IRQHandler : LPUART0 interrupt routine
 */
void LPUART0_RxTx_IRQHandler(void)
{
	uint8_t byte;

	if(LPUART0->STAT & LPUART_STAT_RDRF_MASK)
	{

		byte = LPUART0->DATA;
		xmodem_queue_byte(byte);

	}
	if(LPUART0->STAT & LPUART_STAT_OR_MASK)
	{
		byte = LPUART0->DATA;
		LPUART0->STAT |= LPUART_STAT_OR_MASK;
	}
}
#endif
