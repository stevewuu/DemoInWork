/*
 * boot_uart.h
 *
 *  Created on: 2018Äê1ÔÂ31ÈÕ
 *      Author: zlg-b4337
 */

#ifndef BOOT_UART_H_
#define BOOT_UART_H_

void boot_uart_init();
void lpuart_put_char(LPUART_Type * pLPUART, char send);
void lpuart_get_char(LPUART_Type * pLPUART, char * data);

#endif /* BOOT_UART_H_ */
