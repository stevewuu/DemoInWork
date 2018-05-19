/*
 * bl.c
 *
 *  Created on: 2017Äê8ÔÂ5ÈÕ
 *      Author: 4337
 */

#include "bl.h"
#include "flash.h"
#include "xmodem.h"

/**
 *  \brief boot up to application
 *  
 *  \param [in] appEntry application pointer to 
 *  \param [in] appStack stack_pointer to 
 *  \return None
 *  
 *  \details you can use this function to boot to application
 */
void bootup_application(uint32_t appEntry, uint32_t appStack)
{
    static void (*jump_to_application)(void);
    static uint32_t stack_pointer;

    //shutdown_drivers();

    jump_to_application = (void (*)(void))appEntry;
    stack_pointer = appStack;
    S32_SCB->VTOR = APP_IMAGE_START;
    __asm volatile ("MSR msp, %0\n" : : "r" (stack_pointer) : "sp");
    __asm volatile ("MSR psp, %0\n" : : "r" (stack_pointer) : "sp");
    jump_to_application();
}

/**
 *  \brief SREC line check sum caclate
 *  
 *  \param [in] data buffer to store SREC line
 *  \param [in] data_size S-line size
 *  \return checksum of S-line
 *  
 *  \details do sline checksum check
 */
uint8_t check_sum(uint8_t *data,uint8_t data_size)
{
	uint8_t i;
	uint32_t sum = 0;
	for( i = 0; i < data_size; i++)
	{
		sum += data[i];
	}
	return sum & 0xFF;
}

/**
 *  \brief download_app
 *  
 *  \return None
 *  
 *  \details boot download application function
 */
void download_app(void)
{
	xmodem_download();
}
