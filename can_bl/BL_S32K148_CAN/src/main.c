/* main.c                              
 * Description: S32K144 CAN Bootloader,
 *              Application flash start address is 0x5000,
 *   Revisions:
 *   - rev. 0.1 (2018-05-16) - 4337 H.W. b180516
 *       Initial version.
 */

#include "device_registers.h" /* include peripheral declarations S32K144 */
#include "flexcan.h"
#include "LPSPI.h"
#include "clocks_and_modes.h"
#include "sbc_uja11691.h"

#include "config.h"
#include "xmodem.h"
#include "flash.h"
#include "bl.h"


int main(void)
{
    uint32_t appEntry, appStack;

    WDOG_disable();
    SOSC_init_16MHz();      /* Initialize system oscillator for 16 MHz xtal   */
    SPLL_init_160MHz();     /* Initialize SPLL to 160 MHz with 8 MHz SOSC     */
    NormalRUNmode_80MHz();  /* Init clocks: 80 MHz sysclk & core, 40 MHz bus,
                                                                20 MHz flash  */
    CAN_GlobeVaribleInit();
    flexcan_init();         /* Init FlexCAN0                                  */
    PORT_init();            /* Configure ports                                */
#if 0                  
    /*
     * UJA1169 still in factory mode don't need to config UJA1169
     * if configured should serve watchdog handler
     */
    /* SPI and transceiver initialization is required */
    LPSPI1_init_master();   /*Initialize LPSPI1 for communication with UJA1169*/
    SBC_Init(&sbc_uja11691_InitConfig0, 1U); /* Configure UJA1169 */
#endif
    flash_init();                     /* flash Init (FTFC) */
    xmodem_init();
    if(is_enter_triggered())          /* BTN0(sw2) pressed to download app */
    {
    	while(1)
    	{
    	    if(xmodem_is_active())
    	    {
    	    	download_app();
    	    	break;
    	    }
    	}
    }

    appStack = *(uint32_t*) APP_IMAGE_START;  /* setup app jump */
    appEntry = *(uint32_t*)(APP_IMAGE_START + 4);

    bootup_application(appEntry,appStack);
    while(1)                        /* should never be here */
    {

    }

}

