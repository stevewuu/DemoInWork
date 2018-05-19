/*
 * mian.c
 * app_for_s32k_boot
 */
#include "clocks_and_modes.h"
#include "device_registers.h"

#define APP_PCC_CAN PCC_PORTB_INDEX
#define APP_PCC_LED PCC_PORTE_INDEX
#define APP_PCC_BTN PCC_PORTC_INDEX

#define PORT_CAN PORTB
#define PORT_LED PORTE
#define PORT_BTN PORTC

#define GPIO_LED PTE
#define GPIO_BTN PTC

#define LED0 21 /* RED LED -- E21 */
#define LED1 22 /* Green LED -- E22 */
#define BTN  13 /* BTN index -- C13 */

int main (void)
{
	uint8_t i = 0;
	uint16_t j;

	SOSC_init_16MHz();      /* Initialize system oscillator for 8 MHz xtal_pin */
	SPLL_init_160MHz();    /* Initialize SPLL to 160 MHz with 8 MHz SOSC */
	NormalRUNmode_80MHz(); /* Initialize clocks: 80 MHz sysclk & core, 40 MHz bus,
								  20 MHz flash */

	PCC->PCCn[APP_PCC_LED] |= PCC_PCCn_CGC_MASK;
	PORT_LED->PCR[LED1] |= PORT_PCR_MUX(1);
	GPIO_LED->PDDR |= 1 << LED1;
	GPIO_LED->PCOR |= 1 << LED1;

	for(;;)
	{
		GPIO_LED->PTOR |= 1 << LED1;
		for(i = 0; i < 8; i++)
		{
			for(j = 0; j < 65535; j++);
		}
	}
}
