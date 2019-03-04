#include "derivative.h" /* include peripheral declarations */
#include "ics.h"
#include "nvic.h"
#include "gpio.h"
#include "uart.h"
#include "printf.h"

#include "lin.h"
#include "lin_lin21tl_api.h"
/**********************************************************************************************
* Constants and macros
**********************************************************************************************/
#define MASTER 			0

#define LED0_TOGGLE		OUTPUT_TOGGLE(PTC,PTC0)
#define LED1_TOGGLE		OUTPUT_TOGGLE(PTC,PTC1)
#define LED2_TOGGLE		OUTPUT_TOGGLE(PTC,PTC2)
#define LED3_TOGGLE		OUTPUT_TOGGLE(PTC,PTC3)

#define LED0_OFF		OUTPUT_CLEAR(PTC,PTC0);
#define LED1_OFF		OUTPUT_CLEAR(PTC,PTC1);
#define LED2_OFF		OUTPUT_CLEAR(PTC,PTC2);
#define LED3_OFF		OUTPUT_CLEAR(PTC,PTC3);

#define LED0_ON			OUTPUT_SET(PTC,PTC0);
#define LED1_ON			OUTPUT_SET(PTC,PTC1);
#define LED2_ON			OUTPUT_SET(PTC,PTC2);
#define LED3_ON			OUTPUT_SET(PTC,PTC3);


#define MASTER_REQ_LENGTH_MAX   20

l_u8 LIN_counter =0;
volatile l_u8 master_req_dat[MASTER_REQ_LENGTH_MAX];
volatile l_u16 master_req_length;
unsigned char io_control_status = 0xFF;
volatile l_u8 slave_resp_dat[MASTER_REQ_LENGTH_MAX];
volatile l_u16 slave_resp_length;
/***********************************************************************************************
*
* @brief    CLK_Init - Initialize the clocks to run at 20 MHz from the 10Mhz external XTAL
* @param    none
* @return   none
*
************************************************************************************************/
void Clk_Init()
{
  	ICS_ConfigType ICS_set;					/* Declaration of ICS_setup structure */

	ICS_set.u8ClkMode=ICS_CLK_MODE_FEI;
	ICS_set.bdiv=1;
	ICS_Init(&ICS_set);             		/*Initialization of Core clock at 48 MHz, Bus clock at 24 MHz*/
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
	CONFIG_PIN_AS_GPIO(PTC,PTC0,OUTPUT); 		/* Configure RED LED (PTH0) as an output */
	CONFIG_PIN_AS_GPIO(PTC,PTC1,OUTPUT); 		/* Configure GREEN LED (PTH1) as an output */
	CONFIG_PIN_AS_GPIO(PTC,PTC2,OUTPUT); 		/* Configure BLUE (PTE7) as an output */

	CONFIG_PIN_AS_GPIO(PTD,PTD0,INPUT); 		/* Configure BTN0 (SW2-PTE4) as an input */
	CONFIG_PIN_AS_GPIO(PTD,PTD1,INPUT); 		/* Configure BTN1 (SW3-PTE5) as an input */
	ENABLE_INPUT(PTD,PTD0);			 			/* Enable input SW2 */
	ENABLE_INPUT(PTD,PTD1);						/* Enable input SW3 */

	LED0_OFF;									/* Turn off RED */
	LED1_OFF;									/* Turn off GREEN */
	LED2_OFF;									/* Turn off BLUE*/
}
/***********************************************************************************************
*
* @brief   lin_application_timer_FTM0 - Initialize the timer for LIN application
* @param    none
* @return   none
*
************************************************************************************************/
void UartInit(void)
{
    UART_ConfigType Uart_Config={{0}};

    Uart_Config.sctrl1settings.bits.bM=0; 	/* 8 bit Mode */
    Uart_Config.sctrl1settings.bits.bPe=0; 	/* Parity disable */
    Uart_Config.bSbns=0;					/* One stop bit*/
    Uart_Config.sctrl2settings.bits.bRe=1;	/* Receiver enable*/
    Uart_Config.sctrl2settings.bits.bTe=1;	/* Transmitter enable*/
    //Uart_Config.sctrl2settings.bits.bTie=1;	/* Transmit buffer empty interrupt enable*/
    Uart_Config.u32SysClkHz = 24000000;   	/* Bus clock in Hz*/
    Uart_Config.u32Baudrate = 115200;     	/* Baud rate*/

    /*Initialization of UART module*/
    //UART_SetCallback(UART2, UART2_ISR);
    UART_Init(UART2,&Uart_Config);
    printf("\r\n Hello \r\n");
}
/***********************************************************************************************
*
* @brief   lin_application_timer_FTM0 - Initialize the timer for LIN application
* @param    none
* @return   none
*
************************************************************************************************/
void diagsrv_io_control_by_identifier(void)
{
	/* process recieved data from master request */
	ld_receive_message((l_u16*)&master_req_length, (l_u8*)master_req_dat);
	io_control_status = master_req_dat[4];

  slave_resp_dat[0] = master_req_dat[0] + 0x40;
	slave_resp_dat[1] = master_req_dat[1];
	slave_resp_dat[2] = master_req_dat[2];
	slave_resp_dat[3] = master_req_dat[3];
	slave_resp_dat[4] = io_control_status;
	slave_resp_length = 0x05;
	/* send a message to master */
  ld_send_message((l_u16)slave_resp_length, (l_u8*)slave_resp_dat);
  /*clear diagnostic flag */
  diag_clear_flag(DIAGSRV_IO_CONTROL_BY_IDENTIFIER_ORDER);
}
void DiagSrvRoutineControlByIdentify(void)
{
	/* process recieved data from master request */
	ld_receive_message((l_u16*)&master_req_length, (l_u8*)master_req_dat);
	io_control_status = master_req_dat[4];
	//LED_value = ((byte)~io_control_status) << 4; /*ToanLN*/
  /* Display Led inidcator*/
  //   PTCD = ( (~io_control_status) << 2);
  /* send response to master */
  slave_resp_dat[0] = master_req_dat[0] + 0x40;
	slave_resp_dat[1] = master_req_dat[1];
	slave_resp_dat[2] = master_req_dat[2];
	slave_resp_dat[3] = master_req_dat[3];
	slave_resp_dat[4] = master_req_dat[4];
	slave_resp_length = 0x05;
	/* send a message to master */
  ld_send_message((l_u16)slave_resp_length, (l_u8*)slave_resp_dat);
  /*clear diagnostic flag */
  diag_clear_flag(DIAGSRV_ROUTINECONTROL_ORDER);
}
//fixme:negative response data necessary?
void NegativeAck(uint8_t Sid, uint8_t RespNrc)
{
	//Todo:define a macro to replace Acknowledge ID
	slave_resp_dat[0] = 0x7F;
	slave_resp_dat[1] = Sid;
	slave_resp_dat[2] = RespNrc;
	ld_send_message((l_u16)3, (l_u8*)slave_resp_dat);
}
void DiagSrvRoutineCtrl(void)
{
	//todo:define a flag of flash erase state
	uint8_t erase_state = 0;
	uint8_t i;
	ld_receive_message((l_u16*)&master_req_length, (l_u8*)master_req_dat);
	if(master_req_length < 4)
	{
		//Todo:define a macro to replace NRC value
		NegativeAck(master_req_dat[0], 0x13);
		diag_clear_flag(DIAGSRV_ROUTINECONTROL_ORDER);
		printf("IMLOIF\r\n");
		return ;
	}
	if(erase_state != 0xFF)
	{
		NegativeAck(master_req_dat[0], 0x31);
		diag_clear_flag(DIAGSRV_ROUTINECONTROL_ORDER);
		printf("ROOR\r\n");
		return ;
	}
	ld_receive_message((l_u16*)&master_req_length, (l_u8*)master_req_dat);
	io_control_status = master_req_dat[4];
	//LED_value = ((byte)~io_control_status) << 4; /*ToanLN*/
	/* Display Led inidcator*/
	//   PTCD = ( (~io_control_status) << 2);
	/* send response to master */
	slave_resp_dat[0] = master_req_dat[0] + 0x40;

	slave_resp_length = master_req_length;
	for(i = 1; i < slave_resp_length; i++)
	{
		slave_resp_dat[i] = master_req_dat[i];
	}
	/* send a message to master */
	ld_send_message((l_u16)slave_resp_length, (l_u8*)slave_resp_dat);
	/*clear diagnostic flag */
	diag_clear_flag(DIAGSRV_ROUTINECONTROL_ORDER);

}
int main(void)
{

	Clk_Init();
	GPIO_Init();
	UartInit();

	l_sys_init();
	l_ifc_init(BootProtocol);

	NVIC_ClearPendingIRQ(UART0_IRQn);
	NVIC_EnableIRQ(UART0_IRQn);

	l_u16_wr_BootProtocol_BootStatus(0x1);
	for(;;)
	{
		//if(0x1 == l_u8_rd_BootProtocol_BootCMD())
		//if (diag_get_flag(DIAGSRV_IO_CONTROL_BY_IDENTIFIER_ORDER))
		if (diag_get_flag(DIAGSRV_ROUTINECONTROL_ORDER))
		{
			LED0_ON;
			//diagsrv_io_control_by_identifier();
			//DiagSrvRoutineControlByIdentify();
			//DiagSrvRoutineControlByIdentify();
			DiagSrvRoutineCtrl();
		}
		else
		{
			LED0_OFF;
		}

	}
	return 0;
}

