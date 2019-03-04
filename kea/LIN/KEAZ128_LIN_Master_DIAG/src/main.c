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
#define MASTER 1

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


#define MAX_LENGTH_DIAG_SERVICE  20

#define IO_CONTROL_SET_RECEIV		0x08

l_u8 LIN_counter =0;//, LED_counter = 0;
volatile l_u8 master_req_dat[MAX_LENGTH_DIAG_SERVICE];
volatile l_u16 slave_resp_length;
volatile l_u8 slave_resp_dat[MAX_LENGTH_DIAG_SERVICE];

volatile l_u8 receiv_response_flag = 0x00;
volatile l_u8 NAD;

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
    CONFIG_PIN_AS_GPIO(PTC,PTC0,OUTPUT); 		/* Configure  LED (PTC0) as an output */
    CONFIG_PIN_AS_GPIO(PTC,PTC1,OUTPUT); 		/* Configure  LED (PTC1) as an output */
    CONFIG_PIN_AS_GPIO(PTC,PTC2,OUTPUT); 		/* Configure  LED (PTC2) as an output */

    CONFIG_PIN_AS_GPIO(PTD,PTD0,INPUT); 		/* Configure BTN0 (SW2-PTD0) as an input */
    CONFIG_PIN_AS_GPIO(PTD,PTD1,INPUT); 		/* Configure BTN1 (SW3-PTD1) as an input */
    ENABLE_INPUT(PTD,PTD0);			 			/* Enable input SW2 */
    ENABLE_INPUT(PTD,PTD1);						/* Enable input SW3 */

    LED0_OFF;									/* Turn LED0 */
    LED1_OFF;									/* Turn LED1 */
    LED2_OFF;									/* Turn LED2 */

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
#if MASTER
void lin_application_timer_FTM0()
{
    SIM_SCGC |= SIM_SCGC_FTM0_MASK; /* Enable Clock for FTM0 */
    FTM0_SC |= FTM_SC_PS(7);	/* Select Preescaler in this case 128. 20 Mhz /128 =156.25 Khz. */
                                    /* Counter increase by one every 6.4 us */
        /* Enable Channle 0*/
    FTM0_C0SC |= FTM_CnSC_CHIE_MASK; /* Enable channel 0 interrupt */
    FTM0_C0SC |= FTM_CnSC_MSA_MASK;  /* Channel as Output compare mode */
        /*Select interrupt frequency*/
    FTM0_C0V = FTM_CnV_VAL(189) ;	 	/* Interrupt every 2.5ms */

    FTM0_SC |= FTM_SC_CLKS(1); /*FTM0 use system clock*/

    /* Set the ICPR and ISER registers accordingly */
    NVIC_ClearPendingIRQ(FTM0_IRQn);
    NVIC_EnableIRQ(FTM0_IRQn);
}
#endif
void DiagRoutineControl(l_ifc_handle iii, const l_u8 NAD, l_u16 data_length, l_u8 *const data) /* 0x31 */
{
    l_u8 Buff[MAX_LENGTH_SERVICE];
    l_u8 i;

    const lin_configuration * conf;
    /* Get the current configuration */
    conf = &lin_ifc_configuration[iii];

    /* Check if length of data is greater than length maximum */
    if (data_length < MAX_LENGTH_SERVICE)
    {
        /* check whether service status is idle or not */
        if (*conf->tl_service_status != LD_SERVICE_BUSY)
        {
            /* pack data */
            Buff[0] = 0x31;

            for (i = 0; i < data_length; i++)
            {
                Buff[i+1] = data[i];
            }
            ld_send_message(iii, data_length+1, NAD, Buff);
            /* set service status to busy */
            *conf->tl_service_status = LD_SERVICE_BUSY;
        }
    }
}
void RoutineCtrlSendReq(uint8_t SubId, uint16_t RoutineId, uint8_t *RoutineOpt)
{
    uint8_t master_req_length;
    uint8_t i;
    /*---Set service ID --------*/
     master_req_dat[0] = 0x31;
     master_req_dat[1] = SubId;
     master_req_dat[2] = (uint8_t) ((RoutineId & 0xFF00) >> 8);
     master_req_dat[3] = (uint8_t) (RoutineId & 0xFF);
     for(i = 0; i < 8; i++)
     {
        master_req_dat[i + 4] = RoutineOpt[i];
     }
    /*---Display service ID --------*/
     master_req_length = 12;
     DiagRoutineControl(BootProtocol, 0x2, (l_u16)master_req_length-1, (l_u8* const)(master_req_dat+1));
}
#if 1
void TestRoutineCtrlSrv(void)
{
    uint8_t SubId = 0x01;
    uint8_t i;
    uint16_t RoutineId = 0xFF00;
    uint8_t RoutineOpt[8] = { 0x00, 0x00, 0x19, 0x00, 0x00, 0x00, 0xA5, 0x00};
    uint8_t RcvFlag = 0;
    RoutineCtrlSendReq(SubId, RoutineId, RoutineOpt);

    while(!RcvFlag)
    {
        if(diag_get_flag(BootProtocol, BootProtocol_DIAGSRV_ROUTINECONTROL_ORDER))
        {
            RcvFlag = 0xF;
            DiagSrvRoutineControlByIdentify();
        }
    }

}
#endif
void RoutineControlSendReq(void)
{
    l_u8 master_req_length;
    /*---Set service ID --------*/
     master_req_dat[0] = 0x31;
     master_req_dat[1] = 0x01;
     master_req_dat[2] = 0xFF;
     master_req_dat[3] = 0x00;
     master_req_dat[4] = 0x44;
     master_req_dat[5] = 0x00;
     master_req_dat[6] = 0x19;
     master_req_dat[7] = 0x19;
    /*---Display service ID --------*/
     master_req_length = 0x06;
     DiagRoutineControl(BootProtocol, 0x2, (l_u16)master_req_length-1, (l_u8* const)(master_req_dat+1));
}
void io_control_set_send_req(void)
{
    l_u8 master_req_length;
    volatile l_u8 io_control_status = 0x00; /*Led indicator*/
    /*---Set service ID --------*/
     master_req_dat[0] = 0x2F;
     master_req_dat[1] = 0x00;
     master_req_dat[2] = 0x80;
     master_req_dat[3] = 0x03;
//		 master_req_dat[4] = io_control_status;
    /*---Display service ID --------*/
     master_req_length = 0x05;
     master_req_dat[4] = io_control_status;

     diag_IO_control(BootProtocol, 0x2, (l_u16)master_req_length-1, (l_u8* const)(master_req_dat+1));
}
void diagsrv_io_control_by_identifier(void)
{
  /* Implement code of this service here */
  ld_receive_message(BootProtocol, (l_u16*)&slave_resp_length, (l_u8*)&NAD, (l_u8*)slave_resp_dat);
  receiv_response_flag = IO_CONTROL_SET_RECEIV;
  /*clear diagnostic flag */
  //lin_diag_services_flag[DIAGSRV_IO_CONTROL_BY_IDENTIFIER_ORDER] = 0;
  diag_clear_flag(BootProtocol, BootProtocol_DIAGSRV_IO_CONTROL_BY_IDENTIFIER_ORDER);
}
#if 1
void DiagSrvRoutineControlByIdentify(void)
{
  /* Implement code of this service here */
  ld_receive_message(BootProtocol, (l_u16*)&slave_resp_length, (l_u8*)&NAD, (l_u8*)slave_resp_dat);
  receiv_response_flag = IO_CONTROL_SET_RECEIV;
  /*clear diagnostic flag */
  //lin_diag_services_flag[DIAGSRV_IO_CONTROL_BY_IDENTIFIER_ORDER] = 0;
  diag_clear_flag(BootProtocol, BootProtocol_DIAGSRV_ROUTINECONTROL_ORDER);
}
#endif
int main(void)
{

#if MASTER
    l_u8 ret;
    l_u32 i;
    l_u16 LIN_resp;
    Clk_Init();
    GPIO_Init();
    UartInit();

    l_sys_init();
    l_ifc_init(BootProtocol);
    lin_application_timer_FTM0();
    l_sch_set(BootProtocol, BootProtocol_NormalTable, 0);

    Enable_Interrupt(UART0_IRQn);

    for(;;)
    {

        /* When pressed SW1*/
        if(GPIOA_PDIR & (1 << 24))
        {
            //l_u8_wr_BootProtocol_BootCMD(0x01);
            GPIOA_PSOR |= (1 << 16);
            TestRoutineCtrlSrv();
#if 0
            io_control_set_send_req();
            //RoutineControlSendReq();
            while(receiv_response_flag != IO_CONTROL_SET_RECEIV)
            {
                //if(diag_get_flag(BootProtocol, BootProtocol_DIAGSRV_ROUTINECONTROL_ORDER))
                if(diag_get_flag(BootProtocol, BootProtocol_DIAGSRV_IO_CONTROL_BY_IDENTIFIER_ORDER))
                {
                    diagsrv_io_control_by_identifier();
                    //DiagSrvRoutineControlByIdentify();
                }
            }
#endif
        }
        else
        {
            //l_u8_wr_BootProtocol_BootCMD(0x00);
            GPIOA_PCOR |= (1 << 16);
        }

    }
#else
    Clk_Init();
    GPIO_Init();

    l_sys_init();
    l_ifc_init(BootProtocol);

    NVIC_ClearPendingIRQ(UART0_IRQn);
    NVIC_EnableIRQ(UART0_IRQn);

    l_u16_wr_BootProtocol_BootStatus(0x1);
    for(;;)
    {
        if(0x1 == l_u8_rd_BootProtocol_BootCMD())
        {
            LED0_ON;
        }
        else
        {
            LED0_OFF;
        }

    }
#endif
    return 0;
}
#if MASTER
void FTM0_IRQHandler()
  {
    if (1==((FTM0_C0SC & FTM_CnSC_CHF_MASK)>>FTM_CnSC_CHF_SHIFT) )  /* If the CHF of the channel is equal to 0 */
      {
          (void)FTM0_C0SC;  							/* Read to clear flag */
          FTM0_C0SC ^= FTM_CnSC_CHF_MASK;  			/* Clear flag */
          FTM0_C0V = FTM0_C0V + 189 ; /* Refresh interrupt period */

          if (LIN_counter>=2){
              /* Activate LIN frame transfer for every 15ms */
              l_sch_tick(BootProtocol);
              /* Reset counter */
              LIN_counter = 0;
            }

      LIN_counter++;
      }
  }
#endif
