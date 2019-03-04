/******************************************************************************
* 
* Freescale Semiconductor Inc.
* (c) Copyright 2013-2016 Freescale Semiconductor, Inc.
* Copyright 2016-2019 NXP
* ALL RIGHTS RESERVED.
* 
****************************************************************************//*!
*
* @file      lin_cfg.c
*
* @author    FPT Software
*
* @version   1.0
*
* @date      Wed Feb 27 16:00:39 CST 2019
*
* @brief     Common LIN configuration, data structure
*
******************************************************************************/
#include "lin_cfg.h"
#include "lin.h"
/* Mapping interface with hardware */
const lin_hardware_name lin_virtual_ifc[LIN_NUM_OF_IFCS] = {UART0_};
/* Low level response buffer */
l_u8 lin_lld_response_buffer[LIN_NUM_OF_IFCS][10];
 /* Successful transfer flags */
l_u8 lin_successful_transfer[LIN_NUM_OF_IFCS];
/* Error in response */
l_u8 lin_error_in_response[LIN_NUM_OF_IFCS];
/* Goto sleep flag */
l_u8 lin_goto_sleep_flg[LIN_NUM_OF_IFCS];
/* Save configuration flag */
l_u8 lin_save_configuration_flg[LIN_NUM_OF_IFCS] = {0};
 /* Next transmit tick */
l_u8 lin_next_transmit[LIN_NUM_OF_IFCS];
 /* lin word status */
lin_word_status_str lin_word_status[LIN_NUM_OF_IFCS];
/* current pid */
l_u8 lin_current_pid[LIN_NUM_OF_IFCS];

/* definition and initialization of signal array */
l_u8    lin_pFrameBuf[LIN_FRAME_BUF_SIZE] =
{


  0x00 /* 0 : 00000000 */ /* start of frame BootProtocol_BootWriteCMD */



  ,0x00 /* 1 : 00000000 */ /* start of frame BootProtocol_BootWriteData */

  ,0x00 /* 2 : 00000000 */
  
  ,0x00 /* 3 : 00000000 */
  
  ,0x00 /* 4 : 00000000 */
  
  ,0x00 /* 5 : 00000000 */
  
  ,0x00 /* 6 : 00000000 */
  
  ,0x00 /* 7 : 00000000 */
  
  ,0x00 /* 8 : 00000000 */
  

  ,0x00 /* 9 : 00000000 */ /* start of frame BootProtocol_BootReadStatus */

  ,0x00 /* 10 : 00000000 */
  
};

/* definition and initialization of signal array */
l_u8    lin_flag_handle_tbl[LIN_FLAG_BUF_SIZE] =
{


  0x00 /* 0: start of flag frame BootProtocol_BootWriteCMD */



  ,0x00 /* 1: start of flag frame BootProtocol_BootWriteData */


  ,0x00 /* 2: start of flag frame BootProtocol_BootReadStatus */

};

/*************************** Flag set when signal is updated ******************/
/* Diagnostic signal */
l_u8 lin_diag_signal_tbl[LIN_NUM_OF_IFCS][16];
/*****************************event trigger frame*****************************/
/* all event trigger frames for master node */


/*****************************sporadic frame*****************************/
/*all sporadic frames for master node*/

const l_u8 BootProtocol_BootWrite_info_data[2] ={

   BootProtocol_BootWriteCMD
  

   ,BootProtocol_BootWriteData
  
};
const lin_associate_frame_struct BootProtocol_BootWrite_info ={
   2
   ,&BootProtocol_BootWrite_info_data[0]   
   ,0xFF
};
/**********************************  Frame table **********************************/
const lin_frame_struct lin_frame_tbl[LIN_NUM_OF_FRMS] ={

    { LIN_FRM_UNCD, 1, LIN_RES_PUB, 0, 0, 1   , (l_u8*)0 }

   ,{ LIN_FRM_UNCD, 8, LIN_RES_PUB, 1, 1, 1 , (l_u8*)0 }
  
   ,{ LIN_FRM_UNCD, 2, LIN_RES_SUB, 9, 2, 1 , (l_u8*)0 }
  
   ,{ LIN_FRM_DIAG, 8, LIN_RES_PUB, 0, 0, 0 , (l_u8*)0 }
  
   ,{ LIN_FRM_DIAG, 8, LIN_RES_SUB, 0, 0, 0 , (l_u8*)0 }
  
   ,{ LIN_FRM_SPRDC, 1, LIN_RES_PUB, 0, 0, 0 , (l_u8*)&BootProtocol_BootWrite_info }
  
};

/*********************************** Frame flag Initialization **********************/
l_bool lin_frame_flag_tbl[LIN_NUM_OF_FRMS] = {0, 0, 0, 0, 0, 0};

/**************************** Lin configuration Initialization ***********************/
/* max_response_frame_timeout = round((1.4x(10+Nx10)xTbit)/Tbase_period) + 3 */
const l_u16 lin_max_frame_res_timeout_val[LIN_NUM_OF_IFCS][8]= {

   {6, 7, 9, 10, 12, 13, 15, 16 }
  
};


l_u8 BootProtocol_lin_configuration_RAM[BootProtocol_LIN_SIZE_OF_CFG]= {0x00, 0x3B, 0x39, 0x01, 0x3C, 0x3D, 0xFF ,0xFF};


const l_u16  BootProtocol_lin_configuration_ROM[BootProtocol_LIN_SIZE_OF_CFG]= {0x0000, 0x3B, 0x39, 0x01, 0x3C, 0x3D, 0xFF ,0xFFFF};

/**************** Node attributes Initialization  ****************************/





/********************** Go to sleep Initialization *************************/
/* Delay of this schedule table is: (1.4*(34+10*(8+1))*1000/LIN_speed+jitter) ms */
/* then rounded up to a value is multiple of time base */
const lin_schedule_data BootProtocol_lin_gotosleep_data[1] = {
   {BootProtocol_MasterReq, 10, {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}}
};


/******************** Schedule table Initialization ************************/
const lin_schedule_data BootProtocol_NormalTable_data[5] = {

   {BootProtocol_BootWrite, 10, 0}
  

   , {BootProtocol_BootWrite, 10, 0}
  
   , {BootProtocol_BootWrite, 10, 0}
  
   , {BootProtocol_BootWrite, 10, 0}
  
   , {BootProtocol_BootReadStatus, 5, 0}
  
};



/******************** Schedule table Initialization ************************/
/* If not specify by users, then by default delay of this schedule table is: */
/* (1.4*(34+10*(8+1))*1000/LIN_speed+jitter) ms, then rounded up to a value */
/* that is multiple of time base */
const lin_schedule_data BootProtocol_MasterReqTable_data[1] = {

   {BootProtocol_MasterReq, 10, 0}
  
};

/******************** Schedule table Initialization ************************/
/* If not specify by users, then by default delay of this schedule table is: */
/* (1.4*(34+10*(8+1))*1000/LIN_speed+jitter) ms, then rounded up to a value */
/* that is multiple of time base */
const lin_schedule_data BootProtocol_SlaveRespTable_data[1] = {

   {BootProtocol_SlaveResp, 10, 0}
  
};



/********************* Schedule table structure Initialization ***************/
const lin_schedule_struct lin_schedule_tbl[LIN_NUM_OF_SCHD_TBL] ={

   /*interface_name = BootProtocol */
   {0, LIN_SCH_TBL_NULL  ,(lin_schedule_data*)0 }
  

   ,{1, LIN_SCH_TBL_GOTO  , &BootProtocol_lin_gotosleep_data[0] }
  
   ,{1, LIN_SCH_TBL_DIAG  , &BootProtocol_MasterReqTable_data[0] }
  
   ,{1, LIN_SCH_TBL_DIAG  , &BootProtocol_SlaveRespTable_data[0] }
  
   ,{5, LIN_SCH_TBL_NORM  , &BootProtocol_NormalTable_data[0] }
  
};
/********************** Schedule table status Initialization ******************/
l_u8 lin_schedule_start_entry[LIN_NUM_OF_SCHD_TBL] = {0, 0, 0, 0, 0};
l_u8 lin_active_schedule_id[LIN_NUM_OF_IFCS] = {0};
l_u8 lin_previous_schedule_id[LIN_NUM_OF_IFCS] = {0};



lin_diagnostic_state diagnostic_state[LIN_NUM_OF_IFCS] = {
   LD_DIAG_IDLE 
};
l_u8 lin_diag_frame_to_send[LIN_NUM_OF_IFCS] = {0};

lin_service_status   service_status[LIN_NUM_OF_IFCS]   = {
   LD_SERVICE_IDLE 
};
l_diagnostic_mode diag_mode[LIN_NUM_OF_IFCS] = {DIAG_NONE};



lin_tl_pdu_data BootProtocol_tl_tx_queue_data[MAX_QUEUE_SIZE];
lin_tl_pdu_data BootProtocol_tl_rx_queue_data[MAX_QUEUE_SIZE];



lin_transport_layer_queue lin_tl_tx_queue[LIN_NUM_OF_IFCS] = {

   { 0, 0, LD_QUEUE_EMPTY, 0, MAX_QUEUE_SIZE, BootProtocol_tl_tx_queue_data }

};
lin_transport_layer_queue lin_tl_rx_queue[LIN_NUM_OF_IFCS] = {

   { 0, 0, LD_QUEUE_EMPTY, 0, MAX_QUEUE_SIZE, BootProtocol_tl_rx_queue_data }

};
/* diagnostic interleaved mode */
diag_interleaved_state lin_diag_interleaved_state[LIN_NUM_OF_IFCS] = {DIAG_NOT_START};

/****************************Support SID Initialization ***********************/

const l_u8 BootProtocol_lin_diag_services_supported[_BootProtocol_DIAG_NUMBER_OF_SERVICES_] = {0xB2,0xB6,0xB7,0x22,0x10,0x2F,0x19,0x14,0x31,0xB1};
l_u8 BootProtocol_lin_diag_services_flag[_BootProtocol_DIAG_NUMBER_OF_SERVICES_] = {0,0,0,0,0,0,0,0,0,0};


/****************************Transport Layer Initialization ***********************/

lin_tl_descriptor lin_tl_desc[LIN_NUM_OF_IFCS] = {

   /* interface_name = BootProtocol */
   {
   &lin_tl_tx_queue[BootProtocol],          /* *pointer to transmit queue on TL */
   &lin_tl_rx_queue[BootProtocol],          /* *pointer to receive queue on TL */

   /* Declaration only for Master interface */
   /* message in transmit queue */
   LD_COMPLETED,           /* status of message in transmit queue */
   0,                /* index of message in queue */
   0,                /* Size of message in queue */
   /* message in receive queue */
   LD_NO_MSG,            /* status of receiving message */
   LD_COMPLETED,           /* status of message in transmit queue */
   0,                /* index of message in queue */
   0,                /* Size of message in queue */
   LD_SUCCESS,           /* Status of the last configuration service in LIN 2.0, J2602 */
   0,                /* RSID of the last node configuration service */
   0,                /* Error code in case of positive response */
   0,                /* number of received pdu */
   0,                /* frame counter in received message */
   LD_NO_CHECK_TIMEOUT,            /* timeout type */
   0,
   /* Declaration only for Slave interface */
   (l_u8*)0,
   0,                   /* Slave Response data counter */
   _BootProtocol_DIAG_NUMBER_OF_SERVICES_,
   (l_u8*) &BootProtocol_lin_diag_services_supported,
   (l_u8*) &BootProtocol_lin_diag_services_flag,           /* diagnostic services flags*/
   0           /* Interleaved time out counter */
   }

};

/****************************LIN interface configuration ****************************/
const lin_configuration lin_ifc_configuration[LIN_NUM_OF_IFCS] = {

   /* Interface_name = BootProtocol */
   {
   LIN_PROTOCOL_21,         /*lin_protocol_version */
   LIN_PROTOCOL_21,         /*lin_language_version */
   19200,            /*  baud_rate */
   _MASTER_,                 /*  function _SLAVE_ | _MASTER_*/
   0,                        /*  node attribute is only used for slave node*/
   /* LIN data pointer */
   &lin_lld_response_buffer[BootProtocol][0],        /*  *response_buffer */
   &lin_successful_transfer[BootProtocol],           /*  *lin_successful_transfer */
   &lin_error_in_response[BootProtocol],             /*  *lin_error_in_response */
   &lin_goto_sleep_flg[BootProtocol],              /*  *goto_sleep_flg */
   &lin_current_pid[BootProtocol],                 /*  *current_pid */
   &lin_word_status[BootProtocol],                 /*  *word_status */
   /* Protocol */
   1,                          /*  time_base */
   &lin_diag_signal_tbl[BootProtocol][0],          /*  *diag_signal_tbl */
   6,                            /*  num_of_frames */
   0,                              /*  frame_start */
   &lin_frame_tbl[0],                                          /*  frame_tbl */
   &lin_frame_flag_tbl[0],                                       /*  *frame_flg */

   5,                     /*  num_of_schedules */
   0,                       /*  schedule_start */
   &lin_schedule_tbl[0],                                   /*  schedule_tbl */
   &lin_schedule_start_entry[0],                             /*  schedule_start_entry */
   &lin_next_transmit[BootProtocol],             /*  next_transmit_tick */
   &lin_active_schedule_id[BootProtocol],          /*  active_schedule_id */
   &lin_previous_schedule_id[BootProtocol],        /*  previous_schedule_id */
   &lin_diag_frame_to_send[BootProtocol],          /*  *diagnostic_frame_to_send */
   &diag_mode[BootProtocol],   /*  diagnostic_mode */
   &BootProtocol_lin_configuration_RAM[0],   /*  *configuration_RAM */
   &BootProtocol_lin_configuration_ROM[0],   /*  *configuration_ROM */
   &lin_tl_desc[BootProtocol],
   &diagnostic_state[BootProtocol],
   &service_status[BootProtocol],
   &lin_diag_interleaved_state[BootProtocol]
   }

};
/*************************** Node hardware configuration definition *************************/
/* Node hardware configuration */
lin_node lin_node_descrs[NUM_OF_UART_CHANNEL]={
   {(tUART*)UART0_ADDR, 0, 0xFF, UNINIT,  0, 0,(l_u8*)0, 0x00, (l_u8*)0, 0x80, 0, 0, 0, 0 },
   {(tUART*)UART1_ADDR, 0, 0xFF, UNINIT,  0, 0,(l_u8*)0, 0x00, (l_u8*)0, 0x80, 0, 0, 0, 0 },
   {(tUART*)UART2_ADDR, 0, 0xFF, UNINIT,  0, 0,(l_u8*)0, 0x00, (l_u8*)0, 0x80, 0, 0, 0, 0 }
};


/*This ld_read_by_id_callout() function is used when the master node transmits a read by
 identifier request with an identifier in the user defined area (id from 32 to 63).
 The driver will call this function when such request is received.
 * id: the identifier in the user defined area (32 to 63)
 * data: pointer points to a data area with 5 bytes, used to give the positive response.
  Driver uses 0xFF "do not care value" for unassigned data values.
  Data length in PCI is (1 + number of assigned meaningful data values).
  Driver will take as data for all data before and including the last value in the frame that is different from 0xFF.
  PCI is 0x02-0x06, so data should have at least one value different from 0xFF.
  For example, a response frame, (NAD) (PCI) (0xF2) (0xFF) (0x00) (0xFF) (0xFF) (0xFF),
  PCI will be 0x03, since in this case driver takes all data before 0x00 and 0x00 as meaningful data,
  and values after 0x00 are do not care value.
 * return: LD_NEGATIVE_RESPONSE Respond with a negative response.
           LD_POSTIVE_RESPONSE Respond with a positive response.
           LD_ID_NO_RESPONSE The slave node will not answer.
 */
l_u8 ld_read_by_id_callout(l_ifc_handle iii, l_u8 id, l_u8 *data){
    (void) iii;
    (void) id;
    (void) data;
    return LD_NEGATIVE_RESPONSE;
}