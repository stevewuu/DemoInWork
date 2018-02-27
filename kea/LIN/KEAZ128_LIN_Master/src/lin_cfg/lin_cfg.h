/******************************************************************************
* 
* Freescale Semiconductor Inc.
* (c) Copyright 2013-2016 Freescale Semiconductor, Inc.
* Copyright 2016-2018 NXP
* ALL RIGHTS RESERVED.
* 
****************************************************************************//*!
* 
* @file      lin_cfg.h 
* 
* @author    FPT Software
*  
* @version   1.0 
*  
* @date      Mon Feb 26 16:26:11 CST 2018
*  
* @brief     Hardware configuration file
* 
******************************************************************************/
#ifndef    _LIN_CFG_H_  
#define    _LIN_CFG_H_  
#include "lin_hw_cfg.h" 
/* Define operating mode */
#define _MASTER_MODE_     0 
#define _SLAVE_MODE_      1 
#define LIN_MODE   _MASTER_MODE_ 
/* Define protocol version */
#define PROTOCOL_21       0  
#define PROTOCOL_J2602    1  
#define PROTOCOL_20       2 
#define LIN_PROTOCOL    PROTOCOL_21
#define LIN_NUM_OF_IFCS  1		/* For master */
/**********************************************************************/
/***************          Diagnostic class selection  *****************/
/**********************************************************************/
#define _DIAG_CLASS_I_          0
#define _DIAG_CLASS_II_         1
#define _DIAG_CLASS_III_        2

#define _DIAG_CLASS_SUPPORT_    _DIAG_CLASS_I_

#define MAX_LENGTH_SERVICE 6

#define MAX_QUEUE_SIZE 1


#define _BootProtocol_DIAG_NUMBER_OF_SERVICES_    2

#define BootProtocol_DIAGSRV_READ_BY_IDENTIFIER_ORDER    0

#define BootProtocol_DIAGSRV_ASSIGN_FRAME_ID_RANGE_ORDER    1


/**************** FRAME SUPPORT DEFINITION ******************/
#define _TL_SINGLE_FRAME_       0
#define _TL_MULTI_FRAME_        1

#define _TL_FRAME_SUPPORT_      _TL_SINGLE_FRAME_

/* frame buffer size */
#define LIN_FRAME_BUF_SIZE			11
#define LIN_FLAG_BUF_SIZE			3

/**********************************************************************/
/***************               Interfaces           *******************/
/**********************************************************************/
typedef enum { 
   BootProtocol
}l_ifc_handle; 

/**********************************************************************/
/***************               Signals              *******************/
/**********************************************************************/
/* Number of signals */
#define LIN_NUM_OF_SIGS  4
/* List of signals */   
typedef enum {

   /* Interface_name = BootProtocol */

   BootProtocol_BootStatus

   , BootProtocol_BootCMD
  
   , BootProtocol_BootErr
  
   , BootProtocol_BootData
  
  
} l_signal_handle; 
/**********************************************************************/
/*****************               Frame             ********************/
/**********************************************************************/
/* Number of frames */
#define LIN_NUM_OF_FRMS  6 
/* List of frames */
typedef enum {
/* All frames for master node */

   /* Interface_name = BootProtocol */

   BootProtocol_BootWriteCMD

   , BootProtocol_BootWriteData
  
   , BootProtocol_BootReadStatus
  
   , BootProtocol_MasterReq
  
   , BootProtocol_SlaveResp
  
   , BootProtocol_BootWrite
  
  
} l_frame_handle; 
/**********************************************************************/
/***************        schedule table       *******************/
/**********************************************************************/
/* Number of schedule tables */
#define  LIN_NUM_OF_SCHD_TBL   5 
/* List of schedules */
typedef enum {

   /* Interface_name = BootProtocol */

   BootProtocol_LIN_NULL_SCHEDULE

   ,BootProtocol_GOTO_SLEEP_SCHEDULE
   
   ,BootProtocol_MasterReqTable
   
   ,BootProtocol_SlaveRespTable
   
   ,BootProtocol_NormalTable
   
}l_schedule_handle;  
/**********************************************************************/
/***************             Configuration          *******************/
/**********************************************************************/

/* Size of configuration in ROM and RAM used for interface: BootProtocol */
#define BootProtocol_LIN_SIZE_OF_CFG  8
 
/*********************************************************************
 * global macros
 *********************************************************************/
#define l_bool_rd(SIGNAL) l_bool_rd_##SIGNAL()
#define l_bool_wr(SIGNAL, A) l_bool_wr_##SIGNAL(A)
#define l_u8_rd(SIGNAL) l_u8_rd_##SIGNAL()
#define l_u8_wr(SIGNAL, A) l_u8_wr_##SIGNAL(A)
#define l_u16_rd(SIGNAL) l_u16_rd_##SIGNAL()
#define l_u16_wr(SIGNAL, A) l_u16_wr_##SIGNAL(A)
#define l_bytes_rd(SIGNAL, start, count, data)  l_bytes_rd_##SIGNAL(start, count, data)
#define l_bytes_wr(SIGNAL, start, count, data) l_bytes_wr_##SIGNAL(start, count, data)
#define l_flg_tst(FLAG) l_flg_tst_##FLAG()
#define l_flg_clr(FLAG) l_flg_clr_##FLAG()
#define LIN_TEST_BIT(A,B) ((l_bool)((((A) & (1U << (B))) != 0U) ? 1U : 0U))
#define LIN_SET_BIT(A,B)                      ((A) |= (l_u8) (1U << (B)))
#define LIN_CLEAR_BIT(A,B)               ((A) &= ((l_u8) (~(1U << (B)))))
#define LIN_BYTE_MASK  ((l_u16)(((l_u16)((l_u16)1 << CHAR_BIT)) - (l_u16)1))
#define LIN_FRAME_LEN_MAX                                             10U

/* Returns the low byte of the 32-bit value    */
#define BYTE_0(n)                              ((l_u8)((n) & (l_u8)0xFF))
/* Returns the second byte of the 32-bit value */
#define BYTE_1(n)                        ((l_u8)(BYTE_0((n) >> (l_u8)8)))
/* Returns the third byte of the 32-bit value  */
#define BYTE_2(n)                       ((l_u8)(BYTE_0((n) >> (l_u8)16)))
/* Returns high byte of the 32-bit value       */
#define BYTE_3(n)                       ((l_u8)(BYTE_0((n) >> (l_u8)24)))

/*
 * defines for signal access
 */


#define LIN_BYTE_OFFSET_BootProtocol_BootStatus    9
#define LIN_BIT_OFFSET_BootProtocol_BootStatus    0
#define LIN_SIGNAL_SIZE_BootProtocol_BootStatus    15
#define LIN_FLAG_BYTE_OFFSET_BootProtocol_BootStatus    2
#define LIN_FLAG_BIT_OFFSET_BootProtocol_BootStatus    0
#define LIN_FLAG_UPDATE_BYTE_OFFSET_BootProtocol_BootStatus    2

#define LIN_BYTE_OFFSET_BootProtocol_BootCMD    0
#define LIN_BIT_OFFSET_BootProtocol_BootCMD    0
#define LIN_SIGNAL_SIZE_BootProtocol_BootCMD    8
#define LIN_FLAG_BYTE_OFFSET_BootProtocol_BootCMD    0
#define LIN_FLAG_BIT_OFFSET_BootProtocol_BootCMD    0
#define LIN_FLAG_UPDATE_BYTE_OFFSET_BootProtocol_BootCMD    0

#define LIN_BYTE_OFFSET_BootProtocol_BootErr    10
#define LIN_BIT_OFFSET_BootProtocol_BootErr    7
#define LIN_SIGNAL_SIZE_BootProtocol_BootErr    1
#define LIN_FLAG_BYTE_OFFSET_BootProtocol_BootErr    2
#define LIN_FLAG_BIT_OFFSET_BootProtocol_BootErr    1
#define LIN_FLAG_UPDATE_BYTE_OFFSET_BootProtocol_BootErr    2

#define LIN_BYTE_OFFSET_BootProtocol_BootData    1
#define LIN_BIT_OFFSET_BootProtocol_BootData    0
#define LIN_SIGNAL_SIZE_BootProtocol_BootData    64
#define LIN_FLAG_BYTE_OFFSET_BootProtocol_BootData    1
#define LIN_FLAG_BIT_OFFSET_BootProtocol_BootData    0
#define LIN_FLAG_UPDATE_BYTE_OFFSET_BootProtocol_BootData    1




#define LIN_FLAG_BYTE_OFFSET_BootProtocol_BootWriteCMD             0
#define LIN_FLAG_BIT_OFFSET_BootProtocol_BootWriteCMD              0

#define LIN_FLAG_BYTE_OFFSET_BootProtocol_BootWriteData             1
#define LIN_FLAG_BIT_OFFSET_BootProtocol_BootWriteData              0

#define LIN_FLAG_BYTE_OFFSET_BootProtocol_BootReadStatus             2
#define LIN_FLAG_BIT_OFFSET_BootProtocol_BootReadStatus              0


/**********************************************************************/
/***************        Static API Functions        *******************/
/**********************************************************************/
/*
 * the static signal access macros
 */



/* static access macros for signal BootProtocol_BootCMD */
   
#define l_u8_rd_BootProtocol_BootCMD() \
    ((l_u8) ((lin_pFrameBuf[LIN_BYTE_OFFSET_BootProtocol_BootCMD] & \
    (((1U << LIN_SIGNAL_SIZE_BootProtocol_BootCMD) - 1) \
    << LIN_BIT_OFFSET_BootProtocol_BootCMD )) >> LIN_BIT_OFFSET_BootProtocol_BootCMD))
#define l_u8_wr_BootProtocol_BootCMD(A) \
    {lin_pFrameBuf[LIN_BYTE_OFFSET_BootProtocol_BootCMD] = \
    ((l_u8) (((l_u8) (lin_pFrameBuf[LIN_BYTE_OFFSET_BootProtocol_BootCMD] & \
    ((l_u8) (~(((1U << LIN_SIGNAL_SIZE_BootProtocol_BootCMD) - 1) << LIN_BIT_OFFSET_BootProtocol_BootCMD)))))  | \
    ((l_u8) ((((1U << LIN_SIGNAL_SIZE_BootProtocol_BootCMD) - 1) & (A)) << LIN_BIT_OFFSET_BootProtocol_BootCMD))));\
    lin_frame_flag_tbl[LIN_FLAG_UPDATE_BYTE_OFFSET_BootProtocol_BootCMD] = 1;}

/* static access macros for signal BootProtocol_BootErr */
   
#define l_bool_rd_BootProtocol_BootErr() \
   	(LIN_TEST_BIT(lin_pFrameBuf[LIN_BYTE_OFFSET_BootProtocol_BootErr], \
   	LIN_BIT_OFFSET_BootProtocol_BootErr))
#define l_bool_wr_BootProtocol_BootErr(A) \
	{(A) ? \
  	(LIN_SET_BIT(lin_pFrameBuf[LIN_BYTE_OFFSET_BootProtocol_BootErr], \
  	LIN_BIT_OFFSET_BootProtocol_BootErr)):\
  	(LIN_CLEAR_BIT(lin_pFrameBuf[LIN_BYTE_OFFSET_BootProtocol_BootErr], \
  	LIN_BIT_OFFSET_BootProtocol_BootErr));\
  	lin_frame_flag_tbl[LIN_FLAG_UPDATE_BYTE_OFFSET_BootProtocol_BootErr] = 1;}

/* static access macros for signal BootProtocol_BootData */
  
#define l_bytes_rd_BootProtocol_BootData(start, count, data) \
  	{l_u8       i; \
     for (i = 0; i < (count); ++i)  (data)[i] = lin_pFrameBuf[LIN_BYTE_OFFSET_BootProtocol_BootData + i + (start)];}
#define l_bytes_wr_BootProtocol_BootData(start, count, data) \
  	{l_u8       i; \
     for (i = 0; i < (count); ++i)lin_pFrameBuf[LIN_BYTE_OFFSET_BootProtocol_BootData + i + (start)]  = (data)[i]; \
  	 lin_frame_flag_tbl[LIN_FLAG_UPDATE_BYTE_OFFSET_BootProtocol_BootData] = 1;}



/* static access macros for signal BootProtocol_BootStatus */
#define l_u16_rd_BootProtocol_BootStatus() \
    ((l_u16) ((lin_pFrameBuf[LIN_BYTE_OFFSET_BootProtocol_BootStatus + 1] & 0x7f) << 8) + ((l_u16) (lin_pFrameBuf[LIN_BYTE_OFFSET_BootProtocol_BootStatus]) >> 0x00))
#define l_u16_wr_BootProtocol_BootStatus(A) \
    {lin_pFrameBuf[LIN_BYTE_OFFSET_BootProtocol_BootStatus + 1] = \
	((l_u8) (((l_u8) (lin_pFrameBuf[LIN_BYTE_OFFSET_BootProtocol_BootStatus + 1] &  (0x80))) | \
	((l_u8) ((l_u8) ((A) >> 8)) & 0x7f))); \
	lin_pFrameBuf[LIN_BYTE_OFFSET_BootProtocol_BootStatus] = \
	((l_u8) (((l_u8) (lin_pFrameBuf[LIN_BYTE_OFFSET_BootProtocol_BootStatus] & (0x00))) | \
	((l_u8) (A) << LIN_BIT_OFFSET_BootProtocol_BootStatus))); \
	lin_frame_flag_tbl[LIN_FLAG_UPDATE_BYTE_OFFSET_BootProtocol_BootStatus] = 1;}





/* Signal flag APIs */

#define l_flg_tst_BootProtocol_BootStatus_flag() \
         LIN_TEST_BIT(lin_flag_handle_tbl[LIN_FLAG_BYTE_OFFSET_BootProtocol_BootStatus],\
         LIN_FLAG_BIT_OFFSET_BootProtocol_BootStatus)
#define l_flg_clr_BootProtocol_BootStatus_flag() \
         LIN_CLEAR_BIT(lin_flag_handle_tbl[LIN_FLAG_BYTE_OFFSET_BootProtocol_BootStatus],\
         LIN_FLAG_BIT_OFFSET_BootProtocol_BootStatus)

#define l_flg_tst_BootProtocol_BootCMD_flag() \
         LIN_TEST_BIT(lin_flag_handle_tbl[LIN_FLAG_BYTE_OFFSET_BootProtocol_BootCMD],\
         LIN_FLAG_BIT_OFFSET_BootProtocol_BootCMD)
#define l_flg_clr_BootProtocol_BootCMD_flag() \
         LIN_CLEAR_BIT(lin_flag_handle_tbl[LIN_FLAG_BYTE_OFFSET_BootProtocol_BootCMD],\
         LIN_FLAG_BIT_OFFSET_BootProtocol_BootCMD)

#define l_flg_tst_BootProtocol_BootErr_flag() \
         LIN_TEST_BIT(lin_flag_handle_tbl[LIN_FLAG_BYTE_OFFSET_BootProtocol_BootErr],\
         LIN_FLAG_BIT_OFFSET_BootProtocol_BootErr)
#define l_flg_clr_BootProtocol_BootErr_flag() \
         LIN_CLEAR_BIT(lin_flag_handle_tbl[LIN_FLAG_BYTE_OFFSET_BootProtocol_BootErr],\
         LIN_FLAG_BIT_OFFSET_BootProtocol_BootErr)

#define l_flg_tst_BootProtocol_BootData_flag() \
         LIN_TEST_BIT(lin_flag_handle_tbl[LIN_FLAG_BYTE_OFFSET_BootProtocol_BootData],\
         LIN_FLAG_BIT_OFFSET_BootProtocol_BootData)
#define l_flg_clr_BootProtocol_BootData_flag() \
         LIN_CLEAR_BIT(lin_flag_handle_tbl[LIN_FLAG_BYTE_OFFSET_BootProtocol_BootData],\
         LIN_FLAG_BIT_OFFSET_BootProtocol_BootData)



/* Frame flag APIs */

#define l_flg_tst_BootProtocol_BootWriteCMD_flag() \
         LIN_TEST_BIT(lin_flag_handle_tbl[LIN_FLAG_BYTE_OFFSET_BootProtocol_BootWriteCMD],\
         LIN_FLAG_BIT_OFFSET_BootProtocol_BootWriteCMD)
#define l_flg_clr_BootProtocol_BootWriteCMD_flag() \
         LIN_CLEAR_BIT(lin_flag_handle_tbl[LIN_FLAG_BYTE_OFFSET_BootProtocol_BootWriteCMD],\
         LIN_FLAG_BIT_OFFSET_BootProtocol_BootWriteCMD)

#define l_flg_tst_BootProtocol_BootWriteData_flag() \
         LIN_TEST_BIT(lin_flag_handle_tbl[LIN_FLAG_BYTE_OFFSET_BootProtocol_BootWriteData],\
         LIN_FLAG_BIT_OFFSET_BootProtocol_BootWriteData)
#define l_flg_clr_BootProtocol_BootWriteData_flag() \
         LIN_CLEAR_BIT(lin_flag_handle_tbl[LIN_FLAG_BYTE_OFFSET_BootProtocol_BootWriteData],\
         LIN_FLAG_BIT_OFFSET_BootProtocol_BootWriteData)

#define l_flg_tst_BootProtocol_BootReadStatus_flag() \
         LIN_TEST_BIT(lin_flag_handle_tbl[LIN_FLAG_BYTE_OFFSET_BootProtocol_BootReadStatus],\
         LIN_FLAG_BIT_OFFSET_BootProtocol_BootReadStatus)
#define l_flg_clr_BootProtocol_BootReadStatus_flag() \
         LIN_CLEAR_BIT(lin_flag_handle_tbl[LIN_FLAG_BYTE_OFFSET_BootProtocol_BootReadStatus],\
         LIN_FLAG_BIT_OFFSET_BootProtocol_BootReadStatus)



/* SCHEDULE MANAGEMENT */

#define l_sch_tick_BootProtocol() l_sch_tick(BootProtocol)



#define l_sch_set_BootProtocol(schedule, entry) l_sch_set(BootProtocol, schedule, entry)

/* INTERFACE MANAGEMENT */

#define l_ifc_init_BootProtocol() l_ifc_init(BootProtocol)



#define l_ifc_goto_sleep_BootProtocol() l_ifc_goto_sleep(BootProtocol)


#define l_ifc_wake_up_BootProtocol() l_ifc_wake_up(BootProtocol)



#define l_ifc_rx_BootProtocol() l_ifc_rx(BootProtocol)



#define l_ifc_tx_BootProtocol() l_ifc_tx(BootProtocol)



#define l_ifc_aux_BootProtocol() l_ifc_aux(BootProtocol)



#define l_ifc_read_status_BootProtocol() l_ifc_read_status(BootProtocol)


#endif    /* _LIN_CFG_H_ */