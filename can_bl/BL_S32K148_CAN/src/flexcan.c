/*
 * flexcan.c
 *
 *  Created on: 2017年8月1日
 *      Author: 4337
 */

#include "flexcan.h"
#include <stdbool.h>
#include "s32_core_cm4.h"

#define MSG_BUF_SIZE  4    /* Msg Buffer Size. (CAN 2.0AB: 2 hdr +  2 data= 4 words) */

#define MBR0_BASE	((unsigned char *)0x40024080)

#define PL_LEN_8	16
#define PL_LEN_16	24
#define PL_LEN_32	40
#define PL_LEN_64	72

#define CAN0_MBR0(pl, idx)	((MB_Struct*)((MBR0_BASE) + (PL_LEN_##pl) * (idx)))

uint8_t u8RxFrameBufferIndex;
uint8_t u8RxFrameHeader;
uint8_t u8RxFrameBufferFreeLength;

CAN_FrameType  sRxFrame[CAN_BUFFER_LENGTH];
uint8_t   DLCLen[] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
uint32_t  RxDATA[2];           /* Received message data (2 words) */
uint32_t  RxTIMESTAMP;         /* Received message time */
/**
 *  \brief flexcan_init
 *  
 *  \return None
 *  
 *  \details init flexcan0
 */
void flexcan_init()
{
    uint32_t   i=0;

    PCC->PCCn[PCC_FlexCAN0_INDEX] |= PCC_PCCn_CGC_MASK; /* CGC=1: enable clock to FlexCAN0 */
    CAN0->MCR |= CAN_MCR_MDIS_MASK;         /* MDIS=1: Disable module before selecting clock */
    CAN0->CTRL1 &= ~CAN_CTRL1_CLKSRC_MASK;  /* CLKSRC=0: Clock Source = oscillator (8 MHz) */
    CAN0->MCR &= ~CAN_MCR_MDIS_MASK;        /* MDIS=0; Enable module config. (Sets FRZ, HALT)*/
    while (!((CAN0->MCR & CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT))  {}
                 /* Good practice: wait for FRZACK=1 on freeze mode entry/exit */
    CAN0->CTRL1 = 0x01DB0006; /* Configure for 500 KHz bit time */
                            /* Time quanta freq = 16 time quanta x 500 KHz bit time= 8MHz */
                            /* PRESDIV+1 = Fclksrc/Ftq = 8 MHz/8 MHz = 2 */
                            /*    so PRESDIV = 0 */
                            /* PSEG2 = Phase_Seg2 - 1 = 4 - 1 = 3 */
                            /* PSEG1 = PSEG2 = 3 */
                            /* PROPSEG= Prop_Seg - 1 = 7 - 1 = 6 */
                            /* RJW: since Phase_Seg2 >=4, RJW+1=4 so RJW=3. */
                            /* SMP = 1: use 3 bits per CAN sample */
                            /* CLKSRC=0 (unchanged): Fcanclk= Fosc= 8 MHz */
    CAN0->CTRL2 |= 1 << 16;
    for(i=0; i<128; i++ ) {   /* CAN0: clear 32 msg bufs x 4 words/msg buf = 128 words*/
    CAN0->RAMn[i] = 0;      /* Clear msg buf word */
    }
    for(i=0; i<16; i++ ) {          /* In FRZ mode, init CAN0 16 msg buf filters */
    CAN0->RXIMR[i] = 0x0;//0xFFFFFFFF;  /* 0x0 don't Check all ID bits for incoming messages */
    }
    CAN0->RXMGMASK = 0x0;//0x1FFFFFFF;    /* 0x0 Global acceptance mask: don't check all ID bits */
    CAN0->RAMn[ 4*MSG_BUF_SIZE + 0] = 0x04000000; /* Msg Buf 4, word 0: Enable for reception */
                                                /* EDL,BRS,ESI=0: CANFD not used */
                                                /* CODE=4: MB set to RX inactive */
                                                /* IDE=0: Standard ID */
                                                /* SRR, RTR, TIME STAMP = 0: not applicable */
//TODO:modify filter ID
    CAN0->RAMn[ 4*MSG_BUF_SIZE + 1] = 0x15540000; /* Msg Buf 4, word 1: Standard ID = 0x555 */
                                                /* PRIO = 0: CANFD not used */
    CAN0->MCR = 0x0002001F;   //0x0000001F;       /* Negate FlexCAN 1 halt state for 32 MBs */
	CAN0->IMASK1 = 0x00000010;	/* 使能中断向量标志 */
	//S32_NVIC->ISER[(uint32_t)(irqNumber) >> 5U] = (uint32_t)(1UL << ((uint32_t)(irqNumber) & (uint32_t)0x1FU));
	S32_NVIC->ISER[(uint32_t)CAN0_ORed_0_15_MB_IRQn>>5U] =
			(uint32_t)(1UL << ((uint32_t)(CAN0_ORed_0_15_MB_IRQn) & (uint32_t)0x1FU)); /* 开启NVIC中断 */
    while ((CAN0->MCR && CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT)  {}
                 /* Good practice: wait for FRZACK to clear (not in freeze mode) */
    while ((CAN0->MCR && CAN_MCR_NOTRDY_MASK) >> CAN_MCR_NOTRDY_SHIFT)  {}
                 /* Good practice: wait for NOTRDY to clear (module ready)  */
}
/**
 *  \brief flexcan_send_frame
 *  
 *  \param [in] ptr_tx_frame pointer to rx_frame struct
 *  \return None
 *  
 *  \details 
 */
void flexcan_send_frame(CAN_FramePtr ptr_tx_frame)
{
	CAN0->IFLAG1 = 0x00000001;       /* Clear CAN 0 MB 0 flag without clearing others*/
	CAN0->RAMn[ 0*MSG_BUF_SIZE + 2] = ptr_tx_frame->DATA.W[0]; /* MB0 word 2: data word 0 */
	CAN0->RAMn[ 0*MSG_BUF_SIZE + 3] = ptr_tx_frame->DATA.W[1]; /* MB0 word 3: data word 1 */
	CAN0->RAMn[ 0*MSG_BUF_SIZE + 1] = ptr_tx_frame->ID << CAN_WMBn_ID_ID_SHIFT; /* MB0 word 1: Tx msg with ID */
	CAN0->RAMn[ 0*MSG_BUF_SIZE + 0] = 0x0C400000 | ptr_tx_frame->DLC <<CAN_WMBn_CS_DLC_SHIFT; /* MB0 word 0: */
	                                                /* EDL,BRS,ESI=0: CANFD not used */
	                                                /* CODE=0xC: Activate msg buf to transmit */
	                                                /* IDE=0: Standard ID */
	                                                /* SRR=1 Tx frame (not req'd for std ID) */
	                                                /* RTR = 0: data, not remote tx request frame*/
	                                                /* DLC = 8 bytes */
}
/**
 *  \brief flexcan_send_data
 *  
 *  \param [in] msgid data message ID
 *  \param [in] data Message data
 *  \param [in] length Message data length
 *  \return send can data
 *  
 *  \details 
 */
void flexcan_send_data(uint32_t msgid, uint8_t *data, uint8_t length)
{
	uint8_t j;
	uint32_t txdata[2];
	CAN_FrameType tx_frame_info;
	tx_frame_info.ID = msgid << 18;
	tx_frame_info.DLC = length;
	for(j = 0; j < 2; j++)
	{
		txdata[j] =
		(data[4*j+0]<<24)|(data[4*j+1]<<16)|(data[4*j+2]<<8)|(data[4*j+3]);
	}
	CAN0->IFLAG1 = 0x00000001;
	CAN0->RAMn[ 0*MSG_BUF_SIZE + 2] = txdata[0];
	CAN0->RAMn[ 0*MSG_BUF_SIZE + 3] = txdata[1];
	CAN0->RAMn[ 0*MSG_BUF_SIZE + 1] = tx_frame_info.ID << CAN_WMBn_ID_ID_SHIFT;
	CAN0->RAMn[ 0*MSG_BUF_SIZE + 0] = 0x0C400000 | tx_frame_info.DLC <<CAN_WMBn_CS_DLC_SHIFT;
	//flexcan_send_frame(&tx_frame_info);
}

/*******************************************************************************
 * CAN API
 ******************************************************************************/
 /**
 *  \brief CAN_GlobeVaribleInit
 *  
 *  \return None
 *  
 *  \details CAN frame info varible init 
 */
void CAN_GlobeVaribleInit( void )
{

    u8RxFrameBufferFreeLength = CAN_BUFFER_LENGTH;
    u8RxFrameBufferIndex = 0;
	u8RxFrameHeader = 0;

}
/**
 *  \brief CAN_ReadOneFramefromBufferQueue
 *  
 *  \param [in] pRxFrameInfo pointer to rx_frame_info
 *  \return is there read data from Queus 
 *  
 *  \details read data from queue, if read any data return true else return false
 */
uint8_t CAN_ReadOneFramefromBufferQueue(CAN_FramePtr pRxFrameInfo)
{
    // disable interrupt
	DISABLE_INTERRUPTS();
    if( (u8RxFrameHeader!=u8RxFrameBufferIndex)||
        (u8RxFrameBufferFreeLength == 0) )
    {
    	//_New_CANRx_Data =1;
        *pRxFrameInfo = sRxFrame[u8RxFrameHeader++];
        if(u8RxFrameHeader >= CAN_BUFFER_LENGTH)
        {
            u8RxFrameHeader = 0;
        }
        u8RxFrameBufferFreeLength ++;
    }
    else
    {
        // enable interrupt
    	ENABLE_INTERRUPTS();
        return false;
    }
    // enable interrupt
    ENABLE_INTERRUPTS();
    return true;
}

void CAN_ReadOneFrameFromBuff(CAN_FramePtr pRxFrame)
{

    uint8_t i,j;

	{
    	pRxFrame->CODE   = CAN0_MBR0(8, 4)->CS.B.CODE; 		/* Read CODE, ID, LENGTH, DATA, TIMESTAMP*/
    	pRxFrame->ID     = CAN0_MBR0(8, 4)->ID.B.ID_STD;
    	pRxFrame->DLC = CAN0_MBR0(8, 4)->CS.B.DLC;
    	pRxFrame->LENGTH = DLCLen[pRxFrame->DLC];


    	for (j=0, i=0; i<pRxFrame->LENGTH; j=j+4,i=i+4)
    	{
    		pRxFrame->DATA.B[i] = (CAN0_MBR0(8, 4)->DATA.B[j+3]);
    		pRxFrame->DATA.B[i+1] = (CAN0_MBR0(8, 4)->DATA.B[j+2]);
    		pRxFrame->DATA.B[i+2] = (CAN0_MBR0(8, 4)->DATA.B[j+1]);
    		pRxFrame->DATA.B[i+3] = (CAN0_MBR0(8, 4)->DATA.B[j]);
    	}

    	pRxFrame->TIMESTAMP = CAN0_MBR0(8, 4)->CS.B.TIMESTAMP;

	}

}

/**
 *  \brief CAN0_ORed_0_15_MB_IRQHandler
 *  
 *  \return None
 *  
 *  \details CAN0 msg buffer 0-15 interrupt handler
 */
void CAN0_ORed_0_15_MB_IRQHandler(void)
{
	uint8_t j;
	uint32_t dummy;
	CAN_FrameType rx_frame_info;
	rx_frame_info.ID = (CAN0->RAMn[ 4*MSG_BUF_SIZE + 1] & CAN_WMBn_ID_ID_MASK)  >> CAN_WMBn_ID_ID_SHIFT;
	rx_frame_info.CODE = (CAN0->RAMn[ 4*MSG_BUF_SIZE + 0] & 0x07000000) >> 24;  /* Read CODE field */
	rx_frame_info.DLC = (CAN0->RAMn[ 4*MSG_BUF_SIZE + 0] & CAN_WMBn_CS_DLC_MASK) >> CAN_WMBn_CS_DLC_SHIFT;
	for (j=0; j<2; j++)
	{
		rx_frame_info.DATA.W[j] = CAN0->RAMn[ 4*MSG_BUF_SIZE + 2 + j];
		RxDATA[j] = (unsigned int)(rx_frame_info.DATA.B[4 * j + 3] << 0) +
				    (unsigned int)(rx_frame_info.DATA.B[4 * j + 2] << 8) +
				    (unsigned int)(rx_frame_info.DATA.B[4 * j + 1] << 16)+
				    (unsigned int)(rx_frame_info.DATA.B[4 * j + 0] << 24) ;
		rx_frame_info.DATA.W[j] = RxDATA[j];
	}
	for(j = 0; j < rx_frame_info.DLC; j++)
	{
		xmodem_queue_byte(rx_frame_info.DATA.B[j]);
	}
	RxTIMESTAMP = (CAN0->RAMn[ 0*MSG_BUF_SIZE + 0] & 0x000FFFF);
	dummy = CAN0->TIMER;             /* Read TIMER to unlock message buffers */
	CAN0->IFLAG1 = 0x00000010;       /* Clear CAN 0 MB 4 flag without clearing others*/

}
