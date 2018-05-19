/*
 * flexcan.h
 *
 *  Created on: 2017Äê8ÔÂ1ÈÕ
 *      Author: 4337
 */

#ifndef FLEXCAN_H_
#define FLEXCAN_H_

#include "device_registers.h"

#define SBC   /* SBC requires SPI init + max 1MHz bit rate */
#define CAN_BUFFER_LENGTH		  16

#if 0
typedef struct
{									    /*!< identifier union */
	uint32_t  CODE; 					/* Received message buffer code */
	uint32_t  ID;                		/* Received message ID */
	uint32_t  DLC;            			/* Recieved message number of data bytes */
	uint32_t  LENGTH;            		/* Recieved message number of data bytes */
	//uint32  DATA[16];           	    /* Received message data string*/
	uint8_t   DATA[64];           	    /* Received message data string*/
	uint32_t  TIMESTAMP;         		/* Received message time */
}CAN_FrameType,*CAN_FramePtr;
#endif
typedef struct
{									    /*!< identifier union */
	uint32_t  CODE; 					/* Received message buffer code */
	uint32_t  ID;                		/* Received message ID */
	uint32_t  DLC;            		/* Recieved message number of data bytes */
	uint32_t  LENGTH;            		/* Recieved message number of data bytes */
	//uint32  DATA[16];           	    /* Received message data string*/
	//uint8_t   DATA[64];           	    /* Received message data string*/
	uint32_t  TIMESTAMP;         		/* Received message time */
	union {
	    	uint8_t B[64];
	    	uint16_t H[32];
	    	uint32_t W[16];
	    } DATA;
}CAN_FrameType,*CAN_FramePtr;
typedef struct
{
    union {                            /* Message Buffer 0 CS Register */
    	uint32_t R;
      struct {
    		uint32_t TIMESTAMP:16;        /* Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
    		uint32_t DLC:4;               /* Length of the data to be stored/transmitted. */
    		uint32_t RTR:1;               /* Remote Transmission Request. One/zero for remote/data frame. */
    		uint32_t IDE:1;               /* ID Extended. One/zero for extended/standard format frame. */
    		uint32_t SRR:1;               /* Substitute Remote Request. Contains a fixed recessive bit. */
    		uint32_t  :1;
    		uint32_t CODE:4;              /* Message Buffer Code */
    		uint32_t  :1;
    		uint32_t ESI:1;
    		uint32_t BRS:1;
    		uint32_t EDL:1;
      } B;
    } CS;
    union {                            /* Message Buffer 0 ID Register */
    	uint32_t R;
      struct {
    		uint32_t ID_EXT:18;           /* Frame Identifier Extended */
    		uint32_t ID_STD:11;           /* Frame Identifier Standard */
    		uint32_t PRIO:3;              /* Local Priority */
      } B;
    } ID;
    union {
    	uint8_t B[64];
    	uint16_t H[32];
    	uint32_t W[16];
    } DATA;
}MB_Struct;









void flexcan_init();
void flexcan_send_frame(CAN_FramePtr ptr_tx_frame);
void flexcan_send_data(uint32_t msgid, uint8_t *data, uint8_t length);

void CAN_GlobeVaribleInit( void );
uint8_t CAN_ReadOneFramefromBufferQueue(CAN_FramePtr pRxFrameInfo);
void CAN_ReadOneFrameFromBuff(CAN_FramePtr pRxFrame);
#endif /* FLEXCAN_H_ */
