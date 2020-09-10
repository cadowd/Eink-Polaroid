
/******************************************************************************
  * \attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under ST MYLIBERTY SOFTWARE LICENSE AGREEMENT (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        www.st.com/myliberty
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
******************************************************************************/

/*
 *      PROJECT:   ST25R391x firmware
 *      Revision:
 *      LANGUAGE:  ISO C99
 */

/*! \file rfal_nfcv.c
 *
 *  \author Gustavo Patricio
 *
 *  \brief Implementation of NFC-V Poller (ISO15693) device
 *
 *  The definitions and helpers methods provided by this module are 
 *  aligned with NFC-V (ISO15693)
 *
 */

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include "rfal_nfcv.h"
#include "utils.h"

/*
 ******************************************************************************
 * ENABLE SWITCH
 ******************************************************************************
 */

#ifndef RFAL_FEATURE_NFCV
    #error " RFAL: Module configuration missing. Please enable/disable NFC-V module by setting: RFAL_FEATURE_NFCV "
#endif

#if RFAL_FEATURE_NFCV

/*
 ******************************************************************************
 * GLOBAL DEFINES
 ******************************************************************************
 */

#define RFAL_NFCV_INV_REQ_FLAG            0x06U  /*!< INVENTORY_REQ  INV_FLAG  Digital  2.0  9.6.1                      */
#define RFAL_NFCV_INV_REQ_FLAG            0x06U  /*!< INVENTORY_REQ  INV_FLAG  Digital  2.0  9.6.1                      */
#define RFAL_NFCV_MASKVAL_MAX_LEN         8U     /*!< Mask value max length: 64 bits  (UID length)                      */
#define RFAL_NFCV_MASKVAL_MAX_1SLOT_LEN   64U    /*!< Mask value max length in 1 Slot mode in bits  Digital 2.0 9.6.1.6 */
#define RFAL_NFCV_MASKVAL_MAX_16SLOT_LEN  60U    /*!< Mask value max length in 16 Slot mode in bits Digital 2.0 9.6.1.6 */
#define RFAL_NFCV_INV_REQ_HEADER_LEN      3U     /*!< INVENTORY_REQ header length (INV_FLAG, CMD, MASK_LEN)             */
#define RFAL_NFCV_INV_RES_LEN             10U    /*!< INVENTORY_RES length                                              */
#define RFAL_NFCV_CRC_LEN                 2U     /*!< NFC-V CRC length                                                  */
#define RFAL_NFCV_MAX_SLOTS               16U    /*!< NFC-V max number of Slots                                         */
#define RFAL_NFCV_MAX_GEN_DATA_LEN        (RFAL_NFCV_MAX_BLOCK_LEN + RFAL_NFCV_UID_LEN) /*!< Max number of generic data */


#define RFAL_CMD_LEN                      1U     /*!< Commandbyte length                                                */
#define RFAL_NFCV_FLAG_POS                0U     /*!< Flag byte position                                                */
#define RFAL_NFCV_FLAG_LEN                1U     /*!< Flag byte length                                                  */
#define RFAL_NFCV_DATASTART_POS           1U     /*!< Position of start of data                                         */
#define RFAL_NFCV_DSFI_LEN                1U     /*!< DSFID length                                                      */
#define RFAL_NFCV_SLPREQ_REQ_FLAG         0x22U  /*!< SLPV_REQ request flags Digital 2.0 (Candidate) 9.7.1.1            */

#define RFAL_NFCV_MAX_COLL_SUPPORTED      16U    /*!< Maximum number of collisions supported by the Anticollision loop  */

#define RFAL_FDT_POLL_MAX                 rfalConvMsTo1fc(20) /*!< Maximum Wait time                                    */   



/*! Time from special frame to EOF 
 *                    ISO15693 2009 10.4.2                 : 20ms
 *                    NFC Forum defines Digital 2.0  9.7.4 : FDTV,EOF = [10 ; 20]ms 
 */
#define RFAL_NFCV_FDT_EOF                 20U



/*! Time between slots - ISO 15693 defines t3min depending on modulation depth and data rate.
 *  With only high-bitrate supported, AM modulation and a length of 12 bytes (96bits) for INV_RES we get:
 *                    - ISO t3min = 96/26 ms + 300us = 4 ms
 *                    - NFC Forum defines FDTV,INVENT_NORES = (4394 + 2048)/fc. Digital 2.0  B.5*/
#define RFAL_NFCV_FDT_V_INVENT_NORES      4U



/*
 ******************************************************************************
 * GLOBAL MACROS
 ******************************************************************************
 */


/*
******************************************************************************
* GLOBAL TYPES
******************************************************************************
*/

/*! NFC-V INVENTORY_REQ format   Digital 2.0 9.6.1 */
typedef struct
{
    uint8_t  INV_FLAG;                              /*!< Inventory Flags    */
    uint8_t  CMD;                                   /*!< Command code: 01h  */
    uint8_t  MASK_LEN;                              /*!< Mask Value Length  */
    uint8_t  MASK_VALUE[RFAL_NFCV_MASKVAL_MAX_LEN]; /*!< Mask Value         */
} rfalNfcvInventoryReq;


/*! NFC-V SLP_REQ format   Digital 2.0 (Candidate) 9.7.1 */
typedef struct
{
    uint8_t  REQ_FLAG;                              /*!< Request Flags      */
    uint8_t  CMD;                                   /*!< Command code: 02h  */
    uint8_t  UID[RFAL_NFCV_UID_LEN];                /*!< Mask Value         */
} rfalNfcvSlpvReq;


/*! NFC-V Generic Req format  */
typedef struct
{
    uint8_t  REQ_FLAG;                              /*!< Request Flags      */
    uint8_t  CMD;                                   /*!< Command code       */
    union { /*  PRQA S 0750 # MISRA 19.2 - Both members are of the same type, just different names.  Thus no problem can occur. */
        uint8_t  UID[RFAL_NFCV_UID_LEN];            /*!< Mask Value         */
        uint8_t  data[RFAL_NFCV_MAX_GEN_DATA_LEN];  /*!< Data               */
    }payload;
} rfalNfcvGenericReq;


/*! NFC-V Generic Response format */
typedef struct
{
    uint8_t  RES_FLAG;                              /*!< Response Flags     */
    uint8_t  data[RFAL_NFCV_MAX_GEN_DATA_LEN];      /*!< Data               */
} rfalNfcvGenericRes;


/*! Container for a collision found during Anticollsion loop */
typedef struct
{
    uint8_t  maskLen;
    uint8_t  maskVal[RFAL_NFCV_MASKVAL_MAX_LEN];
}rfalNfcvCollision;


/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/
static ReturnCode rfalNfcvParseError( uint8_t err );

/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/

/*******************************************************************************/
static ReturnCode rfalNfcvParseError( uint8_t err )
{
    switch(err)
    {
        case RFAL_NFCV_ERROR_CMD_NOT_SUPPORTED:
        case RFAL_NFCV_ERROR_OPTION_NOT_SUPPORTED:
            return ERR_NOTSUPP;
            
        case RFAL_NFCV_ERROR_CMD_NOT_RECOGNIZED:
            return ERR_PROTO;
            
        case RFAL_NFCV_ERROR_WRITE_FAILED:
            return ERR_WRITE;
            
        default:
            return ERR_REQUEST;
    }
}

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/

/*******************************************************************************/
ReturnCode rfalNfcvPollerInitialize( void )
{
    ReturnCode ret;
            
    EXIT_ON_ERR( ret, rfalSetMode( RFAL_MODE_POLL_NFCV, RFAL_BR_26p48, RFAL_BR_26p48 ) );
    rfalSetErrorHandling( RFAL_ERRORHANDLING_NFC );
    
    rfalSetGT( RFAL_GT_NFCV_ADJUSTED );
    rfalSetFDTListen( RFAL_FDT_LISTEN_NFCV_POLLER );
    rfalSetFDTPoll( RFAL_FDT_POLL_NFCV_POLLER );
    
    return ERR_NONE;
}

/*******************************************************************************/
ReturnCode rfalNfcvPollerCheckPresence( rfalNfcvInventoryRes *invRes )
{
    ReturnCode ret;
    
    /* INVENTORY_REQ with 1 slot and no Mask   Activity 2.0 (Candidate) 9.2.3.32 */
    ret = rfalNfcvPollerInventory( RFAL_NFCV_NUM_SLOTS_1, 0, NULL, invRes, NULL );
    
    if( (ret == ERR_RF_COLLISION) || (ret == ERR_CRC)  || 
        (ret == ERR_FRAMING)      || (ret == ERR_PROTO)  )
    {
        ret = ERR_NONE;
    }
    
    return ret;
}

/*******************************************************************************/
ReturnCode rfalNfcvPollerInventory( rfalNfcvNumSlots nSlots, uint8_t maskLen, const uint8_t *maskVal, rfalNfcvInventoryRes *invRes, uint16_t* rcvdLen )
{
    ReturnCode           ret;
    rfalNfcvInventoryReq invReq;
    uint16_t             rxLen;
    
    if( ((maskVal == NULL) && (maskLen != 0U)) || (invRes == NULL) )
    {
        return ERR_PARAM;
    }
    
    invReq.INV_FLAG = (RFAL_NFCV_INV_REQ_FLAG | (uint8_t)nSlots);
    invReq.CMD      = RFAL_NFCV_CMD_INVENTORY;
    invReq.MASK_LEN = (uint8_t)MIN( maskLen, ((nSlots == RFAL_NFCV_NUM_SLOTS_1) ? RFAL_NFCV_MASKVAL_MAX_1SLOT_LEN : RFAL_NFCV_MASKVAL_MAX_16SLOT_LEN) );   /* Digital 2.0  9.6.1.6 */
    
    if( rfalConvBitsToBytes(invReq.MASK_LEN) > 0U )  /* MISRA 21.18 */
    {
        ST_MEMCPY( invReq.MASK_VALUE, maskVal, rfalConvBitsToBytes(invReq.MASK_LEN) );
    }
    
    ret = rfalISO15693TransceiveAnticollisionFrame( (uint8_t*)&invReq, (uint8_t)(RFAL_NFCV_INV_REQ_HEADER_LEN + rfalConvBitsToBytes(invReq.MASK_LEN)), (uint8_t*)invRes, sizeof(rfalNfcvInventoryRes), &rxLen );
    
    /* Check for optional output parameter */
    if( rcvdLen != NULL )
    {
        *rcvdLen = rxLen;
    }
    
    if( ret == ERR_NONE )
    {
        if( rxLen != rfalConvBytesToBits(RFAL_NFCV_INV_RES_LEN + RFAL_NFCV_CRC_LEN) )
        {
            return ERR_PROTO;
        }
    }
    
    return ret;
}

/*******************************************************************************/
ReturnCode rfalNfcvPollerCollisionResolution( uint8_t devLimit, rfalNfcvListenDevice *nfcvDevList, uint8_t *devCnt )
{
    ReturnCode ret;
    uint8_t           slotNum;
    uint16_t          rcvdLen;
    uint8_t           colIt;
    uint8_t           colCnt;
    bool              colPending;
    rfalNfcvCollision colFound[RFAL_NFCV_MAX_COLL_SUPPORTED];
    
    
    if( (nfcvDevList == NULL) || (devCnt == NULL) )
    {
        return ERR_PARAM;
    }
    
    /* Initialize parameters */
    *devCnt = 0;
    colIt         = 0;
    colCnt        = 0;
    colPending    = false;
    ST_MEMSET(colFound, 0x00, (sizeof(rfalNfcvCollision)*RFAL_NFCV_MAX_COLL_SUPPORTED) );
    
    if( devLimit > 0U )       /* MISRA 21.18 */
    {
        ST_MEMSET(nfcvDevList, 0x00, (sizeof(rfalNfcvListenDevice)*devLimit) );
    }
    
    
    NO_WARNING(colPending);   /* colPending is not exposed externally, in future it might become exposed/ouput parameter */
    
    
    /* Send INVENTORY_REQ with one slot   Activity 2.0  9.3.7.1  (Symbol 0)  */
    ret = rfalNfcvPollerInventory( RFAL_NFCV_NUM_SLOTS_1, 0, NULL, &nfcvDevList->InvRes, NULL );
    
    if( ret == ERR_TIMEOUT )  /* Exit if no device found     Activity 2.0  9.3.7.2 (Symbol 1)  */
    {
        return ERR_NONE;
    }
    if( ret == ERR_NONE )     /* Device found without transmission error/collision    Activity 2.0  9.3.7.3 (Symbol 2)  */
    {
        (*devCnt)++;
        return ERR_NONE;
    }
    
    /* A Collision has been identified  Activity 2.0  9.3.7.2  (Symbol 3) */
    colPending = true;
    colCnt        = 1;

    /* Check if the Collision Resolution is set to perform only Collision detection   Activity 2.0  9.3.7.5 (Symbol 4)*/
    if( devLimit == 0U )
    {
        return ERR_RF_COLLISION;
    }

    platformDelay(RFAL_NFCV_FDT_V_INVENT_NORES);
    
    /*******************************************************************************/
    /* Collisions pending, Anticollision loop must be executed                     */
    /*******************************************************************************/
    
    /* Execute until all collisions are resolved Activity 2.0  9.3.7.16  (Symbol 17) */
    do
    {
        /* Activity 2.0  9.3.7.5  (Symbol 6) */
        colPending = false;
        slotNum    = 0;
        
        do
        {

            if (0U==slotNum)
            {
                /* Send INVENTORY_REQ with 16 slots   Activity 2.0  9.3.7.7  (Symbol 8) */
                ret = rfalNfcvPollerInventory( RFAL_NFCV_NUM_SLOTS_16, colFound[colIt].maskLen, colFound[colIt].maskVal, &nfcvDevList[(*devCnt)].InvRes, &rcvdLen );
            }
            else
            {
                ret = rfalISO15693TransceiveEOFAnticollision( (uint8_t*)&nfcvDevList[(*devCnt)].InvRes, sizeof(rfalNfcvInventoryRes), &rcvdLen );
            }
            slotNum++;
            /*******************************************************************************/
            if( ret != ERR_TIMEOUT )
            {
                if( rcvdLen < rfalConvBytesToBits(RFAL_NFCV_INV_RES_LEN + RFAL_NFCV_CRC_LEN) )
                { /* If only a partial frame was received make sure the FDT_V_INVENT_NORES is fulfilled */
                    platformDelay(RFAL_NFCV_FDT_V_INVENT_NORES);
                }

                if( ret == ERR_NONE )
                {
                    /* Check if the device found is already on the list and its response is a valid INVENTORY_RES */
                    if( rcvdLen == rfalConvBytesToBits(RFAL_NFCV_INV_RES_LEN + RFAL_NFCV_CRC_LEN) )
                    {
                        /* Activity 2.0  9.3.7.15  (Symbol 16) */
                        (*devCnt)++;
                    }
                }
                else if( ret == ERR_RF_COLLISION )
                {
                    /* Activity 2.0  9.3.7.15  (Symbol 16) */
                    colPending = true;
                    
                    /* Ensure that the frame received has at least the FLAGS + DSFI */
                    if( rcvdLen <= rfalConvBytesToBits( RFAL_NFCV_FLAG_LEN + RFAL_NFCV_DSFI_LEN ) )
                    {
                        return ERR_RF_COLLISION;
                    }

                    /*******************************************************************************/
                    /* Ensure that this collision still fits on the container */
                    if( colCnt < RFAL_NFCV_MAX_COLL_SUPPORTED )
                    {
                        /* Store this collision on the container to be resolved later */
                        colFound[colCnt].maskLen = (uint8_t)( rcvdLen - rfalConvBytesToBits( RFAL_NFCV_FLAG_LEN + RFAL_NFCV_DSFI_LEN ) );
                        ST_MEMCPY(colFound[colCnt].maskVal, nfcvDevList[(*devCnt)].InvRes.UID, RFAL_NFCV_UID_LEN);
                        colCnt++;
                    }
                }
                else
                {
                    /* MISRA 15.7 - Empty else */
                }
            }
            else 
            { /* Timeout */
                platformDelay(RFAL_NFCV_FDT_V_INVENT_NORES);
            }
            
            /* Check if devices found have reached device limit   Activity 2.0  9.3.7.15  (Symbol 16) */
            if( *devCnt >= devLimit )
            {
                return ERR_NONE;
            }
        } while( slotNum < RFAL_NFCV_MAX_SLOTS );  /* Slot loop             */
        colIt++; 
    } while( colIt < colCnt );                    /* Collisions found loop */
    
    return ERR_NONE;
}

/*******************************************************************************/
ReturnCode rfalNfcvPollerSleep( uint8_t flags, const uint8_t* uid )
{
    ReturnCode      ret;
    rfalNfcvSlpvReq slpReq;
    uint8_t         rxBuf;    /* dummy buffer, just to perform Rx */
    
    if( uid == NULL )
    {
        return ERR_PARAM;
    }
    
    /* Compute SLPV_REQ */
    slpReq.REQ_FLAG = (flags | (uint8_t)RFAL_NFCV_REQ_FLAG_ADDRESS);   /* Should be with UID according Digital 2.0 (Candidate) 9.7.1.1 */
    slpReq.CMD      = RFAL_NFCV_CMD_SLPV;
    ST_MEMCPY( slpReq.UID, uid, RFAL_NFCV_UID_LEN );
    
    /* NFC Forum device SHALL wait at least FDTVpp to consider the SLPV acknowledged (FDTVpp = FDTVpoll)  Digital 2.0 (Candidate)  9.7  9.8.2  */
    ret = rfalTransceiveBlockingTxRx( (uint8_t*)&slpReq, sizeof(rfalNfcvSlpvReq), &rxBuf, sizeof(rxBuf), NULL, RFAL_TXRX_FLAGS_DEFAULT, RFAL_FDT_POLL_NFCV_POLLER );
    if( ret != ERR_TIMEOUT )
    {
        return ret;
    }
    
    return ERR_NONE;
}

/*******************************************************************************/
ReturnCode rfalNfcvPollerSelect( uint8_t flags, const uint8_t* uid )
{
    uint16_t           rcvLen;
    ReturnCode         ret;
    rfalNfcvGenericReq req;
    rfalNfcvGenericRes res;

    if( uid == NULL )
    {
        return ERR_PARAM;
    }
    
    /* Compute Request Command */
    req.REQ_FLAG = (flags | (uint8_t)RFAL_NFCV_REQ_FLAG_ADDRESS);
    req.CMD      = RFAL_NFCV_CMD_SELECT;
    ST_MEMCPY( req.payload.UID, uid, RFAL_NFCV_UID_LEN );
    
    ret = rfalTransceiveBlockingTxRx( (uint8_t*)&req, (RFAL_CMD_LEN + RFAL_NFCV_FLAG_LEN + RFAL_NFCV_UID_LEN), (uint8_t*)&res, sizeof(rfalNfcvGenericRes), &rcvLen, RFAL_TXRX_FLAGS_DEFAULT, RFAL_FDT_POLL_MAX );
    if( ret != ERR_NONE )
    {
        return ret;
    }
    
    /* Check if the response minimum length has been received */
    if( rcvLen < RFAL_NFCV_FLAG_LEN )
    {
        return ERR_PROTO;
    }
    
    /* Check if an error has been signalled */
    if( (res.RES_FLAG & (uint8_t)RFAL_NFCV_RES_FLAG_ERROR) != 0U )
    {
        return rfalNfcvParseError( *res.data );
    }
    
    return ERR_NONE;
}

/*******************************************************************************/
ReturnCode rfalNfcvPollerReadSingleBlock( uint8_t flags, const uint8_t* uid, uint8_t blockNum, uint8_t* rxBuf, uint16_t rxBufLen, uint16_t *rcvLen )
{
    ReturnCode         ret;
    rfalNfcvGenericReq req;
    uint8_t            msgIt;
    
    msgIt = 0;
    
    /* Compute Request Command */
    req.REQ_FLAG  = (uint8_t)(flags & (~((uint32_t)RFAL_NFCV_REQ_FLAG_ADDRESS) & ~((uint32_t)RFAL_NFCV_REQ_FLAG_SELECT)));
    req.CMD       = RFAL_NFCV_CMD_READ_SINGLE_BLOCK;
    
    /* Check if request is to be sent in Addressed or Selected mode */
    if( uid != NULL )
    {
        req.REQ_FLAG |= (uint8_t)RFAL_NFCV_REQ_FLAG_ADDRESS;
        ST_MEMCPY( req.payload.UID, uid, RFAL_NFCV_UID_LEN );
        msgIt += RFAL_NFCV_UID_LEN;
    }
    else
    {
        req.REQ_FLAG |= (uint8_t)RFAL_NFCV_REQ_FLAG_SELECT;
    }
    req.payload.data[msgIt++] = blockNum;
    
    /* Transceive Command */
    ret = rfalTransceiveBlockingTxRx( (uint8_t*)&req, (RFAL_CMD_LEN + RFAL_NFCV_FLAG_LEN +(uint16_t)msgIt), rxBuf, rxBufLen, rcvLen, RFAL_TXRX_FLAGS_DEFAULT, RFAL_FDT_POLL_MAX );
    if( ret != ERR_NONE )
    {
        return ret;
    }
    
    /* Check if the response minimum length has been received */
    if( (*rcvLen) < (uint8_t)RFAL_NFCV_FLAG_LEN )
    {
        return ERR_PROTO;
    }
    
    /* Check if an error has been signalled */
    if( (rxBuf[RFAL_NFCV_FLAG_POS] & (uint8_t)RFAL_NFCV_RES_FLAG_ERROR) != 0U )
    {
        return rfalNfcvParseError( rxBuf[RFAL_NFCV_DATASTART_POS] );
    }
    
    return ERR_NONE;
}

/*******************************************************************************/
ReturnCode rfalNfcvPollerWriteSingleBlock( uint8_t flags, const uint8_t* uid, uint8_t blockNum, const uint8_t* wrData, uint8_t blockLen )
{
    ReturnCode         ret;
    uint16_t           rcvLen;
    rfalNfcvGenericReq req;
    rfalNfcvGenericRes res;
    uint8_t            msgIt;

    if( blockLen > (uint8_t)RFAL_NFCV_MAX_BLOCK_LEN )
    {
        return ERR_PARAM;
    }
    
    msgIt = 0;
    
    /* Compute Request Command */
    req.REQ_FLAG  = (uint8_t)(flags & (~((uint32_t)RFAL_NFCV_REQ_FLAG_ADDRESS) & ~((uint32_t)RFAL_NFCV_REQ_FLAG_SELECT)));
    req.CMD       = RFAL_NFCV_CMD_WRITE_SINGLE_BLOCK;
    
    /* Check if request is to be sent in Addressed or Selected mode */
    if( uid != NULL )
    {
        req.REQ_FLAG |= (uint8_t)RFAL_NFCV_REQ_FLAG_ADDRESS;
        ST_MEMCPY( req.payload.UID, uid, RFAL_NFCV_UID_LEN );
        msgIt += (uint8_t)RFAL_NFCV_UID_LEN;
    }
    else
    {
        req.REQ_FLAG |= (uint8_t)RFAL_NFCV_REQ_FLAG_SELECT;
    }
    req.payload.data[msgIt++] = blockNum;
    
    if( blockLen > 0U )         /* MISRA 21.18 */
    {
        ST_MEMCPY( &req.payload.data[msgIt], wrData, blockLen );
        msgIt += blockLen;
    }
    
    /* Transceive Command */
    ret = rfalTransceiveBlockingTxRx( (uint8_t*)&req, (RFAL_CMD_LEN + RFAL_NFCV_FLAG_LEN + (uint16_t)msgIt), (uint8_t*)&res, sizeof(rfalNfcvGenericRes), &rcvLen, RFAL_TXRX_FLAGS_DEFAULT, RFAL_FDT_POLL_MAX );

    if( ret != ERR_NONE )
    {
        return ret;
    }
    
    /* Check if the response minimum length has been received */
    if( rcvLen < (uint8_t)RFAL_NFCV_FLAG_LEN )
    {
        return ERR_PROTO;
    }
    
    /* Check if an error has been signalled */
    if( (res.RES_FLAG & (uint8_t)RFAL_NFCV_RES_FLAG_ERROR) != 0U )
    {
        return rfalNfcvParseError( *res.data );
    }
    
    return ERR_NONE;
}

/*******************************************************************************/
ReturnCode rfalNfcvPollerLockBlock( uint8_t flags, const uint8_t* uid, uint8_t blockNum )
{
    ReturnCode         ret;
    uint16_t           rcvLen;
    rfalNfcvGenericReq req;
    rfalNfcvGenericRes res;
    uint8_t            msgIt;

    msgIt = 0;
    
    /* Compute Request Command */
    req.REQ_FLAG  = (uint8_t)(flags & (~((uint32_t)RFAL_NFCV_REQ_FLAG_ADDRESS) & ~((uint32_t)RFAL_NFCV_REQ_FLAG_SELECT)));
    req.CMD       = RFAL_NFCV_CMD_LOCK_BLOCK;
    
    /* Check if request is to be sent in Addressed or Selected mode */
    if( uid != NULL )
    {
        req.REQ_FLAG |= (uint8_t)RFAL_NFCV_REQ_FLAG_ADDRESS;
        ST_MEMCPY( req.payload.UID, uid, RFAL_NFCV_UID_LEN );
        msgIt += (uint8_t)RFAL_NFCV_UID_LEN;
    }
    else
    {
        req.REQ_FLAG |= (uint8_t)RFAL_NFCV_REQ_FLAG_SELECT;
    }
    req.payload.data[msgIt++] = blockNum;
       
    /* Transceive Command */
    ret = rfalTransceiveBlockingTxRx( (uint8_t*)&req, (RFAL_CMD_LEN + RFAL_NFCV_FLAG_LEN + (uint16_t)msgIt), (uint8_t*)&res, sizeof(rfalNfcvGenericRes), &rcvLen, RFAL_TXRX_FLAGS_DEFAULT, RFAL_FDT_POLL_MAX );

    if( ret != ERR_NONE )
    {
        return ret;
    }
    
    /* Check if the response minimum length has been received */
    if( rcvLen < (uint8_t)RFAL_NFCV_FLAG_LEN )
    {
        return ERR_PROTO;
    }
    
    /* Check if an error has been signalled */
    if( (res.RES_FLAG & (uint8_t)RFAL_NFCV_RES_FLAG_ERROR) != 0U )
    {
        return rfalNfcvParseError( *res.data );
    }
    
    return ERR_NONE;
}

/*******************************************************************************/
ReturnCode rfalNfcvPollerReadMultipleBlocks( uint8_t flags, const uint8_t* uid, uint8_t firstBlockNum, uint8_t numOfBlocks, uint8_t* rxBuf, uint16_t rxBufLen, uint16_t *rcvLen )
{
    ReturnCode          ret;
    rfalNfcvGenericReq  req;
    uint8_t             msgIt;
    
    msgIt = 0;
    
    /* Compute Request Command */
    req.REQ_FLAG  = (uint8_t)(flags & (~((uint32_t)RFAL_NFCV_REQ_FLAG_ADDRESS) & ~((uint32_t)RFAL_NFCV_REQ_FLAG_SELECT)));
    req.CMD       = RFAL_NFCV_CMD_READ_MULTIPLE_BLOCKS;
    
    /* Check if request is to be sent in Addressed or Selected mode */
    if( uid != NULL )
    {
        req.REQ_FLAG |= (uint8_t)RFAL_NFCV_REQ_FLAG_ADDRESS;
        ST_MEMCPY( req.payload.UID, uid, RFAL_NFCV_UID_LEN );
        msgIt += (uint8_t)RFAL_NFCV_UID_LEN;
    }
    else
    {
        req.REQ_FLAG |= (uint8_t)RFAL_NFCV_REQ_FLAG_SELECT;
    }
    req.payload.data[msgIt++] = firstBlockNum;
    req.payload.data[msgIt++] = numOfBlocks;
    
    /* Transceive Command */
    ret = rfalTransceiveBlockingTxRx( (uint8_t*)&req, (RFAL_CMD_LEN + RFAL_NFCV_FLAG_LEN + (uint16_t)msgIt), rxBuf, rxBufLen, rcvLen, RFAL_TXRX_FLAGS_DEFAULT, RFAL_FDT_POLL_MAX );
    if( ret != ERR_NONE )
    {
        return ret;
    }
    
    /* Check if the response minimum length has been received */
    if( (*rcvLen) < (uint8_t)RFAL_NFCV_FLAG_LEN )
    {
        return ERR_PROTO;
    }
    
    /* Check if an error has been signalled */
    if( (rxBuf[RFAL_NFCV_FLAG_POS] & (uint8_t)RFAL_NFCV_RES_FLAG_ERROR) != 0U )
    {
        return rfalNfcvParseError( rxBuf[RFAL_NFCV_DATASTART_POS] );
    }
    
    return ERR_NONE;
}

/*******************************************************************************/
ReturnCode rfalNfcvPollerExtendedReadSingleBlock( uint8_t flags, const uint8_t* uid, uint16_t blockNum, uint8_t* rxBuf, uint16_t rxBufLen, uint16_t *rcvLen )
{
    ReturnCode         ret;
    rfalNfcvGenericReq req;
    uint8_t            msgIt;
    
    msgIt = 0;
    
    /* Compute Request Command */
    req.REQ_FLAG  = (uint8_t)(flags & (~((uint32_t)RFAL_NFCV_REQ_FLAG_ADDRESS) & ~((uint32_t)RFAL_NFCV_REQ_FLAG_SELECT)));
    req.CMD       = RFAL_NFCV_CMD_EXTENDED_READ_SINGLE_BLOCK;
    
    /* Check if request is to be sent in Addressed or Selected mode */
    if( uid != NULL )
    {
        req.REQ_FLAG |= (uint8_t)RFAL_NFCV_REQ_FLAG_ADDRESS;
        ST_MEMCPY( req.payload.UID, uid, RFAL_NFCV_UID_LEN );
        msgIt += RFAL_NFCV_UID_LEN;
    }
    else
    {
        req.REQ_FLAG |= (uint8_t)RFAL_NFCV_REQ_FLAG_SELECT;
    }
    req.payload.data[msgIt++] = (uint8_t)blockNum; /* TS T5T 1.0 BNo is considered as a multi-byte field. TS T5T 1.0 5.1.1.13 multi-byte field follows [DIGITAL]. [DIGITAL] 9.3.1 A multiple byte field is transmitted LSB first. */
    req.payload.data[msgIt++] = (uint8_t)(blockNum >> 8U);
    
    /* Transceive Command */
    ret = rfalTransceiveBlockingTxRx( (uint8_t*)&req, (RFAL_CMD_LEN + RFAL_NFCV_FLAG_LEN +(uint16_t)msgIt), rxBuf, rxBufLen, rcvLen, RFAL_TXRX_FLAGS_DEFAULT, RFAL_FDT_POLL_MAX );
    if( ret != ERR_NONE )
    {
        return ret;
    }
    
    /* Check if the response minimum length has been received */
    if( (*rcvLen) < (uint8_t)RFAL_NFCV_FLAG_LEN )
    {
        return ERR_PROTO;
    }
    
    /* Check if an error has been signalled */
    if( (rxBuf[RFAL_NFCV_FLAG_POS] & (uint8_t)RFAL_NFCV_RES_FLAG_ERROR) != 0U )
    {
        return rfalNfcvParseError( rxBuf[RFAL_NFCV_DATASTART_POS] );
    }
    
    return ERR_NONE;
}

/*******************************************************************************/
ReturnCode rfalNfcvPollerExtendedWriteSingleBlock( uint8_t flags, const uint8_t* uid, uint16_t blockNum, const uint8_t* wrData, uint8_t blockLen )
{
    ReturnCode         ret;
    uint16_t           rcvLen;
    rfalNfcvGenericReq req;
    rfalNfcvGenericRes res;
    uint8_t            msgIt;

    if( blockLen > (uint8_t)RFAL_NFCV_MAX_BLOCK_LEN )
    {
        return ERR_PARAM;
    }
    
    msgIt = 0;
    
    /* Compute Request Command */
    req.REQ_FLAG  = (uint8_t)(flags & (~((uint32_t)RFAL_NFCV_REQ_FLAG_ADDRESS) & ~((uint32_t)RFAL_NFCV_REQ_FLAG_SELECT)));
    req.CMD       = RFAL_NFCV_CMD_EXTENDED_WRITE_SINGLE_BLOCK;
    
    /* Check if request is to be sent in Addressed or Selected mode */
    if( uid != NULL )
    {
        req.REQ_FLAG |= (uint8_t)RFAL_NFCV_REQ_FLAG_ADDRESS;
        ST_MEMCPY( req.payload.UID, uid, RFAL_NFCV_UID_LEN );
        msgIt += (uint8_t)RFAL_NFCV_UID_LEN;
    }
    else
    {
        req.REQ_FLAG |= (uint8_t)RFAL_NFCV_REQ_FLAG_SELECT;
        
    }
    req.payload.data[msgIt++] = (uint8_t)blockNum; /* TS T5T 1.0 BNo is considered as a multi-byte field. TS T5T 1.0 5.1.1.13 multi-byte field follows [DIGITAL]. [DIGITAL] 9.3.1 A multiple byte field is transmitted LSB first. */
    req.payload.data[msgIt++] = (uint8_t)(blockNum >> 8U);;
    
    if( blockLen > 0U )         /* MISRA 21.18 */
    {
        ST_MEMCPY( &req.payload.data[msgIt], wrData, blockLen );
        msgIt += blockLen;
    }
    
    /* Transceive Command */
    ret = rfalTransceiveBlockingTxRx( (uint8_t*)&req, (RFAL_CMD_LEN + RFAL_NFCV_FLAG_LEN + (uint16_t)msgIt), (uint8_t*)&res, sizeof(rfalNfcvGenericRes), &rcvLen, RFAL_TXRX_FLAGS_DEFAULT, RFAL_FDT_POLL_MAX );

    if( ret != ERR_NONE )
    {
        return ret;
    }
    
    /* Check if the response minimum length has been received */
    if( rcvLen < (uint8_t)RFAL_NFCV_FLAG_LEN )
    {
        return ERR_PROTO;
    }
    
    /* Check if an error has been signalled */
    if( (res.RES_FLAG & (uint8_t)RFAL_NFCV_RES_FLAG_ERROR) != 0U )
    {
        return rfalNfcvParseError( *res.data );
    }
    
    return ERR_NONE;
}

/*******************************************************************************/
ReturnCode rfalNfcvPollerExtendedLockSingleBlock( uint8_t flags, const uint8_t* uid, uint16_t blockNum )
{
    ReturnCode         ret;
    uint16_t           rcvLen;
    rfalNfcvGenericReq req;
    rfalNfcvGenericRes res;
    uint8_t            msgIt;

    msgIt = 0;
    
    /* Compute Request Command */
    req.REQ_FLAG  = (uint8_t)(flags & (~((uint32_t)RFAL_NFCV_REQ_FLAG_ADDRESS) & ~((uint32_t)RFAL_NFCV_REQ_FLAG_SELECT)));
    req.CMD       = RFAL_NFCV_CMD_EXTENDED_LOCK_SINGLE_BLOCK;
    
    /* Check if request is to be sent in Addressed or Selected mode */
    if( uid != NULL )
    {
        req.REQ_FLAG |= (uint8_t)RFAL_NFCV_REQ_FLAG_ADDRESS;
        ST_MEMCPY( req.payload.UID, uid, RFAL_NFCV_UID_LEN );
        msgIt += (uint8_t)RFAL_NFCV_UID_LEN;
    }
    else
    {
        req.REQ_FLAG |= (uint8_t)RFAL_NFCV_REQ_FLAG_SELECT;
    }
    req.payload.data[msgIt++] = (uint8_t)blockNum; /* TS T5T 1.0 BNo is considered as a multi-byte field. TS T5T 1.0 5.1.1.13 multi-byte field follows [DIGITAL]. [DIGITAL] 9.3.1 A multiple byte field is transmitted LSB first. */
    req.payload.data[msgIt++] = (uint8_t)(blockNum >> 8U);;
       
    /* Transceive Command */
    ret = rfalTransceiveBlockingTxRx( (uint8_t*)&req, (RFAL_CMD_LEN + RFAL_NFCV_FLAG_LEN + (uint16_t)msgIt), (uint8_t*)&res, sizeof(rfalNfcvGenericRes), &rcvLen, RFAL_TXRX_FLAGS_DEFAULT, RFAL_FDT_POLL_MAX );

    if( ret != ERR_NONE )
    {
        return ret;
    }
    
    /* Check if the response minimum length has been received */
    if( rcvLen < (uint8_t)RFAL_NFCV_FLAG_LEN )
    {
        return ERR_PROTO;
    }
    
    /* Check if an error has been signalled */
    if( (res.RES_FLAG & (uint8_t)RFAL_NFCV_RES_FLAG_ERROR) != 0U )
    {
        return rfalNfcvParseError( *res.data );
    }
    
    return ERR_NONE;
}

/*******************************************************************************/
ReturnCode rfalNfcvPollerExtendedReadMultipleBlocks( uint8_t flags, const uint8_t* uid, uint16_t firstBlockNum, uint16_t numOfBlocks, uint8_t* rxBuf, uint16_t rxBufLen, uint16_t *rcvLen )
{
    ReturnCode          ret;
    rfalNfcvGenericReq  req;
    uint8_t             msgIt;
    
    msgIt = 0;
    
    /* Compute Request Command */
    req.REQ_FLAG  = (uint8_t)(flags & (~((uint32_t)RFAL_NFCV_REQ_FLAG_ADDRESS) & ~((uint32_t)RFAL_NFCV_REQ_FLAG_SELECT)));
    req.CMD       = RFAL_NFCV_CMD_EXTENDED_READ_MULTIPLE_BLOCK;
    
    /* Check if request is to be sent in Addressed or Selected mode */
    if( uid != NULL )
    {
        req.REQ_FLAG |= (uint8_t)RFAL_NFCV_REQ_FLAG_ADDRESS;
        ST_MEMCPY( req.payload.UID, uid, RFAL_NFCV_UID_LEN );
        msgIt += (uint8_t)RFAL_NFCV_UID_LEN;
    }
    else
    {
        req.REQ_FLAG |= (uint8_t)RFAL_NFCV_REQ_FLAG_SELECT;
    }
    
    req.payload.data[msgIt++] = (uint8_t)((firstBlockNum >> 8) & 0xFFU);
    req.payload.data[msgIt++] = (uint8_t)((firstBlockNum >> 0) & 0xFFU);
    req.payload.data[msgIt++] = (uint8_t)((numOfBlocks >> 8) & 0xFFU);
    req.payload.data[msgIt++] = (uint8_t)((numOfBlocks >> 0) & 0xFFU);

    
    /* Transceive Command */
    ret = rfalTransceiveBlockingTxRx( (uint8_t*)&req, (RFAL_CMD_LEN + RFAL_NFCV_FLAG_LEN + (uint16_t)msgIt), rxBuf, rxBufLen, rcvLen, RFAL_TXRX_FLAGS_DEFAULT, RFAL_FDT_POLL_MAX );
    if( ret != ERR_NONE )
    {
        return ret;
    }
    
    /* Check if the response minimum length has been received */
    if( (*rcvLen) < (uint8_t)RFAL_NFCV_FLAG_LEN )
    {
        return ERR_PROTO;
    }
    
    /* Check if an error has been signalled */
    if( (rxBuf[RFAL_NFCV_FLAG_POS] & (uint8_t)RFAL_NFCV_RES_FLAG_ERROR) != 0U )
    {
        return rfalNfcvParseError( rxBuf[RFAL_NFCV_DATASTART_POS] );
    }
    
    return ERR_NONE;
}

#endif /* RFAL_FEATURE_NFCV */
