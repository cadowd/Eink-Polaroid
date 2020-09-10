
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

/*! \file rfal_nfcf.c
 *
 *  \author Gustavo Patricio
 *
 *  \brief Implementation of NFC-F Poller (FeliCa PCD) device
 *
 *  The definitions and helpers methods provided by this module are 
 *  aligned with NFC-F (FeliCa - JIS X6319-4)
 *
 */

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include "rfal_nfcf.h"
#include "utils.h"

/*
 ******************************************************************************
 * ENABLE SWITCH
 ******************************************************************************
 */

#ifndef RFAL_FEATURE_NFCF
    #error " RFAL: Module configuration missing. Please enable/disable NFC-F module by setting: RFAL_FEATURE_NFCF "
#endif

#if RFAL_FEATURE_NFCF

/*
 ******************************************************************************
 * GLOBAL DEFINES
 ******************************************************************************
 */
#define RFAL_NFCF_SENSF_REQ_LEN_MIN                5U     /*!< SENSF_RES minimum length                           */

#define RFAL_NFCF_READ_WO_ENCRYPTION_MIN_LEN       15U    /*!< Minimum length for a Check Command   -  T3T  5.4.1 */
#define RFAL_NFCF_WRITE_WO_ENCRYPTION_MIN_LEN      31U    /*!< Minimum length for an Update Command -  T3T  5.5.1 */


/*
 ******************************************************************************
 * GLOBAL MACROS
 ******************************************************************************
 */
#define rfalNfcfSlots2CardNum( s )                 ((uint8_t)(s)+1U) /*!< Converts Time Slot Number (TSN) into num of slots  */

/*
******************************************************************************
* GLOBAL TYPES
******************************************************************************
*/

/*! Structure/Buffer to hold the SENSF_RES with LEN byte prepended                                 */
typedef struct{
    uint8_t           LEN;                                /*!< NFC-F LEN byte                      */
    rfalNfcfSensfRes  SENSF_RES;                          /*!< SENSF_RES                           */
} rfalNfcfSensfResBuf;


/*! Greedy collection for NFCF GRE_POLL_F  Activity 1.0 Table 10                                   */
typedef struct{
    uint8_t              pollFound;                       /*!< Number of devices found by the Poll */
    uint8_t              pollCollision;                   /*!< Number of collisions detected       */
    rfalFeliCaPollRes    POLL_F[RFAL_NFCF_POLL_MAXCARDS]; /*!< GRE_POLL_F   Activity 1.0 Table 10  */
} rfalNfcfGreedyF;


/*! NFC-F SENSF_REQ format  Digital 1.1  8.6.1                     */
typedef struct
{
    uint8_t  CMD;                          /*!< Command code: 00h  */
    uint8_t  SC[RFAL_NFCF_SENSF_SC_LEN];   /*!< System Code        */
    uint8_t  RC;                           /*!< Request Code       */
    uint8_t  TSN;                          /*!< Time Slot Number   */
} rfalNfcfSensfReq;


/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/
static rfalNfcfGreedyF gRfalNfcfGreedyF;   /*!< Activity's NFCF Greedy collection */


/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/
static void rfalNfcfComputeValidSENF( rfalNfcfListenDevice *outDevInfo, uint8_t *curDevIdx, uint8_t devLimit, bool overwrite, bool *nfcDepFound );


/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/

/*******************************************************************************/
static void rfalNfcfComputeValidSENF( rfalNfcfListenDevice *outDevInfo, uint8_t *curDevIdx, uint8_t devLimit, bool overwrite, bool *nfcDepFound )
{
    uint8_t             tmpIdx;
    bool                duplicate;    
    rfalNfcfSensfResBuf *sensfBuf;
    rfalNfcfSensfResBuf sensfCopy;
    
    
    /*******************************************************************************/
    /* Go through all responses check if valid and duplicates                      */
    /*******************************************************************************/
    while( (gRfalNfcfGreedyF.pollFound > 0U) && ((*curDevIdx) < devLimit) )
    {
        duplicate = false;
        gRfalNfcfGreedyF.pollFound--;
        
        /* MISRA 11.3 - Cannot point directly into different object type, use local copy */
        ST_MEMCPY( (uint8_t*)&sensfCopy, (uint8_t*)&gRfalNfcfGreedyF.POLL_F[gRfalNfcfGreedyF.pollFound], sizeof(rfalNfcfSensfResBuf) );
        
        
        /* Point to received SENSF_RES */
        sensfBuf = &sensfCopy;
        
        
        /* Check for devices that are already in device list */
        for( tmpIdx = 0; tmpIdx < (*curDevIdx); tmpIdx++ )
        {
            if( ST_BYTECMP( sensfBuf->SENSF_RES.NFCID2, outDevInfo[tmpIdx].sensfRes.NFCID2, RFAL_NFCF_NFCID2_LEN ) == 0 )
            {
                duplicate = true;
                break;
            }
        }
        
        /* If is a duplicate skip this (and not to overwrite)*/        
        if(duplicate && !overwrite)
        {
            continue;
        }
        
        /* Check if response length is OK */
        if( (( sensfBuf->LEN - RFAL_NFCF_HEADER_LEN) < RFAL_NFCF_SENSF_RES_LEN_MIN) || ((sensfBuf->LEN - RFAL_NFCF_HEADER_LEN) > RFAL_NFCF_SENSF_RES_LEN_MAX) )
        {
            continue;
        }
        
        /* Check if the response is a SENSF_RES / Polling response */
        if( sensfBuf->SENSF_RES.CMD != (uint8_t)RFAL_NFCF_CMD_POLLING_RES )
        {
            continue;
        }
        
        /* Check if is an overwrite request or new device*/
        if(duplicate && overwrite)
        {
            /* overwrite deviceInfo/GRE_SENSF_RES with SENSF_RES */
            outDevInfo[tmpIdx].sensfResLen = (sensfBuf->LEN - RFAL_NFCF_LENGTH_LEN);
            ST_MEMCPY( &outDevInfo[tmpIdx].sensfRes, &sensfBuf->SENSF_RES, outDevInfo[tmpIdx].sensfResLen );
            continue;
        }
        else
        {
            /* fill deviceInfo/GRE_SENSF_RES with new SENSF_RES */
            outDevInfo[(*curDevIdx)].sensfResLen = (sensfBuf->LEN - RFAL_NFCF_LENGTH_LEN);
            ST_MEMCPY( &outDevInfo[(*curDevIdx)].sensfRes, &sensfBuf->SENSF_RES, outDevInfo[(*curDevIdx)].sensfResLen );            
        }
        
        /* Check if this device supports NFC-DEP and signal it (ACTIVITY 1.1   9.3.6.63) */        
        *nfcDepFound = rfalNfcfIsNfcDepSupported( &outDevInfo[(*curDevIdx)] );
                
        (*curDevIdx)++;
    }
}

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/

/*******************************************************************************/
ReturnCode rfalNfcfPollerInitialize( rfalBitRate bitRate )
{
    ReturnCode ret;
    
    if( (bitRate != RFAL_BR_212) && (bitRate != RFAL_BR_424) )
    {
        return ERR_PARAM;
    }
    
    EXIT_ON_ERR( ret, rfalSetMode( RFAL_MODE_POLL_NFCF, bitRate, bitRate ) );
    rfalSetErrorHandling( RFAL_ERRORHANDLING_NFC );
    
    rfalSetGT( RFAL_GT_NFCF );
    rfalSetFDTListen( RFAL_FDT_LISTEN_NFCF_POLLER );
    rfalSetFDTPoll( RFAL_FDT_POLL_NFCF_POLLER );
    
    return ERR_NONE;
}



/*******************************************************************************/
ReturnCode rfalNfcfPollerPoll( rfalFeliCaPollSlots slots, uint16_t sysCode, uint8_t reqCode, rfalFeliCaPollRes *cardList, uint8_t *devCnt, uint8_t *collisions )
{
    return rfalFeliCaPoll( slots, sysCode, reqCode, cardList, rfalNfcfSlots2CardNum(slots), devCnt, collisions );
}

/*******************************************************************************/
ReturnCode rfalNfcfPollerCheckPresence( void )
{
    gRfalNfcfGreedyF.pollFound     = 0;
    gRfalNfcfGreedyF.pollCollision = 0;
        
    /* ACTIVITY 1.0 & 1.1 - 9.2.3.17 SENSF_REQ  must be with number of slots equal to 4
     *                                SC must be 0xFFFF
     *                                RC must be 0x00 (No system code info required) */
    return rfalFeliCaPoll( RFAL_FELICA_4_SLOTS, RFAL_NFCF_SYSTEMCODE, RFAL_FELICA_POLL_RC_NO_REQUEST, gRfalNfcfGreedyF.POLL_F, rfalNfcfSlots2CardNum(RFAL_FELICA_4_SLOTS), &gRfalNfcfGreedyF.pollFound, &gRfalNfcfGreedyF.pollCollision );
}


/*******************************************************************************/
ReturnCode rfalNfcfPollerCollisionResolution( rfalComplianceMode compMode, uint8_t devLimit, rfalNfcfListenDevice *nfcfDevList, uint8_t *devCnt )
{
    ReturnCode  ret;
    bool        nfcDepFound;
    
    if( (nfcfDevList == NULL) || (devCnt == NULL) )
    {
        return ERR_PARAM;
    }
            
    *devCnt      = 0;
    nfcDepFound  = false;
    
    
    /*******************************************************************************************/
    /* ACTIVITY 1.0 - 9.3.6.3 Copy valid SENSF_RES in GRE_POLL_F into GRE_SENSF_RES            */
    /* ACTIVITY 1.0 - 9.3.6.6 The NFC Forum Device MUST remove all entries from GRE_SENSF_RES[]*/
    /* ACTIVITY 1.1 - 9.3.63.59 Populate GRE_SENSF_RES with data from GRE_POLL_F               */
    /*                                                                                         */
    /* CON_DEVICES_LIMIT = 0 Just check if devices from Tech Detection exceeds -> always true  */
    /* Allow the number of slots open on Technology Detection                                  */
    /*******************************************************************************************/
    rfalNfcfComputeValidSENF( nfcfDevList, devCnt, ((devLimit == 0U) ? rfalNfcfSlots2CardNum( RFAL_FELICA_4_SLOTS ) : devLimit), false, &nfcDepFound );

    
    /*******************************************************************************/
    /* ACTIVITY 1.0 - 9.3.6.4                                                      */
    /* ACTIVITY 1.1 - 9.3.63.60 Check if devices found are lower than the limit    */
    /* and send a SENSF_REQ if so                                                  */
    /*******************************************************************************/
    if( *devCnt < devLimit )
    {
        /* ACTIVITY 1.0 - 9.3.6.5  Copy valid SENSF_RES and then to remove it
         * ACTIVITY 1.1 - 9.3.6.65 Copy and filter duplicates                                           
         * For now, due to some devices keep generating different nfcid2, we use 1.0  
         * Phones detected: Samsung Galaxy Nexus,Samsung Galaxy S3,Samsung Nexus S */
        *devCnt = 0;
        
        ret = rfalNfcfPollerPoll( RFAL_FELICA_16_SLOTS, RFAL_NFCF_SYSTEMCODE, RFAL_FELICA_POLL_RC_NO_REQUEST, gRfalNfcfGreedyF.POLL_F, &gRfalNfcfGreedyF.pollFound, &gRfalNfcfGreedyF.pollCollision );
        if( ret == ERR_NONE )
        {
            rfalNfcfComputeValidSENF( nfcfDevList, devCnt, devLimit, false, &nfcDepFound );
        }
      
      /*******************************************************************************/
      /* ACTIVITY 1.1 -  9.3.6.63 Check if any device supports NFC DEP               */
      /*******************************************************************************/
      if( nfcDepFound && (compMode == RFAL_COMPLIANCE_MODE_NFC) )
      {
          ret = rfalNfcfPollerPoll( RFAL_FELICA_16_SLOTS, RFAL_NFCF_SYSTEMCODE, RFAL_FELICA_POLL_RC_SYSTEM_CODE, gRfalNfcfGreedyF.POLL_F, &gRfalNfcfGreedyF.pollFound, &gRfalNfcfGreedyF.pollCollision );
          if( ret == ERR_NONE )
          {
              rfalNfcfComputeValidSENF( nfcfDevList, devCnt, devLimit, true, &nfcDepFound );
          }
      }
    }
    
    return ERR_NONE;
}


/*******************************************************************************/
bool rfalNfcfListenerIsT3TReq( const uint8_t* buf, uint16_t bufLen, uint8_t* nfcid2 )
{
    /* Check cmd byte */
    switch( *buf )
    {
        case RFAL_NFCF_CMD_READ_WITHOUT_ENCRYPTION:
            if( bufLen < RFAL_NFCF_READ_WO_ENCRYPTION_MIN_LEN )
            {
                return false;
            }
            break;
            
        case RFAL_NFCF_CMD_WRITE_WITHOUT_ENCRYPTION:
            if( bufLen < RFAL_NFCF_WRITE_WO_ENCRYPTION_MIN_LEN )
            {
                return false;
            }
            break;
            
        default:
            return false;       
    }
    
    /* Output NFID2 if requested */
    if( nfcid2 != NULL )
    {
        ST_MEMCPY( nfcid2, &buf[RFAL_NFCF_CMD_LEN], RFAL_NFCF_NFCID2_LEN );
    }
    
    return true;
}

#endif /* RFAL_FEATURE_NFCF */
