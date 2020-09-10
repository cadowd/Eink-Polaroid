
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

/*! \file rfal_nfcv.h
 *
 *  \author Gustavo Patricio
 *
 *  \brief Implementation of NFC-V Poller (ISO15693) device
 *
 *  The definitions and helpers methods provided by this module 
 *  are aligned with NFC-V Digital 2.0 (Candidate)  
 *
 *
 * \addtogroup RFAL
 * @{
 *
 * \addtogroup RFAL-AL
 * \brief RFAL Abstraction Layer
 * @{
 *
 * \addtogroup NFC-V
 * \brief RFAL NFC-V Module
 * @{
 * 
 */

#ifndef RFAL_NFCV_H
#define RFAL_NFCV_H

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include "platform.h"
#include "st_errno.h"
#include "rfal_rf.h"

/*
 ******************************************************************************
 * GLOBAL DEFINES
 ******************************************************************************
 */
#define RFAL_NFCV_UID_LEN                  8U    /*!< NFC-V UID length                                            */
#define RFAL_NFCV_MAX_BLOCK_LEN           32U    /*!< Max Block size: can be of up to 256 bits  ISO 15693 2000  5 */



/*! NFC-V RequestFlags   ISO15693 2000 7.3.1 */
enum{
    RFAL_NFCV_REQ_FLAG_DEFAULT           = 0x02,       /*!< Default Request Flags           */
    RFAL_NFCV_REQ_FLAG_SUB_CARRIER       = 0x01,       /*!< Sub Carrier flag                */
    RFAL_NFCV_REQ_FLAG_DATA_RATE         = 0x02,       /*!< Data Rate flag                  */
    RFAL_NFCV_REQ_FLAG_INVENTORY         = 0x04,       /*!< Inventory flag                  */
    RFAL_NFCV_REQ_FLAG_PROTOCOL_EXT      = 0x08,       /*!< Protocol Extension flag         */
    RFAL_NFCV_REQ_FLAG_SELECT            = 0x10,       /*!< Select flag                     */
    RFAL_NFCV_REQ_FLAG_ADDRESS           = 0x20,       /*!< Address flag                    */
    RFAL_NFCV_REQ_FLAG_OPTION            = 0x40,       /*!< Option flag                     */
    RFAL_NFCV_REQ_FLAG_RFU               = 0x80,       /*!< RFU flag                        */
    RFAL_NFCV_REQ_FLAG_AFI               = 0x10,       /*!< AFI flag                        */
    RFAL_NFCV_REQ_FLAG_NB_SLOTS          = 0x20,       /*!< Number of Slots flag            */
};

/*! NFC-V Response Flags   ISO15693 2000 7.4.1 */
enum{
    RFAL_NFCV_RES_FLAG_ERROR             = 0x01,       /*!< Error flag                      */
    RFAL_NFCV_RES_FLAG_RFU1              = 0x02,       /*!< RFU flag                        */
    RFAL_NFCV_RES_FLAG_RFU2              = 0x04,       /*!< RFU flag                        */
    RFAL_NFCV_RES_FLAG_EXTENSION         = 0x08,       /*!< Extension flag                  */
    RFAL_NFCV_RES_FLAG_RFU3              = 0x10,       /*!< RFU flag                        */
    RFAL_NFCV_RES_FLAG_RFU4              = 0x20,       /*!< RFU flag                        */
    RFAL_NFCV_RES_FLAG_RFU5              = 0x40,       /*!< RFU flag                        */
    RFAL_NFCV_RES_FLAG_RFU6              = 0x80,       /*!< RFU flag                        */
};

/*! NFC-V Error code  ISO15693 2000 7.4.2 */
enum{
    RFAL_NFCV_ERROR_CMD_NOT_SUPPORTED    = 0x01,       /*!< The command is not supported, code is not recognised */
    RFAL_NFCV_ERROR_CMD_NOT_RECOGNIZED   = 0x02,       /*!< The command is not recognised, format error occurred */
    RFAL_NFCV_ERROR_OPTION_NOT_SUPPORTED = 0x03,       /*!< The option is not supported                          */
    RFAL_NFCV_ERROR_UNKNOWN              = 0x0F,       /*!< Unknown error                                        */
    RFAL_NFCV_ERROR_BLOCK_NOT_AVALIABLE  = 0x10,       /*!< The specified block is not available                 */
    RFAL_NFCV_ERROR_BLOCK_ALREDY_LOCKED  = 0x11,       /*!< The specified block is already locked                */
    RFAL_NFCV_ERROR_BLOCK_LOCKED         = 0x12,       /*!< The specified block is locked                        */
    RFAL_NFCV_ERROR_WRITE_FAILED         = 0x13,       /*!< The specified block was not successfully programmed  */
    RFAL_NFCV_ERROR_BLOCK_FAILED         = 0x14,       /*!< The specified block was not successfully locked      */
};


/*! NFC-V command set   ISO15693 2000 9.1 */
enum 
{
    RFAL_NFCV_CMD_INVENTORY                     = 0x01,      /*!< INVENTORY_REQ (Inventory) command                            */
    RFAL_NFCV_CMD_SLPV                          = 0x02,      /*!< SLPV_REQ (Stay quiet) command                                */
    RFAL_NFCV_CMD_READ_SINGLE_BLOCK             = 0x20,      /*!< Read single block command                                    */
    RFAL_NFCV_CMD_WRITE_SINGLE_BLOCK            = 0x21,      /*!< Write single block command                                   */
    RFAL_NFCV_CMD_LOCK_BLOCK                    = 0x22,      /*!< Lock block command                                           */
    RFAL_NFCV_CMD_READ_MULTIPLE_BLOCKS          = 0x23,      /*!< Read multiple blocks command                                 */
    RFAL_NFCV_CMD_WRITE_MULTIPLE_BLOCKS         = 0x24,      /*!< Write multiple blocks command                                */
    RFAL_NFCV_CMD_SELECT                        = 0x25,      /*!< Select command                                               */
    RFAL_NFCV_CMD_RESET_TO_READY                = 0x26,      /*!< Reset To Ready command                                       */
    RFAL_NFCV_CMD_GET_SYS_INFO                  = 0x2B,      /*!< Get System Information command                               */
    RFAL_NFCV_CMD_EXTENDED_READ_SINGLE_BLOCK    = 0x30,      /*!< Extended read single block command                           */
    RFAL_NFCV_CMD_EXTENDED_WRITE_SINGLE_BLOCK   = 0x31,      /*!< Extended write single block command                          */
    RFAL_NFCV_CMD_EXTENDED_LOCK_SINGLE_BLOCK    = 0x32,      /*!< Extended lock single block command                           */
    RFAL_NFCV_CMD_EXTENDED_READ_MULTIPLE_BLOCK  = 0x33,      /*!< Extended read multiple block command                         */
    
};

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

/*! NFC-V Number of slots  Digital 2.0  9.6.1 */
typedef enum 
{
    RFAL_NFCV_NUM_SLOTS_1    =  0x20,   /*!< Number of slots: 1             */
    RFAL_NFCV_NUM_SLOTS_16   =  0x00,   /*!< Number of slots: 16            */
} rfalNfcvNumSlots;


/*! NFC-V INVENTORY_RES format   Digital 2.0  9.6.2 */
typedef struct 
{
    uint8_t RES_FLAG;                   /*!< Response Flags                 */
    uint8_t DSFID;                      /*!< Data Storage Format Identifier */
    uint8_t UID[RFAL_NFCV_UID_LEN];     /*!< NFC-V device UID               */
    uint8_t crc[RFAL_CRC_LEN];          /*!< CRC                            */
} rfalNfcvInventoryRes;


/*! NFC-V listener device (VICC) struct  */
typedef struct
{
    rfalNfcvInventoryRes    InvRes;     /*!< INVENTORY_RES                  */
    bool                    isSleep;    /*!< Device sleeping flag           */
} rfalNfcvListenDevice;


/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/

/*! 
 *****************************************************************************
 * \brief  Initialize NFC-V Poller mode
 *  
 * This methods configures RFAL RF layer to perform as a 
 * NFC-F Poller/RW (ISO15693) including all default timings 
 *
 * \return ERR_WRONG_STATE  : RFAL not initialized or mode not set
 * \return ERR_PARAM        : Incorrect bitrate
 * \return ERR_NONE         : No error
 *****************************************************************************
 */
ReturnCode rfalNfcvPollerInitialize( void );

/*! 
 *****************************************************************************
 * \brief  NFC-V Poller Check Presence
 *  
 * This method checks if a NFC-V Listen device (VICC) is present on the field
 * by sending an Inventory (INVENTORY_REQ) 
 *  
 * \param[out] invRes : If received, the INVENTORY_RES
 *
 * \return ERR_WRONG_STATE  : RFAL not initialized or incorrect mode
 * \return ERR_PARAM        : Invalid parameters
 * \return ERR_IO           : Generic internal error
 * \return ERR_TIMEOUT      : Timeout error, no listener device detectedd
 * \return ERR_NONE         : No error, one or more device in the field
 *****************************************************************************
 */
ReturnCode rfalNfcvPollerCheckPresence( rfalNfcvInventoryRes *invRes );

/*! 
 *****************************************************************************
 * \brief NFC-F Poller Poll
 * 
 * This function sends to all VICCs in field the INVENTORY command with the 
 * given number of slots
 * 
 * If more than one slot is used the following EOF need to be handled
 * by the caller using rfalISO15693TransceiveEOFAnticollision()
 *
 * \param[in]  nSlots  : Number of Slots to be sent (1 or 16)
 * \param[in]  maskLen : Number bits on the Mask value
 * \param[in]  maskVal : location of the Mask value
 * \param[out] invRes  : location to place the INVENTORY_RES
 * \param[out] rcvdLen : number of bits received (without collision)
 * 
 * \return ERR_WRONG_STATE  : RFAL not initialized or incorrect mode
 * \return ERR_PARAM        : Invalid parameters
 * \return ERR_IO           : Generic internal error
 * \return ERR_RF_COLLISION : Collision detected 
 * \return ERR_CRC          : CRC error detected
 * \return ERR_PROTO        : Protocol error detected
 * \return ERR_NONE         : No error
 *****************************************************************************
 */ 
ReturnCode rfalNfcvPollerInventory( rfalNfcvNumSlots nSlots, uint8_t maskLen, const uint8_t *maskVal, rfalNfcvInventoryRes *invRes, uint16_t* rcvdLen );

/*! 
 *****************************************************************************
 * \brief  NFC-V Poller Full Collision Resolution
 *  
 * Performs a full Collision resolution as defined in Activity 2.0   9.3.7
 * Once done, the devCnt will indicate how many (if any) devices have 
 * been identified and their details are contained on nfcvDevList
 *
 * \param[in]  devLimit     : device limit value, and size nfcaDevList
 * \param[out] nfcvDevList  : NFC-v listener devices list
 * \param[out] devCnt       : Devices found counter
 *
 * \return ERR_WRONG_STATE  : RFAL not initialized or mode not set
 * \return ERR_PARAM        : Invalid parameters
 * \return ERR_IO           : Generic internal error
 * \return ERR_NONE         : No error
 *****************************************************************************
 */
ReturnCode rfalNfcvPollerCollisionResolution( uint8_t devLimit, rfalNfcvListenDevice *nfcvDevList, uint8_t *devCnt );

/*! 
 *****************************************************************************
 * \brief  NFC-V Poller Sleep
 *  
 * This function is used to send the SLPV_REQ (Stay Quiet) command to put the VICC 
 * with the given UID to state QUIET so that they do not reply to more Inventory
 * 
 * \param[in]  flags        : Flags to be used: Sub-carrier; Data_rate; Option
 *                            for NFC-Forum use: RFAL_NFCV_REQ_FLAG_DEFAULT
 * \param[in]  uid          : UID of the device to be put to Sleep
 *  
 * \return ERR_WRONG_STATE  : RFAL not initialized or incorrect mode
 * \return ERR_PARAM        : Invalid parameters
 * \return ERR_IO           : Generic internal error
 * \return ERR_NONE         : No error
 *****************************************************************************
 */
ReturnCode rfalNfcvPollerSleep( uint8_t flags, const uint8_t* uid );

/*! 
 *****************************************************************************
 * \brief  NFC-V Poller Select
 *  
 * Selects a device (VICC) by its UID 
 *
 * \param[in]  flags        : Flags to be used: Sub-carrier; Data_rate; Option
 *                            for NFC-Forum use: RFAL_NFCV_REQ_FLAG_DEFAULT
 * \param[in]  uid          : UID of the device to be put to be Selected
 *  
 * \return ERR_WRONG_STATE  : RFAL not initialized or incorrect mode
 * \return ERR_PARAM        : Invalid parameters
 * \return ERR_IO           : Generic internal error 
 * \return ERR_CRC          : CRC error detected
 * \return ERR_FRAMING      : Framing error detected
 * \return ERR_PROTO        : Protocol error detected
 * \return ERR_TIMEOUT      : Timeout error
 * \return ERR_NONE         : No error
 *****************************************************************************
 */
ReturnCode rfalNfcvPollerSelect( uint8_t flags, const uint8_t* uid );

/*! 
 *****************************************************************************
 * \brief  NFC-V Poller Read Single Block
 *  
 * Reads a Single Block from a device (VICC)  
 *
 * \param[in]  flags        : Flags to be used: Sub-carrier; Data_rate; Option
 *                            for NFC-Forum use: RFAL_NFCV_REQ_FLAG_DEFAULT
 * \param[in]  uid          : UID of the device to be put to be read
 *                             if not provided Select mode will be used 
 * \param[in]  blockNum     : Number of the block to read
 * \param[out] rxBuf        : buffer to store response (also with RES_FLAGS)
 * \param[in]  rxBufLen     : length of rxBuf
 * \param[out] rcvLen       : number of bytes received
 *  
 * \return ERR_WRONG_STATE  : RFAL not initialized or incorrect mode
 * \return ERR_PARAM        : Invalid parameters
 * \return ERR_IO           : Generic internal error 
 * \return ERR_CRC          : CRC error detected
 * \return ERR_FRAMING      : Framing error detected
 * \return ERR_PROTO        : Protocol error detected
 * \return ERR_TIMEOUT      : Timeout error
 * \return ERR_NONE         : No error
 *****************************************************************************
 */
ReturnCode rfalNfcvPollerReadSingleBlock( uint8_t flags, const uint8_t* uid, uint8_t blockNum, uint8_t* rxBuf, uint16_t rxBufLen, uint16_t *rcvLen );

/*! 
 *****************************************************************************
 * \brief  NFC-V Poller Write Single Block
 *  
 * Writes a Single Block from a device (VICC)
 *
 * \param[in]  flags        : Flags to be used: Sub-carrier; Data_rate; Option
 *                            for NFC-Forum use: RFAL_NFCV_REQ_FLAG_DEFAULT
 * \param[in]  uid          : UID of the device to be put to be written
 *                             if not provided Select mode will be used 
 * \param[in]  blockNum     : Number of the block to write
 * \param[in]  wrData       : data to be written on the given block
 * \param[in]  blockLen     : number of bytes of a block
 *  
 * \return ERR_WRONG_STATE  : RFAL not initialized or incorrect mode
 * \return ERR_PARAM        : Invalid parameters
 * \return ERR_IO           : Generic internal error 
 * \return ERR_CRC          : CRC error detected
 * \return ERR_FRAMING      : Framing error detected
 * \return ERR_PROTO        : Protocol error detected
 * \return ERR_TIMEOUT      : Timeout error
 * \return ERR_NONE         : No error
 *****************************************************************************
 */
ReturnCode rfalNfcvPollerWriteSingleBlock( uint8_t flags, const uint8_t* uid, uint8_t blockNum, const uint8_t* wrData, uint8_t blockLen );

/*! 
 *****************************************************************************
 * \brief  NFC-V Poller Read Multiple Blocks
 *  
 * Reads Multiple Blocks from a device (VICC)  
 *
 * \param[in]  flags          : Flags to be used: Sub-carrier; Data_rate; Option
 *                              for NFC-Forum use: RFAL_NFCV_REQ_FLAG_DEFAULT
 * \param[in]  uid            : UID of the device to be put to be read
 *                               if not provided Select mode will be used 
 * \param[in]  firstBlockNum  : first block to be read
 * \param[in]  numOfBlocks    : number of block to read
 * \param[out] rxBuf          : buffer to store response (also with RES_FLAGS)
 * \param[in]  rxBufLen       : length of rxBuf
 * \param[out] rcvLen         : number of bytes received
 *  
 * \return ERR_WRONG_STATE    : RFAL not initialized or incorrect mode
 * \return ERR_PARAM          : Invalid parameters
 * \return ERR_IO             : Generic internal error 
 * \return ERR_CRC            : CRC error detected
 * \return ERR_FRAMING        : Framing error detected
 * \return ERR_PROTO          : Protocol error detected
 * \return ERR_TIMEOUT        : Timeout error
 * \return ERR_NONE           : No error
 *****************************************************************************
 */
ReturnCode rfalNfcvPollerReadMultipleBlocks( uint8_t flags, const uint8_t* uid, uint8_t firstBlockNum, uint8_t numOfBlocks, uint8_t* rxBuf, uint16_t rxBufLen, uint16_t *rcvLen );


/*! 
 *****************************************************************************
 * \brief  NFC-V Poller Extended Lock Single Block
 *  
 * Blocks a Single Block from a device (VICC)  
 *
 * \param[in]  flags        : Flags to be used: Sub-carrier; Data_rate; Option
 *                            for NFC-Forum use: RFAL_NFCV_REQ_FLAG_DEFAULT
 * \param[in]  uid          : UID of the device
 *                             if not provided Select mode will be used 
 * \param[in]  blockNum     : Number of the block to be locked
 *  
 * \return ERR_WRONG_STATE  : RFAL not initialized or incorrect mode
 * \return ERR_PARAM        : Invalid parameters
 * \return ERR_IO           : Generic internal error 
 * \return ERR_CRC          : CRC error detected
 * \return ERR_FRAMING      : Framing error detected
 * \return ERR_PROTO        : Protocol error detected
 * \return ERR_TIMEOUT      : Timeout error
 * \return ERR_NONE         : No error
 *****************************************************************************
 */
ReturnCode rfalNfcvPollerLockBlock( uint8_t flags, const uint8_t* uid, uint8_t blockNum );
    
/*! 
 *****************************************************************************
 * \brief  NFC-V Poller Extended Lock Single Block
 *  
 * Blocks a Single Block from a device (VICC)  
 *
 * \param[in]  flags        : Flags to be used: Sub-carrier; Data_rate; Option
 *                            for NFC-Forum use: RFAL_NFCV_REQ_FLAG_DEFAULT
 * \param[in]  uid          : UID of the device
 *                             if not provided Select mode will be used 
 * \param[in]  blockNum     : Number of the block to be locked (16 bits)
 *  
 * \return ERR_WRONG_STATE  : RFAL not initialized or incorrect mode
 * \return ERR_PARAM        : Invalid parameters
 * \return ERR_IO           : Generic internal error 
 * \return ERR_CRC          : CRC error detected
 * \return ERR_FRAMING      : Framing error detected
 * \return ERR_PROTO        : Protocol error detected
 * \return ERR_TIMEOUT      : Timeout error
 * \return ERR_NONE         : No error
 *****************************************************************************
 */
ReturnCode rfalNfcvPollerExtendedLockSingleBlock( uint8_t flags, const uint8_t* uid, uint16_t blockNum );

/*! 
 *****************************************************************************
 * \brief  NFC-V Poller Extended Read Single Block
 *  
 * Reads a Single Block from a device (VICC)  
 *
 * \param[in]  flags        : Flags to be used: Sub-carrier; Data_rate; Option
 *                            for NFC-Forum use: RFAL_NFCV_REQ_FLAG_DEFAULT
 * \param[in]  uid          : UID of the device to be put to be read
 *                             if not provided Select mode will be used 
 * \param[in]  blockNum     : Number of the block to read (16 bits)
 * \param[out] rxBuf        : buffer to store response (also with RES_FLAGS)
 * \param[in]  rxBufLen     : length of rxBuf
 * \param[out] rcvLen       : number of bytes received
 *  
 * \return ERR_WRONG_STATE  : RFAL not initialized or incorrect mode
 * \return ERR_PARAM        : Invalid parameters
 * \return ERR_IO           : Generic internal error 
 * \return ERR_CRC          : CRC error detected
 * \return ERR_FRAMING      : Framing error detected
 * \return ERR_PROTO        : Protocol error detected
 * \return ERR_TIMEOUT      : Timeout error
 * \return ERR_NONE         : No error
 *****************************************************************************
 */
ReturnCode rfalNfcvPollerExtendedReadSingleBlock( uint8_t flags, const uint8_t* uid, uint16_t blockNum, uint8_t* rxBuf, uint16_t rxBufLen, uint16_t *rcvLen );

/*! 
 *****************************************************************************
 * \brief  NFC-V Poller Extendec Write Single Block
 *  
 * Writes a Single Block from a device (VICC)
 *
 * \param[in]  flags        : Flags to be used: Sub-carrier; Data_rate; Option
 *                            for NFC-Forum use: RFAL_NFCV_REQ_FLAG_DEFAULT
 * \param[in]  uid          : UID of the device
 *                             if not provided Select mode will be used 
 * \param[in]  blockNum     : Number of the block to write (16 bits)
 * \param[in]  wrData       : data to be written on the given block
 * \param[in]  blockLen     : number of bytes of a block
 *  
 * \return ERR_WRONG_STATE  : RFAL not initialized or incorrect mode
 * \return ERR_PARAM        : Invalid parameters
 * \return ERR_IO           : Generic internal error 
 * \return ERR_CRC          : CRC error detected
 * \return ERR_FRAMING      : Framing error detected
 * \return ERR_PROTO        : Protocol error detected
 * \return ERR_TIMEOUT      : Timeout error
 * \return ERR_NONE         : No error
 *****************************************************************************
 */
ReturnCode rfalNfcvPollerExtendedWriteSingleBlock( uint8_t flags, const uint8_t* uid, uint16_t blockNum, const uint8_t* wrData, uint8_t blockLen );

/*! 
 *****************************************************************************
 * \brief  NFC-V Poller Read Multiple Blocks
 *  
 * Reads Multiple Blocks from a device (VICC)  
 *
 * \param[in]  flags          : Flags to be used: Sub-carrier; Data_rate; Option
 *                              for NFC-Forum use: RFAL_NFCV_REQ_FLAG_DEFAULT
 * \param[in]  uid            : UID of the device to be put to be read
 *                               if not provided Select mode will be used 
 * \param[in]  firstBlockNum  : first block to be read (16 bits)
 * \param[in]  numOfBlocks    : number of consecutive blocks to read (16 bits)
 * \param[out] rxBuf          : buffer to store response (also with RES_FLAGS)
 * \param[in]  rxBufLen       : length of rxBuf
 * \param[out] rcvLen         : number of bytes received
 *  
 * \return ERR_WRONG_STATE    : RFAL not initialized or incorrect mode
 * \return ERR_PARAM          : Invalid parameters
 * \return ERR_IO             : Generic internal error 
 * \return ERR_CRC            : CRC error detected
 * \return ERR_FRAMING        : Framing error detected
 * \return ERR_PROTO          : Protocol error detected
 * \return ERR_TIMEOUT        : Timeout error
 * \return ERR_NONE           : No error
 *****************************************************************************
 */
ReturnCode rfalNfcvPollerExtendedReadMultipleBlocks( uint8_t flags, const uint8_t* uid, uint16_t firstBlockNum, uint16_t numOfBlocks, uint8_t* rxBuf, uint16_t rxBufLen, uint16_t *rcvLen );

#endif /* RFAL_NFCV_H */

/**
  * @}
  *
  * @}
  *
  * @}
  */

