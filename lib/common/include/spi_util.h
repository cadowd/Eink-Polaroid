/******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under ST MYLIBERTY SOFTWARE LICENSE AGREEMENT (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/myliberty
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
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file spi.h
 *
 *  \brief SPI driver
 *
 *  \author Gustavo Patricio
 *
 *
 */


#ifndef __SPI_UTIL_H
#define __SPI_UTIL_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"
#include "driver/spi_master.h"
//#include <Arduino.h>
//#include <SPI.h>


spi_device_handle_t NFC_ESP32_SPI;
#define DMA_CHAN    SPI_DMA_CH2 //2
#define NFC_HOST    SPI2_HOST //VSPI_HOST


 /*!
 *****************************************************************************
 * \brief  SPI initialize
 *
 * Initializes the SPI peripheral and Chip select pin
 *
 *****************************************************************************
 */
void spiInitialize( void );


 /*!
 *****************************************************************************
 * \brief  SPI Transceive
 *
 *
 * \param[in]  txData : buffer containing the bytes to be transmitted
 * \param[in]  rxData : buffer to place received bytes
 * \param[in]  len    : number of clocks to be performed on the SPI bus
 *
 *****************************************************************************
 */
void spiTxRx( uint8_t* txData, uint8_t* rxData, uint16_t len );



#endif /* __SPI_H */
