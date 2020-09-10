
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
/*! \file
 *
 *  \author
 *
 *  \brief Platform header file. Defining platform independent functionality.
 *
 */


/*
 *      PROJECT:
 *      $Revision: $
 *      LANGUAGE:  ISO C99
 */

/*! \file platform.h
 *
 *  \author Gustavo Patricio
 *
 *  \brief Platform specific definition layer
 *
 *  This should contain all platform and hardware specifics such as
 *  GPIO assignment, system resources, locks, IRQs, etc
 *
 *  Each distinct platform/system/board must provide this definitions
 *  for all SW layers to use
 *
 */

#ifndef PLATFORM_H
#define PLATFORM_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

//#include "stdint.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "limits.h"
//#include "freertos/semphr.h"

//#include "stm8s.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"



#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif
#include <timer.h>
#include "tick.h"
#include "spi_util.h"
//#include "log.h"
//#include "main.h"


//#define true TRUE
//#define false FALSE

/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/
#define ST25R391X_SS_PIN            15          /*!< GPIO pin used for ST25R3911 SPI SS                */
//#define ST25R391X_SS_PORT           GPIOE               /*!< GPIO port used for ST25R3911 SPI SS port          */

#define ST25R391X_INT_PIN           GPIO_NUM_2           /*!< GPIO pin used for ST25R3911 External Interrupt    */
//#define ST25R391X_INT_PORT          GPIOB               /*!< GPIO port used for ST25R3911 External Interrupt   */

#define BUTTON_PIN           GPIO_NUM_3           /*!< GPIO pin used for button input    */
#define LED_PIN           GPIO_NUM_4           /*!< GPIO pin used for LED output    */


#define GPIO_INPUT_PIN_SEL  ((1ULL<<ST25R391X_INT_PIN))
#define GPIO_BUTTON_PIN_SEL  ((1ULL<<BUTTON_PIN))


#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   15

#define CAMERA_IMAGE_WIDTH      640
#define CAMERA_IMAGE_HEIGHT     480
#define CAMERA_PIXELS           (CAMERA_IMAGE_WIDTH * CAMERA_IMAGE_HEIGHT)

SemaphoreHandle_t IRQ_semaphore;
SemaphoreHandle_t COM_semaphore;
//static BaseType_t xHigherPriorityTaskWoken1;
TaskHandle_t callbackTaskHandle;

/*
******************************************************************************
* GLOBAL MACROS
******************************************************************************
*/

#define platformProtectST25R391xComm()               xSemaphoreTake(COM_semaphore, portMAX_DELAY) //gpio_intr_disable(ST25R391X_INT_PIN ) noInterrupts() //disableInterrupts() ST25R3911_INT_PORT->CR2 &= (uint8_t)(~(ST25R3911_INT_PIN))      < Disables ST353911 MCU's external interrupt
#define platformUnprotectST25R391xComm()             xSemaphoreGive(COM_semaphore) //gpio_intr_enable(ST25R391X_INT_PIN ) interrupts()  ST25R3911_INT_PORT->CR2 |= (uint8_t)ST25R3911_INT_PIN           < Enables ST353911 MCU's external interrupt


#define platformIrqST25R3911SetCallback( cb )              // hook isr handler for specific gpio pin
#define platformIrqST25R3911PinInitialize()

#define platformProtectST25R391xIrqStatus()           xSemaphoreTake(IRQ_semaphore, portMAX_DELAY)                /*!< Protect unique access to IRQ status var - IRQ disable on single thread environment (MCU) ; Mutex lock on a multi thread environment */
#define platformUnprotectST25R391xIrqStatus()         xSemaphoreGive(IRQ_semaphore)              /*!< Unprotect the IRQ status var - IRQ enable on a single thread environment (MCU) ; Mutex unlock on a multi thread environment         */


#define platformLedsInitialize()                      //printf("LEDS initialise")                                              /*!< Initializes the pins used as LEDs to outputs*/

//#define platformLedOff( port, pin )                   platformGpioClear(port, pin)                  /*!< Turns the given LED Off                     */
//#define platformLedOn( port, pin )                    platformGpioSet(port, pin)                    /*!< Turns the given LED On                      */
//#define platformLedToogle( port, pin )                platformGpioToogle(port, pin)                 /*!< Toogle the given LED                        */

#define platformGpioSet( port, pin )                  gpio_set_level(pin, 1)                     /*!< Turns the given GPIO High                   */
#define platformGpioClear( port, pin )                gpio_set_level(pin, 0)                      /*!< Turns the given GPIO Low                    */
#define platformGpioToogle( port, pin )               gpio_set_level(pin, !gpio_get_level(pin)(pin))                  /*!< Toogles the given GPIO                      */
#define platformGpioIsHigh( port, pin )               (gpio_get_level(pin)==1) //((((uint32_t)((port)->IDR) & (uint32_t)(pin)) != 0U)) /*(GPIO_ReadInputPin(port, pin) == SET)         < Checks if the given LED is High             */
#define platformGpioIsLow( port, pin )                (!platformGpioIsHigh(port, pin))              /*!< Checks if the given LED is Low              */

#define platformTimerCreate( t )                      timerCalculateTimer(t)                        /*!< Create a timer with the given time (ms)     */
#define platformTimerIsExpired( timer )               timerIsExpired(timer)                         /*!< Checks if the given timer is expired        */
#define platformDelay( t )                            vTaskDelay(t / portTICK_PERIOD_MS)                                /*!< Performs a delay for the given time (ms)    */

#define platformGetSysTick()                          ((uint32_t)esp_timer_get_time())/1000                                     /*!< Get System Tick ( 1 tick = 1 ms)            */

#define platformSpiSelect()                           //gpio_set_level( PIN_NUM_CS, 0 ) /*!< SPI SS\CS: Chip|Slave Select                */
#define platformSpiDeselect()                         //gpio_set_level( PIN_NUM_CS, 1 )   /*!< SPI SS\CS: Chip|Slave Deselect              */
#define platformSpiTxRx( txBuf, rxBuf, len )          spiTxRx(txBuf, rxBuf, len)                    /*!< SPI transceive                              */


#define platformI2CTx( txBuf, len )                                                                 /*!< I2C Transmit                                */
#define platformI2CRx( txBuf, len )                                                                 /*!< I2C Receive                                 */
#define platformI2CStart()                                                                          /*!< I2C Start condition                         */
#define platformI2CStop()                                                                           /*!< I2C Stop condition                          */
#define platformI2CRepeatStart()                                                                    /*!< I2C Repeat Start                            */
#define platformI2CSlaveAddrWR(add)                                                                 /*!< I2C Slave address for Write operation       */
#define platformI2CSlaveAddrRD(add)                                                                 /*!< I2C Slave address for Read operation        */

#define platformLog(...)                             printf(__VA_ARGS__)                          /*!< Log  method                                 */

#define ST25R391X_COM_SINGLETXRX //Allows use of CS by performing SPI transactions in single command

/*
******************************************************************************
* RFAL FEATURES CONFIGURATION
******************************************************************************
*/

#define RFAL_FEATURE_NFCA                      true       /*!< Enable/Disable RFAL support for NFC-A (ISO14443A)                         */
#define RFAL_FEATURE_NFCB                      true       /*!< Enable/Disable RFAL support for NFC-B (ISO14443B)                         */
#define RFAL_FEATURE_NFCF                      true       /*!< Enable/Disable RFAL support for NFC-F (FeliCa)                            */
#define RFAL_FEATURE_NFCV                      true       /*!< Enable/Disable RFAL support for NFC-V (ISO15693)                          */
#define RFAL_FEATURE_T1T                       true       /*!< Enable/Disable RFAL support for T1T (Topaz)                               */
#define RFAL_FEATURE_T2T                       false       /*!< Enable/Disable RFAL support for T2T                                      */
#define RFAL_FEATURE_ST25TB                    true       /*!< Enable/Disable RFAL support for ST25TB                                    */
#define RFAL_FEATURE_DYNAMIC_ANALOG_CONFIG     false      /*!< Enable/Disable Analog Configs to be dynamically updated (RAM)             */
#define RFAL_FEATURE_DPO                       false      /*!< Enable/Disable RFAL dynamic power support                                 */
#define RFAL_FEATURE_ISO_DEP                   true       /*!< Enable/Disable RFAL support for ISO-DEP (ISO14443-4)                      */
#define RFAL_FEATURE_ISO_DEP_POLL              true       /*!< Enable/Disable RFAL support for Poller mode (PCD) ISO-DEP (ISO14443-4)    */
#define RFAL_FEATURE_ISO_DEP_LISTEN            false      /*!< Enable/Disable RFAL support for Listen mode (PICC) ISO-DEP (ISO14443-4)   */
#define RFAL_FEATURE_NFC_DEP                   true       /*!< Enable/Disable RFAL support for NFC-DEP (NFCIP1/P2P)                      */


#define RFAL_FEATURE_ISO_DEP_IBLOCK_MAX_LEN    256        /*!< ISO-DEP I-Block max length. Please use values as defined by rfalIsoDepFSx */
#define RFAL_FEATURE_ISO_DEP_APDU_MAX_LEN      1024       /*!< ISO-DEP APDU max length. Please use multiples of I-Block max length       */

#ifdef __cplusplus
}
#endif
#endif /* PLATFORM_H */
