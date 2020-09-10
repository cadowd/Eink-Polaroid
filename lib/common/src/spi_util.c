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

/*! \file spi.c
 *
 *  \brief SPI driver
 *
 *  \author Gustavo Patricio
 *
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "spi_util.h"


/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/

/*******************************************************************************/
void spiInitialize( void )
{
	//SPI_DeInit();
  //SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_4, SPI_MODE_MASTER, SPI_CLOCKPOLARITY_LOW, SPI_CLOCKPHASE_2EDGE, SPI_DATADIRECTION_2LINES_FULLDUPLEX, SPI_NSS_SOFT, 0x07);
	esp_err_t ret;
	spi_bus_config_t buscfg={};
	spi_device_interface_config_t devcfg = {};

		buscfg.mosi_io_num = PIN_NUM_MOSI;
		buscfg.miso_io_num = PIN_NUM_MISO;
		buscfg.sclk_io_num = PIN_NUM_CLK;
		buscfg.quadwp_io_num = -1;
		buscfg.quadhd_io_num = -1;
		buscfg.max_transfer_sz = SPI_MAX_DMA_LEN;

		devcfg.command_bits = 0;
		devcfg.address_bits = 0;
		devcfg.dummy_bits = 0;
		devcfg.mode = 1;
		devcfg.duty_cycle_pos = 128;
		devcfg.cs_ena_pretrans = 0;
		devcfg.cs_ena_posttrans = 0; // Keep the CS low 3 cycles after transaction, to stop the master from missing the last bit when CS has less propagation delay than CLK
		devcfg.clock_speed_hz = 10*1000*1000;
		//.input_delay_ns = 20,
		devcfg.spics_io_num = PIN_NUM_CS; //Perform CS manually with platform platformSpiSelect function
		devcfg.flags = 0;
		devcfg.queue_size = 1;
		devcfg.pre_cb = NULL;
		devcfg.post_cb = NULL;

	spi_bus_free(NFC_HOST);
	//Initialize the SPI bus
	ret=spi_bus_initialize(NFC_HOST, &buscfg, DMA_CHAN);
	ESP_ERROR_CHECK(ret);
	//Attach the NFC board to the SPI bus
	ret=spi_bus_add_device(NFC_HOST, &devcfg, &NFC_ESP32_SPI);
	ESP_ERROR_CHECK(ret);

	//SPI.begin(PIN_NUM_CLK, PIN_NUM_MISO, PIN_NUM_MOSI, ST25R391X_SS_PIN); // sck, miso, mosi, ss (ss can be any GPIO)

  //SPI_Cmd(ENABLE); //  * @brief  Enables or disables the SPI peripheral.

	/* Configure CS pin */
	//pinMode( ST25R391X_SS_PIN,  OUTPUT );
	//digitalWrite( ST25R391X_SS_PIN, HIGH );

//Not necessary to manually set SS pin high as CS pin is defined for SPI?
	// gpio_pad_select_gpio(PIN_NUM_CS);
  //   /* Set the GPIO as a push/pull output */
  // gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
	// gpio_set_level(PIN_NUM_CS, 1);

}


/*******************************************************************************/
void spiTxRx( uint8_t* txData, uint8_t* rxData, uint16_t len )
{
	//uint16_t rxByte;
	//uint8_t txByte;
	//SPISettings settingsNFC(2000000, MSBFIRST, SPI_MODE1);

	// SPI.beginTransaction(settingsNFC);
  // //Loop over sending and receiving bytes
  // while( len-- )
  // {
  //     txByte = 0;
  //     //while( SPI_GetFlagStatus(SPI_FLAG_TXE) != SET);
	//
  //     if( 0 != txData )
  //     {
  //         txByte = *txData;
  //         txData++;
  //     }
  //     SPI.transfer(txByte);
	//
	// 		//while( SPI_GetFlagStatus(SPI_FLAG_BSY) == SET );
	//
	// 		rxByte = SPI.transfer(0);
	//
	//
  //     if( 0 != rxData )
  //     {
  //         *rxData = (uint8_t)rxByte;
  //         rxData++;
  //     }
	//
	// 		//while( SPI_GetFlagStatus(SPI_FLAG_BSY) == SET );
	// 		SPI.endTransaction();
	//
  // }
	esp_err_t ret;
	//spi_device_handle_t spi;
	spi_transaction_t t;
	if (len==0) return;             //no need to send anything
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length=len*8;                 //Len is in bytes, transaction length is in bits.
	if (txData)
		t.tx_buffer=txData;               //Data
	if (rxData)
		t.rx_buffer=rxData;
	//vTaskSuspendAll(); //Suspend other tasks to prevent dual access to SPI
	ret=spi_device_polling_transmit(NFC_ESP32_SPI, &t);  //Transmit!
	//ret=spi_device_transmit(NFC_ESP32_SPI, &t);  //Transmit!
	//xTaskResumeAll();
	//platformDelay(1);
	assert(ret==ESP_OK);            //Should have had no issues.
	// for (int i = 0; i < len; i++) {
	//printf("RX: %d\n", rxData[0]);
 // }
  return;
}
