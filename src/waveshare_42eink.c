/**
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/*! \file
 *
 *  \author
 *
 *  \brief Demo application
 *
 *  This demo shows how to poll for several types of NFC cards/devices and how
 *  to exchange data with these devices, using the RFAL library.
 *
 *  This demo does not fully implement the activities according to the standards,
 *  it performs the required to communicate with a card/device and retrieve
 *  its UID. Also blocking methods are used for data exchange which may lead to
 *  long periods of blocking CPU/MCU.
 *  For standard compliant example please refer to the Examples provided
 *  with the RFAL library.
 *
 */

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include "waveshare_42eink.h"
#include "utils.h"
#include "rfal_rf.h"

/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/

/* Definition of possible states the demo state machine could have */

#define DEMO_BUF_LEN                  255
#define DEMO_NFCV_BLOCK_LEN           4


/*
 ******************************************************************************
 * LOCAL VARIABLES
 ******************************************************************************
 */
 // array declaration
//static int img_size=400*300;
//uint8_t iArr[120000];


 uint8_t  pic_send_array[15000]; //img_width*img_height/8, each byte of pic array is binary data for 8 pixels


 void epaper_init(uint8_t * ImageBuffer)
 {

   ReturnCode            err;
   uint8_t               rxBuf[20];
   uint16_t              rxBufLen=20; //Max RX length
   uint8_t               txBuf[2];
   uint16_t              txBufLen=2;
   uint16_t              actLen[20];
   uint32_t              flags=RFAL_TXRX_FLAGS_DEFAULT;
   uint32_t              fwt=rfalConvMsTo1fc(400); //Frame waiting time 2157+2048?

   //uint8_t  bArr_packet[103];

   // int k;
   // // initializing array elements
   // for (k = 0; k < img_size; k++){
   //    iArr[k] = 0;
   // }

   //NFC driver board power on
   txBuf[0]=0xCD;
   txBuf[1]=0x0D;
   rxBuf[0]=1; rxBuf[1]=1; //set response to false
   err=rfalTransceiveBlockingTxRx( txBuf, txBufLen, rxBuf, rxBufLen, actLen, flags, fwt );
   if (err != ERR_NONE)
   {
     printf("Transcieve error\n");
     return;
   }
   else if ((rxBuf[0] != 0x00) || (rxBuf[1] != 0x00))
   {
     printf("NFC card error\n");
     return;
   }
   printf("Power on\n");

   //NFCTag reset and choose e-Paper
   uint8_t               txBuf3[3];
   txBufLen=3;
   txBuf3[0]=0xCD;
   txBuf3[1]=0x00;
   txBuf3[2]=0x0A;
   rxBuf[0]=1; rxBuf[1]=1; //set response to false
   err=rfalTransceiveBlockingTxRx( txBuf3, txBufLen, rxBuf, rxBufLen, actLen, flags, fwt );
   if (err != ERR_NONE)
   {
     printf("Transcieve error\n");
     return;
   }
   else if ((rxBuf[0] != 0x00) || (rxBuf[1] != 0x00))
   {
     printf("NFC card error\n");
     return;
   }
   platformDelay(50);

   printf("Reseted\n");

   //NFCTag configure command 1
   txBuf[0]=0xCD;
   txBuf[1]=0x01;
   txBufLen=2;
   rxBuf[0]=1; rxBuf[1]=1; //set response to false
   err=rfalTransceiveBlockingTxRx( txBuf, txBufLen, rxBuf, rxBufLen, actLen, flags, fwt );
   if (err != ERR_NONE)
   {
     printf("Transcieve error\n");
     return;
   }
   else if ((rxBuf[0] != 0x00) || (rxBuf[1] != 0x00))
   {
     printf("NFC card error\n");
     return;
   }
   platformDelay(10);
   printf("Configured\n");
   //NFCTag configure command 2
   txBuf[0]=0xCD;
   txBuf[1]=0x02;
   rxBuf[0]=1; rxBuf[1]=1; //set response to false
   err=rfalTransceiveBlockingTxRx(txBuf, txBufLen, rxBuf, rxBufLen, actLen, flags, fwt );
   if (err != ERR_NONE)
   {
     printf("Transcieve error\n");
     return;
   }
   else if ((rxBuf[0] != 0x00) || (rxBuf[1] != 0x00))
   {
     printf("NFC card error\n");
     return;
   }
   platformDelay(10);
   //NFCTag power on
   txBuf[0]=0xCD;
   txBuf[1]=0x03;
   rxBuf[0]=1; rxBuf[1]=1; //set response to false
   err=rfalTransceiveBlockingTxRx( txBuf, txBufLen, rxBuf, rxBufLen, actLen, flags, fwt );
   if (err != ERR_NONE)
   {
     printf("Transcieve error\n");
     return;
   }
   else if ((rxBuf[0] != 0x00) || (rxBuf[1] != 0x00))
   {
     printf("NFC card error\n");
     return;
   }
   platformDelay(10);
   //NFCTag configure command 3
   txBuf[0]=0xCD;
   txBuf[1]=0x05;
   rxBuf[0]=1; rxBuf[1]=1; //set response to false
   err=rfalTransceiveBlockingTxRx( txBuf, txBufLen, rxBuf, rxBufLen, actLen, flags, fwt );
   if (err != ERR_NONE)
   {
     printf("Transcieve error\n");
     return;
   }
   else if ((rxBuf[0] != 0x00) || (rxBuf[1] != 0x00))
   {
     printf("NFC card error\n");
     return;
   }
   platformDelay(10);
   //NFCTag configure command 4
   txBuf[0]=0xCD;
   txBuf[1]=0x06;
   rxBuf[0]=1; rxBuf[1]=1; //set response to false
   err=rfalTransceiveBlockingTxRx( txBuf, txBufLen, rxBuf, rxBufLen, actLen, flags, fwt );
   if (err != ERR_NONE)
   {
     printf("Transcieve error\n");
     return;
   }
   else if ((rxBuf[0] != 0x00) || (rxBuf[1] != 0x00))
   {
     printf("NFC card error\n");
     return;
   }
   platformDelay(10);

    //Ready to write image, convert BW to binary buffer
    for (int iy = 0; iy < 300; iy++) { //for each image row
      for (int ix = 0; ix < 50; ix++) { //for each image column
        uint8_t  image_byte = 0;
        for (int ibyte = 0; ibyte < 8; ibyte++) { //for each bit in array byte
          image_byte = (image_byte << 1); //Move all bits left by 1
          if ((uint8_t)(ImageBuffer[(ix * 8) + ibyte + (iy * 400)] & 255) > 128) {  //& 255 is to convert to unsigned int. If colour value of int >128 then it is considered black
            image_byte |= 0x01; //Add 1 to end of bit
            // if (ix<50){
            //   printf("x");
            // }
          }
          // else{
          //   // if (ix<50){
          //   //   printf("o");
          //   // }
          // }
        }
        pic_send_array[(iy * 50) + ix] = image_byte;
      }
      // printf("\n");
    }

   //NFCTag ready to send data
   txBuf3[0]=0xCD;
   txBuf3[1]=0x07;
   txBuf3[2]=0x00;
   rxBuf[0]=1; rxBuf[1]=1; //set response to false
   err=rfalTransceiveBlockingTxRx( txBuf3, 3, rxBuf, rxBufLen, actLen, flags, fwt );
   if (err != ERR_NONE)
   {
     printf("Transcieve error\n");
     return;
   }
   else if ((rxBuf[0] != 0x00) || (rxBuf[1] != 0x00))
   {
     printf("NFC card error\n");
     return;
   }

   printf("Starting transfer\n");
   //Transfer image
   int i = 0;
   uint8_t bArr_packet[103];
   memset(&bArr_packet, 0, sizeof(bArr_packet));
   bArr_packet[0]=0xCD;
   bArr_packet[1]=8;
   bArr_packet[2]=100;


   while (i < 150) {

      int i2 = i * 100;
      memcpy( &bArr_packet[3], &pic_send_array[i2], 100*sizeof(uint8_t));

      // printf("%% %d\n", i);
      //printf("%d\n", sizeof(bArr_packet) / sizeof(bArr_packet[0]));
      rxBuf[0]=1; rxBuf[1]=1; //set response to false
      err=rfalTransceiveBlockingTxRx( bArr_packet, 103, rxBuf, rxBufLen, actLen, flags, fwt );
      if (err != ERR_NONE)
      {
        printf("Transcieve error\n");
        return;
      }
      else if ((rxBuf[0] != 0x00) || (rxBuf[1] != 0x00))
      {
        printf("Image transfer failed\n");
        return;
      }
      i++;
    }

    //NFCTag refresh e-Paper
    txBuf[0]=0xCD;
    txBuf[1]=0x09;
    err=rfalTransceiveBlockingTxRx( txBuf, txBufLen, rxBuf, rxBufLen, actLen, flags, fwt );
    if (err != ERR_NONE)
    {
      printf("Transcieve error\n");
      return;
    }
    else if ((rxBuf[0] != 0x00) || (rxBuf[1] != 0x00))
    {
      printf("NFC card error\n");
      return;
    }
    platformDelay(200);
    printf("Waiting for update\n");

    //Wait for NFCTag update
    txBuf[0]=0xCD;
    txBuf[1]=0x0A;
    int i3=0;
    while (true) {
        i3++;
        err=rfalTransceiveBlockingTxRx(txBuf, txBufLen, rxBuf, rxBufLen, actLen, flags, fwt );
        if (err != ERR_NONE)
        {
          printf("Transcieve error\n");
          return;
        }
        if ((rxBuf[0] == 0xFF) && (rxBuf[1] == 0x00))
        {
          printf("Transfer success\n");
          break;
        }
        if(i3>200){
            printf("Wait timeout");
            return;
        }
        platformDelay(25);
    }

    //platformDelay(200);
    //Shutdown nfctag
    txBuf[0]=0xCD;
    txBuf[1]=0x04;
    err=rfalTransceiveBlockingTxRx( txBuf, txBufLen, rxBuf, rxBufLen, actLen, flags, fwt );
    if (err != ERR_NONE)
    {
      printf("Transcieve error\n");
      return;
    }
    else if ((rxBuf[0] != 0x00) || (rxBuf[1] != 0x00))
    {
      printf("NFC card error\n");
      return;
    }

   printf("Success\n");
 }
