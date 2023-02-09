#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_camera.h"

#include <rfal_rf.h>
#include <rfal_analogConfig.h>
#include <rfal_nfca.h>
//#include "rfal_st25tb.h"
#include <spi_util.h>
#include "st25r3911_interrupt.h"

#include <platform.h>

#include <waveshare_42eink.h>
#include "image_funcs.h"

static const char *TAG = "nfc_camera";

int screen_width = 400;
int screen_height = 300;

#define LOG_MAX_HEX_STR_LENGTH  64                                  /*<! Max length of a hex string             */
#define LOG_MAX_HEX_STR_NUM     2                                   /*<! Max number of simultaneous hex steings */

static char gHexStr[LOG_MAX_HEX_STR_NUM][LOG_MAX_HEX_STR_LENGTH];
static uint8_t gHexStrIdx;

uint8_t* ImageBuffer;

//#define BUTTONPIN 14
//#define DEBOUNCETIMEOUT 50

volatile int numberOfButtonInterrupts = 0;
volatile bool lastState;
volatile uint32_t debounceTime = 0;
volatile uint32_t lastDebounceTime = 0;

SemaphoreHandle_t buttonBinarySemaphore;

TaskHandle_t flashLED_handle = NULL;
TaskHandle_t flashLEDSlow_handle = NULL;

#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    21
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27

#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      19
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM       5
#define Y2_GPIO_NUM       4
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    .xclk_freq_hz = 20000000,//EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_GRAYSCALE, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_VGA,   //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .grab_mode = CAMERA_GRAB_LATEST //Get latest image
};

static void *_malloc(size_t size)
{
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

char* hex2Str( uint8_t *data, uint16_t dataLen )
{
	const char* hex = "0123456789ABCDEF";
	unsigned char* pin;
	char* pout;
	char* ret;
	uint8_t i;

	pin  = data;
	pout = gHexStr[gHexStrIdx];
	ret  = pout;

	if(dataLen == 0)
	{
		*pout = 0;
	}
	else
	{
		if( dataLen > LOG_MAX_HEX_STR_LENGTH )
	  {
			dataLen = LOG_MAX_HEX_STR_LENGTH;
		}

		for(i=0; i < (dataLen - 1); i++)
		{
				*pout++ = hex[(*pin>>4)&0x0F];
				*pout++ = hex[(*pin++)&0x0F];
		}
		*pout++ = hex[(*pin>>4)&0x0F];
		*pout++ = hex[(*pin)&0x0F];
		*pout = 0;
	}

	gHexStrIdx++;
	gHexStrIdx %= LOG_MAX_HEX_STR_NUM;

	return ret;
}


static void IRAM_ATTR ISR_routine( void* arg )
  {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      // Notify the thread so it will wake up when the ISR is complete
      vTaskNotifyGiveFromISR(callbackTaskHandle,
                           &xHigherPriorityTaskWoken);
      if(xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
  }

  // Interrupt Service Routine - Keep it short!
static void IRAM_ATTR buttonISR(void* arg) {
    /* */
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  //printf("button\n");
  /* un-block the interrupt processing task now */
  gpio_intr_disable(BUTTON_PIN);
  xSemaphoreGiveFromISR( buttonBinarySemaphore, &xHigherPriorityTaskWoken );

    if(xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
  }


void ISR_callback_ESP32( void *pvParameters )
{
  static uint32_t thread_notification;

  while(1)
  {
      /* Sleep until we are notified of a state change by an
      * interrupt handler. Note the first parameter is pdTRUE,
      * which has the effect of clearing the task's notification
      * value back to 0, making the notification value act like
      * a binary (rather than a counting) semaphore.  */
      thread_notification=ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

      if(thread_notification)
      {
          st25r3911Isr();
      }
  }

}

static esp_err_t init_camera()
{
  //initialize the camera
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Camera Init Failed");
    return err;
  }

  sensor_t* Sensor = esp_camera_sensor_get();
  Sensor->set_contrast(Sensor, 1);       // -2 to 2
  Sensor->set_special_effect(Sensor, 2); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  // Sensor->set_brightness(Sensor, 0);
  // Sensor->set_exposure_ctrl(Sensor, 1);
  // Sensor->set_aec2(Sensor, 0);
  // Sensor->set_ae_level(Sensor, 0);
  // Sensor->set_aec_value(Sensor, 300);
  Sensor->set_vflip(Sensor, 1);
  Sensor->set_hmirror(Sensor, 1);

  return ESP_OK;
}

static void flash_LED(void *arg)
{
  int flash_interval=60; //ms flash interval
  while (true) {
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(flash_interval / portTICK_PERIOD_MS);
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(flash_interval / portTICK_PERIOD_MS);
  }
}

static void flash_LED_slow(void *arg)
{
  int flash_interval_slow=500; //ms flash interval
  while (true) {
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(flash_interval_slow / portTICK_PERIOD_MS);
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(flash_interval_slow / portTICK_PERIOD_MS);
  }
}

void app_main()
{
  //init_sdcard();
  //Initialise IRQ mutex
  IRQ_semaphore=xSemaphoreCreateBinary();
  //Set it free
  xSemaphoreGive(IRQ_semaphore);

  //Initialise COM mutex
  COM_semaphore=xSemaphoreCreateBinary();
  //Set it free
  xSemaphoreGive(COM_semaphore);

  //Initialise button notification semaphore
  buttonBinarySemaphore=xSemaphoreCreateBinary();
  printf("Semaphore created \n");
  if (buttonBinarySemaphore != NULL)
  {
  xSemaphoreTake(buttonBinarySemaphore, 10);
  }

  /* Disable all interupts for peripheral config */
  platformProtectST25R391xComm();

  //Start callback task running with priority 5 on core 1
  xTaskCreatePinnedToCore(ISR_callback_ESP32, "ISR_callback_ESP32", 2048, NULL, 5, &callbackTaskHandle, 1);

  //Configure GPIO, first configure ST25R391x interrupt pin
  gpio_config_t io_conf;
  //interrupt of rising edge
  io_conf.intr_type = GPIO_INTR_POSEDGE ; //GPIO_INTR_POSEDGE
  //bit mask of the pins, use GPIO4/5 here
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL ;
  //set as input mode
  io_conf.mode = GPIO_MODE_INPUT;
  //enable pull-up mode
  io_conf.pull_up_en  = GPIO_PULLUP_DISABLE;
  //disable pull-down mode
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_config(&io_conf);
  //
  //Configure button interrupt pin
  gpio_pad_select_gpio(BUTTON_PIN);
  gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
  gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_POSEDGE);
  gpio_set_pull_mode(BUTTON_PIN, GPIO_FLOATING);
  gpio_intr_enable(BUTTON_PIN);

  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

  printf("Installing ISR gpio_install_isr_service \n");
  // //install gpio isr service
  esp_err_t err;
  err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM); //default is 0
  if (err != ESP_OK) {
  printf("handler add failed with error 0x%x \r\n", err);
  }
  //Add interrupts
  gpio_isr_handler_add(ST25R391X_INT_PIN, ISR_routine, (void*) ST25R391X_INT_PIN);
  gpio_isr_handler_add(BUTTON_PIN, buttonISR, (void*) BUTTON_PIN);


  /* Initialize SPI module */
  printf("Initialise SPI \n");
  spiInitialize();


  /* Enable all interupts */
  printf("Enabling interupts \n");
  platformUnprotectST25R391xComm();

  init_camera();

  /* Initialize RFAL */
  printf("Initialising RFAL Analogue config \n");
  rfalAnalogConfigInitialize();

  printf("Config initialised  \n");
  //vTaskDelay(2000 / portTICK_PERIOD_MS);
  //printf("Wait over now RFAL initialise  \n");

  int bufferSize = FRAME_WIDTH*FRAME_HEIGHT;
  ImageBuffer=_malloc(bufferSize);


  if( rfalInitialize() != ERR_NONE )
  {
    /* Initialization failed - SPI or communication problem most likely */
    while(1)
    {
      printf("Initialisation failed  \n");
      //int error_no=rfalInitialize();
      //printf("%d\n", error_no);
      // platformLedToogle(PLATFORM_LED_A_PORT, PLATFORM_LED_A_PIN);
      // platformLedToogle(PLATFORM_LED_B_PORT, PLATFORM_LED_B_PIN);
      // platformLedToogle(PLATFORM_LED_F_PORT, PLATFORM_LED_F_PIN);
      // platformLedToogle(PLATFORM_LED_V_PORT, PLATFORM_LED_V_PIN);
      // platformLedToogle(PLATFORM_LED_AP2P_PORT, PLATFORM_LED_AP2P_PIN);
      // platformLedToogle(PLATFORM_LED_FIELD_PORT, PLATFORM_LED_FIELD_PIN);
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
  }

  /* Infinite loop */
  //while(1)
  printf("Initialisation success  \n");

  //LED flashing task
  xTaskCreate(
    flash_LED,         // Function that should be called
    "Flash LED",      // Name of the task (for debugging)
    1000,          // Stack size (bytes)
    NULL,          // Parameter to pass
    1,             // Task priority
    &flashLED_handle  // Task handle
  );
  vTaskSuspend(flashLED_handle);
    //LED flashing task
  xTaskCreate(
    flash_LED_slow,         // Function that should be called
    "Flash LED slow",      // Name of the task (for debugging)
    1000,          // Stack size (bytes)
    NULL,          // Parameter to pass
    1,             // Task priority
    &flashLEDSlow_handle  // Task handle
  );
  



  /* Initialise RFAL */
  rfalWorker();
  rfalFieldOff();
  // rfalWakeUpModeStop();
  vTaskDelay(300 / portTICK_PERIOD_MS);
  //rfalWakeUpModeStart( NULL ); //Use wakeup mode to reduce power
  //printf("Wakeup mode  \n");

  ReturnCode        errRFAL;
  bool              found = false;
  uint8_t           devIt = 0;
  rfalNfcaSensRes   sensRes;
  /* Infinite loop */
  while(1)
  {

  rfalWorker();

  errRFAL = rfalNfcaPollerInitialize();   /* Initialize for NFC-A */

  errRFAL = rfalFieldOnAndStartGT();      /* Turns the Field On if not already and start GT timer */

  errRFAL = rfalNfcaPollerTechnologyDetection( RFAL_COMPLIANCE_MODE_NFC, &sensRes );
  //printf("%d\n", &sensRes);
  printf("Poller tech detection %d\n", errRFAL);

  if(errRFAL == ERR_NONE) //At least 1 Card is in reader
    {

      rfalNfcaListenDevice nfcaDevList[1];
      uint8_t                   devCnt;
      errRFAL = rfalNfcaPollerFullCollisionResolution( RFAL_COMPLIANCE_MODE_NFC, 1, nfcaDevList, &devCnt);
      // printf("%d\n", errRFAL);

      if ( (errRFAL == ERR_NONE) && (devCnt > 0) )
      {
        found = true;
        devIt = 0;
        // printf("Found \n");


        /* Check if it is Topaz aka T1T */
        if( nfcaDevList[devIt].type == RFAL_NFCA_T1T )
        {
          /********************************************/
          /* NFC-A T1T card found                     */
          /* NFCID/UID is contained in: t1tRidRes.uid */
          printf("ISO14443A/Topaz (NFC-A T1T) TAG found. UID: %s\r\n", hex2Str(nfcaDevList[devIt].ridRes.uid, RFAL_T1T_UID_LEN));
        }
        else
        {
          /*********************************************/
          /* NFC-A device found                        */
          /* NFCID/UID is contained in: nfcaDev.nfcId1 */
          // printf("ISO14443A/NFC-A card found. UID: %s\r\n", hex2Str(nfcaDevList[0].nfcId1, nfcaDevList[0].nfcId1Len));
          //ID should be 5753445A31306D for Waveshare 42inch NFC

          /* task move to Block state to wait for interrupt event */
          //gpio_isr_handler_add(BUTTON_PIN, buttonISR, (void*) BUTTON_PIN);
          // printf("Ready and waiting on button \n");
          vTaskSuspend(flashLEDSlow_handle);
          gpio_set_level(LED_PIN, 1);
          // Wait 200ms for button push, if no button push, poll again to check that frame is still there
          if( xSemaphoreTake( buttonBinarySemaphore, 200 / portTICK_PERIOD_MS ) == pdTRUE )
          {
            printf("BUTTON\n");
          // gpio_set_level(LED_PIN, 0); //TODO make LED flash
          vTaskResume(flashLED_handle);
          // ESP_LOGI(TAG, "Taking picture...");
          camera_fb_t* pic = NULL;
          pic = esp_camera_fb_get();
          // camera_fb_t * fb = NULL;
          // fb = esp_camera_fb_get();
          esp_camera_fb_return(pic); // dispose the buffered image
          pic = NULL; // reset to capture errors
          pic = esp_camera_fb_get(); // get fresh image
          // vTaskDelay(200 / portTICK_RATE_MS);
          // int64_t timestamp = esp_timer_get_time();

          // use pic->buf to access the image
          ESP_LOGI(TAG, "Picture taken! Its size was: %zu bytes", pic->len);

          // printf("allocating memory\n");

          //uint8_t * bin_buf = (uint8_t *)_malloc(bin_buf_size);

          // printf("converting to binary\n");
          fmt2binary(pic->buf, pic->len, 400, 300, pic->format, ImageBuffer);
          // printf("Done\n");

          vTaskDelay(200 / portTICK_RATE_MS);
          rfalWorker();
          epaper_init(ImageBuffer);
          vTaskDelay(1000 / portTICK_RATE_MS);
          esp_camera_fb_return(pic); //release reserved memory
          vTaskSuspend(flashLED_handle);
          gpio_intr_enable(BUTTON_PIN);
          }
          rfalWorker();
          //rfalWakeUpModeStart( NULL );
          //rfalFieldOff();
        }
      }
    }
    else{
    // gpio_set_level(LED_PIN, 0);
    vTaskResume(flashLEDSlow_handle);
    rfalWorker();
    rfalFieldOff();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  rfalWorker();
  rfalFieldOff();
  vTaskDelay(300 / portTICK_PERIOD_MS);

  }

}
