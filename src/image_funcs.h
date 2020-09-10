

/* Define to prevent recursive inclusion -------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_camera.h"

typedef size_t (* jpg_out_cb)(void * arg, size_t index, const void* data, size_t len);

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
bool fmt2binary(const uint8_t *src_buf, size_t src_len, uint16_t width, uint16_t height, pixformat_t format, uint8_t * ImageBuffer);

#ifdef __cplusplus
}
#endif
