#include <stdint.h>
#include <stdio.h>

#include "dht.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <esp_intr_alloc.h>
#include <sx127x.h>

#include "driver/i2c_master.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"

#include "esp_lvgl_port.h"
#include "lvgl.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

#include "lora_comm.h"
#include "oled_display.h"
#include "sensors_util.h"
lv_disp_t *disp;

char *TAG = "MAIN";

// // LORA START

sx127x *device = NULL;
int messages_sent = 0;
TaskHandle_t handle_interrupt;

QueueHandle_t sensorQueue = NULL;

void app_main(void) {
  ESP_LOGI(TAG, "starting up");

  sensorQueue = xQueueCreate(10, sizeof(SensorData));

  gpio_set_direction(TRANSMIT_LED, GPIO_MODE_OUTPUT);
  setupOled();
  setup_adc();

  setup_lora();

  xTaskCreatePinnedToCore(read_sensors_task, "read_sensors", 4096, NULL, 1,
                          NULL, 0);

  xTaskCreatePinnedToCore(enc_transmit_data_task, "enc_transmit_data", 4096,
                          NULL, 1, NULL, 1);

  while (1) {
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}
