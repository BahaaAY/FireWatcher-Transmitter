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

static bool custom_adc_calibration_init(adc_unit_t unit, adc_channel_t channel,
                                        adc_atten_t atten,
                                        adc_cali_handle_t *out_handle);
static void custom_adc_calibration_deinit(adc_cali_handle_t handle);

char *TAG = "MAIN";

// // LORA START

sx127x *device = NULL;
int messages_sent = 0;
TaskHandle_t handle_interrupt;

void app_main(void) {
  ESP_LOGI(TAG, "starting up");

  setupOled();
  setup_adc();

  setup_lora();

  xTaskCreate(read_sensors, "read_sensors", 4096, NULL, 1, NULL);

  while (1) {

    // readADC();
    // rawSmoke = adc_raw[0][0];
    // calSmokeVoltage = voltage[0][0];

    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (lvgl_port_lock(0)) {

      // Release the mutex
      lvgl_port_unlock();
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}
