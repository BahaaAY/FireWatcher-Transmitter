#include "dht.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"

#define DHT_PIN 17

#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"

#include "esp_lvgl_port.h"
#include "lvgl.h"

#define I2C_BUS_PORT 0

#define OLED_PIXEL_CLOCK_HZ (400 * 1000)
#define OLED_PIN_NUM_SDA 4
#define OLED_PIN_NUM_SCL 15
#define OLED_PIN_NUM_RST 16
#define OLED_I2C_HW_ADDR 0x3C

#define OLED_H_RES 128
#define OLED_V_RES 64

#define OLED_CMD_BITS 8
#define OLED_PARAM_BITS 8

lv_disp_t *disp;

void example_lvgl_demo_ui(float *temperature, float *humidity) {

  // clear the screen

  lv_obj_t *scr = lv_disp_get_scr_act(disp);

  lv_obj_clean(scr);

  lv_obj_t *label = lv_label_create(scr);
  // lv_label_set_long_mode(label,
  //                        LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll
  //                        */
  printf("Hello\n");
  printf("Temperature: %.2fC\n", *temperature);
  lv_label_set_text_fmt(label, "Temperature: %.2fC\n", *temperature);
  // lv_label_set_text_fmt(label, "Humidity: %.2f%%\n", *humidity);

  /* Size of the screen (if you use rotation 90 or 270, please set
   * disp->driver->ver_res) */
  lv_obj_set_width(label, disp->driver->hor_res);
  lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
}
void setupOled() {

  ESP_LOGI("SetupOled", "Initialize I2C bus");
  i2c_master_bus_handle_t i2c_bus = NULL;
  i2c_master_bus_config_t bus_config = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .i2c_port = I2C_BUS_PORT,
      .sda_io_num = OLED_PIN_NUM_SDA,
      .scl_io_num = OLED_PIN_NUM_SCL,
      .flags.enable_internal_pullup = true,
  };
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

  ESP_LOGI("SetupOled", "Install panel IO");
  esp_lcd_panel_io_handle_t io_handle = NULL;
  esp_lcd_panel_io_i2c_config_t io_config = {
      .dev_addr = OLED_I2C_HW_ADDR,
      .scl_speed_hz = OLED_PIXEL_CLOCK_HZ,
      .control_phase_bytes = 1,        // According to SSD1306 datasheet
      .lcd_cmd_bits = OLED_CMD_BITS,   // According to SSD1306 datasheet
      .lcd_param_bits = OLED_CMD_BITS, // According to SSD1306 datasheet
      .dc_bit_offset = 6,              // According to SSD1306 datasheet
  };

  ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));
  ESP_LOGI("SetupOled", "Install SSD1306 panel driver");
  esp_lcd_panel_handle_t panel_handle = NULL;
  esp_lcd_panel_dev_config_t panel_config = {
      .bits_per_pixel = 1,
      .reset_gpio_num = OLED_PIN_NUM_RST,
  };
  ESP_ERROR_CHECK(
      esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

  // setup lvgl

  ESP_LOGI("SetupOled", "Initialize LVGL");
  const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
  lvgl_port_init(&lvgl_cfg);

  const lvgl_port_display_cfg_t disp_cfg = {.io_handle = io_handle,
                                            .panel_handle = panel_handle,
                                            .buffer_size =
                                                OLED_H_RES * OLED_V_RES,
                                            .double_buffer = true,
                                            .hres = OLED_H_RES,
                                            .vres = OLED_V_RES,
                                            .monochrome = true,
                                            .rotation = {
                                                .swap_xy = false,
                                                .mirror_x = false,
                                                .mirror_y = false,
                                            }};
  disp = lvgl_port_add_disp(&disp_cfg);

  /* Rotation of the screen */
  lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);

  ESP_LOGI("END", "Display LVGL Scroll Text");
}

void app_main(void) {

  setupOled();

  while (1) {
    float humidity, temperature;
    dht_read_float_data(DHT_TYPE_DHT11, DHT_PIN, &humidity, &temperature);
    printf("Humidity: %.2f%% Temp: %.2fC\n", humidity, temperature);
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (lvgl_port_lock(0)) {

      example_lvgl_demo_ui(&humidity, &temperature);
      // Release the mutex
      lvgl_port_unlock();
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}
