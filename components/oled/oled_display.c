#include "oled_display.h"

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

extern lv_disp_t *disp;

void setupOled() {
  const char *TAG = "SETUP_OLED";
  ESP_LOGI(TAG, "Initialize I2C bus");
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

  ESP_LOGI(TAG, "Install panel IO");
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
  ESP_LOGI(TAG, "Install SSD1306 panel driver");
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
  TAG = "LVGL_SETUP";
  ESP_LOGI(TAG, "Initialize LVGL");
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

  if (disp == NULL) {
    ESP_LOGE(TAG, "Failed to add display");
    return;
  }
  // print address of disp
  ESP_LOGI("SETUP_OLED", "Address of disp after setup: %p", (void *)disp);
}
/* Not Used in Lora Transmitter
#define LV_COLOR_WHITE lv_color_make(255, 255, 255)
#define LV_COLOR_BLACK lv_color_make(0, 0, 0)
void display_oled_qr(char *name, char *transport, char *ver) {
  const char *TAG = "DISPLAY_QR_CODE";
  if (!name || !transport) {
    ESP_LOGW(TAG, "Cannot generate QR code payload. Data missing.");
    return;
  }
  char payload[150] = {0};
  snprintf(payload, sizeof(payload),
           "{\"ver\":\"%s\",\"name\":\"%s\""
           ",\"transport\":\"%s\"}",
           ver, name, transport);
  ESP_LOGI(
      TAG,
      "Scan this QR code from the provisioning application for Provisioning.");

  ESP_LOGI(TAG, "Displaying QR code on OLED");
  lv_obj_t *qrcode_obj =
      lv_qrcode_create(lv_scr_act(), 60, LV_COLOR_BLACK,
                       LV_COLOR_WHITE); // Replace values as needed

  // Update the QR code content
  lv_qrcode_update(qrcode_obj, payload, strlen(payload));
  lv_obj_align(qrcode_obj, LV_ALIGN_CENTER, 0, 0);
  lv_scr_load(lv_scr_act());
}
*/
void display_oled(int16_t *temperature, int16_t *humidity, int16_t *smoke,
                  int16_t *calSmokeVoltage) {

  // print address of disp
  ESP_LOGI("DISPLAY_OLED", "Address of disp in display_oled: %p", (void *)disp);

  if (disp == NULL) {
    ESP_LOGI("DISPLAY_OLED", "disp is NULL");
  }

  if (!disp || !(disp->driver)) {
    ESP_LOGE("DISPLAY_OLED", "Display or display driver is NULL");
    return;
  }

  lv_obj_t *scr = lv_disp_get_scr_act(disp);
  if (!scr) {
    ESP_LOGE("DISPLAY_OLED", "Active screen is NULL");
    return;
  }

  // clear the screen

  lv_obj_clean(scr);

  lv_obj_t *label = lv_label_create(scr);
  // lv_label_set_long_mode(label,
  //                        LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll
  //                        */
  lv_label_set_text_fmt(
      label, "Temperature:%dC\nHumidity: %d%%\nSmoke: %d\nVoltage: %d\n",
      *temperature, *humidity, *smoke, *calSmokeVoltage);
  // lv_label_set_text_fmt(label, "Humidity: %.2f%%\n", *humidity);

  /* Size of the screen (if you use rotation 90 or 270, please set
   * disp->driver->ver_res) */
  lv_obj_set_width(label, disp->driver->hor_res);
  lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
  //   lv_scr_load(lv_scr_act());
}