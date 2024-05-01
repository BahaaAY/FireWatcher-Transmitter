#include "dht.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"

#define DHT_PIN 17 // GPIO 17

#define MQ2_PIN 2 // GPIO 2

#include "driver/i2c_master.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"

#include "esp_lvgl_port.h"
#include "lvgl.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

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

//-------------ADC Config---------------//
#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC2_CHAN2 ADC_CHANNEL_2 // GPIO 2
static int adc_raw[2][10];
static int voltage[2][10];

adc_oneshot_unit_handle_t adc2_handle;
adc_cali_handle_t adc2_cali_chan2_handle = NULL;
bool do_calibration2_chan2 = false;

static bool custom_adc_calibration_init(adc_unit_t unit, adc_channel_t channel,
                                        adc_atten_t atten,
                                        adc_cali_handle_t *out_handle);
static void custom_adc_calibration_deinit(adc_cali_handle_t handle);

lv_disp_t *disp;

void display_oled(int16_t *temperature, int16_t *humidity) {

  lv_obj_t *scr = lv_disp_get_scr_act(disp);

  // clear the screen

  lv_obj_clean(scr);

  lv_obj_t *label = lv_label_create(scr);
  // lv_label_set_long_mode(label,
  //                        LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll
  //                        */
  lv_label_set_text_fmt(label, "Temperature:%dC\nHumidity: %d%%\n",
                        *temperature, *humidity);
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

void setupADC() {
  const static char *TAG = "ADC_SETUP";
  //-------------ADC2 Init---------------//
  adc_oneshot_unit_init_cfg_t init_config2 = {
      .unit_id = ADC_UNIT_2,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

  //-------------ADC2 Config---------------//
  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .atten = ADC_ATTEN,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC2_CHAN2, &config));

  //-------------ADC2 Calibration Init---------------//

  do_calibration2_chan2 = custom_adc_calibration_init(
      ADC_UNIT_2, ADC2_CHAN2, ADC_ATTEN, &adc2_cali_chan2_handle);

  // // Tear Down
  // ESP_ERROR_CHECK(adc_oneshot_del_unit(adc2_handle));
  // if (do_calibration2_chan2) {
  //   custom_adc_calibration_deinit(adc2_cali_chan2_handle);
  // }
}

void readADC() {
  const static char *TAG = "ADC_READ";
  ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, ADC2_CHAN2, &adc_raw[0][0]));
  ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_2 + 1, ADC2_CHAN2,
           adc_raw[0][0]);
  if (do_calibration2_chan2) {
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_chan2_handle,
                                            adc_raw[0][0], &voltage[0][0]));
    ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_2 + 1,
             ADC2_CHAN2, voltage[0][0]);
  }
}
void app_main(void) {

  setupOled();
  setupADC();
  while (1) {
    int16_t humidity, temperature;
    dht_read_data(DHT_TYPE_DHT11, DHT_PIN, &humidity, &temperature);
    humidity = humidity / 10;
    temperature = temperature / 10;

    readADC();

    printf("Humidity: %d%% Temp: %dC\n", humidity, temperature);
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (lvgl_port_lock(0)) {

      display_oled(&temperature, &humidity);
      // Release the mutex
      lvgl_port_unlock();
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

static void custom_adc_calibration_deinit(adc_cali_handle_t handle) {
  const static char *TAG = "ADC_CALI_DEINIT";
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
  ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
  ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}
static bool custom_adc_calibration_init(adc_unit_t unit, adc_channel_t channel,
                                        adc_atten_t atten,
                                        adc_cali_handle_t *out_handle) {
  const static char *TAG = "ADC_CALI_INIT";
  adc_cali_handle_t handle = NULL;
  esp_err_t ret = ESP_FAIL;
  bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  if (!calibrated) {
    ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
      calibrated = true;
    }
  }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  if (!calibrated) {
    ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
      calibrated = true;
    }
  }
#endif

  *out_handle = handle;
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Calibration Success");
  } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
    ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
  } else {
    ESP_LOGE(TAG, "Invalid arg or no memory");
  }

  return calibrated;
}
