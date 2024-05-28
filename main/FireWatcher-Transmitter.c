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

#include "oled_display.h"

lv_disp_t *disp;

#define DHT_PIN 23

// LORA PINS
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26
#define DIO1 35
#define DIO2 34

//-------------ADC Config---------------//
#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC2_CHAN2 ADC_CHANNEL_2 // GPIO 2
static int adc_raw[2][10];
static int voltage[2][10];

adc_oneshot_unit_handle_t adc2_handle;
adc_cali_handle_t adc2_cali_chan2_handle = NULL;
bool do_calibration2_chan2 = false;

uint8_t dataArray[8];

static bool custom_adc_calibration_init(adc_unit_t unit, adc_channel_t channel,
                                        adc_atten_t atten,
                                        adc_cali_handle_t *out_handle);
static void custom_adc_calibration_deinit(adc_cali_handle_t handle);

char *TAG = "MAIN";

void setupADC() {
  TAG = "ADC_SETUP";
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
  TAG = "ADC_READ";
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

// // LORA START

sx127x *device = NULL;
int messages_sent = 0;
TaskHandle_t handle_interrupt;

void transmit_data(sx127x *device, uint8_t data[8], size_t size) {
  TAG = "TRANSMIT";
  ESP_LOGI(TAG, "Transmitting");
  ESP_ERROR_CHECK(sx127x_fsk_ook_tx_set_for_transmission(data, size, device));
  ESP_ERROR_CHECK(
      sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_FSK, device));
}

void IRAM_ATTR handle_interrupt_fromisr(void *arg) {
  xTaskResumeFromISR(handle_interrupt);
}

void handle_interrupt_task(void *arg) {
  while (1) {
    vTaskSuspend(NULL);
    sx127x_handle_interrupt((sx127x *)arg);
  }
}

void tx_callback(sx127x *device) {
  TAG = "TX_CALLBACK";
  ESP_LOGI(TAG, "Message Transmitted %d", messages_sent);
  messages_sent++;
  ESP_ERROR_CHECK(
      sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_FSK, device));
}

void setup_gpio_interrupts(gpio_num_t gpio, sx127x *device,
                           gpio_int_type_t type) {
  gpio_set_direction(gpio, GPIO_MODE_INPUT);
  gpio_pulldown_en(gpio);
  gpio_pullup_dis(gpio);
  gpio_set_intr_type(gpio, type);
  gpio_isr_handler_add(gpio, handle_interrupt_fromisr, (void *)device);
}

// // LORA END

void packData(uint8_t dataArray[8], int16_t humidity, int16_t temperature,
              int16_t rawSmoke, int16_t calSmokeVoltage) {

  // Pack humidity
  dataArray[0] = (uint8_t)(humidity & 0xFF);        // Lower 8 bits
  dataArray[1] = (uint8_t)((humidity >> 8) & 0xFF); // Upper 8 bits

  // Pack temperature
  dataArray[2] = (uint8_t)(temperature & 0xFF);
  dataArray[3] = (uint8_t)((temperature >> 8) & 0xFF);

  // Pack rawSmoke
  dataArray[4] = (uint8_t)(rawSmoke & 0xFF);
  dataArray[5] = (uint8_t)((rawSmoke >> 8) & 0xFF);

  // Pack calSmokeVoltage
  dataArray[6] = (uint8_t)(calSmokeVoltage & 0xFF);
  dataArray[7] = (uint8_t)((calSmokeVoltage >> 8) & 0xFF);
}

void app_main(void) {

  setupOled();
  setupADC();
  int16_t humidity, temperature;
  int16_t rawSmoke, calSmokeVoltage;

  ESP_LOGI(TAG, "starting up");
  spi_bus_config_t config = {
      .mosi_io_num = MOSI,
      .miso_io_num = MISO,
      .sclk_io_num = SCK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0,
  };
  ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &config, 1));
  spi_device_interface_config_t dev_cfg = {.clock_speed_hz = 3E6,
                                           .spics_io_num = SS,
                                           .queue_size = 16,
                                           .command_bits = 0,
                                           .address_bits = 8,
                                           .dummy_bits = 0,
                                           .mode = 0};
  spi_device_handle_t spi_device;
  ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &dev_cfg, &spi_device));
  ESP_ERROR_CHECK(sx127x_create(spi_device, &device));
  ESP_ERROR_CHECK(
      sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_FSK, device));
  ESP_ERROR_CHECK(sx127x_set_frequency(915000000, device));
  ESP_ERROR_CHECK(
      sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_FSK, device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_bitrate(4800.0, device));
  ESP_ERROR_CHECK(sx127x_fsk_set_fdev(5000.0, device));
  ESP_ERROR_CHECK(sx127x_set_preamble_length(4, device)); // FIXME?
  uint8_t syncWord[] = {0x12, 0xAD};
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_syncword(syncWord, 2, device));
  ESP_ERROR_CHECK(
      sx127x_fsk_ook_set_address_filtering(SX127X_FILTER_NONE, 0, 0, device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_packet_encoding(SX127X_NRZ, device));
  ESP_ERROR_CHECK(
      sx127x_fsk_ook_set_packet_format(SX127X_VARIABLE, 255, device));
  ESP_ERROR_CHECK(
      sx127x_fsk_set_data_shaping(SX127X_BT_0_5, SX127X_PA_RAMP_10, device));
  ESP_ERROR_CHECK(sx127x_tx_set_pa_config(SX127x_PA_PIN_BOOST, 20, device));
  ESP_ERROR_CHECK(sx127x_fsk_ook_set_crc(SX127X_CRC_CCITT, device));

  sx127x_tx_set_callback(tx_callback, device);

  BaseType_t task_code =
      xTaskCreatePinnedToCore(handle_interrupt_task, "handle interrupt", 8196,
                              device, 2, &handle_interrupt, xPortGetCoreID());
  if (task_code != pdPASS) {
    ESP_LOGE(TAG, "can't create task %d", task_code);
    sx127x_destroy(device);
    return;
  }

  gpio_install_isr_service(0);
  setup_gpio_interrupts((gpio_num_t)DIO0, device, GPIO_INTR_POSEDGE);
  setup_gpio_interrupts((gpio_num_t)DIO1, device, GPIO_INTR_NEGEDGE);
  setup_gpio_interrupts((gpio_num_t)DIO2, device, GPIO_INTR_POSEDGE);

  while (1) {
    dht_read_data(DHT_TYPE_DHT11, DHT_PIN, &humidity, &temperature);
    humidity = humidity / 10;
    temperature = temperature / 10;

    readADC();
    rawSmoke = adc_raw[0][0];
    calSmokeVoltage = voltage[0][0];

    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (lvgl_port_lock(0)) {

      display_oled(&temperature, &humidity, &rawSmoke, &calSmokeVoltage);
      // Release the mutex
      lvgl_port_unlock();
    }

    packData(dataArray, humidity, temperature, rawSmoke, calSmokeVoltage);
    transmit_data(device, dataArray, 8);
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
  TAG = "ADC_CALI_INIT";
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
