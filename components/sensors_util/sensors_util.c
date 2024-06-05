#include "sensors_util.h"

adc_oneshot_unit_handle_t adc2_handle;
adc_cali_handle_t adc2_cali_chan2_handle = NULL;
bool do_calibration2_chan2 = false;
static int adc_raw[2][10];
static int voltage[2][10];

uint8_t dataArray[8];
int16_t humidity, temperature;
int16_t rawSmoke, calSmokeVoltage;

extern QueueHandle_t sensorQueue;

void setup_adc() {
  char *TAG = "ADC_SETUP";
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

  //   do_calibration2_chan2 = custom_adc_calibration_init(
  //       ADC_UNIT_2, ADC2_CHAN2, ADC_ATTEN, &adc2_cali_chan2_handle);

  // // Tear Down
  // ESP_ERROR_CHECK(adc_oneshot_del_unit(adc2_handle));
  // if (do_calibration2_chan2) {
  //   custom_adc_calibration_deinit(adc2_cali_chan2_handle);
  // }
}

void read_adc() {
  char *TAG = "ADC_READ";
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

void read_sensors_task(void *pvParameters) {
  SensorData sensorReading;
  char *TAG = "SENSORS TASK";
  for (;;) {
    ESP_LOGI(TAG, "Reading sensors on core %d", xPortGetCoreID());
    read_adc();
    rawSmoke = adc_raw[0][0];
    dht_read_data(DHT_TYPE_DHT11, DHT_PIN, &humidity, &temperature);
    humidity = humidity / 10;
    temperature = temperature / 10;

    sensorReading.humidity = humidity;
    sensorReading.temperature = temperature;
    sensorReading.smoke = rawSmoke;
    sensorReading.timestamp = esp_log_timestamp();
    xQueueSend(sensorQueue, &sensorReading, 0);

    // packData(dataArray, humidity, temperature, rawSmoke, calSmokeVoltage);
    // transmit_data(dataArray, 8);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}