#ifndef SENSOR_UTIL_H
#define SENSOR_UTIL_H

#include "dht.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora_comm.h"
#include "oled_display.h"
#include <esp_err.h>
#include <esp_log.h>

#include "sensor_data.h" // Include the sensor data structure definition

//-------------ADC Config---------------//
#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC2_CHAN2 ADC_CHANNEL_2 // GPIO 2

#define DHT_PIN 23

void setup_adc();
void read_sensors_task(void *pvParameters);
#endif
