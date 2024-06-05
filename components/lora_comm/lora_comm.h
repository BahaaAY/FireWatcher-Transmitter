#ifndef LORA_COMM_H
#define LORA_COMM_H

#include <sx127x.h>

#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "mbedtls/gcm.h"
#include "oled_display.h"
#include "sdkconfig.h"
#include "sensors_util.h"
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <esp_err.h>
#include <esp_log.h>
#include <inttypes.h>
#include <mbedtls/gcm.h>

// LORA PINS
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26
#define DIO1 35
#define DIO2 34

#define TRANSMIT_LED 25

void setup_lora();

void handle_interrupt_task(void *arg);

void tx_callback(sx127x *device);

void setup_gpio_interrupts(gpio_num_t gpio, gpio_int_type_t type);

void transmit_data(uint8_t data[8], size_t size);

void packData(uint8_t dataArray[8], int16_t humidity, int16_t temperature,
              int16_t rawSmoke, int16_t calSmokeVoltage);

void enc_transmit_data_task(void *pvParameters);

#endif