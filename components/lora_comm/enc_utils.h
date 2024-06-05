#ifndef ENC_UTILS_H
#define ENC_UTILS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora_comm.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "mbedtls/gcm.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include <mbedtls/gcm.h>
#include <stdio.h>
#include <string.h>


int generate_random_key(unsigned char *key, size_t key_size);

// Function to print bytes for debugging
void print_hex(const char *label, const unsigned char *data, size_t length);
// Encryption function
void enc_data_and_transmit(unsigned char *input, size_t input_length);

#endif