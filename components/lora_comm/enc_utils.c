#include "enc_utils.h"
#define AES_KEY_SIZE 32 // 256 bits
#define AES_IV_SIZE 12  // 96 bits
#define AES_KEY                                                                \
  "d5c72e5e8ef42e5317c412fa59cdd33818b3288414826130fdaa3603301a176a"
// Function to initialize a random key
int generate_random_key(unsigned char *key, size_t key_size) {
  mbedtls_entropy_context entropy;
  mbedtls_ctr_drbg_context ctr_drbg;
  const char *personalization = "MyEntropy";
  int ret;

  mbedtls_entropy_init(&entropy);
  mbedtls_ctr_drbg_init(&ctr_drbg);

  // Seed and setup entropy source for DRBG
  ret = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
                              (const unsigned char *)personalization,
                              strlen(personalization));
  if (ret != 0) {
    printf("Failed in mbedtls_ctr_drbg_seed: %d\n", ret);
    return ret;
  }

  // Generate a random key
  ret = mbedtls_ctr_drbg_random(&ctr_drbg, key, key_size);
  if (ret != 0) {
    printf("Failed in mbedtls_ctr_drbg_random: %d\n", ret);
    return ret;
  }

  mbedtls_entropy_free(&entropy);
  mbedtls_ctr_drbg_free(&ctr_drbg);
  return 0;
}

// Function to print bytes for debugging
void print_hex(const char *label, const unsigned char *data, size_t length) {
  if (label != NULL) {
    printf("%s: ", label);
  }
  for (size_t i = 0; i < length; i++) {
    printf("%02X ", data[i]);
  }
  printf("\n");
}
// Function to convert a hexadecimal string to a byte array
void hex_string_to_byte_array(const char *hex_str, unsigned char *byte_array,
                              size_t byte_array_size) {
  size_t len = strlen(hex_str);
  if (len % 2 != 0 || len / 2 > byte_array_size) {
    printf("Invalid hex string\n");
    return;
  }

  for (size_t i = 0; i < len / 2; ++i) {
    sscanf(hex_str + 2 * i, "%2hhx", &byte_array[i]);
  }
}

void enc_data_and_transmit(unsigned char *input, size_t input_length) {
  mbedtls_gcm_context gcm;
  unsigned char key[32];
  unsigned char nonce[12];
  unsigned char tag[16];
  unsigned char
      output[128 + sizeof(nonce) + sizeof(tag)]; // Adjust size accordingly
  int ret;

  // Convert hex key string to byte array
  hex_string_to_byte_array(AES_KEY, key, sizeof(key));

  // Initialize GCM context
  mbedtls_gcm_init(&gcm);

  // Generate a random nonce
  generate_random_key(nonce, sizeof(nonce));

  // Set up the key in the GCM context
  ret = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, key, 256);
  if (ret != 0) {
    printf("Failed in mbedtls_gcm_setkey: %d\n", ret);
    mbedtls_gcm_free(&gcm);
    return;
  }

  // Encrypt the data
  ret = esp_aes_gcm_crypt_and_tag(&gcm, MBEDTLS_GCM_ENCRYPT, input_length,
                                  nonce, sizeof(nonce), NULL, 0, input, output,
                                  sizeof(tag), tag);
  if (ret != 0) {
    printf("Failed in mbedtls_gcm_crypt_and_tag: %d\n", ret);
    mbedtls_gcm_free(&gcm);
    return;
  }
  printf("Encryption success\n");

  // Copy nonce and tag to the output buffer after the encrypted data
  memcpy(output + input_length, nonce, sizeof(nonce));
  memcpy(output + input_length + sizeof(nonce), tag, sizeof(tag));

  // Total transmission size is the sum of encrypted data, nonce, and tag sizes
  size_t transmission_size = input_length + sizeof(nonce) + sizeof(tag);
  print_hex("Full Transmission", output, transmission_size);

  // Transmit encrypted data along with nonce and tag
  transmit_data(output, transmission_size);

  // Free GCM context
  mbedtls_gcm_free(&gcm);
}
