#include "lora_comm.h"

extern sx127x *device;
extern int messages_sent;
extern TaskHandle_t handle_interrupt;

extern QueueHandle_t sensorQueue;

void transmit_data(uint8_t data[8], size_t size) {
  char *TAG = "TRANSMIT";
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
  char *TAG = "TX_CALLBACK";

  ESP_LOGI(TAG, "Message Transmitted %d", messages_sent);
  messages_sent++;
  ESP_ERROR_CHECK(
      sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_FSK, device));
}

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

void setup_gpio_interrupts(gpio_num_t gpio, gpio_int_type_t type) {
  gpio_set_direction(gpio, GPIO_MODE_INPUT);
  gpio_pulldown_en(gpio);
  gpio_pullup_dis(gpio);
  gpio_set_intr_type(gpio, type);
  gpio_isr_handler_add(gpio, handle_interrupt_fromisr, (void *)device);
}

void setup_lora() {

  char *TAG = "LORA_SETUP";
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
  setup_gpio_interrupts((gpio_num_t)DIO0, GPIO_INTR_POSEDGE);
  setup_gpio_interrupts((gpio_num_t)DIO1, GPIO_INTR_NEGEDGE);
  setup_gpio_interrupts((gpio_num_t)DIO2, GPIO_INTR_POSEDGE);
}

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

void enc_transmit_data_task(void *pvParameters) {
  char *TAG = "ENC_TRANSMIT";

  SensorData sensorReading;
  for (;;) {

    ESP_LOGI(TAG, "Transmitting on core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Transmitting encrypted data");
    if (xQueueReceive(sensorQueue, &sensorReading, portMAX_DELAY) != pdTRUE) {
      ESP_LOGE(TAG, "Failed to receive sensor data");

    } else {
      gpio_set_level(TRANSMIT_LED, 1);
      uint8_t dataArray[8];
      // Lock the mutex due to the LVGL APIs are not thread-safe
      if (lvgl_port_lock(0)) {
        display_oled(&sensorReading.temperature, &sensorReading.humidity,
                     &sensorReading.smoke, &sensorReading.smoke);
        // Release the mutex
        lvgl_port_unlock();
      }
      packData(dataArray, sensorReading.humidity, sensorReading.temperature,
               sensorReading.smoke, 0000);
      transmit_data(dataArray, 8);
    }
    gpio_set_level(TRANSMIT_LED, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}