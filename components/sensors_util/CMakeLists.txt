idf_component_register(SRCS "sensors_util.c"
                       INCLUDE_DIRS "." REQUIRES driver esp_adc dht oled lora_comm) 