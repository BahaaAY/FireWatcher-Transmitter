idf_component_register(SRCS "enc_utils.c" "lora_comm.c"
                       INCLUDE_DIRS "." REQUIRES driver sx127x oled mbedtls sensors_util) 