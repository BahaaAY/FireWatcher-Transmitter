#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include <esp_log.h>

void setupOled();
void display_oled_qr(char *name, char *transport, char *ver);
void display_oled(int16_t *temperature, int16_t *humidity, int16_t *smoke,
                  int16_t *calSmokeVoltage);
#endif // OLED_DISPLAY_H