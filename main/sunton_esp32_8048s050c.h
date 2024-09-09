/**
 * @file sunton_esp32_8048s050c.h
 * @author Sven Fabricius (sven.fabricius@livediesel.de)
 * @brief
 * @version 0.1
 * @date 2024-09-09
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#define SUNTON_ESP32_LCD_WIDTH                  800
#define SUNTON_ESP32_LCD_HEIGHT                 480

#define SUNTON_ESP32_PIN_BCKL                   GPIO_NUM_2

// GT911 Pin config
#define SUNTON_ESP32_TOUCH_PIN_I2C_SCL          GPIO_NUM_20
#define SUNTON_ESP32_TOUCH_PIN_I2C_SDA          GPIO_NUM_19
#define SUNTON_ESP32_TOUCH_PIN_RST              GPIO_NUM_38
#define SUNTON_ESP32_TOUCH_PIN_INT              GPIO_NUM_18

// R5 is bound to VCC so GT911 should have I2C address 0x14, but GPIO18 is pulled down on init, so its 0x5D
#define SUNTON_ESP32_TOUCH_ADDRESS              ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS

// not required, external pullups R3 / R4 in place
//#define SUNTON_ESP32_TOUCH_I2C_PULLUP           y

#define SUNTON_ESP32_BACKLIGHT_LEDC_TIMER       LEDC_TIMER_0
#define SUNTON_ESP32_BACKLIGHT_LEDC_CHANNEL     LEDC_CHANNEL_0

#define LVGL_TICK_PERIOD_MS                     2

void sunton_esp32s3_backlight_init(void);
lv_display_t *sunton_esp32s3_lcd_init(void);
i2c_master_bus_handle_t sunton_esp32s3_i2c_master(void);
void sunton_esp32s3_touch_init(i2c_master_bus_handle_t i2c_master);
