/**
 * @file sunton_esp32s3.h
 * @author Sven Fabricius (sven.fabricius@livediesel.de)
 * @brief
 * @version 0.1
 * @date 2024-08-21
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "esp_lcd_panel_rgb.h"
#include "driver/gpio.h"
#include "lvgl.h"

#if defined(CONFIG_SUNTON_ESP32_8048S050C) || defined(CONFIG_SUNTON_ESP32_8048S050R)
    #include "esp32_8048s050.h"
#else
    #error "Select Display Type"
#endif

#if defined(CONFIG_SUNTON_ESP32_TOUCH_TYPE_CAPACITIVE) || defined(CONFIG_SUNTON_ESP32_TOUCH_TYPE_RESISTIVE)
    #include "esp_lcd_touch.h"

    void sunton_esp32s3_touch_init(void);
#endif

void sunton_esp32s3_backlight_init(void);
lv_display_t *sunton_esp32s3_lcd_init(void);