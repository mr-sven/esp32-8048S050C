/**
 * @file esp32_8048s050.h
 * @author Sven Fabricius (sven.fabricius@livediesel.de)
 * @brief
 * @version 0.1
 * @date 2024-08-21
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#define CONFIG_SUNTON_ESP32_LCD_WIDTH              800
#define CONFIG_SUNTON_ESP32_LCD_HEIGHT             480

#define CONFIG_SUNTON_ESP32_PIN_BCKL               GPIO_NUM_2

#ifdef CONFIG_SUNTON_ESP32_TOUCH_TYPE_CAPACITIVE
    // GT911 Pin config
    #define CONFIG_SUNTON_ESP32_TOUCH_PIN_I2C_SCL  GPIO_NUM_20
    #define CONFIG_SUNTON_ESP32_TOUCH_PIN_I2C_SDA  GPIO_NUM_19
    #define CONFIG_SUNTON_ESP32_TOUCH_PIN_RST      GPIO_NUM_38
    #define CONFIG_SUNTON_ESP32_TOUCH_PIN_INT      GPIO_NUM_18

    // R5 is bound to VCC so GT911 should have I2C address 0x14, but GPIO18 is pulled down on init, so its 0x5D
    #define CONFIG_SUNTON_ESP32_TOUCH_ADDRESS      ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS
    //_BACKUP

    // not required, external pullups R3 / R4 in place
    // #define CONFIG_SUNTON_ESP32_TOUCH_I2C_PULLUP  y
#elif defined(CONFIG_SUNTON_ESP32_TOUCH_TYPE_RESISTIVE)
    #error "Resitive touch currently not implemented"
#endif

