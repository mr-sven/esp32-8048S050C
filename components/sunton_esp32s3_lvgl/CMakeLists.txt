set(requ lvgl esp_lcd)

if (SUNTON_ESP32_TOUCH_TYPE_CAPACITIVE)
    list(APPEND requ esp_lcd_touch_gt911)
endif()

idf_component_register(SRCS "esp32_8048s050.c" "touch.c" "backlight.c" "lcd.c"
                  INCLUDE_DIRS include
                  REQUIRES "${requ}")