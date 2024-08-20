idf_component_register(SRCS "lcd_touch.c" "lcd_backlight.c" "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES lvgl esp_lcd esp_lcd_touch_gt911)