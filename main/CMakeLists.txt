idf_component_register(SRCS "lcd_backlight.c" "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES lvgl esp_lcd touch)