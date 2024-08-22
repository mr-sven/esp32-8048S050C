/**
 * @file touch.c
 * @author Sven Fabricius (sven.fabricius@livediesel.de)
 * @brief
 * @version 0.1
 * @date 2024-08-20
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "freertos/FreeRTOS.h"

#include "driver/i2c_master.h"
#include "esp_lcd_touch_gt911.h"

#include "sunton_esp32s3.h"

static lv_indev_t * indev_touchpad = NULL;

static inline uint16_t map(uint16_t n, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
    return (n - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void process_coordinates(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    *x = map(*x, 0, CONFIG_SUNTON_ESP32_LCD_WIDTH, 0, CONFIG_SUNTON_ESP32_LCD_WIDTH);
    *y = map(*y, 0, CONFIG_SUNTON_ESP32_LCD_HEIGHT, 0, CONFIG_SUNTON_ESP32_LCD_HEIGHT);
}

static void touchpad_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    esp_lcd_touch_handle_t tp = (esp_lcd_touch_handle_t)lv_indev_get_user_data(indev);

    uint16_t touchpad_x;
    uint16_t touchpad_y;
    uint8_t touchpad_cnt = 0;

    esp_lcd_touch_read_data(tp);

    bool touchpad_pressed = esp_lcd_touch_get_coordinates(tp, &touchpad_x, &touchpad_y, NULL, &touchpad_cnt, 1);
    if (touchpad_pressed && touchpad_cnt > 0)
    {
        data->point.x = touchpad_x;
        data->point.y = touchpad_y;
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

#ifdef CONFIG_SUNTON_ESP32_TOUCH_TYPE_CAPACITIVE
static esp_lcd_touch_handle_t touch_init(void)
{
    i2c_master_bus_handle_t touch_i2c_bus_handle = NULL;
    const i2c_master_bus_config_t touch_i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = CONFIG_SUNTON_ESP32_TOUCH_PIN_I2C_SCL,
        .sda_io_num = CONFIG_SUNTON_ESP32_TOUCH_PIN_I2C_SDA,
        .glitch_ignore_cnt = 7,
#if CONFIG_SUNTON_ESP32_TOUCH_I2C_PULLUP
        .flags.enable_internal_pullup = 1,
#endif
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&touch_i2c_bus_config, &touch_i2c_bus_handle));

    esp_lcd_panel_io_handle_t gt911_touch_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t gt911_touch_io_config = {
        .dev_addr = CONFIG_SUNTON_ESP32_TOUCH_ADDRESS,
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
        .control_phase_bytes = 1,
        .dc_bit_offset = 0,
        .lcd_cmd_bits = 16,
        .lcd_param_bits = 0,
        .flags = {
            .dc_low_on_data = 0,
            .disable_control_phase = 1,
        },
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(touch_i2c_bus_handle, &gt911_touch_io_config, &gt911_touch_io_handle));

    esp_lcd_touch_handle_t touch_handle = NULL;
    esp_lcd_touch_io_gt911_config_t tp_gt911_config = {
        .dev_addr = gt911_touch_io_config.dev_addr,
    };
    const esp_lcd_touch_config_t gt911_touch_cfg = {
        .x_max = CONFIG_SUNTON_ESP32_LCD_WIDTH,
        .y_max = CONFIG_SUNTON_ESP32_LCD_HEIGHT,
        .rst_gpio_num = CONFIG_SUNTON_ESP32_TOUCH_PIN_RST,
        .int_gpio_num = CONFIG_SUNTON_ESP32_TOUCH_PIN_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
        .driver_data = &tp_gt911_config,
        .process_coordinates = process_coordinates, // callback to fix coordinates between gt911 and display
        .interrupt_callback = NULL,
    };
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(gt911_touch_io_handle, &gt911_touch_cfg, &touch_handle));
    return touch_handle;
}
#endif

void sunton_esp32s3_touch_init(void)
{
    indev_touchpad = lv_indev_create();
    lv_indev_set_type(indev_touchpad, LV_INDEV_TYPE_POINTER);
    lv_indev_set_user_data(indev_touchpad, touch_init());
    lv_indev_set_read_cb(indev_touchpad, touchpad_read);
}