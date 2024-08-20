/**
 * @file lcd_touch.c
 * @author Sven Fabricius (sven.fabricius@livediesel.de)
 * @brief
 * @version 0.1
 * @date 2024-08-19
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_lcd_touch_gt911.h"

#include "driver/i2c_master.h"

#include "lcd_touch.h"

static const char *TAG = "TOUCH";

static uint16_t lcd_width = 0;
static uint16_t lcd_height = 0;

static inline uint16_t map(uint16_t n, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
    return (n - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void process_coordinates(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    *x = map(*x, 0, lcd_width, 0, lcd_width);
    *y = map(*y, 0, lcd_height, 0, lcd_height);
}

esp_lcd_touch_handle_t lcd_touch_init(uint16_t width, uint16_t height)
{
    ESP_LOGI(TAG, "Init Touch driver");
    lcd_width = width;
    lcd_height = height;
    i2c_master_bus_handle_t touch_i2c_bus_handle = NULL;
    const i2c_master_bus_config_t touch_i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = GPIO_NUM_20,
        .sda_io_num = GPIO_NUM_19,
        .glitch_ignore_cnt = 7,
        // not required, external pullups R3 / R4 in place
        //.flags.enable_internal_pullup = 1,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&touch_i2c_bus_config, &touch_i2c_bus_handle));

    esp_lcd_panel_io_handle_t gt911_touch_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t gt911_touch_io_config = {
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS,
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
    // R5 is bound to VCC so GT911 has I2C address 0x14
    esp_lcd_touch_io_gt911_config_t tp_gt911_config = {
        .dev_addr = 0x14,
    };
    const esp_lcd_touch_config_t gt911_touch_cfg = {
        .x_max = lcd_width,
        .y_max = lcd_height,
        .rst_gpio_num = GPIO_NUM_38,
        .int_gpio_num = GPIO_NUM_18,
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