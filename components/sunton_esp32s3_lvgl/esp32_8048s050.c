/**
 * @file esp32_8048s050.c
 * @author Sven Fabricius (sven.fabricius@livediesel.de)
 * @brief
 * @version 0.1
 * @date 2024-08-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "sunton_esp32s3.h"

const esp_lcd_rgb_panel_config_t panel_config = {
    .data_width = 16,
#if CONFIG_SUNTON_ESP32_DOUBLE_FB
    .num_fbs = 2,
#else
    .num_fbs = 1,
#endif
    .clk_src = LCD_CLK_SRC_PLL160M,
    .timings = {
        .pclk_hz = (16*1000000),
        .h_res = CONFIG_SUNTON_ESP32_LCD_WIDTH,
        .v_res = CONFIG_SUNTON_ESP32_LCD_HEIGHT,
        .hsync_pulse_width = 4,
        .hsync_back_porch = 8,
        .hsync_front_porch = 8,
        .vsync_pulse_width = 4,
        .vsync_back_porch = 8,
        .vsync_front_porch = 8,
        .flags = {
            .hsync_idle_low = true,
            .vsync_idle_low = true,
            .de_idle_high = false,
            .pclk_active_neg = true,
            .pclk_idle_high = false,
        },
    },
    .sram_trans_align = 8,
    .psram_trans_align = 64,
    .hsync_gpio_num = GPIO_NUM_39,
    .vsync_gpio_num = GPIO_NUM_41,
    .de_gpio_num = GPIO_NUM_40,
    .pclk_gpio_num = GPIO_NUM_42,
    .data_gpio_nums = {
        GPIO_NUM_8, GPIO_NUM_3, GPIO_NUM_4, GPIO_NUM_9, GPIO_NUM_1,               // B0 - B4
        GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7, GPIO_NUM_15, GPIO_NUM_16, GPIO_NUM_4, // G0 - G5
        GPIO_NUM_45, GPIO_NUM_48, GPIO_NUM_47, GPIO_NUM_21, GPIO_NUM_14,          // R0 - R4
    },
    .disp_gpio_num = GPIO_NUM_NC,
    .flags = {
        .disp_active_low = false,
        .fb_in_psram = true,
    },
};