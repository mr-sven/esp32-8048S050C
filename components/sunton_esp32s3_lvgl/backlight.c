/**
 * @file backlight.c
 * @author Sven Fabricius (sven.fabricius@livediesel.de)
 * @brief
 * @version 0.1
 * @date 2024-08-19
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "freertos/FreeRTOS.h"

#include "driver/ledc.h"

#include "sunton_esp32s3.h"

//TODO: Add more configs for LEDC Channel etc.
void sunton_esp32s3_backlight_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = 4000,  // Set output frequency at 4 kHz
        .clk_cfg         = LEDC_USE_RC_FAST_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num   = CONFIG_SUNTON_ESP32_PIN_BCKL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0, // Set duty to 0%
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 255));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}