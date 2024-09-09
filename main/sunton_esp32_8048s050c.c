/**
 * @file sunton_esp32_8048s050c.c
 * @author Sven Fabricius (sven.fabricius@livediesel.de)
 * @brief
 * @version 0.1
 * @date 2024-09-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_touch_gt911.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "lvgl.h"

#include "sunton_esp32_8048s050c.h"

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
        .h_res = SUNTON_ESP32_LCD_WIDTH,
        .v_res = SUNTON_ESP32_LCD_HEIGHT,
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

static TaskHandle_t lvgl_port_task_handle = NULL;
static esp_timer_handle_t lvgl_tick_timer_handle = NULL;
static lv_indev_t * indev_touchpad = NULL;

void sunton_esp32s3_backlight_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num       = SUNTON_ESP32_BACKLIGHT_LEDC_TIMER,
        .freq_hz         = 4000,  // Set output frequency at 4 kHz
        .clk_cfg         = LEDC_USE_RC_FAST_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num   = SUNTON_ESP32_PIN_BCKL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = SUNTON_ESP32_BACKLIGHT_LEDC_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = SUNTON_ESP32_BACKLIGHT_LEDC_TIMER,
        .duty       = 0, // Set duty to 0%
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, SUNTON_ESP32_BACKLIGHT_LEDC_CHANNEL, 255));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, SUNTON_ESP32_BACKLIGHT_LEDC_CHANNEL));
}

#if CONFIG_SUNTON_ESP32_DOUBLE_FB_TEARING
IRAM_ATTR bool lvgl_port_task_notify(uint32_t value)
{
    BaseType_t need_yield = pdFALSE;

    // Notify LVGL task
    if (xPortInIsrContext() == pdTRUE)
    {
        xTaskNotifyFromISR(lvgl_port_task_handle, value, eNoAction, &need_yield);
    }
    else
    {
        xTaskNotify(lvgl_port_task_handle, value, eNoAction);
    }

    return (need_yield == pdTRUE);
}

static bool lvgl_port_flush_vsync_ready_callback(esp_lcd_panel_handle_t panel_io, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx)
{
    return lvgl_port_task_notify(ULONG_MAX);
}
#endif

static void lvgl_disp_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
#if CONFIG_SUNTON_ESP32_DOUBLE_FB_TEARING
    if (lv_display_flush_is_last(disp))
    {
#endif
        esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px_map);
#if CONFIG_SUNTON_ESP32_DOUBLE_FB_TEARING
        /* Waiting for the last frame buffer to complete transmission */
        ulTaskNotifyValueClear(NULL, ULONG_MAX);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
#endif
    lv_display_flush_ready(disp);
};

static void lvgl_port_task(void *arg)
{
#ifdef CONFIG_LV_OS_FREERTOS
    // required to ensure the notify task ID is correct
    lv_draw_init();
#endif
    uint32_t task_delay_ms = CONFIG_LVGL_TASK_MAX_DELAY_MS;
    while (1)
    {
        task_delay_ms = lv_timer_handler();
        if (task_delay_ms > CONFIG_LVGL_TASK_MAX_DELAY_MS)
        {
            task_delay_ms = CONFIG_LVGL_TASK_MAX_DELAY_MS;
        }
        else if (task_delay_ms < CONFIG_LVGL_TASK_MIN_DELAY_MS)
        {
            task_delay_ms = CONFIG_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

//region - LVGL Tick Timer
static void lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

lv_display_t *sunton_esp32s3_lcd_init(void)
{
    void *buf1 = NULL;
    void *buf2 = NULL;
    int buffer_size;
    esp_lcd_panel_handle_t panel_handle = NULL;

    // create lcd panel
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    // assign callback and handle
    lv_init();
    lv_display_t * disp = lv_display_create(SUNTON_ESP32_LCD_WIDTH, SUNTON_ESP32_LCD_HEIGHT);
    lv_display_set_user_data(disp, panel_handle);
    lv_display_set_flush_cb(disp, lvgl_disp_flush);

    buffer_size = SUNTON_ESP32_LCD_WIDTH * SUNTON_ESP32_LCD_HEIGHT * sizeof(lv_color_t); // 2 = 16bit color data

#if CONFIG_SUNTON_ESP32_DOUBLE_FB
#if CONFIG_SUNTON_ESP32_DOUBLE_FB_TEARING
    // register flush callback to avoid tearing effect
    const esp_lcd_rgb_panel_event_callbacks_t vsync_cbs = {
        .on_vsync = lvgl_port_flush_vsync_ready_callback,
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &vsync_cbs, disp));
#endif

    ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2));
    lv_display_set_buffers(disp, buf1, buf2, buffer_size, LV_DISPLAY_RENDER_MODE_DIRECT);
#else
    buffer_size = buffer_size / 10;
    buf1 = heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    //buf2 = heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    lv_display_set_buffers(disp, buf1, buf2, buffer_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
#endif

    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lvgl_tick,
        .name = "lvgl_tick"
    };
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer_handle, LVGL_TICK_PERIOD_MS * 1000));

    xTaskCreate(lvgl_port_task, "lvgl_port_task", (CONFIG_LVGL_TASK_STACK_SIZE * 1024), NULL, CONFIG_LVGL_TASK_PRIORITY, &lvgl_port_task_handle);

    return disp;
}

i2c_master_bus_handle_t sunton_esp32s3_i2c_master(void)
{
    i2c_master_bus_handle_t touch_i2c_bus_handle = NULL;
    const i2c_master_bus_config_t touch_i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = SUNTON_ESP32_TOUCH_PIN_I2C_SCL,
        .sda_io_num = SUNTON_ESP32_TOUCH_PIN_I2C_SDA,
        .glitch_ignore_cnt = 7,
#if CONFIG_SUNTON_ESP32_TOUCH_I2C_PULLUP
        .flags.enable_internal_pullup = 1,
#endif
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&touch_i2c_bus_config, &touch_i2c_bus_handle));
    return touch_i2c_bus_handle;
}

static inline uint16_t map(uint16_t n, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
    return (n - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void process_coordinates(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    *x = map(*x, 0, SUNTON_ESP32_LCD_WIDTH, 0, SUNTON_ESP32_LCD_WIDTH);
    *y = map(*y, 0, SUNTON_ESP32_LCD_HEIGHT, 0, SUNTON_ESP32_LCD_HEIGHT);
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

static esp_lcd_touch_handle_t touch_init(i2c_master_bus_handle_t i2c_master)
{
    esp_lcd_panel_io_handle_t gt911_touch_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t gt911_touch_io_config = {
        .dev_addr = SUNTON_ESP32_TOUCH_ADDRESS,
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
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_master, &gt911_touch_io_config, &gt911_touch_io_handle));

    esp_lcd_touch_handle_t touch_handle = NULL;
    esp_lcd_touch_io_gt911_config_t tp_gt911_config = {
        .dev_addr = gt911_touch_io_config.dev_addr,
    };
    const esp_lcd_touch_config_t gt911_touch_cfg = {
        .x_max = SUNTON_ESP32_LCD_WIDTH,
        .y_max = SUNTON_ESP32_LCD_HEIGHT,
        .rst_gpio_num = SUNTON_ESP32_TOUCH_PIN_RST,
        .int_gpio_num = SUNTON_ESP32_TOUCH_PIN_INT,
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

void sunton_esp32s3_touch_init(i2c_master_bus_handle_t i2c_master)
{
    indev_touchpad = lv_indev_create();
    lv_indev_set_type(indev_touchpad, LV_INDEV_TYPE_POINTER);
    lv_indev_set_user_data(indev_touchpad, touch_init(i2c_master));
    lv_indev_set_read_cb(indev_touchpad, touchpad_read);
}
