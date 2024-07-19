#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_touch_gt911.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "lvgl.h"
#include "demos/lv_demos.h"

#define LCD_WIDTH               800
#define LCD_HEIGHT              480
#define LCD_NUM_FB              2
#define LCD_GPIO_BCKL           GPIO_NUM_2

static const char *TAG = "LVGL";

static lv_indev_t * indev_touchpad = NULL;

static void lvgl_disp_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
    esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px_map);
    lv_display_flush_ready(disp);
};

void lvgl_lcd_init(lv_display_t *disp)
{
    void *buf1 = NULL;
    void *buf2 = NULL;
    int buffer_size;
    esp_lcd_panel_handle_t panel_handle = NULL;

    const esp_lcd_rgb_panel_config_t st7262_panel_config = {
        .data_width = 16,
        .num_fbs = LCD_NUM_FB,
        .clk_src = LCD_CLK_SRC_PLL160M,
        .timings = {
            .pclk_hz = (16*1000000),
            .h_res = LCD_WIDTH,
            .v_res = LCD_HEIGHT,
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

    // create lcd panel
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&st7262_panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    // assign callback and handle
    lv_display_set_user_data(disp, panel_handle);
    lv_display_set_flush_cb(disp, lvgl_disp_flush);

#if LCD_NUM_FB == 2
    buffer_size = LCD_WIDTH * LCD_HEIGHT * 2; // 2 = 16bit color data
    esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2);
    lv_display_set_buffers(disp, buf1, buf2, buffer_size, LV_DISPLAY_RENDER_MODE_DIRECT);
#else
    buffer_size = LCD_WIDTH * LCD_HEIGHT / 4;
    buf1 = heap_caps_malloc(sizeof(lv_color_t) * buffer_size, MALLOC_CAP_SPIRAM);
    lv_display_set_buffers(disp, buf1, buf2, buffer_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
#endif
}

uint16_t map(uint16_t n, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t  out_max)
{
    return (n - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void process_coordinates(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    *x = map(*x, 0, LCD_WIDTH, 0, LCD_WIDTH);
    *y = map(*y, 0, LCD_HEIGHT, 0, LCD_HEIGHT);
}

esp_lcd_touch_handle_t lvgl_touch_init(void)
{
    ESP_LOGI(TAG, "Init Touch driver");
    i2c_master_bus_handle_t touch_i2c_bus_handle = NULL;
    const i2c_master_bus_config_t touch_i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = GPIO_NUM_20,
        .sda_io_num = GPIO_NUM_19,
        .glitch_ignore_cnt = 7,
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
    const esp_lcd_touch_config_t gt911_touch_cfg = {
        .x_max = LCD_WIDTH,
        .y_max = LCD_HEIGHT,
        .rst_gpio_num = GPIO_NUM_38,
        .int_gpio_num = GPIO_NUM_NC,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
        .process_coordinates = process_coordinates, // callback to fix coordinates between gt911 and display
        .interrupt_callback = NULL,
    };
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(gt911_touch_io_handle, &gt911_touch_cfg, &touch_handle));

    return touch_handle;
}

static void gt911_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data)
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

#define LVGL_TICK_PERIOD_MS    2
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 1
#define LVGL_TASK_STACK_SIZE   (8 * 1024)
#define LVGL_TASK_PRIORITY     2

static SemaphoreHandle_t lvgl_mux = NULL;

static void lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

bool lvgl_lock(int timeout_ms)
{
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

void lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

static void lvgl_port_task(void *arg)
{
    // required to ensure the notify task ID is correct
    lv_draw_init();

    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1)
    {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (lvgl_lock(-1))
        {
            task_delay_ms = lv_timer_handler();
            // Release the mutex
            lvgl_unlock();
        }
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS)
        {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        }
        else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS)
        {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Turn on LCD backlight");
    ledc_timer_config_t ledc_timer = {
            .speed_mode       = LEDC_LOW_SPEED_MODE,
            .duty_resolution  = LEDC_TIMER_8_BIT,
            .timer_num        = LEDC_TIMER_0,
            .freq_hz          = 4000,  // Set output frequency at 4 kHz
            .clk_cfg          = LEDC_USE_RC_FAST_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
            .gpio_num       = LCD_GPIO_BCKL,
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = LEDC_CHANNEL_0,
            .intr_type      = LEDC_INTR_DISABLE,
            .timer_sel      = LEDC_TIMER_0,
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,(uint32_t)(256/(100/100.00))-1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    lv_init();

    lv_display_t * disp = lv_display_create(LCD_WIDTH, LCD_HEIGHT);

    lvgl_lcd_init(disp);

    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    xTaskCreate(lvgl_port_task, "lvgl_port_task", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);

    // init touch i2c bus
    esp_lcd_touch_handle_t touch_handle = lvgl_touch_init();
    indev_touchpad = lv_indev_create();
    lv_indev_set_type(indev_touchpad, LV_INDEV_TYPE_POINTER);
    lv_indev_set_user_data(indev_touchpad, touch_handle);
    lv_indev_set_read_cb(indev_touchpad, gt911_touchpad_read);

    if (lvgl_lock(-1))
    {
        lv_demo_widgets();
        lvgl_unlock();
    }
}