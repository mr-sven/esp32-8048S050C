#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lvgl.h"
#include "demos/lv_demos.h"

#include "lcd_backlight.h"
#include "touch.h"

#define LCD_WIDTH                   800
#define LCD_HEIGHT                  480

#define LCD_DOUBLE_FB               1

#define LVGL_TICK_PERIOD_MS         2
#define LVGL_TASK_MAX_DELAY_MS      500
#define LVGL_TASK_MIN_DELAY_MS      1
#define LVGL_TASK_STACK_SIZE        (8 * 1024)
#define LVGL_TASK_PRIORITY          2

static const char *TAG = "LVGL";

static lv_indev_t * indev_touchpad = NULL;
static TaskHandle_t lvgl_port_task_handle = NULL;
static esp_timer_handle_t lvgl_tick_timer_handle = NULL;

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

static void lvgl_disp_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
    if (lv_display_flush_is_last(disp))
    {
        esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px_map);
        /* Waiting for the last frame buffer to complete transmission */
        ulTaskNotifyValueClear(NULL, ULONG_MAX);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    lv_display_flush_ready(disp);
};

static void lvgl_lcd_init(lv_display_t *disp)
{
    void *buf1 = NULL;
    void *buf2 = NULL;
    int buffer_size;
    esp_lcd_panel_handle_t panel_handle = NULL;

    const esp_lcd_rgb_panel_config_t st7262_panel_config = {
        .data_width = 16,
#if LCD_DOUBLE_FB
        .num_fbs = 2,
#else
        .num_fbs = 1,
#endif
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

    buffer_size = LCD_WIDTH * LCD_HEIGHT * sizeof(lv_color_t); // 2 = 16bit color data

#if LCD_DOUBLE_FB
    // register flush callback to avoid tearing effect
    const esp_lcd_rgb_panel_event_callbacks_t vsync_cbs = {
        .on_vsync = lvgl_port_flush_vsync_ready_callback,
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &vsync_cbs, disp));

    ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2));
    lv_display_set_buffers(disp, buf1, buf2, buffer_size, LV_DISPLAY_RENDER_MODE_DIRECT);
#else
    buffer_size = buffer_size / 10;
    buf1 = heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    //buf2 = heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    lv_display_set_buffers(disp, buf1, buf2, buffer_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
#endif
}

static void lvgl_port_task(void *arg)
{
    // required to ensure the notify task ID is correct
    lv_draw_init();

    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1)
    {
        task_delay_ms = lv_timer_handler();
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

//region - LVGL Touch components
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

static void lvgl_init_touch(void)
{
    indev_touchpad = lv_indev_create();
    lv_indev_set_type(indev_touchpad, LV_INDEV_TYPE_POINTER);
    lv_indev_set_user_data(indev_touchpad, touch_init(LCD_WIDTH, LCD_HEIGHT));
    lv_indev_set_read_cb(indev_touchpad, touchpad_read);
}
//endregion

//region - LVGL Tick Timer
static void lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void lvgl_init_tick_timer(void)
{
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lvgl_tick,
        .name = "lvgl_tick"
    };
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer_handle, LVGL_TICK_PERIOD_MS * 1000));
}
//endregion


void app_main(void)
{
    lcd_backlight_init();

    lv_init();

    lv_display_t * disp = lv_display_create(LCD_WIDTH, LCD_HEIGHT);

    lvgl_lcd_init(disp);

    lvgl_init_tick_timer();

    xTaskCreate(lvgl_port_task, "lvgl_port_task", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, &lvgl_port_task_handle);

    lvgl_init_touch();

    lv_lock();
    lv_demo_widgets();
    lv_unlock();
}