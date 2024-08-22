/**
 * @file lcd.c
 * @author Sven Fabricius (sven.fabricius@livediesel.de)
 * @brief
 * @version 0.1
 * @date 2024-08-21
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "freertos/FreeRTOS.h"
#include "esp_timer.h"

#include "esp_lcd_panel_ops.h"

#include "sunton_esp32s3.h"

#define LVGL_TICK_PERIOD_MS         2

extern const esp_lcd_rgb_panel_config_t panel_config;

static TaskHandle_t lvgl_port_task_handle = NULL;
static esp_timer_handle_t lvgl_tick_timer_handle = NULL;

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
    lv_display_t * disp = lv_display_create(CONFIG_SUNTON_ESP32_LCD_WIDTH, CONFIG_SUNTON_ESP32_LCD_HEIGHT);
    lv_display_set_user_data(disp, panel_handle);
    lv_display_set_flush_cb(disp, lvgl_disp_flush);

    buffer_size = CONFIG_SUNTON_ESP32_LCD_WIDTH * CONFIG_SUNTON_ESP32_LCD_HEIGHT * sizeof(lv_color_t); // 2 = 16bit color data

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