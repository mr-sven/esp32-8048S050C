#include "freertos/FreeRTOS.h"
#include "lvgl.h"
#include "demos/lv_demos.h"

#include "sunton_esp32s3.h"

void app_main(void)
{
    sunton_esp32s3_backlight_init();

    lv_display_t * disp = sunton_esp32s3_lcd_init();

    sunton_esp32s3_touch_init();

    lv_lock();
    lv_demo_widgets();
    lv_unlock();
}