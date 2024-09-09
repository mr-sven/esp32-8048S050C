#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"
#include "lvgl.h"
#include "demos/lv_demos.h"

#include "sunton_esp32_8048s050c.h"

void app_main(void)
{
    sunton_esp32s3_backlight_init();

    lv_display_t * disp = sunton_esp32s3_lcd_init();

    i2c_master_bus_handle_t i2c_master = sunton_esp32s3_i2c_master();
    sunton_esp32s3_touch_init(i2c_master);

    lv_lock();
    lv_demo_widgets();
    lv_unlock();
}