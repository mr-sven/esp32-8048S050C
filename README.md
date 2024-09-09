# ESP32-8048S050C

**Implementation with FreeRTOS OSAL and LVGL 9.2**

Sunton ESP32-S3 800x480 Capacitive touch display

Example using esp-idf 5.2 and the esp_lcd_touch_gt911 and lvgl components.

In gt911_touch_init, a callback is registered to map the measured touch coordinates to display coordinates, see header file for information.

* Set esp-idf target to ESP32S3, other versions might lack rgb panel support.
* The supplied sdkconfig.defaults configures SPIRAM, regenerate your sdkconfig if needed.

idf.py set-target esp32s3 idf.py build flash monitor

## Branches

* [Main](../../tree/main)
  * LVGL 9.2.0
  * LVGL requires 128kb RAM for demo widgets
  * can use OSAL via `CONFIG_LV_OS_FREERTOS`
  * can use double-FB and direct rendering

* [Test](../../tree/lvgl-test) - test branch
