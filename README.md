# ESP32-8048S050C

**Implementation with FreeRTOS OSAL**

Sunton ESP32-S3 800x480 Capacitive touch display

Example using esp-idf 5.2 and the esp_lcd_touch_gt911 and lvgl components.

In gt911_touch_init, a callback is registered to map the measured touch coordinates to display coordinates, see header file for information.

* Set esp-idf target to ESP32S3, other versions might lack rgb panel support.
* The supplied sdkconfig.defaults configures SPIRAM, regenerate your sdkconfig if needed.

idf.py set-target esp32s3 idf.py build flash monitor

## Branches

* [Main](../../tree/main) - Implementation without FreeRTOS OSAL setting
  * LVGL 9.1.0

* [Freertos LVGL 9.1.0](../../tree/freertos-lvgl-9.1.0) - Implementation FreeRTOS OSAL
  * LVGL 9.1.0
  * LVGL requires 128kb RAM for demo widgets
  * Bar animation is a bit flickery

* [Test](../../tree/lvgl-test) - Implementation FreeRTOS OSAL LVGL git master
  * LVGL git master branch
  * LVGL requires 128kb RAM for demo widgets
  * Bar animation is a bit flickery
