# ESP32-8048S050C

**Implementation with FreeRTOS OSAL and LVGL 9.4**

Sunton ESP32-S3 800x480 Capacitive touch display

Example using esp-idf 5.5.2 and the esp_lcd_touch_gt911 and lvgl components.

In gt911_touch_init, a callback is registered to map the measured touch coordinates to display coordinates, see header file for information.

* Set esp-idf target to ESP32S3, other versions might lack rgb panel support.
* The supplied sdkconfig.defaults configures SPIRAM, regenerate your sdkconfig if needed.

idf.py set-target esp32s3 idf.py build flash monitor

## Additional infos and limitations

Due the reduced size of internal RAM of the ESP32S3, the framebuffer cannot be located in internal RAM.

This limitation leads to transfer problems via the DMA. Progmem and PSRAM share the same SPI bus for read and write data.

* Bounce Buffer Mode
  * Pixelclock at 18 MHz
  * Fluent medium animations
  * Lack of performance in slide and scroll
* Double Buffer Mode
  * Pixelclock at 14 MHz
  * Fluent medium animations
  * Slide and scroll more fluent

If you are facing issues in panel distortion or glitchy animations in your project, this may caused by the DMA/SPI bottleneck, so you must reduce the pixelclock.

In bounce buffer mode, be aware of the lvgl draw buffer which is located in the internal RAM by default and may cause OOM issues.

## Branches

* [Main](../../tree/main)
  * LVGL 9.4.0
  * LVGL requires 128kb RAM for demo widgets
  * can use OSAL via `CONFIG_LV_OS_FREERTOS`
  * can use double-FB and direct rendering

* [Test](../../tree/lvgl-test) - test branch
