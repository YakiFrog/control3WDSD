#ifndef __LED_CONTROLLER_H__
#define __LED_CONTROLLER_H__

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

class LEDController {
    public:
        LEDController(int pin, int num_pixels);
        void init();
        void set_led(int led_num, int r, int g, int b);
        void set_all_led(int r, int g, int b);
        void set_led_cw_ccw(int r, int g, int b, bool cw, int interval);
        void set_led_blink(int r, int g, int b, int interval);
    private:
        Adafruit_NeoPixel led;
};
#endif // __LED_CONTROLLER_H__
