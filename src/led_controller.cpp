#include <Arduino.h>
#include "led_controller.h"
#include <Adafruit_NeoPixel.h>

LEDController::LEDController(int pin, int num_pixels) : led(Adafruit_NeoPixel(num_pixels, pin, NEO_GRB + NEO_KHZ800))
{
}

void LEDController::init() {
    led.begin();
    led.setBrightness(255);
    led.show();
}

void LEDController::set_led(int led_num, int r, int g, int b) {
    led.setPixelColor(led_num, r, g, b);
    led.show();
}

void LEDController::set_all_led(int r, int g, int b) {
    led.clear();
    for (int i = 0; i < led.numPixels(); i++) {
        led.setPixelColor(i, r, g, b);
    }
    led.show();
}

void LEDController::set_led_cw_ccw(int r, int g, int b, bool cw, int interval) {
    if (cw) {
        for (int i = 0; i < led.numPixels(); i++) {
            led.setPixelColor(i, r, g, b);
            led.setPixelColor((i + 13) % led.numPixels(), 0, 0, 0);
            led.show();
            delay(interval / led.numPixels());
        }
    } else {
        for (int i = led.numPixels() - 1; i >= 0; i--) {
            led.setPixelColor(i, r, g, b);
            led.setPixelColor((i + 13) % led.numPixels(), 0, 0, 0);
            led.show();
            delay(interval / led.numPixels());
        }
    }
}

void LEDController::set_led_blink(int r, int g, int b, int interval) {
    for (int i = 0; i < led.numPixels(); i++) {
        led.setPixelColor(i, r, g, b);
    }
    led.show();
    delay(interval / 2);
    led.clear();
    delay(interval / 2);
}