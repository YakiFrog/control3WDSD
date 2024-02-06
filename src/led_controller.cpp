#include <Arduino.h>
#include "led_controller.h"
#include <Adafruit_NeoPixel.h>

LEDController::LEDController(int pin, int num_pixels) : led(Adafruit_NeoPixel(num_pixels, pin, NEO_GRB + NEO_KHZ800))
{
}

void LEDController::init() {
    led.begin();
    led.show();
}

void LEDController::set_led(int led_num, int r, int g, int b) {
    led.setPixelColor(led_num, r, g, b);
    led.show();
}

void LEDController::set_all_led(int r, int g, int b) {
    for (int i = 0; i < led.numPixels(); i++) {
        led.setPixelColor(i, r, g, b);
    }
    led.show();
}

void LEDController::set_led_cw_ccw(int r, int g, int b, bool cw, int interval) {
    for (int i = 0; i < led.numPixels(); i++) {
        if (i % 2 == 0) {
            led.setPixelColor(i, r, g, b);
        } else {
            led.setPixelColor(i, 0, 0, 0);
        }
    }
    led.show();
    delay(interval);
    for (int i = 0; i < led.numPixels(); i++) {
        if (i % 2 == 0) {
            led.setPixelColor(i, 0, 0, 0);
        } else {
            led.setPixelColor(i, r, g, b);
        }
    }
    led.show();
    delay(interval);
}

void LEDController::set_led_blink(int r, int g, int b, int interval) {
    for (int i = 0; i < led.numPixels(); i++) {
        led.setPixelColor(i, r, g, b);
    }
    led.show();
    delay(interval);
    for (int i = 0; i < led.numPixels(); i++) {
        led.setPixelColor(i, 0, 0, 0);
    }
    led.show();
    delay(interval);
}