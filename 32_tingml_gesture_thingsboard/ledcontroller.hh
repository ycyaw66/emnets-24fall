// LEDController.h  
#pragma once
#ifndef LEDCONTROLLER_H  
#define LEDCONTROLLER_H  
#include <stdint.h>
#include "timex.h"

class LEDController {  
public:  
    /**
     * LEDController constructor.
     * Initializes the LED controller with the specified GPIO pin.
     *
     * @param gpio The GPIO pin to be used for controlling the LED.
     */
    LEDController(int gpio);  
    /**
     * Blinks the LED at a faster rate for a specified number of times.
     *
     * @param times The number of times to blink the LED.
     * @param delay_time_per_blink The duration of each blink in milliseconds.
     */
    void blink_faster(int times, uint32_t delay_time_per_blink);
    /**
     * Changes the state of the LED to the specified state.
     *
     * @param state The state to set the LED to (0 for OFF, 1 for ON).
     */
    void change_led_state(int state);
    /**
     * Delays execution for the specified duration in milliseconds.
     *
     * @param sleep_ms The duration to sleep in milliseconds.
     */
    void delay(uint32_t sleep_ms);  // sleep time
private:  
    int led_gpio;               // led in gpio
    int led_state;              // 1 | 0
    /**
     * Updates the state of the LED based on the current LED state.
     */
    void update_led(void);
};  
  
#endif  