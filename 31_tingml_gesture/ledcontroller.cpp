#include "ztimer.h"
#include "ledcontroller.hh"
#include "periph/gpio.h"  
#include <iostream>

LEDController::LEDController(int gpio){  
    std::cout << "LED Controller initialized with GPIO " << led_gpio << std::endl;  
    led_gpio = gpio;
    gpio_init(led_gpio, GPIO_OUT);
}  

void LEDController::delay(uint32_t sleep_ms) {
    ztimer_sleep(ZTIMER_USEC, sleep_ms * US_PER_MS);
}

/**
 * Updates the state of the LED based on the current LED state.
 * ------------------------------------------------------------
 * @note Method 1
 * Utilizes the gpio_write function to set the GPIO pin connected to the LED to the current LED state.
 * void gpio_write(gpio_t pin, int value);
 * @param pin The GPIO pin connected to the LED.
 * @param value The value to set the GPIO pin to (0 for LOW, 1 for HIGH).
 * ------------------------------------------------------------
 * @note Method 2
 * Uses the gpio_set and gpio_clear functions to set the GPIO pin connected to the LED to the current LED state.
 * void gpio_set(); void gpio_clear();
 * @param pin The GPIO pin connected to the LED.
 */


void LEDController::change_led_state(int state) {
    led_state = state;
    update_led();
}

void LEDController::update_led(void){
    // input your code
    gpio_write(led_gpio, led_state);
}
/**
 * Blinks the LED at a faster rate for a specified number of times.
 *
 * @param times The number of times to blink the LED.
 * @param delay_time_per_blink The duration of each blink in milliseconds.
 */
void LEDController::blink_faster(int times, uint32_t delay_time_per_blink) {
    //  input your code
    for (int i = 0; i < times * 2; ++i)
    {
        led_state = !led_state;
        update_led();
        delay(delay_time_per_blink / 2);
    }
}
