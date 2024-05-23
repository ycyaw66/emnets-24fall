#ifndef _BLINK
#define _BLINK
#include "periph/gpio.h"
#include <xtimer.h>
#define BLINK_GPIO GPIO12
static void configure_led(void)
{
    /* Set the GPIO as a push/pull output */
    gpio_init(BLINK_GPIO, GPIO_OUT);
}
static uint8_t s_led_state = 0;
static void blink_led(void)
{
    gpio_write(BLINK_GPIO, s_led_state);
}
void blink_faster(int times, int delay_time)
{
    s_led_state = 0;
    for (int i = 0; i < times * 2; ++i)
    {
        s_led_state = !s_led_state;
        blink_led();
        xtimer_msleep(delay_time / 2);
    }
    s_led_state = 0;
}
#endif
