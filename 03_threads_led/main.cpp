#include <cstdio>

#include "clk.h"
#include "board.h"
#include "periph_conf.h"
#include "timex.h"
#include "ztimer.h"
#include "periph/gpio.h"  
#include "thread.h"
#include "msg.h"
#include "shell.h"

#include "ledcontroller.hh"
#define LED_GPIO GPIO12
#define THREAD_STACKSIZE        (THREAD_STACKSIZE_IDLE)
static char stack[THREAD_STACKSIZE];
static char stack1[THREAD_STACKSIZE];

static kernel_pid_t _led_pid;
#define LED_MSG_TYPE_ISR     (0x3456)
LEDController led(LED_GPIO);

void *_led_thread(void *arg)
{
    (void) arg;
    // input your code
    while(1){
        printf("[LED_THREAD] WAIT\n");
        msg_t msg;
        msg_receive(&msg);
        if (msg.type == LED_MSG_TYPE_ISR)
        {
            led.blink_faster(5, 100);
            printf("[LED_THREAD]: LED start to work!!\n");
        }
    }
    return NULL;
}

void *_sleep_thread(void *arg)
{
    uint16_t sleep_ms = 2000;
    (void) arg;
    while(1){
        ztimer_sleep(ZTIMER_USEC, sleep_ms * US_PER_MS);
        printf("[SLEEP_THREAD]: SLEEPFINISH\n");
        msg_t msg;
        msg.type = LED_MSG_TYPE_ISR;
        if (msg_send(&msg, _led_pid) <= 0){
            printf("[SLEEP_THREAD]: possibly lost interrupt.\n");
        }
        else{
            printf("[SLEEP_THREAD]: Successfully set interrupt.\n");

        }
    }
    return NULL;
}

static const shell_command_t shell_commands[] = {
    { NULL, NULL, NULL }
};

int main(void)
{
    led.blink_faster(10, 100);
    _led_pid = thread_create(stack, sizeof(stack), THREAD_PRIORITY_MAIN - 2,
                            THREAD_CREATE_STACKTEST, _led_thread, NULL,
                            "led");
    // kernel_pid_t *ppid = _led_pid;
    if (_led_pid <= KERNEL_PID_UNDEF) {
        printf("[MAIN] Creation of receiver thread failed\n");
        return 1;
    }
    else
    {
        printf("[MAIN] LED_PID: %d\n", _led_pid);
    }
    thread_create(stack1, sizeof(stack1), THREAD_PRIORITY_MAIN - 1,
                            THREAD_CREATE_STACKTEST, _sleep_thread, &_led_pid,
                            "sleep");
    printf("[Main] Initialization successful - starting the shell now\n");
    char line_buf[SHELL_DEFAULT_BUFSIZE];

    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
    return 0;
}
