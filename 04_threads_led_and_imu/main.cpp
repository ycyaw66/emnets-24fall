/*
 * Copyright (C) 2021 Otto-von-Guericke-Universität Magdeburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Blinky application
 *
 * @author      Marian Buschsieweke <marian.buschsieweke@ovgu.de>
 *
 * @}
 */

#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <string>
#include <log.h>
#include <errno.h>
#include "clk.h"
#include "board.h"
#include "periph_conf.h"
#include "timex.h"
#include "ztimer.h"
#include "periph/gpio.h"  
#include "thread.h"
#include "msg.h"
#include "shell.h"
// #include "xtimer.h"
#include "ledcontroller.hh"
#include "mpu6050.h"
#define LED_GPIO GPIO12
#define THREAD_STACKSIZE        (THREAD_STACKSIZE_IDLE)
static char stack[THREAD_STACKSIZE];
static char stack1[THREAD_STACKSIZE];

static kernel_pid_t _led_pid;
#define LED_MSG_TYPE_ISR     (0x3456)
LEDController led(LED_GPIO);
struct MPU6050Data
{
    float ax, ay, az;
    float gx, gy, gz;
};
enum MoveState{Stationary, Tilted, Rotating, Moving};
/**
 * LED control thread function.
 * This thread initially blinks the LED 10 times with a delay of 100 milliseconds per blink.
 * Then, it enters an infinite loop where it waits for messages to control the LED.
 * When receiving a message of type LED_MSG_TYPE_ISR, it blinks the LED 4 times with a delay of 100 milliseconds per blink.
 * @param arg Unused argument.
 * @return NULL.
 */
void *_led_thread(void *arg)
{
    (void) arg;
    // input your code
    // Initially blink the LED 10 times with a delay of 100 milliseconds per blink
    led.blink_faster(10, 100);
    while(1){
        // Wait for a message to control the LED
        msg_t msg;
        msg_receive(&msg);
        // Check if the received message is of type LED_MSG_TYPE_ISR
        switch (msg.type)
        {
        case Stationary:
            led.change_led_state(0);
            // always dark
            break;
        case Tilted:
            led.change_led_state(1);
            // always ligth
            break;
        case Rotating:
            led.blink_faster(4, 100);
            break;
        case Moving:
            led.blink_faster(2, 200);
            break;
        default:
            break;
        }
        printf("[LED_THREAD]: Movement state:%d\n", msg.type);
    }
    return NULL;
}
void delay(uint32_t sleep_ms)
{
    ztimer_sleep(ZTIMER_USEC, sleep_ms * US_PER_MS);
    return;
}


#define g_acc (9.8)
/**
 * IMU (Inertial Measurement Unit) thread function.
 * This thread initializes the MPU6050 sensor, retrieves device information, and configures sensor settings.
 * It then enters an infinite loop where it reads sensor data, calculates balance angles, and sends interrupt messages to the LED thread if a significant change in angle is detected.
 * 
 * @param arg Unused argument.
 * @return NULL.
 */


MoveState detectMovement(MPU6050Data &data)
{
    // input your code
    float g_threshold = 5;
    float rotate_threshold = 25;
    float a_threshold = 1.0;
    if (fabs(data.gx) > rotate_threshold || fabs(data.gy) > rotate_threshold || fabs(data.gz) > rotate_threshold) {
        return Rotating;
    }
    else{
        if(fabs(data.gx) < g_threshold && fabs(data.gy) < g_threshold && fabs(data.gz) < g_threshold) {
           if(fabs(data.ax) < a_threshold && fabs(data.ay) < a_threshold) {
                return Stationary;
            }
            else if (fabs(data.az) < 10.0){
                return Tilted;
            }
            else {
                return Moving;
            }
        }
        else
        {
            return Moving;
        }
    }
}

void *_imu_thread(void *arg)
{
    (void) arg;
    // Initialize MPU6050 sensor
    MPU6050 mpu;
    uint8_t device_id = mpu.getDeviceID();
    uint16_t rate = mpu.getRate();
    printf("[IMU_THREAD] DEVICE_ID:0x%x, RATE:0x%x\n", device_id, rate);
    mpu.initialize();
    mpu.setTempSensorEnabled(false);

    // Configure gyroscope and accelerometer full scale ranges
    uint8_t gyro_fs = mpu.getFullScaleGyroRange();
    uint8_t accel_fs_g = mpu.getFullScaleAccelRange();
    uint16_t accel_fs_real = 1;
    float gyro_fs_convert = 1.0;

    // Convert gyroscope full scale range to conversion factor
    if (gyro_fs == MPU6050_GYRO_FS_250)
        gyro_fs_convert = 131.0;
    else if (gyro_fs == MPU6050_GYRO_FS_500)
        gyro_fs_convert = 65.5;
    else if (gyro_fs == MPU6050_GYRO_FS_1000)
        gyro_fs_convert = 32.8;
    else if (gyro_fs == MPU6050_GYRO_FS_2000)
        gyro_fs_convert = 16.4;
    else
        printf("[IMU_THREAD] Unknown GYRO_FS: 0x%x\n", gyro_fs);

    // Convert accelerometer full scale range to real value
    if (accel_fs_g == MPU6050_ACCEL_FS_2)
        accel_fs_real = g_acc * 2;
    else if (accel_fs_g == MPU6050_ACCEL_FS_4)
        accel_fs_real = g_acc * 4;
    else if (accel_fs_g == MPU6050_ACCEL_FS_8)
        accel_fs_real = g_acc * 8;
    else if (accel_fs_g == MPU6050_ACCEL_FS_16)
        accel_fs_real = g_acc * 16;
    else
        printf("[IMU_THREAD] Unknown ACCEL_FS: 0x%x\n", accel_fs_g);

    // Calculate accelerometer conversion factor
    float accel_fs_convert = 32768.0 / accel_fs_real;

    // Initialize variables
    int16_t ax, ay, az, gx, gy, gz;

    // Initial delay
    delay(1000);
    int idx = 0;
    // Main loop
    while (1) {
        // Read sensor data
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        ztimer_sleep(ZTIMER_USEC, 200 * US_PER_MS);

        MPU6050Data data;
        // Convert raw sensor data to real values
        data.ax = ax / accel_fs_convert;
        data.ay = ay / accel_fs_convert;
        data.az = az / accel_fs_convert;
        data.gx = gx / gyro_fs_convert;
        data.gy = gy / gyro_fs_convert;
        data.gz = gz / gyro_fs_convert;

        // Print sensor data and balance angle
        printf("----------------------------------------\n");
        printf("[IMU_THREAD] (X,Y,Z):(%.02f,%.02f,%.02f), (XG,YG,ZG):(%.02f,%.02f,%.02f)\n", data.ax, data.ay, data.az, data.gx, data.gy, data.gz);

        idx += 1;
        idx %= 1024;
        if (idx % 2 == 0)
        {
            MoveState state = detectMovement(data);
            // Check for significant change in angle
            msg_t msg;  
            msg.type = state;
            if (msg_send(&msg, _led_pid) <= 0){
                printf("[IMU_THREAD]: possibly lost interrupt.\n");
            }
        }
    }
    return NULL;
}
static const shell_command_t shell_commands[] = {
    { NULL, NULL, NULL }
};

int main(void)
{
    gpio_init(GPIO23, GPIO_OUT);
    // gpio_init(GPIO14, GPIO_OUT);
    gpio_set(GPIO23);
    // gpio_set(GPIO14);
    delay(1000);

    _led_pid = thread_create(stack, sizeof(stack), THREAD_PRIORITY_MAIN - 2,
                            THREAD_CREATE_STACKTEST, _led_thread, NULL,
                            "led_controller_thread");
    if (_led_pid <= KERNEL_PID_UNDEF) {
        printf("[MAIN] Creation of receiver thread failed\n");
        return 1;
    }
    thread_create(stack1, sizeof(stack1), THREAD_PRIORITY_MAIN - 1,
                            THREAD_CREATE_STACKTEST, _imu_thread, &_led_pid,
                            "imu_read_thread");
    printf("[Main] Initialization successful - starting the shell now\n");

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
    return 0;
}
