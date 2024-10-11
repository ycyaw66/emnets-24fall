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
#define THREAD_STACKSIZE        (THREAD_STACKSIZE_IDLE)
static char stack_for_led_thread[THREAD_STACKSIZE];
static char stack_for_imu_thread[THREAD_STACKSIZE];

static kernel_pid_t _led_pid;
#define LED_MSG_TYPE_ISR     (0x3456)
#define LED_MSG_TYPE_NONE    (0x3110)
#define LED_MSG_TYPE_RED     (0x3111)
#define LED_MSG_TYPE_GREEN   (0x3112)
#define LED_MSG_TYPE_YELLOW  (0x3113)
#define LED_MSG_TYPE_BLUE    (0x3114)
#define LED_MSG_TYPE_MAGENTA (0x3115)
#define LED_MSG_TYPE_CYAN    (0x3116)
#define LED_MSG_TYPE_WHITE   (0x3117)
#define LED_GPIO_R GPIO26
#define LED_GPIO_G GPIO25
#define LED_GPIO_B GPIO27
struct MPU6050Data
{
    float ax, ay, az;
    float gx, gy, gz;
};
enum MoveState{Stationary, Tilted, Rotating, MovingX, MovingY, MovingZ, Moving};

void delay_ms(uint32_t sleep_ms)
{
    ztimer_sleep(ZTIMER_USEC, sleep_ms * US_PER_MS);
    return;
}
/**
 * LED control thread function.
 * Then, it enters an infinite loop where it waits for messages to control the LED.
 * @param arg Unused argument.
 * @return NULL.
 */
void *_led_thread(void *arg)
{
    (void) arg;
    LEDController led(LED_GPIO_R, LED_GPIO_G, LED_GPIO_B);
    led.change_led_color(0);
    while(1){
        // Input your codes
        // Wait for a message to control the LED
        // Display different light colors based on the motion state of the device.
        printf("[LED_THREAD] WAIT\n");
        msg_t msg;
        msg_receive(&msg);
        switch (msg.type) {
            case LED_MSG_TYPE_NONE:
                led.change_led_color(0);
                printf("[LED_THREAD]: LED TURN OFF!! DEVICE IS Stationary!!\n");
                break;
            case LED_MSG_TYPE_RED:
                led.change_led_color(1);
                printf("[LED_THREAD]: LED TURN RED!! DEVICE IS Tilted!!\n");
                break;
            case LED_MSG_TYPE_GREEN:
                led.change_led_color(2);
                printf("[LED_THREAD]: LED TURN GREEN!! DEVICE IS Moving!!\n");
                break;
            case LED_MSG_TYPE_YELLOW:
                led.change_led_color(3);
                printf("[LED_THREAD]: LED TURN YELLOW!! DEVICE IS MovingX!!\n");
                break;
            case LED_MSG_TYPE_BLUE:
                led.change_led_color(4);
                printf("[LED_THREAD]: LED TURN BLUE!! DEVICE IS Rotating!!\n");
                break;
            case LED_MSG_TYPE_MAGENTA:
                led.change_led_color(5);
                printf("[LED_THREAD]: LED TURN MAGENTA!! DEVICE IS MovingY!!\n");
                break;
            case LED_MSG_TYPE_WHITE:
                led.change_led_color(7);
                printf("[LED_THREAD]: LED TURN WHITE!!\n");
                break;
            default:
                break;
        }
    }
    return NULL;
}

#define g_acc (9.8)
MoveState detectMovement(MPU6050Data &data)
{
    // Input your code
    // Please use your imagination or search online 
    // to determine the motion state of the device 
    // based on the data obtained from the MPU6050 sensor.
    if (abs(data.gx) > 70 || abs(data.gy) > 70 || abs(data.gz) > 70) {
        return Rotating;
    }
    // 合成三个方向加速度判断是否移动
    float acc = sqrt(data.ax * data.ax + data.ay * data.ay + data.az * data.az);
    if (abs(acc - g_acc) < 0.5 && abs(data.gx) < 5 && abs(data.gy) < 5 && abs(data.gz) < 5) {
        if (abs(data.az - g_acc) > 0.1) {
            return Tilted;
        }
        return Stationary;
    }
    // 判断哪个轴在平移
    float deltax = abs(data.ax), deltay = abs(data.ay);
    if (deltax > 1 && deltay < 1) {
        return MovingX;
    }
    else if (deltay > 1 && deltax < 1) {
        return MovingY;
    }
    else {
        return Moving;
    }
}

void *_imu_thread(void *arg)
{
    (void) arg;
    // Input your code
    // 1. initial mpu6050 sensor
    MPU6050 mpu;
    uint8_t device_id = mpu.getDeviceID();
    printf("[IMU_THREAD] DEVICE_ID:0x%x\n", device_id);
    mpu.initialize();
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
    float avg_ax = 0, avg_ay = 0, avg_az = 0, avg_gx = 0, avg_gy = 0, avg_gz = 0;
    // 花5s调零
    for (int i = 0; i < 100; i++) {
        msg_t msg;
        msg.type = LED_MSG_TYPE_WHITE;
        if (msg_send(&msg, _led_pid) <= 0){
            printf("[IMU_THREAD]: possibly lost interrupt.\n");
        }
        else{
            printf("[IMU_THREAD]: Successfully set interrupt.\n");
        }
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        avg_ax += ax / accel_fs_convert;
        avg_ay += ay / accel_fs_convert;
        avg_az += az / accel_fs_convert;
        avg_gx += gx / gyro_fs_convert;
        avg_gy += gy / gyro_fs_convert;
        avg_gz += gz / gyro_fs_convert;
        delay_ms(50);
    }
    avg_ax /= 100; avg_ay /= 100; avg_az /= 100;
    avg_gx /= 100; avg_gy /= 100; avg_gz /= 100;
    
    delay_ms(1000);
    // Main loop
    MoveState lastState = Stationary;
    while (1) {
        // Read sensor data
        // 2. Acquire sensor data every 100ms
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        delay_ms(100);    
        MPU6050Data data;
        // Convert raw sensor data to real values
        data.ax = ax / accel_fs_convert - avg_ax;
        data.ay = ay / accel_fs_convert - avg_ay;
        data.az = az / accel_fs_convert - avg_az + g_acc;
        data.gx = gx / gyro_fs_convert - avg_gx;
        data.gy = gy / gyro_fs_convert - avg_gy;
        data.gz = gz / gyro_fs_convert - avg_gz;
        // Print sensor data and balance angle
        printf("----------------------------------------\n");
        printf("[IMU_THREAD] (X,Y,Z):(%.02f,%.02f,%.02f)(m/s^2), (XG,YG,ZG):(%.02f,%.02f,%.02f)(°/s)\n", data.ax, data.ay, data.az, data.gx, data.gy, data.gz);
        
        // 3. Determine the motion state
        MoveState state = detectMovement(data);
        if (state != lastState) {
            lastState = state;
            continue;
        }
        lastState = state;
        msg_t msg;

        // 4. notify the LED thread to display the light color through a message.
        switch (state) {
            case Stationary:
                msg.type = LED_MSG_TYPE_NONE;
                break;
            case Tilted:
                msg.type = LED_MSG_TYPE_RED;
                break;
            case Rotating:
                msg.type = LED_MSG_TYPE_BLUE;
                break;
            case Moving:
                msg.type = LED_MSG_TYPE_GREEN;
                break;
            case MovingX:
                msg.type = LED_MSG_TYPE_YELLOW;
                break;
            case MovingY:
                msg.type = LED_MSG_TYPE_MAGENTA;
                break;
            default:
                break;
        }
        if (msg_send(&msg, _led_pid) <= 0){
            printf("[IMU_THREAD]: possibly lost interrupt.\n");
        }
        else{
            printf("[IMU_THREAD]: Successfully set interrupt.\n");
        }
    }
    return NULL;
}
static const shell_command_t shell_commands[] = {
    { NULL, NULL, NULL }
};

int main(void)
{
    _led_pid = thread_create(stack_for_led_thread, sizeof(stack_for_led_thread), THREAD_PRIORITY_MAIN - 2,
                            THREAD_CREATE_STACKTEST, _led_thread, NULL,
                            "led_controller_thread");
    if (_led_pid <= KERNEL_PID_UNDEF) {
        printf("[MAIN] Creation of receiver thread failed\n");
        return 1;
    }
    thread_create(stack_for_imu_thread, sizeof(stack_for_imu_thread), THREAD_PRIORITY_MAIN - 1,
                            THREAD_CREATE_STACKTEST, _imu_thread, NULL,
                            "imu_read_thread");
    printf("[Main] Initialization successful - starting the shell now\n");

    while(1)
    {
        
    }
    return 0;
}
