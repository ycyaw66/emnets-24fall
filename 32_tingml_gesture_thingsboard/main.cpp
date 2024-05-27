/*
 * Copyright (C) 2019 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 *
 * @file
 * @brief       TensorFlow Lite test application
 *
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 */

/* Provide an Arduino-Like API to be able to easily reuse the code from
   upstream examples */
#include "ledcontroller.hh"
#include "mqtt_thingsboard.hh"
#include "mpu6050.h"
#include "periph_conf.h"
#include "periph/gpio.h"
#include "periph/i2c.h"
#include "shell.h"

#include <log.h>
#include <xtimer.h>

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "timex.h"
#include "ztimer.h"
#include "shell.h"
#include "thread.h"
#include "mutex.h"
#include "paho_mqtt.h"
#include "MQTTClient.h"
#include <string>


using namespace std;

#define BUF_SIZE                        1024
#define MQTT_VERSION_v311               4       /* MQTT v3.1.1 version is 4 */
#define COMMAND_TIMEOUT_MS              4000
/**
 * @brief Default MQTT port
 */
#define DEFAULT_MQTT_PORT               1883

/**
 * @brief Keepalive timeout in seconds
 */
#define DEFAULT_KEEPALIVE_SEC           10

#ifndef MAX_LEN_TOPIC
#define MAX_LEN_TOPIC                   100
#endif

#ifndef MAX_TOPICS
#define MAX_TOPICS                      4
#endif

#define IS_CLEAN_SESSION                1
#define IS_RETAINED_MSG                 0
MQTTPacket_connectData data;
static MQTTClient client;
static Network network;
static MQTT_Thingsboard mqtt_thingsboard{&client, &network};
void setup();
int predict(float *imu_data, int data_len);
#define THREAD_STACKSIZE        (THREAD_STACKSIZE_IDLE)
static unsigned char buf[BUF_SIZE];
static unsigned char readbuf[BUF_SIZE];
static char stack_publish[THREAD_STACKSIZE * 2];
static char stack_predict[THREAD_STACKSIZE];

#define VCC_GPIO GPIO23
#define g_acc (9.8)
#define SAMPLES_PER_GESTURE (50)
MPU6050 mpu;
LEDController led(GPIO12);
float gyro_fs_convert = 1.0;
float accel_fs_convert;
float imu_data[SAMPLES_PER_GESTURE * 6] = {0};
int data_len = SAMPLES_PER_GESTURE * 6;
struct Mpu_data {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    uint8_t s_led_state;
    uint8_t motion;
};
Mpu_data mpu_data; 
string DEFAULT_MQTT_CLIENT_ID = "esp32_test";
string DEFAULT_MQTT_USER = "esp32";
string DEFAULT_MQTT_PWD = "esp32";
string DEFAULT_IPV6 = "fe80::125:3159:d478:19e9";
string DEFAULT_IPV4 = "192.168.31.229";
string DEFAULT_TOPIC = "v1/devices/me/telemetry";
void init_mpu(void){
    uint8_t device_id = mpu.getDeviceID();
    uint16_t rate = mpu.getRate();
    mpu.initialize();
    mpu.setTempSensorEnabled(false);
    uint8_t gyro_fs = mpu.getFullScaleGyroRange();
    uint8_t accel_fs_g = mpu.getFullScaleAccelRange();
    if (gyro_fs == MPU6050_GYRO_FS_250)
          // gyro_fs_real = 250;
          gyro_fs_convert = 131.0;
      else if (gyro_fs == MPU6050_GYRO_FS_500)
          // gyro_fs_real = 500;
          gyro_fs_convert = 65.5;
      else if (gyro_fs == MPU6050_GYRO_FS_1000)
          // gyro_fs_real = 1000;
          gyro_fs_convert = 32.8;
      else if (gyro_fs == MPU6050_GYRO_FS_2000)
          // gyro_fs_real = 2000;
          gyro_fs_convert = 16.4;
      else
          LOG_ERROR("[main] Unknown GYRO_FS: 0x%x\n", gyro_fs);
    uint16_t accel_fs_real = 1;

    if (accel_fs_g == MPU6050_ACCEL_FS_2)
        accel_fs_real = g_acc * 2;
    else if (accel_fs_g == MPU6050_ACCEL_FS_4)
        accel_fs_real = g_acc * 4;
    else if (accel_fs_g == MPU6050_ACCEL_FS_8)
        accel_fs_real = g_acc * 8;
    else if (accel_fs_g == MPU6050_ACCEL_FS_16)
        accel_fs_real = g_acc * 16;
    else
        LOG_ERROR("[main] Unknown ACCEL_FS: 0x%x\n", accel_fs_g);
    accel_fs_convert = 32768.0 / accel_fs_real;
    LOG_INFO("[main]DEVICE_ID:0x%x, RATE:0x%x\n", device_id, rate);
    uint8_t id = getDeviceID();
    printf("addr:[%x]\n", id);
}
void get_imu_data(void){
    int16_t ax, ay, az, gx, gy, gz;
    for(int i = 0; i < SAMPLES_PER_GESTURE; ++i)
    {
        i += 1;
        /* code */
        xtimer_msleep(20);
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        imu_data[i*6 + 0] = ax / accel_fs_convert;
        imu_data[i*6 + 1] = ay / accel_fs_convert;
        imu_data[i*6 + 2] = az / accel_fs_convert;
        imu_data[i*6 + 3] = gx / gyro_fs_convert;
        imu_data[i*6 + 4] = gy / gyro_fs_convert;
        imu_data[i*6 + 5] = gz / gyro_fs_convert;
    }
    mpu_data.ax = ax / accel_fs_convert;
    mpu_data.ay = ay / accel_fs_convert;
    mpu_data.az = az / accel_fs_convert;
    mpu_data.gx = gx / accel_fs_convert;
    mpu_data.gy = gy / accel_fs_convert;
    mpu_data.gz = gz / accel_fs_convert;
    

} 
static kernel_pid_t _thingsboard_pid;

void *_publish_thread(void *arg)
{
    // Your code here.
    (void) arg;
    MPU6050 mpu;
    MQTTMessage msg;
    LEDController led_controller(GPIO12);
    mpu.initialize();
    xtimer_msleep(2000);
    data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = MQTT_VERSION_v311;
    data.clientID.cstring = (char *)DEFAULT_MQTT_CLIENT_ID.c_str();
    data.username.cstring = (char *)DEFAULT_MQTT_USER.c_str();
    data.password.cstring = (char *)DEFAULT_MQTT_PWD.c_str();
    data.keepAliveInterval = DEFAULT_KEEPALIVE_SEC;
    data.cleansession = IS_CLEAN_SESSION;
    data.willFlag = 0;
    char json[200];  
    while(1){
        msg_t msg1;
        msg_receive(&msg1);
        mpu_data.motion = msg1.content.value;
        sprintf(json, "{ax:%d, ay:%d, az:%d, gx:%d, gy:%d, gz:%d, motion:%d}", mpu_data.ax, mpu_data.ay, mpu_data.az, mpu_data.gx, mpu_data.gy, mpu_data.gz, mpu_data.motion);
        msg.qos = QOS0;
        msg.retained = IS_RETAINED_MSG;
        msg.payload = json;
        msg.payloadlen = strlen((char *)msg.payload);
        mqtt_thingsboard.mqtt_connect(data, DEFAULT_IPV4.c_str(), DEFAULT_MQTT_PORT);
        mqtt_thingsboard.mqtt_publish(msg, DEFAULT_TOPIC.c_str());
    }
    return NULL;
}

void *_predict_thread(void *arg) {
    (void) arg;
    setup();
    msg_t msg;
    int res;
    while(true) {
        get_imu_data();
        res = predict(imu_data, data_len);
        LOG_INFO("RES: %d\n", res);
        msg.content.value = res;
        if (msg_send(&msg, _thingsboard_pid) <= 0)
            printf("[PREDICT_THREAD]: possibly lost interrupt.\n");
    }
    return NULL;
}


static const shell_command_t shell_commands[] = {
    { NULL, NULL, NULL }
};
int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;
    gpio_init(VCC_GPIO, GPIO_OUT);
    gpio_set(VCC_GPIO);
    gpio_init(GPIO14, GPIO_OUT);
    gpio_set(GPIO14);
    xtimer_msleep(1000);
    init_mpu();  
    setup();
    NetworkInit(&network);

    MQTTClientInit(&client, &network, COMMAND_TIMEOUT_MS, buf, BUF_SIZE,
                   readbuf,
                   BUF_SIZE);

    MQTTStartTask(&client);
    xtimer_msleep(2000);
      /* code */
    _thingsboard_pid = thread_create(stack_publish, sizeof(stack_publish), THREAD_PRIORITY_MAIN, THREAD_CREATE_STACKTEST, 
        _publish_thread, NULL, "MQTT_Thingsboard_Publish");
    thread_create(stack_predict, sizeof(stack_predict), THREAD_PRIORITY_MAIN, THREAD_CREATE_STACKTEST, 
        _predict_thread, NULL, "PREDICT");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
    return 0;
}
