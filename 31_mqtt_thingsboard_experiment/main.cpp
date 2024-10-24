/*
 * Copyright (C) 2019 Javier FILEIV <javier.fileiv@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file        main.c
 * @brief       Example using MQTT Paho package from RIOT
 *
 * @author      Javier FILEIV <javier.fileiv@gmail.com>
 *
 * @}
 */

#include <cassert>
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
#include "xtimer.h"
#include <string>
#include "ledcontroller.hh"
#include "mpu6050.h"
#include "periph/gpio.h"  
#include "periph_conf.h"
#include "board.h"
#include "clk.h"
#include "msg.h"
extern "C" {
    #include "nimble_riot.h"
    #include "nimble_autoadv.h"
    #include "log.h"
    #include "host/ble_hs.h"
    #include "host/util/util.h"
    #include "host/ble_gatt.h"
    #include "services/gap/ble_svc_gap.h"
    #include "services/gatt/ble_svc_gatt.h"
    #include "lwip/netif.h"

}
using namespace std;
void setup();
int predict(float *imu_data, int data_len, float threashold, int class_num);

#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

#define BUF_SIZE                        1024
#define MQTT_VERSION_v311               4       /* MQTT v3.1.1 version is 4 */
#define COMMAND_TIMEOUT_MS              4000

string DEFAULT_MQTT_CLIENT_ID = "esp32_test";
string DEFAULT_MQTT_USER = "esp32";
string DEFAULT_MQTT_PWD = "esp32";
// Please enter the IP of the computer on which you have ThingsBoard installed.
string DEFAULT_IPV4 = "192.168.0.103";
string DEFAULT_TOPIC = "v1/devices/me/telemetry";



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

static MQTTClient client;
static Network network;
static int topic_cnt = 0;
static int led_state;

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
#define g_acc (9.8)
#define class_num (4)
#define SAMPLES_PER_GESTURE (10)
#define THREAD_STACKSIZE        (THREAD_STACKSIZE_IDLE)
static char stack_for_led_thread[THREAD_STACKSIZE];
static char stack_for_motion_thread[THREAD_STACKSIZE];

static kernel_pid_t _led_pid;
static kernel_pid_t _motion_pid;

// input your code, 自定义想要的UUID
static const ble_uuid128_t gatt_svr_svc_rw_demo_uuid
        = {{128}, {0x15, 0xa3, 0xc7, 0x14, 0x3e, 0x03, 0x3e, 0xa1, 0xff,
                0x48, 0x37, 0xd1, 0xb3, 0x38, 0xce, 0x1b}};

static const ble_uuid128_t gatt_svr_chr_rw_demo_predict_uuid
        = {{128}, {0x11, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

static const ble_uuid128_t gatt_svr_chr_rw_demo_threshold_uuid
        = {{128}, {0x22, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

static const ble_uuid128_t gatt_svr_chr_rw_demo_frequency_uuid
        = {{128}, {0x33, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

static const ble_uuid128_t gatt_svr_chr_rw_demo_led_uuid
        = {{128}, {0x44, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

static const ble_uuid128_t gatt_svr_chr_rw_demo_mqtt_uuid
        = {{128}, {0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

static int gatt_svr_chr_access_rw_demo(
        uint16_t conn_handle, uint16_t attr_handle,
        struct ble_gatt_access_ctxt *ctxt, void *arg);
#define DEMO_BUFFER_SIZE 100
static char rm_demo_threshold_data[DEMO_BUFFER_SIZE] = "This characteristic is read and writeable!";
static char rm_demo_frequency_data[DEMO_BUFFER_SIZE] = "This characteristic is read and writeable!";
static char rm_demo_led_data[DEMO_BUFFER_SIZE] = "\x00\0";
static char rm_demo_mqtt_data[DEMO_BUFFER_SIZE] = "This characteristic is read and writeable!";
#define STR_ANSWER_BUFFER_SIZE 100
static char str_answer[STR_ANSWER_BUFFER_SIZE];

int mqtt_interval_ms = 2000;
struct MPU6050Data
{
    float ax, ay, az; // acceler_x_axis, acceler_y_axis, acceler_z_axis
    float gx, gy, gz; // gyroscope_x_axis, gyroscope_y_axis, gyroscope_z_axis
};
void delay_ms(uint32_t sleep_ms)
{
    ztimer_sleep(ZTIMER_USEC, sleep_ms * US_PER_MS);
    return;
}
static int led_control = 0;
void *_led_thread(void *arg)
{
    (void) arg;
    LEDController led(LED_GPIO_R, LED_GPIO_G, LED_GPIO_B);
    led.change_led_color(0);
    while(1){
        // Wait for a message to control the LED
        // Display different light colors based on the motion state of the device.
        msg_t msg;
        msg_receive(&msg);
        switch (msg.type) {
            case LED_MSG_TYPE_NONE:
                led.change_led_color(0);
                break;
            case LED_MSG_TYPE_RED:
                led.change_led_color(1);
                break;
            case LED_MSG_TYPE_GREEN:
                led.change_led_color(2);
                break;
            case LED_MSG_TYPE_YELLOW:
                led.change_led_color(3);
                break;
            case LED_MSG_TYPE_BLUE:
                led.change_led_color(4);
                break;
            case LED_MSG_TYPE_MAGENTA:
                led.change_led_color(5);
                break;
            case LED_MSG_TYPE_WHITE:
                led.change_led_color(7);
                break;
            default:
                break;
        }
    }
    return NULL;
}

float gyro_fs_convert = 1.0;
float accel_fs_convert;
float avg_ax = 0, avg_ay = 0, avg_az = 0, avg_gx = 0, avg_gy = 0, avg_gz = 0;
static int collect_interval_ms = 20;
static int predict_interval_ms = 200;
void get_imu_data(MPU6050 mpu, float *imu_data){
    int16_t ax, ay, az, gx, gy, gz;
    for(int i = 0; i < SAMPLES_PER_GESTURE; ++i)
    {
        delay_ms(collect_interval_ms);
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        imu_data[i*6 + 0] = ax / accel_fs_convert - avg_ax;
        imu_data[i*6 + 1] = ay / accel_fs_convert - avg_ay;
        imu_data[i*6 + 2] = az / accel_fs_convert - avg_az + g_acc;
        imu_data[i*6 + 3] = gx / gyro_fs_convert - avg_gx;
        imu_data[i*6 + 4] = gy / gyro_fs_convert - avg_gy;
        imu_data[i*6 + 5] = gz / gyro_fs_convert - avg_gz;
    }
}

static int predict_state = 0;
static float threshold = 0.7;
float t_ax = 0, t_ay = 0, t_az = 0, t_gx = 0, t_gy = 0, t_gz = 0;

void *_motion_thread(void *arg)
{
    (void) arg;
    // Initialize MPU6050 sensor
    MPU6050 mpu;
    // get mpu6050 device id
    uint8_t device_id = mpu.getDeviceID();
    printf("[IMU_THREAD] DEVICE_ID:0x%x\n", device_id);
    mpu.initialize();
    // Configure gyroscope and accelerometer full scale ranges
    uint8_t gyro_fs = mpu.getFullScaleGyroRange();
    uint8_t accel_fs_g = mpu.getFullScaleAccelRange();
    uint16_t accel_fs_real = 1;

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
    accel_fs_convert = 32768.0 / accel_fs_real;
    float imu_data[SAMPLES_PER_GESTURE * 6] = {0};
    int data_len = SAMPLES_PER_GESTURE * 6;
    int16_t ax, ay, az, gx, gy, gz;
    // 花5s调零
    for (int i = 0; i < 100; i++) {
        msg_t msg;
        msg.type = LED_MSG_TYPE_WHITE;
        if (msg_send(&msg, _led_pid) <= 0){
            printf("[IMU_THREAD]: possibly lost interrupt.\n");
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

    delay_ms(200);
    // Main loop
    int ret = 0;
#define class_num (4)
    string motions[class_num] = {"Stationary", "Tilted", "Rotating", "Moving"};
    while (1) {
        delay_ms(predict_interval_ms);    
        // Read sensor data
        get_imu_data(mpu, imu_data);
        ret = predict(imu_data, data_len, threshold, class_num);
        t_ax = imu_data[0]; t_ay = imu_data[1]; t_az = imu_data[2];
        t_gx = imu_data[3]; t_gy = imu_data[4]; t_gz = imu_data[5];
        // tell the led thread to do some operations
        if (led_control > 0) {
            ret = led_control;
        }
        led_state = ret;
        msg_t msg;
        switch (ret) {
            case 0:
                msg.type = LED_MSG_TYPE_NONE;
                break;
            case 1:
                msg.type = LED_MSG_TYPE_RED;
                break;
            case 2:
                msg.type = LED_MSG_TYPE_BLUE;
                break;
            case 3:
                msg.type = LED_MSG_TYPE_GREEN;
                break;
            default:
                break;
        }
        if (msg_send(&msg, _led_pid) <= 0){
            printf("[IMU_THREAD]: possibly lost interrupt.\n");
        }
        // Print result
        // printf("Predict: %d, %s\n", ret, motions[ret].c_str());
        // save motion for gatt server
        predict_state = ret;
    }
    return NULL;
}


/* define several bluetooth services for our device */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    /*
     * access_cb defines a callback for read and write access events on
     * given characteristics
     */
    {
        /* Service: Read/Write Demo */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (ble_uuid_t*) &gatt_svr_svc_rw_demo_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) { {
            /* Characteristic: Read Demo predict */
            .uuid = (ble_uuid_t*) &gatt_svr_chr_rw_demo_predict_uuid.u,
            .access_cb = gatt_svr_chr_access_rw_demo,
            .flags = BLE_GATT_CHR_F_READ,
        }, {
            /* Characteristic: Read/Write Demo threshold */
            .uuid = (ble_uuid_t*) &gatt_svr_chr_rw_demo_threshold_uuid.u,
            .access_cb = gatt_svr_chr_access_rw_demo,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
        }, {
            /* Characteristic: Read/Write Demo frequency */
            .uuid = (ble_uuid_t*) &gatt_svr_chr_rw_demo_frequency_uuid.u,
            .access_cb = gatt_svr_chr_access_rw_demo,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
        }, {
            /* Characteristic: Read/Write Demo led */
            .uuid = (ble_uuid_t*) &gatt_svr_chr_rw_demo_led_uuid.u,
            .access_cb = gatt_svr_chr_access_rw_demo,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
        }, {
            /* Characteristic: Read/Write Demo mqtt */
            .uuid = (ble_uuid_t*) &gatt_svr_chr_rw_demo_mqtt_uuid.u,
            .access_cb = gatt_svr_chr_access_rw_demo,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
        }, {
            0, /* No more characteristics in this service */
        }, }
    },
    {
        0, /* No more services */
    },
};

static int gatt_svr_chr_access_rw_demo(
        uint16_t conn_handle, uint16_t attr_handle,
        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    printf("service 'rw demo' callback triggered\n");
    (void) conn_handle;
    (void) attr_handle;
    (void) arg;

    int rc = 0;
    ble_uuid_t* predict_uuid = (ble_uuid_t*) &gatt_svr_chr_rw_demo_predict_uuid.u;
    ble_uuid_t* threshold_uuid = (ble_uuid_t*) &gatt_svr_chr_rw_demo_threshold_uuid.u;
    ble_uuid_t* frequency_uuid = (ble_uuid_t*) &gatt_svr_chr_rw_demo_frequency_uuid.u;
    ble_uuid_t* led_uuid = (ble_uuid_t*) &gatt_svr_chr_rw_demo_led_uuid.u;
    ble_uuid_t* mqtt_uuid = (ble_uuid_t*) &gatt_svr_chr_rw_demo_mqtt_uuid.u;

    if (ble_uuid_cmp(ctxt->chr->uuid, predict_uuid) == 0) {
        printf("access to characteristic 'rw demo (predict)'\n");
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            string motions[class_num] = {"Stationary", "Tilted", "Rotating", "Moving"};
            snprintf(str_answer, STR_ANSWER_BUFFER_SIZE, "PREDICT STATE: %s", motions[predict_state].c_str());
            printf("%s\n", str_answer);
            rc = os_mbuf_append(ctxt->om, &str_answer, strlen(str_answer));
            return rc;
        }
        return 0;
    }
    else if (ble_uuid_cmp(ctxt->chr->uuid, threshold_uuid) == 0) {
        printf("access to characteristic 'rw demo (threshold)'\n");
        switch (ctxt->op) {
            case BLE_GATT_ACCESS_OP_READ_CHR:
                printf("read from characteristic\n");
                printf("current value of threshold: '%.2f'\n", threshold);
                snprintf(str_answer, STR_ANSWER_BUFFER_SIZE, "THRESHOLD: %.2f", threshold);
                printf("%s\n", str_answer);
                rc = os_mbuf_append(ctxt->om, &str_answer, strlen(str_answer));
                break;
            case BLE_GATT_ACCESS_OP_WRITE_CHR:
                printf("write to characteristic\n");
                printf("old value of threshold: %.2f\n", threshold);
                uint16_t om_len;
                om_len = OS_MBUF_PKTLEN(ctxt->om);
                rc = ble_hs_mbuf_to_flat(ctxt->om, &rm_demo_threshold_data, sizeof(rm_demo_threshold_data), &om_len);
                rm_demo_threshold_data[om_len] = '\0';
                // 将读到的字符数据转换为浮点数
                threshold = atof(rm_demo_threshold_data) / 100;
                printf("new value of threshold: %.2f\n", threshold);
                break;
            default:
                printf("unhandled operation!\n");
                rc = 1;
                break;
        }
        return rc;
    }
    else if (ble_uuid_cmp(ctxt->chr->uuid, frequency_uuid) == 0) {
        printf("access to characteristic 'rw demo (frequency)'\n");
        switch (ctxt->op) {
            case BLE_GATT_ACCESS_OP_READ_CHR:
                printf("read from characteristic\n");
                printf("current value of frequency: %dms\n", collect_interval_ms);
                snprintf(str_answer, STR_ANSWER_BUFFER_SIZE, "FREQUENCY: %dms", collect_interval_ms);
                printf("%s\n", str_answer);
                rc = os_mbuf_append(ctxt->om, &str_answer, strlen(str_answer));
                break;
            case BLE_GATT_ACCESS_OP_WRITE_CHR:
                printf("write to characteristic\n");
                printf("old value of frequency: %dms\n", collect_interval_ms);
                uint16_t om_len;
                om_len = OS_MBUF_PKTLEN(ctxt->om);
                rc = ble_hs_mbuf_to_flat(ctxt->om, &rm_demo_frequency_data, sizeof(rm_demo_frequency_data), &om_len);
                rm_demo_frequency_data[om_len] = '\0';
                collect_interval_ms = atoi(rm_demo_frequency_data);
                printf("new value of frequency: %dms\n", collect_interval_ms);
                break;
            default:
                printf("unhandled operation!\n");
                rc = 1;
                break;
        }
        return rc;
    }
    else if (ble_uuid_cmp(ctxt->chr->uuid, led_uuid) == 0) {
        printf("access to characteristic 'rw demo (led)'\n");
        switch (ctxt->op) {
            case BLE_GATT_ACCESS_OP_READ_CHR:
                printf("read from characteristic\n");
                printf("current value of rm_demo_led_data: '%d'\n", led_control);
                rc = os_mbuf_append(ctxt->om, &rm_demo_led_data, strlen(rm_demo_led_data));
                break;
            case BLE_GATT_ACCESS_OP_WRITE_CHR:
                printf("write to characteristic\n");
                printf("old value of rm_demo_write_data: '%d'\n", led_control);
                uint16_t om_len;
                om_len = OS_MBUF_PKTLEN(ctxt->om);
                rc = ble_hs_mbuf_to_flat(ctxt->om, &rm_demo_led_data, sizeof(rm_demo_led_data), &om_len);
                rm_demo_led_data[om_len] = '\0';
                switch (rm_demo_led_data[0]) {
                    case 0x00:
                        led_control = 0;
                        break;
                    case 0x01:
                        led_control = 1;
                        break;
                    case 0x02:
                        led_control = 2;
                        break;
                    case 0x03:
                        led_control = 3;
                        break;
                    default:
                        break;
                }
                printf("new value of rm_demo_write_data: '%d'\n", led_control);
                break;
            default:
                printf("unhandled operation!\n");
                rc = 1;
                break;
        }
        return rc;
    }
    else if (ble_uuid_cmp(ctxt->chr->uuid, mqtt_uuid) == 0) {
        printf("access to characteristic 'rw demo (mqtt)'\n");
        switch (ctxt->op) {
            case BLE_GATT_ACCESS_OP_READ_CHR:
                printf("read from characteristic\n");
                printf("current value of mqtt: %dms\n", mqtt_interval_ms);
                snprintf(str_answer, STR_ANSWER_BUFFER_SIZE, "MQTT_INTERVAL: %dms", mqtt_interval_ms);
                printf("%s\n", str_answer);
                rc = os_mbuf_append(ctxt->om, &str_answer, strlen(str_answer));
                break;
            case BLE_GATT_ACCESS_OP_WRITE_CHR:
                printf("write to characteristic\n");
                printf("old value of mqtt: %dms\n", mqtt_interval_ms);
                uint16_t om_len;
                om_len = OS_MBUF_PKTLEN(ctxt->om);
                rc = ble_hs_mbuf_to_flat(ctxt->om, &rm_demo_mqtt_data, sizeof(rm_demo_mqtt_data), &om_len);
                rm_demo_mqtt_data[om_len] = '\0';
                mqtt_interval_ms = atoi(rm_demo_mqtt_data);
                printf("new value of mqtt: %dms\n", mqtt_interval_ms);
                break;
            default:
                printf("unhandled operation!\n");
                rc = 1;
                break;
        }
        return rc;
    }
    printf("unhandled uuid!\n");
    return 1;
}

int mqtt_disconnect(void)
{
    topic_cnt = 0;
    int res = MQTTDisconnect(&client);
    if (res < 0) {
        printf("mqtt_example: Unable to disconnect\n");
    }
    else {
        printf("mqtt_example: Disconnect successful\n");
    }

    NetworkDisconnect(&network);
    return res;
}

int mqtt_connect(void)
{
    const char *remote_ip;
    remote_ip = DEFAULT_IPV4.c_str();
    if (client.isconnected) {
        printf("mqtt_example: client already connected, disconnecting it\n");
        MQTTDisconnect(&client);
        NetworkDisconnect(&network);
    }
    int port = DEFAULT_MQTT_PORT;

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = MQTT_VERSION_v311;

    data.clientID.cstring = (char *)DEFAULT_MQTT_CLIENT_ID.c_str();
    data.username.cstring = (char *)DEFAULT_MQTT_USER.c_str();
    data.password.cstring = (char *)DEFAULT_MQTT_PWD.c_str();
    data.keepAliveInterval = DEFAULT_KEEPALIVE_SEC;
    data.cleansession = IS_CLEAN_SESSION;
    data.willFlag = 0;

    NetworkConnect(&network, (char *)remote_ip, port);
    int ret = MQTTConnect(&client, &data);
    if (ret < 0) {
        printf("mqtt_example: Unable to connect client %d\n", ret);
        mqtt_disconnect();
        return ret;
    }
    else {
        printf("mqtt_example: Connection successfully\n");
    }

    return (ret > 0) ? 0 : 1;
}


int mqtt_pub(void)
{
    enum QoS qos = QOS0;

    MQTTMessage message;
    message.qos = qos;
    message.retained = IS_RETAINED_MSG;
    char json[500];  
    // 一: 6轴数据， 二: 设备姿态，三: LED灯R，G，B状态
    string motions[class_num] = {"Stationary", "Tilted", "Rotating", "Moving"};
    int R = 0, G = 0, B = 0;
    switch (led_state) {
        case 1: R = 1; break;
        case 2: B = 1; break;
        case 3: G = 1; break;
        default: break;
    }
    snprintf(json, 500, "{ax:%.2f, ay:%.2f, az:%.2f, gx:%.2f, gy:%.2f, gz:%.2f, predict:%s, led_state:%d, led_R:%d, led_G:%d, led_B:%d, mqtt_interval_ms:%d}", t_ax, t_ay, t_az, t_gx, t_gy, t_gz, motions[led_state].c_str(), led_state, R, G, B, mqtt_interval_ms);
    printf("[Send] Message:%s\n", json);
    message.payload = json;
    message.payloadlen = strlen((char *)message.payload);

    int rc;
    if ((rc = MQTTPublish(&client, DEFAULT_TOPIC.c_str(), &message)) < 0) {
        printf("mqtt_example: Unable to publish (%d)\n", rc);
    }
    else {
        printf("mqtt_example: Message (%s) has been published to topic %s"
               "with QOS %d\n",
               (char *)message.payload, DEFAULT_TOPIC.c_str(), (int)message.qos);
    }

    return rc;
}

void send(void)
{
    mqtt_connect();
    mqtt_pub();
    mqtt_disconnect();
}

static unsigned char buf[BUF_SIZE];
static unsigned char readbuf[BUF_SIZE];

int main(void)
{
    if (IS_USED(MODULE_GNRC_ICMPV6_ECHO)) {
        msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    }
#ifdef MODULE_LWIP
    /* let LWIP initialize */
    delay_ms(10);
#endif

    NetworkInit(&network);
    MQTTClientInit(&client, &network, COMMAND_TIMEOUT_MS, buf, BUF_SIZE,
                   readbuf,
                   BUF_SIZE);
    printf("Running mqtt paho example. Type help for commands info\n");

    MQTTStartTask(&client);
    // create led thread
    _led_pid = thread_create(stack_for_led_thread, sizeof(stack_for_led_thread), THREAD_PRIORITY_MAIN - 2,
                            THREAD_CREATE_STACKTEST, _led_thread, NULL,
                            "led");
    if (_led_pid <= KERNEL_PID_UNDEF) {
        printf("[MAIN] Creation of receiver thread failed\n");
        return 1;
    }
    else
    {
        printf("[MAIN] LED_PID: %d\n", _led_pid);
    }
    setup();
    // create motion thread
    _motion_pid = thread_create(stack_for_motion_thread, sizeof(stack_for_motion_thread), THREAD_PRIORITY_MAIN - 2,
                            THREAD_CREATE_STACKTEST, _motion_thread, NULL,
                            "motion_predict_thread");
    if (_motion_pid <= KERNEL_PID_UNDEF) {
        printf("[MAIN] Creation of receiver thread failed\n");
        return 1;
    }
    else
    {
        printf("[MAIN] MOTION_PID: %d\n", _motion_pid);
    }


    int rc = 0;
    (void)rc;

    /* verify and add our custom services */
    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    assert(rc == 0);

    /* set the device name */
    ble_svc_gap_device_name_set(CONFIG_NIMBLE_AUTOADV_DEVICE_NAME);
    /* reload the GATT server to link our added services */
    ble_gatts_start();

    // 获取蓝牙设备的默认 MAC 地址
    uint8_t own_addr_type;
    uint8_t own_addr[6];
    ble_hs_id_infer_auto(0, &own_addr_type);
    ble_hs_id_copy_addr(own_addr_type, own_addr, NULL);

    // 打印 MAC 地址
    LOG_INFO("Default MAC address: %02X:%02X:%02X:%02X:%02X:%02X\n",
             own_addr[5], own_addr[4], own_addr[3],
             own_addr[2], own_addr[1], own_addr[0]);
    /* start to advertise this node */
    nimble_autoadv_start(NULL);

    // waiting for get IP 
    extern struct netif *netif_default;
    uint32_t addr;
    do
    {
        addr = netif_ip_addr4(netif_default)->addr;
        printf("Waiting for getting IP, current IP addr:");
        ip_addr_debug_print(LWIP_DBG_ON, netif_ip_addr4(netif_default));
        printf("\n");
        delay_ms(1000);
    }while (addr == 0x0);


    while (1)
    {
        send();
        delay_ms(mqtt_interval_ms);
    }
    
    return 0;
}
