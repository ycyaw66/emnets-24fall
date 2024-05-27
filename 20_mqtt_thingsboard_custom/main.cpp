#include <cstdio>
#include <cstring>
#include <cstdbool>
#include <string>

#include "timex.h"
#include "xtimer.h"
#include "ztimer.h"
#include "shell.h"
#include "thread.h"
#include "mutex.h"
#include "paho_mqtt.h"
#include "MQTTClient.h"
#include "periph/gpio.h"  
#include "periph_conf.h"

#include "mpu6050.h"

#include "ledcontroller.hh"
#include "mqtt_thingsboard.hh"

using namespace std;
// #define THREAD_STACKSIZE        (THREAD_STACKSIZE_IDLE)
// static char stack[THREAD_STACKSIZE];
#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

#define BUF_SIZE                        1024
#define MQTT_VERSION_v311               4       /* MQTT v3.1.1 version is 4 */
#define COMMAND_TIMEOUT_MS              4000
#define DEFAULT_MQTT_PORT               1883
#define DEFAULT_KEEPALIVE_SEC           10

#ifndef MAX_LEN_TOPIC
#define MAX_LEN_TOPIC                   100
#endif

#ifndef MAX_TOPICS
#define MAX_TOPICS                      4
#endif

#define IS_CLEAN_SESSION                1
#define IS_RETAINED_MSG                 0

string DEFAULT_MQTT_CLIENT_ID = "esp32_test";
string DEFAULT_MQTT_USER = "esp32";
string DEFAULT_MQTT_PWD = "esp32";
string DEFAULT_IPV6 = "fe80::125:3159:d478:19e9";
string DEFAULT_IPV4 = "192.168.31.229";
string DEFAULT_TOPIC = "v1/devices/me/telemetry";

static MQTTClient client;
static Network network;

// static int topic_cnt = 0;
// static char _topic_to_subscribe[MAX_TOPICS][MAX_LEN_TOPIC];

struct Mpu_data {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    uint8_t s_led_state;
};

void publish_thingsboard(MQTT_Thingsboard &mqtt_thingsboard)
{
    // Your code here.
    MPU6050 mpu;
    Mpu_data mpu_data; 
    MQTTMessage msg;
    LEDController led_controller(GPIO12);
    mpu.initialize();
    xtimer_msleep(10000);
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = MQTT_VERSION_v311;

    data.clientID.cstring = (char *)DEFAULT_MQTT_CLIENT_ID.c_str();
    data.username.cstring = (char *)DEFAULT_MQTT_USER.c_str();
    data.password.cstring = (char *)DEFAULT_MQTT_PWD.c_str();
    data.keepAliveInterval = DEFAULT_KEEPALIVE_SEC;
    data.cleansession = IS_CLEAN_SESSION;
    data.willFlag = 0;
    while(1){
        mpu.getMotion6(&mpu_data.ax, &mpu_data.ay, &mpu_data.az, &mpu_data.gx, &mpu_data.gy, &mpu_data.gz);
        mpu_data.s_led_state = led_controller.led_state;
        char json[200];  
        sprintf(json, "{ax:%d, ay:%d, az:%d, gx:%d, gy:%d, gz:%d, led_state:%d}", mpu_data.ax, mpu_data.ay, mpu_data.az, mpu_data.gx, mpu_data.gy, mpu_data.gz, mpu_data.s_led_state);
        printf("{ax:%d, ay:%d, az:%d, gx:%d, gy:%d, gz:%d, led_state:%d}\n", mpu_data.ax, mpu_data.ay, mpu_data.az, mpu_data.gx, mpu_data.gy, mpu_data.gz, mpu_data.s_led_state);
        msg.qos = QOS0;
        msg.retained = IS_RETAINED_MSG;
        msg.payload = json;
        msg.payloadlen = strlen((char *)msg.payload);
        mqtt_thingsboard.mqtt_connect(data, DEFAULT_IPV4.c_str(), DEFAULT_MQTT_PORT);
        mqtt_thingsboard.mqtt_publish(msg, DEFAULT_TOPIC.c_str());
        // ztimer_sleep(ZTIMER_MSEC, 500 * US_PER_MS);
        xtimer_msleep(500);
    }
}


static unsigned char buf[BUF_SIZE];
static unsigned char readbuf[BUF_SIZE];

static const shell_command_t shell_commands[] = {
    { NULL, NULL, NULL }
};
int main(void)
{
    if (IS_USED(MODULE_GNRC_ICMPV6_ECHO)) {
        msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    }
#ifdef MODULE_LWIP
    /* let LWIP initialize */
    ztimer_sleep(ZTIMER_MSEC, 1 * MS_PER_SEC);
#endif

    NetworkInit(&network);

    MQTTClientInit(&client, &network, COMMAND_TIMEOUT_MS, buf, BUF_SIZE,
                   readbuf,
                   BUF_SIZE);

    MQTTStartTask(&client);
    // thread_create(stack, sizeof(stack), THREAD_PRIORITY_MAIN - 3, THREAD_CREATE_STACKTEST, 
    //     _publish_thread, NULL, "MQTT_Thingsboard_Publish");
    static MQTT_Thingsboard mqtt_thingsboard{&client, &network};
    publish_thingsboard(mqtt_thingsboard);   
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
       
    return 0;
}
