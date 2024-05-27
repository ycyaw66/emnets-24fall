
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
#include <xtimer.h>
#include <string>
using namespace std;

#define BUF_SIZE                        1024
#define MQTT_VERSION_v311               4       /* MQTT v3.1.1 version is 4 */
#define COMMAND_TIMEOUT_MS              4000

string DEFAULT_MQTT_CLIENT_ID = "esp32_test";
string DEFAULT_MQTT_USER = "esp32";
string DEFAULT_MQTT_PWD = "esp32";
string DEFAULT_IPV6 = "fe80::125:3159:d478:19e9";
string DEFAULT_IPV4 = "192.168.31.229";
string DEFAULT_TOPIC = "v1/devices/me/telemetry";
