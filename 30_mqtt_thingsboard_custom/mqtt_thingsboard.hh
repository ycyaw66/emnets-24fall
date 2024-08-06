#pragma once
#include "paho_mqtt.h"
#include "MQTTClient.h"
#include <string>
class MQTT_Thingsboard {
    public:
        explicit MQTT_Thingsboard(MQTTClient *client, Network *network);
        int mqtt_connect(MQTTPacket_connectData &data, std::string remote_ip, int port);
        int mqtt_publish(MQTTMessage &msg, std::string topic);
        int mqtt_disconnect();
    private:
        MQTTClient *client_;
        Network *network_;
};