#include "mqtt_thingsboard.hh"
#include <iostream>

MQTT_Thingsboard::MQTT_Thingsboard(MQTTClient *client, Network *network){
    client_ = client;
    network_ = network;
}
/**
 * 1. Network connect: int NetworkConnect(Network* n, char* addr, int port);
 * 2. MQTT Connect: int MQTTConnect(MQTTClient* c, MQTTPacket_connectData* options);
 * * if mqtt connect fail: disconnect network; 
*/
int MQTT_Thingsboard::mqtt_connect(MQTTPacket_connectData &data, std::string remote_ip, int port){
    // Your code here.
    if (client_->isconnected){
        mqtt_disconnect();
    }

    NetworkConnect(network_, (char *)remote_ip.c_str(), port);
    
    int ret = MQTTConnect(client_, &data);
    if (ret < 0) {
        std::cout << "mqtt_example: Unable to connect client " << ret << std::endl;
        mqtt_disconnect();
        return ret;
    }
    std::cout << "mqtt_example: Connection successfully" << std::endl;
    return ret;
}

/**
 * 1. MQTT disconnect: int MQTTDisconnect(MQTTClient* c);
 * 2. Network disconnect: void NetworkDisconnect(Network *n);
*/
int MQTT_Thingsboard::mqtt_disconnect(){
    // Your code here.
    int ret = MQTTDisconnect(client_);
    if (ret < 0)
        std::cout << "mqtt_example: Unable to disconnect" << std::endl;
    else
        std::cout << "mqtt_example: Disconnect successful" << std::endl;
    
    NetworkDisconnect(network_);
    return ret;
}
/**
 * 1. MQTTPublish: int MQTTPublish(MQTTClient* c, const char* topicName, MQTTMessage* message);
 * 
*/
int MQTT_Thingsboard::mqtt_publish(MQTTMessage &msg, std::string topic){
    // Your code here.
    int ret = MQTTPublish(client_, topic.c_str(), &msg);
    if (ret < 0)
        std::cout << "mqtt_example: Unable to publish (" << ret << ")\n";
    else
        std::cout << "mqtt_example: Message (" << msg.payload << ") has been published to topic "
            << topic << "with QOS " << msg.qos << std::endl; 
    return ret;
}
