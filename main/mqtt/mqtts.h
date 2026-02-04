//
// Created by nebula on 2026/2/4.
//

#ifndef MQTT_TCP_MQTTS_H
#define MQTT_TCP_MQTTS_H
#include "mqtt_client.h"

extern esp_mqtt_client_handle_t client;
void mqtt_task(void* param);
#endif //MQTT_TCP_MQTTS_H
