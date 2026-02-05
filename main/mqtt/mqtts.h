//
// Created by nebula on 2026/2/4.
//

#ifndef MQTT_TCP_MQTTS_H
#define MQTT_TCP_MQTTS_H
#include "mqtt_client.h"

void mqtt_task(void* param);
bool mqtt_is_ready(void);
esp_mqtt_client_handle_t mqtt_get_client(void);

#endif //MQTT_TCP_MQTTS_H
