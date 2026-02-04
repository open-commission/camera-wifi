//
// Created by nebula on 2026/2/4.
//

#include "mqtts.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "tool.h"

static const char* TAG = "MQTT_EXAMPLE";

esp_mqtt_client_handle_t client = NULL;

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    char buf[256];
    client = event->client;
    int msg_id;

    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        uart_tool_send("[MQTT] CONNECTED\r\n");
        break;

    case MQTT_EVENT_DISCONNECTED:
        uart_tool_send("[MQTT] DISCONNECTED\r\n");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        snprintf(buf, sizeof(buf),
                 "[MQTT] SUBSCRIBED, msg_id=%d\r\n",
                 event->msg_id);
        uart_tool_send(buf);
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        snprintf(buf, sizeof(buf),
                 "[MQTT] UNSUBSCRIBED, msg_id=%d\r\n",
                 event->msg_id);
        uart_tool_send(buf);
        break;

    case MQTT_EVENT_PUBLISHED:
        snprintf(buf, sizeof(buf),
                 "[MQTT] PUBLISHED, msg_id=%d\r\n",
                 event->msg_id);
        uart_tool_send(buf);
        break;

    case MQTT_EVENT_DATA:
        uart_tool_send("[MQTT] DATA RECEIVED\r\n");

        snprintf(buf, sizeof(buf),
                 "TOPIC=%.*s\r\n",
                 event->topic_len, event->topic);
        uart_tool_send(buf);

        snprintf(buf, sizeof(buf),
                 "DATA=%.*s\r\n",
                 event->data_len, event->data);
        uart_tool_send(buf);
        break;

    case MQTT_EVENT_ERROR:
        uart_tool_send("[MQTT] ERROR\r\n");
        break;

    default:
        snprintf(buf, sizeof(buf),
                 "[MQTT] OTHER EVENT id=%d\r\n",
                 event->event_id);
        uart_tool_send(buf);
        break;
    }

    return ESP_OK;
}

static void mqtt_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

void mqtt_task(void* param)
{
    uart_tool_send("[APP] Startup..");

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(example_connect());

    mqtt_app_start();
}
