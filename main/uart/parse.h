//
// Created by nebula on 2026/2/4.
//

#ifndef MQTT_TCP_PARSE_H
#define MQTT_TCP_PARSE_H

#pragma once

#include <stdint.h>

typedef enum {
    UART_CMD_NONE = 0,
    UART_CMD_PUB,
    UART_CMD_SUB,
    UART_CMD_UNSUB,
} uart_cmd_t;

typedef struct {
    uart_cmd_t cmd;
    char topic[128];
    char payload[256];
} uart_cmd_frame_t;

/* UART 字节喂入接口 */
void uart_parser_feed(uint8_t byte);

/* 解析状态清空 */
void uart_parser_reset(void);

/* 解析完成回调（你已有 MQTT 逻辑） */
void uart_parser_on_frame(uart_cmd_frame_t* frame);


#endif //MQTT_TCP_PARSE_H