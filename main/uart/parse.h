//
// Created by nebula on 2026/2/4.
//

#ifndef MQTT_TCP_PARSE_H
#define MQTT_TCP_PARSE_H

#include <stdint.h>

typedef enum {
    UART_CMD_NONE = 0,
    UART_CMD_PUB,
    UART_CMD_SUB,
    UART_CMD_UNSUB,
} uart_cmd_t;

typedef struct {
    uart_cmd_t cmd;
    char topic[64];
    char payload[128];   // 仅 pub 使用
} uart_cmd_frame_t;

/* 输入字节 */
void uart_parser_feed(uint8_t byte);

/* 解析完成后的回调（你可在此对接 MQTT） */
void uart_parser_on_frame(uart_cmd_frame_t *frame);

/* 主动清空解析器 */
void uart_parser_reset(void);

#endif //MQTT_TCP_PARSE_H