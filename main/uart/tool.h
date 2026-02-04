//
// Created by nebula on 2026/2/4.
//

#ifndef MQTT_TCP_TOOL_H
#define MQTT_TCP_TOOL_H

#include <stdint.h>

void uart_tool_init(uint32_t baudrate);
void uart_tool_send(const char* data);
void uart_tool_send_len(const uint8_t* data, uint16_t len);

/* 在 UART 中断或轮询中调用 */
void uart_tool_rx_byte(uint8_t byte);
void uart_rx_task(void* arg);
#endif //MQTT_TCP_TOOL_H
