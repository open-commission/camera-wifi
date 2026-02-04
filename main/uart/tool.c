//
// Created by nebula on 2026/2/4
//

#include "tool.h"

#include <string.h>

#include "parse.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"

#define UART_PORT      UART_NUM_0
#define UART_RX_BUF    256

/* 一键初始化 UART0（RTOS 正确方式） */
void uart_tool_init(uint32_t baudrate)
{
    uart_config_t cfg = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(UART_PORT, &cfg);

    uart_driver_install(
        UART_PORT,
        UART_RX_BUF, // RX buffer
        0, // TX buffer（不用）
        0,
        NULL,
        0
    );
}

/* 发送字符串 */
void uart_tool_send(const char* data)
{
    uart_write_bytes(UART_PORT, data, strlen(data));
}

/* 发送指定长度 */
void uart_tool_send_len(const uint8_t* data, uint16_t len)
{
    uart_write_bytes(UART_PORT, (const char*)data, len);
}

/* UART RX task（代替中断） */
void uart_rx_task(void* arg)
{
    uint8_t buf[64];

    while (1)
    {
        int len = uart_read_bytes(
            UART_PORT,
            buf,
            sizeof(buf),
            pdMS_TO_TICKS(100)
        );

        for (int i = 0; i < len; i++)
        {
            uart_tool_rx_byte(buf[i]);
        }
    }
}

/* 统一交给解析器 */
void uart_tool_rx_byte(uint8_t byte)
{
    uart_parser_feed(byte);
}
