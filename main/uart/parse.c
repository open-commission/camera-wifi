//
// Created by nebula on 2026/2/4.
//

#include "parse.h"

#include <string.h>

#include "esp_attr.h"
#include "os.h"

#define PARSER_BUF_SIZE 256

static char parser_buf[PARSER_BUF_SIZE];
static uint16_t parser_len = 0;
static uint8_t at_count = 0;

static void ICACHE_FLASH_ATTR parser_try_parse(void);

/* 关键：重置状态 */
void ICACHE_FLASH_ATTR uart_parser_reset(void)
{
    parser_len = 0;
    at_count = 0;
    os_memset(parser_buf, 0, sizeof(parser_buf));
}

/* 字节流入口 */
void ICACHE_FLASH_ATTR uart_parser_feed(uint8_t byte)
{
    /* 新命令强制清空旧缓冲 */
    if (byte == '@') {
        /* 如果 buffer 中已经包含命令关键字，说明是新命令 */
        if (parser_len > 0 &&
            (os_strstr(parser_buf, "pub") ||
             os_strstr(parser_buf, "sub") ||
             os_strstr(parser_buf, "unsub"))) {
            uart_parser_reset();
        }
        at_count++;
    }

    if (parser_len < PARSER_BUF_SIZE - 1) {
        parser_buf[parser_len++] = byte;
        parser_buf[parser_len] = '\0';
    } else {
        /* 溢出直接丢弃 */
        uart_parser_reset();
        return;
    }

    /* pub 至少 4 个 @，sub/unsub 至少 3 个 @ */
    if (at_count >= 3) {
        parser_try_parse();
    }
}

/* 实际解析 */
static void ICACHE_FLASH_ATTR parser_try_parse(void)
{
    uart_cmd_frame_t frame;
    os_memset(&frame, 0, sizeof(frame));

    char *p = parser_buf;
    char *cmd = strtok(p, "@");
    char *arg1 = strtok(NULL, "@");
    char *arg2 = strtok(NULL, "@");

    if (!cmd || !arg1) {
        return;
    }

    if (!os_strcmp(cmd, "pub") && arg2) {
        frame.cmd = UART_CMD_PUB;
        os_strncpy(frame.topic, arg1, sizeof(frame.topic) - 1);
        os_strncpy(frame.payload, arg2, sizeof(frame.payload) - 1);
    } else if (!os_strcmp(cmd, "sub")) {
        frame.cmd = UART_CMD_SUB;
        os_strncpy(frame.topic, arg1, sizeof(frame.topic) - 1);
    } else if (!os_strcmp(cmd, "unsub")) {
        frame.cmd = UART_CMD_UNSUB;
        os_strncpy(frame.topic, arg1, sizeof(frame.topic) - 1);
    } else {
        return;
    }

    /* 回调给用户 */
    uart_parser_on_frame(&frame);

    /* 一帧解析完成后必须清空 */
    uart_parser_reset();
}

void uart_parser_on_frame(uart_cmd_frame_t *frame)
{
    switch (frame->cmd) {
    case UART_CMD_PUB:
        // mqtt_publish(frame->topic, frame->payload);
        break;
    case UART_CMD_SUB:
        // mqtt_subscribe(frame->topic);
        break;
    case UART_CMD_UNSUB:
        // mqtt_unsubscribe(frame->topic);
        break;
    default:
        break;
    }
}
