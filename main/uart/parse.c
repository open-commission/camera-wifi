//
// Created by nebula on 2026/2/4.
// FIXED & SAFE VERSION FOR ESP8266
//

#include "parse.h"

#include <string.h>
#include <stdio.h>

#include "esp_attr.h"
#include "mqtt_client.h"
#include "mqtts.h"
#include "os.h"
#include "tool.h"

#define PARSER_BUF_SIZE 256

static char     parser_buf[PARSER_BUF_SIZE];
static uint16_t parser_len = 0;

/* =========================================================
 * 工具
 * ========================================================= */

void ICACHE_FLASH_ATTR uart_parser_reset(void)
{
    parser_len = 0;
    os_memset(parser_buf, 0, sizeof(parser_buf));
}

/* =========================================================
 * 核心解析（原地指针切分，无 snprintf）
 * ========================================================= */

void ICACHE_FLASH_ATTR parser_try_parse(void)
{
    uart_cmd_frame_t frame;
    os_memset(&frame, 0, sizeof(frame));

    /* 必须 @ 开头 */
    if (parser_buf[0] != '@')
        goto out;

    uart_tool_send(parser_buf);

    char *cmd, *arg1, *arg2;
    char *p1, *p2, *p3;

    cmd = parser_buf + 1;
    p1 = os_strchr(cmd, '@');
    if (!p1) goto out;
    *p1 = 0;

    arg1 = p1 + 1;
    p2 = os_strchr(arg1, '@');
    if (!p2) goto out;
    *p2 = 0;

    arg2 = p2 + 1;
    p3 = os_strchr(arg2, '@');   /* pub 才需要 */

    if (!os_strcmp(cmd, "sub"))
    {
        uart_tool_send("sub");
        frame.cmd = UART_CMD_SUB;
        os_strncpy(frame.topic, arg1, sizeof(frame.topic) - 1);
    }
    else if (!os_strcmp(cmd, "unsub"))
    {
        uart_tool_send("unsub");
        frame.cmd = UART_CMD_UNSUB;
        os_strncpy(frame.topic, arg1, sizeof(frame.topic) - 1);
    }
    else if (!os_strcmp(cmd, "pub"))
    {
        uart_tool_send("pub");
        if (!p3) goto out;
        *p3 = 0;

        frame.cmd = UART_CMD_PUB;
        os_strncpy(frame.topic, arg1, sizeof(frame.topic) - 1);
        os_strncpy(frame.payload, arg2, sizeof(frame.payload) - 1);
    }
    else
    {
        goto out;
    }

    uart_parser_on_frame(&frame);

    out:
        uart_parser_reset();
}

/* =========================================================
 * UART 字节流入口（逐字节）
 * ========================================================= */

void ICACHE_FLASH_ATTR uart_parser_feed(uint8_t byte)
{
    /* 1. 所有字符强制回显 */
    char echo[2] = { (char)byte, 0 };
    uart_tool_send(echo);

    /* 2. 缓冲区保护 */
    if (parser_len >= PARSER_BUF_SIZE - 1)
    {
        uart_parser_reset();
        return;
    }

    /* 3. 先写入缓冲区 */
    parser_buf[parser_len++] = byte;
    parser_buf[parser_len]   = 0;

    /* 4. 检测新的合法命令起点（扫描整个 buffer） */
    for (int i = parser_len - 1; i >= 0; i--)
    {
        if (parser_buf[i] != '@')
            continue;

        if (!os_strncmp(&parser_buf[i], "@sub", 4) ||
            !os_strncmp(&parser_buf[i], "@pub", 4) ||
            !os_strncmp(&parser_buf[i], "@unsub", 6))
        {
            if (i > 0)
            {
                uint16_t remain = parser_len - i;
                os_memmove(parser_buf, &parser_buf[i], remain);
                parser_len = remain;
                parser_buf[parser_len] = 0;

                uart_tool_send("[UART] frame restart\r\n");
            }
            break;
        }
    }


    /* 5. MQTT 未就绪不解析 */
    if (!mqtt_is_ready())
        return;

    /* 6. 统计 @ 数量 */
    uint8_t at_cnt = 0;
    for (uint16_t i = 0; i < parser_len; i++)
    {
        if (parser_buf[i] == '@')
            at_cnt++;
    }

    /* 7. 根据命令类型决定是否尝试解析 */
    if (!os_strncmp(parser_buf, "@sub@", 5) && at_cnt >= 3)
    {
        uart_tool_send("[UART] parse sub\r\n");
        parser_try_parse();
    }
    else if (!os_strncmp(parser_buf, "@unsub@", 7) && at_cnt >= 3)
    {
        uart_tool_send("[UART] parse unsub\r\n");
        parser_try_parse();
    }
    else if (!os_strncmp(parser_buf, "@pub@", 5) && at_cnt >= 4)
    {
        uart_tool_send("[UART] parse pub\r\n");
        parser_try_parse();
    }
}

/* =========================================================
 * 命令执行（全部带 UART 回显）
 * ========================================================= */

void uart_parser_on_frame(uart_cmd_frame_t* frame)
{
    if (!mqtt_is_ready())
    {
        uart_tool_send("[UART] MQTT not ready\r\n");
        return;
    }

    esp_mqtt_client_handle_t client = mqtt_get_client();
    if (!client)
    {
        uart_tool_send("[UART] MQTT client null\r\n");
        return;
    }

    char buf[96];
    int msg_id;

    switch (frame->cmd)
    {
    case UART_CMD_PUB:
        msg_id = esp_mqtt_client_publish(
            client,
            frame->topic,
            frame->payload,
            os_strlen(frame->payload),
            1,
            0
        );
        sprintf(buf, "[UART] PUB OK id=%d\r\n", msg_id);
        uart_tool_send(buf);
        break;

    case UART_CMD_SUB:
        msg_id = esp_mqtt_client_subscribe(client, frame->topic, 0);
        sprintf(buf, "[UART] SUB OK id=%d\r\n", msg_id);
        uart_tool_send(buf);
        break;

    case UART_CMD_UNSUB:
        msg_id = esp_mqtt_client_unsubscribe(client, frame->topic);
        sprintf(buf, "[UART] UNSUB OK id=%d\r\n", msg_id);
        uart_tool_send(buf);
        break;

    default:
        break;
    }
}
