#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tool.h"
#include "parse.h"

void app_main(void)
{
    /* 1. 初始化 UART */
    uart_tool_init(115200);

    /* 2. （可选）发一条启动提示 */
    uart_tool_send("UART ready\r\n");

    /* 3. app_main 不能退出，留一个空循环 */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
