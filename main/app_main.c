#include "mqtts.h"
#include "task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tool.h"

void app_main(void)
{
    // /* 1. 初始化 UART */
    // uart_tool_init(115200);
    //
    // xTaskCreate(mqtt_task,
    //             "mqtt_task",
    //             12288,
    //             NULL,
    //             5,
    //             NULL);
    //
    // vTaskDelay(pdMS_TO_TICKS(5000));
    //
    // xTaskCreate(
    //     uart_rx_task,
    //     "uart_rx",
    //     2048,
    //     NULL,
    //     10,
    //     NULL
    // );
    //
    // /* 2. （可选）发一条启动提示 */
    // uart_tool_send("UART ready\r\n");
    //
    // /* 3. app_main 不能退出，留一个空循环 */
    // while (1)
    // {
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }

    xTaskCreate(pn532_task, "pn532_task", 4096, NULL, 5, NULL);
}
