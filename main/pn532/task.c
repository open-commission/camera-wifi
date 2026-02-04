//
// Created by nebula on 2026/2/4.
//

#include "task.h"

#include "532.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

#include "dto.h"

void pn532_task(void* pvParameters)
{
    bool init_flag = init_PN532_I2C(4, 5, 16, 13, I2C_NUM_0);
    ESP_LOGI("app_main", "init_flag = %d", init_flag);

    while (1)
    {
        SAMConfig();

        vTaskDelay(1000 / portTICK_PERIOD_MS);

        uint8_t uid[4];
        uint8_t uidLength = 0;

        // 尝试读取卡
        bool success = readPassiveTargetID(0x00, uid, &uidLength, 1000); // 1秒超时

        if (success && uidLength > 0)
        {
            ESP_LOGI("app_main", "找到卡，UID长度=%d, UID=", uidLength);
            /* UID 转 HEX 字符串 */
            dto tmp;
            size_t pos = 0;
            size_t max_len = sizeof(tmp.card_id);
            for (int i = 0; i < uidLength; i++)
            {
                ESP_LOGI("app_main", "%02X ", uid[i]);
                if (pos + 2 >= max_len)
                    break;

                pos += snprintf(&tmp.card_id[pos],
                                max_len - pos,
                                "%02X",
                                uid[i]);
            }
            /* 确保字符串结束 */
            tmp.card_id[max_len - 1] = '\0';

            /* 一次性写回 volatile */
            memcpy((void*)&dto_data, &tmp, sizeof(tmp));
        }
        else
        {
            ESP_LOGI("app_main", "未检测到卡或读取失败\n");
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
