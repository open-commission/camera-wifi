//
// Created by nebula on 2026/2/4.
//

#ifndef MQTT_TCP_DTO_H
#define MQTT_TCP_DTO_H

typedef struct mqtt_dto
{
    float wendu;
    float shidu;
    float shuiningjie;
    float zhongliang;

    int jiare;
    int fengmingqi;

    char card_id[32];
} dto;

extern dto dto_data;

#endif //MQTT_TCP_DTO_H
