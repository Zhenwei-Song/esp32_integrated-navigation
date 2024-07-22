/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-04-08 14:37:08
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-04-09 09:51:19
 * @FilePath: \esp32_positioning\components\down_sampling\inc\down_sampling.h
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
#ifndef GET_ACC_WITHOUT_G_H_
#define GET_ACC_WITHOUT_G_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "../../../main/main.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"

typedef struct {
    float x;
    float y;
    float z;
} vector3D;

typedef struct {
    float x;
    float y;
    float z;
    float Gx;
    float Gy;
    float Gz;
} vector3D_G;

typedef struct {
    float pitch;
    float roll;
    float yaw;
} eulerAngles;

vector3D_G get_acc_without_g(vector3D linearAcc, vector3D gravity, eulerAngles angles);

#ifdef __cplusplus
}
#endif

#endif