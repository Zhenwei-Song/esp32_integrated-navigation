/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-04-08 14:37:08
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-04-09 09:51:19
 * @FilePath: \esp32_positioning\components\down_sampling\inc\down_sampling.h
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
#ifndef DOWN_SAMPLING_H_
#define DOWN_SAMPLING_H_

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
#ifdef DOWN_SAMPLING
void down_sampling(short *gx, short *gy, short *gz, short *ax, short *ay, short *az, short *mx, short *my, short *mz);
#endif
#ifdef __cplusplus
}
#endif

#endif