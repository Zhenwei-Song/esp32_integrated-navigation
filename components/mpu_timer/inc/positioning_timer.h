/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-01-16 15:05:23
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-03-21 10:07:46
 * @FilePath: \esp32_positioning\components\mpu_timer\inc\positioning_timer.h
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */

#ifndef BLE_TIMER_H_
#define BLE_TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "./../../../main/main.h"

// #define TIME1_TIMER_PERIOD 10000    // 定时中断频率100Hz，读MPU9250

#ifdef USING_SPI

extern bool timer1_flag;

extern esp_timer_handle_t positioning_time1_timer;

extern SemaphoreHandle_t xCountingSemaphore_timeout1;

#define TIME1_TIMER_PERIOD 100000 // 100ms

void time1_timer_cb(void);

#endif // USING_SPI

#ifdef PSINS_UART

extern bool timer1_flag;

extern esp_timer_handle_t positioning_time1_timer;

extern SemaphoreHandle_t xCountingSemaphore_timeout1;

#define TIME1_TIMER_PERIOD (1 * 1000000 / DEFAULT_HZ)

void time1_timer_cb(void);

extern bool timer2_flag;

extern esp_timer_handle_t positioning_time2_timer;

extern SemaphoreHandle_t xCountingSemaphore_timeout2;
#ifdef DOWN_SAMPLING
#define TIME2_TIMER_PERIOD (1 * 1000000 / OUT_SAMPING_RATE)
#else
#define TIME2_TIMER_PERIOD (1 * 1000000 / (SAMPLE_RATE/2))
#endif
void time2_timer_cb(void);
#endif // PSINS_UART
#ifdef USING_RAW
extern bool timer3_flag;

extern esp_timer_handle_t positioning_time3_timer;

extern SemaphoreHandle_t xCountingSemaphore_timeout3;

// #define TIME3_TIMER_PERIOD 500000 // 500ms

#define TIME3_TIMER_PERIOD (1 * 1000000 / SAMPLE_RATE) // 根据mpu频率而定

void time3_timer_cb(void);
#endif // USING_RAW

void positioning_timer_init(void);

#ifdef __cplusplus
}
#endif

#endif