/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-03-25 15:11:51
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-04-17 15:14:16
 * @FilePath: \esp32_positioning\main\all_tasks.h
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
#ifndef _ALL_TASKS_H
#define _ALL_TASKS_H

#include "./main.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#ifdef USING_DMP
// extern float magCalibration[3];
extern bool data_updated;
void ins_init(void);
void get_dmp_data(void *arg);
#endif

#ifdef USING_RAW
extern SemaphoreHandle_t xCountingSemaphore_push_data;
void get_raw_data_i2c(void *pvParameters);
void push_raw_data(void *pvParameters);
#endif // USING_RAW

extern SemaphoreHandle_t xCountingSemaphore_data_update;

#ifdef ONLY_ATT
void psins_att_data_update(void *pvParameters);
#endif // ONLY_ATT

#ifdef PSINS_POS
#ifdef PSINS_UART
void psins_uart_pop_data(void *pvParameters);
#endif // PSINS_UART
void psins_static_pos_data_update(void *pvParameters);
#endif // PSINS_POS

#ifdef USING_SFANN_SINS
void sins_pos_data_update(void *pvParameters);
#endif // USING_SFANN

#ifdef USING_SPI
void get_raw_data_spi(void *pvParameters);
#endif // USING_SPI

#endif