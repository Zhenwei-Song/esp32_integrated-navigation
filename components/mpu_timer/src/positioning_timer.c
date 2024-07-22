
/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-01-16 15:05:32
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-04-18 19:20:49
 * @FilePath: \esp32_positioning\components\mpu_timer\src\positioning_timer.c
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
#include "./../inc/positioning_timer.h"

#ifdef USING_SPI
bool timer1_flag = false;

esp_timer_handle_t positioning_time1_timer;

SemaphoreHandle_t xCountingSemaphore_timeout1;
#endif

#ifdef USING_I2C
#ifdef PSINS_UART
bool timer2_flag = false;

esp_timer_handle_t positioning_time1_timer;

SemaphoreHandle_t xCountingSemaphore_timeout1;

esp_timer_handle_t positioning_time2_timer;

SemaphoreHandle_t xCountingSemaphore_timeout2;
#endif // PSINS_UART
#ifdef USING_RAW
bool timer3_flag = false;

esp_timer_handle_t positioning_time3_timer;

SemaphoreHandle_t xCountingSemaphore_timeout3;
#endif // USING_RAW
#endif // USING_I2C

/**
 * @description: 定时器初始化函数
 * @return {*}
 */
void positioning_timer_init(void)
{
#ifdef USING_SPI
    const esp_timer_create_args_t time1_timer_args = {
        .callback = &time1_timer_cb,
        .name = "timer1"}; // 定时器名字
    ESP_ERROR_CHECK(esp_timer_create(&time1_timer_args, &positioning_time1_timer));
#endif // USING_SPI
#ifdef USING_I2C
#ifdef PSINS_UART
    const esp_timer_create_args_t time1_timer_args = {
        .callback = &time1_timer_cb,
        .name = "timer1"}; // 定时器名字
    ESP_ERROR_CHECK(esp_timer_create(&time1_timer_args, &positioning_time1_timer));
    const esp_timer_create_args_t time2_timer_args = {
        .callback = &time2_timer_cb,
        .name = "timer2"}; // 定时器名字
    ESP_ERROR_CHECK(esp_timer_create(&time2_timer_args, &positioning_time2_timer));
#endif // PSINS_UART

#ifdef USING_RAW
    const esp_timer_create_args_t time3_timer_args = {
        .callback = &time3_timer_cb,
        .name = "timer3"}; // 定时器名字
    ESP_ERROR_CHECK(esp_timer_create(&time3_timer_args, &positioning_time3_timer));
#endif // USING_RAW
#endif // USING_I2C
    printf("timer init finished\n");
}

#ifdef USING_SPI
/**
 * @description: 用于定时直接从mpu9250寄存器读取数据（raw，spi）
 * @return {*}
 */
void time1_timer_cb(void)
{
    if (timer1_flag == false) {
        xSemaphoreGive(xCountingSemaphore_timeout1);
#ifdef DEBUG
        printf("time1_timeout\n");
#endif
        timer1_flag = true;
    }
}
#endif // USING_SPI

#ifdef PSINS_UART
/**
 * @description:用于串口输出数据给上位机时，更新开发板运行时间
 * @return {*}
 */
void time1_timer_cb(void)
{
    OUT_cnt = OUT_cnt + 1000 / DEFAULT_HZ;
    // printf("OUT_cnt %ld\n", OUT_cnt);
}

/**
 * @description: 用于串口输出数据给上位机
 * @return {*}
 */
void time2_timer_cb(void)
{
    if (timer2_flag == false) {
        xSemaphoreGive(xCountingSemaphore_timeout2);
#ifdef DEBUG
        printf("time2_timeout\n");
#endif
        // OUT_cnt = OUT_cnt + 1000 / DEFAULT_HZ;
        timer2_flag = true;
    }
    else {
        printf("\n\ntimer 2 block warning\n\n");
    }
}
#endif // PSINS_UART

#ifdef USING_RAW
/**
 * @description: 用于定时直接从mpu9250寄存器读取数据（raw，i2c）
 * @return {*}
 */
void time3_timer_cb(void)
{
    if (timer3_flag == false) {
        xSemaphoreGive(xCountingSemaphore_timeout3);
        // printf("time3_timeout\n");
#ifdef DEBUG
        printf("time3_timeout\n");
#endif
        timer3_flag = true;
    }
    else {
        printf("\n\ntimer 3 warning\n\n");
    }
}
#endif // USING_RAW