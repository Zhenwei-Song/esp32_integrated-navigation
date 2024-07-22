/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-05-24 10:23:13
 * @LastEditors: Zhenwei-Song zhenwei.song@qq.com
 * @LastEditTime: 2024-05-30 08:41:54
 * @FilePath: \esp32_positioning\esp32_positioning\components\shift_window\src\shift_window.c
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
#include "./../inc/shift_window.h"

float zupt_f_window_x[ZUPT_F_WINDOW_SIZE] = {0};
float zupt_f_window_y[ZUPT_F_WINDOW_SIZE] = {0};
float zupt_f_window_z[ZUPT_F_WINDOW_SIZE] = {0};

static int right = 0;

/**
 * @description:滑动窗口
 * @param {float} f_x
 * @param {float} f_y
 * @param {float} f_z
 * @return {*}
 */
void zupt_shift_window(float f_x, float f_y, float f_z)
{
    if (right != (ZUPT_F_WINDOW_SIZE - 1)) { // 初始window还没有填满
        zupt_f_window_x[right] = f_x;
        zupt_f_window_y[right] = f_y;
        zupt_f_window_z[right] = f_z;
        right = right + 1;
    }
    else {                                             // window已经填满，开始滑动
        for (int j = 1; j < ZUPT_F_WINDOW_SIZE; j++) { // 整体左移
            zupt_f_window_x[j - 1] = zupt_f_window_x[j];
            zupt_f_window_y[j - 1] = zupt_f_window_y[j];
            zupt_f_window_z[j - 1] = zupt_f_window_z[j];
        }
        zupt_f_window_x[ZUPT_F_WINDOW_SIZE - 1] = f_x; // 新值填充到窗口右边，完成滑动
        zupt_f_window_y[ZUPT_F_WINDOW_SIZE - 1] = f_y;
        zupt_f_window_z[ZUPT_F_WINDOW_SIZE - 1] = f_z;
    }
}

/**
 * @description: 计算窗口内均值
 * @param {float} *window
 * @param {int} size
 * @return {*}
 */
static float zupt_calculate_mean(float *window, int size)
{
    float sum = 0.0;
    float mean = 0;
    for (int i = 0; i < size; i++) {
        sum += window[i];
    }
    mean = sum / size;
    return mean;
}

/**
 * @description: 计算方差
 * @param {float} *window
 * @param {int} size
 * @return {*}
 */
static float zupt_calculate_variance(float *window, int size)
{
    float mean = zupt_calculate_mean(window, size);
    float variance = 0.0;
    for (int i = 0; i < size; i++) {
        variance += (window[i] - mean) * (window[i] - mean);
    }
    variance = variance / size;
    return variance;
}

/**
 * @description: 方差阈值检测
 * @param {float} *window_x
 * @param {float} *window_y
 * @param {float} *window_z
 * @param {int} size
 * @return {*}
 */
bool check_zupt_f_window_state(float *window_x, float *window_y, float *window_z, int size)
{
    float var_x = zupt_calculate_variance(window_x, size);
    float var_y = zupt_calculate_variance(window_y, size);
    float var_z = zupt_calculate_variance(window_z, size);
    // printf("window_x: %f, window_y: %f,window_z: %f\n",var_x,var_y,var_z);
    if (var_x < ZUPT_VARIANCE_THRESHOLD && var_y < ZUPT_VARIANCE_THRESHOLD && var_z < ZUPT_VARIANCE_THRESHOLD) {
        return true;
    }
    return false;
}