/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-05-24 10:23:13
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-05-31 15:53:48
 * @FilePath: \esp32_positioning\components\band_pass_filter\src\band_pass_filter.c
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
#include "./../inc/band_pass_filter.h"

#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846

/* -------------------------------------------------------------------------- */
/*                                 初始化带通滤波器参数                                 */
/* -------------------------------------------------------------------------- */
void init_band_pass_filter(band_pass_filter_s *filter, double center_freq, double bandwidth, double sample_rate)
{
    double omega = 2 * PI * center_freq / sample_rate;
    double alpha = sin(omega) * sinh(log(2.0) / 2.0 * bandwidth * omega / sin(omega));

    double cos_omega = cos(omega);
    double a0_inv = 1.0 / (1.0 + alpha);

    filter->a0 = alpha * a0_inv;
    filter->a1 = 0.0;
    filter->a2 = -alpha * a0_inv;
    filter->b1 = -2.0 * cos_omega * a0_inv;
    filter->b2 = (1.0 - alpha) * a0_inv;

    filter->x1 = filter->x2 = filter->y1 = filter->y2 = 0.0;
}

/* -------------------------------------------------------------------------- */
/*                                 带通滤波器处理函数                                */
/* -------------------------------------------------------------------------- */

static float process_band_pass_filter(band_pass_filter_s *filter, float input)
{
    float output = filter->a0 * input + filter->a1 * filter->x1 + filter->a2 * filter->x2 - filter->b1 * filter->y1 - filter->b2 * filter->y2;

    filter->x2 = filter->x1;
    filter->x1 = input;

    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}

/* -------------------------------------------------------------------------- */
/*                                  步数检测信号处理                                  */
/* -------------------------------------------------------------------------- */
float process_step_detection(float input_signal, band_pass_filter_s *filter)
{
    float output_signal = 0;
    // for (int i = 0; i < length; i++) {
    //     output_signal[i] = process_band_pass_filter(filter, input_signal[i]);
    // }
    output_signal = process_band_pass_filter(filter, input_signal);
    return output_signal;
}