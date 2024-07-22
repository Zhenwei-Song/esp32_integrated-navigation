#ifndef BAND_PASS_FILTER_H
#define BAND_PASS_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "./../../../main/main.h"

// 定义滤波器参数
typedef struct band_pass_filter {
    double a0, a1, a2, b1, b2;
    double x1, x2, y1, y2;
} band_pass_filter_s;

void init_band_pass_filter(band_pass_filter_s *filter, double center_freq, double bandwidth, double sample_rate);

float process_step_detection(float input_signal, band_pass_filter_s *filter);

#ifdef __cplusplus
}
#endif

#endif