#ifndef SHIFT_WINDOW_H
#define SHIFT_WINDOW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "./../../../main/main.h"

extern float zupt_f_window_x[ZUPT_F_WINDOW_SIZE];
extern float zupt_f_window_y[ZUPT_F_WINDOW_SIZE];
extern float zupt_f_window_z[ZUPT_F_WINDOW_SIZE];

void zupt_shift_window(float f_x, float f_y, float f_z);

bool check_zupt_f_window_state(float *window_x, float *window_y, float *window_z, int size);

#ifdef __cplusplus
}
#endif

#endif