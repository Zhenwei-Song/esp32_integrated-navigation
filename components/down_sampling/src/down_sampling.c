/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-03-26 16:11:33
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-04-08 15:29:47
 * @FilePath: \esp32_positioning\components\down_sampling\src\down_sampling.c
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
#include "./../inc/down_sampling.h"
#include "./../../mpu9250/inc/mpu9250_raw.h"
#ifdef DOWN_SAMPLING
void down_sampling(short *gx, short *gy, short *gz, short *ax, short *ay, short *az, short *mx, short *my, short *mz)
{
    short temp_gyr[3];
    short temp_acc[3];
    short temp_mag[3];
    static short sum_gyr[3];
    static short sum_acc[3];
    static short sum_mag[3];
    for (int i = 0; i < DOWNSAMPLE_FACTOR; i++) {
        RAW_MPU_Get_Gyroscope(&temp_gyr[0], &temp_gyr[1], &temp_gyr[2]);     // 读取角速度原始数据
        RAW_MPU_Get_Accelerometer(&temp_acc[0], &temp_acc[1], &temp_acc[2]); // 读取角加速度原始数据
        RAW_MPU_Get_Magnetometer(&temp_mag[0], &temp_mag[1], &temp_mag[2]);  // 读取磁力计原始数据
        for (int j = 0; j < 3; j++) {
            sum_gyr[j] = sum_gyr[j] + temp_gyr[j];
            sum_acc[j] = sum_acc[j] + temp_acc[j];
            sum_mag[j] = sum_mag[j] + temp_mag[j];
        }
    }
    for (int j = 0; j < 3; j++) {
        sum_gyr[j] = sum_gyr[j] / DOWNSAMPLE_FACTOR;
        sum_acc[j] = sum_acc[j] / DOWNSAMPLE_FACTOR;
        sum_mag[j] = sum_mag[j] / DOWNSAMPLE_FACTOR;
    }
    *gx = sum_gyr[0];
    *gy = sum_gyr[1];
    *gz = sum_gyr[2];

    *ax = sum_acc[0];
    *ay = sum_acc[1];
    *az = sum_acc[2];

    *mx = sum_mag[0];
    *my = sum_mag[1];
    *mz = sum_mag[2];
}
#endif