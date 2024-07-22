/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-02-28 18:53:28
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-07-16 15:31:14
 * @FilePath: \esp32_positioning\components\mpu9250\inc\mpu_dmp_driver.h
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
#ifndef _MPU_DMP_DRIVER_H
#define _MPU_DMP_DRIVER_H

#include "./../../../main/main.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6500

#define MPU_I2C_SCL I2C_SCL
#define MPU_I2C_SDA I2C_SDA
#define DEFAULT_MPU_HZ DEFAULT_HZ // 设置MPU9250的采样率

uint8_t mpu_init_i2c(void);

void gyro_data_ready_cb(void);

// 初始化mpu6050
uint8_t mpu_dmp_init(void);

uint8_t MPU_Get_Magnetometer(short *mx, short *my, short *mz);

#ifdef __cplusplus
}
#endif

#endif