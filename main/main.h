/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-03-11 15:46:52
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-07-18 14:32:01
 * @FilePath: \esp32_positioning\main\main.h
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
#ifndef __MAIN_H
#define __MAIN_H

#include "./../components/mpu9250/inc/range.h"
#include "./../components/psins/inc/mcu_init.h"

// #define DEBUG

#define IS_ESP32_S3

/* -------------------------------------------------------------------------- */
/*                                  数据读取方式设定                                  */
/* -------------------------------------------------------------------------- */
// #define USING_DMP // I2C读DMP

#if !defined USING_DMP && !defined USING_RAW
#define USING_RAW // 用直接读寄存器的方式
// #define DOWN_SAMPLING // 降采样
#define YAW_INIT // 利用磁力初始化yaw
// #define GET_ACC_WITHOUT_G //RAW模式下获取去除了重力分量之后的加速度，用于加速度零偏处理
#endif

#if defined USING_DMP || defined USING_RAW
#define USING_I2C
#endif

#if !defined USING_I2C && !defined USING_SPI
#define USING_SPI
#endif

/* -------------------------------------------------------------------------- */
/*                                  数据解析方式设定                                  */
/* -------------------------------------------------------------------------- */
//  #define PSINS_ATT //仅获取姿态角
#define PSINS_POS          // 获取位置(包括姿态)
#define PSINS_POS_ON_BOARD // 板上更新信息
#define PSINS_UART         // 上传给上位机数据

// #define USING_PDR
// #define USING_ZUPT
// #difned FORCED_CONVERGENCE //强制收敛

#ifdef USING_I2C

#define GET_RAW_INFO // 从寄存器直接获取角速度和加速度（和SPI一样）

#ifdef IS_ESP32_S3
#define I2C_SDA 11
#define I2C_SCL 12
#define GPIO_INTR 13
#else
#define I2C_SDA 21
#define I2C_SCL 22
#define GPIO_INTR 23
#endif

#define DEFAULT_HZ (1000)   // 设置MPU9250的采样率
#define G_RANGE G_RANGE_250 // 传感器角速度量程
#define A_RANGE A_RANGE_2   // 传感器加速度量程

#define LONGITUDE 108.91860023655954 // 经度
#define LATITUDE 34.2339893101992    // 纬度
#define ALTITUDE 430                 // 海拔

#define WAIT_TIME 3000      // 秒乘1000
#define ALIGN_TIME 300 - 00 // 秒乘1000
// #define ALIGN_TIME 10000 // 秒乘1000
#define ZERO_BIAS_CAL_TIME 5000
/* -------------------------------------------------------------------------- */
/*                                  降采样速率相关设置                                 */
/* -------------------------------------------------------------------------- */
#define SAMPLE_RATE (220)
#define MAG_SAMPLE_RATE 200
#define MAG_SAMPLE_FACTOR (SAMPLE_RATE / MAG_SAMPLE_RATE)

#ifdef DOWN_SAMPLING
#define OUT_SAMPING_RATE 50 // 将采样后输出频率
#define DOWNSAMPLE_FACTOR (SAMPLE_RATE / OUT_SAMPING_RATE)
#define my_TS (1.0 / OUT_SAMPING_RATE)
#else
#define my_TS (1.0 / SAMPLE_RATE)
#endif // DOWN_SAMPLING

#ifdef USING_DMP
#define GET_LINEAR_ACC_AND_G // 获取除去重力的线性加速度
#endif                       // USING_DMP

/* -------------------------------------------------------------------------- */
/*                                   ZUPT参数                                   */
/* -------------------------------------------------------------------------- */
#define ZUPT_F_MIN 9.0    // m/s^2
#define ZUPT_F_MAX 11     // m/s^2
#define ZUPT_W_THESH 0.08 // rad/s ==1.0313240312354817757823667866539 度/s
#define ZUPT_F_WINDOW_SIZE 50
#define ZUPT_VARIANCE_THRESHOLD 0.2
/* -------------------------------------------------------------------------- */
/*                                     PDR                                    */
/* -------------------------------------------------------------------------- */
#define PDR_STEP_LENGTH 0.6 // m
#define PDR_STEP_K 0.53     // 步长估计参数
#endif                      // USING_I2C

#if (A_RANGE == A_RANGE_2)
#define A_RANGE_NUM A_RANGE_2_NUM
#elif (A_RANGE == A_RANGE_4)
#define A_RANGE_NUM A_RANGE_4_NUM
#elif (A_RANGE == A_RANGE_8)
#define A_RANGE_NUM A_RANGE_8_NUM
#elif (A_RANGE == A_RANGE_16)
#define A_RANGE_NUM A_RANGE_16_NUM
#endif // A_RANGE

#if defined PSINS_ATT || defined PSINS_POS
#define USING_PSINS
#endif

/* -------------------------------------------------------------------------- */
/*                                   编译警示信息                                   */
/* -------------------------------------------------------------------------- */
#ifdef USING_SPI
#warning Using spi!!!!!!
#endif

#ifdef USING_RAW
#warning Using raw!!!!!!
#endif
/* -------------------------------------------------------------------------- */
/*                                   编译错误信息                                   */
/* -------------------------------------------------------------------------- */
#if defined USING_I2C && defined USING_SPI
#error "Can't use I2C and SPI at the same time!"
#endif

#if defined USING_RAW && defined USING_DMP
#error "Can't use RAW and DMP at the same time!"
#endif

extern MPU_AD_value mpu_AD_value;
extern MPU_Data_value mpu_Data_value;
extern GPS_Data_value gps_Data_value;
extern INS_Data_value ins_Data_value;
extern Out_Data out_data;

extern uint8_t MS5611_cnt;

extern uint8_t mcu_init_gpscfg;
extern uint8_t Usart1_out_DATA[200];
extern uint8_t Usart1_out_Length;

extern uint8_t Rx2_data[120], Rx2_data1[120];
extern uint8_t Rx2_complete;
extern uint16_t Length2;

extern uint8_t PPs_cnt;
extern uint8_t GPS_exist;
extern uint16_t GPS_break_cnt;

extern uint32_t OUT_cnt;
extern uint32_t GPS_Delay;

extern uint8_t GAMT_OK_flag;
extern uint8_t GPS_OK_flag;
extern uint8_t Bar_OK_flag;

#endif /* __MAIN_H */
