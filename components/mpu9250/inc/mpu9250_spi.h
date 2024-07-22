/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-03-11 10:13:45
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-03-12 16:55:11
 * @FilePath: \esp32_positioning\components\mpu9250\inc\mpu9250_spi.h
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
#ifndef __MPU9250_SPI_H_
#define __MPU9250_SPI_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/spi_master.h"

extern spi_device_handle_t my_spi;
/* -------------------------------------------------------------------------- */
/*                                     SPI                                    */
/* -------------------------------------------------------------------------- */
// #define DMA_CHAN 2
// #define PIN_NUM_MISO 12
// #define PIN_NUM_MOSI 13
// #define PIN_NUM_CLK 14
// #define PIN_NUM_CS 15
#define PIN_NUM_MISO 32
#define PIN_NUM_MOSI 33
#define PIN_NUM_CLK 25
#define PIN_NUM_CS 26

// 定义MPU9250内部地址
/*****************************************************************/
#define SMPLRT_DIV 0x19 // 陀螺采样率
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG_2 0x1D
#define LP_ACCEL_ODR 0x1E
#define FIFO_EN 0x23

#define INT_PIN_CFG 0x37 // 中断配置
#define USER_CTRL 0x6a
#define I2C_MST_CTRL 0x24
#define I2C_MST_DELAY_CTRL 0x67
//--------------------i2c slv0-------------------------------//
#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_CTRL 0x27
#define I2C_SLV0_DO 0x63 // output reg
//--------------------i2c slv1-------------------------------//
#define I2C_SLV1_ADDR 0x28
#define I2C_SLV1_REG 0x29
#define I2C_SLV1_CTRL 0x2A
#define I2C_SLV1_DO 0x64 // output reg
//--------------------AK8963 reg addr------------------------//
#define MPU9250_AK8963_ADDR 0x0C // AKM addr
#define AK8963_WHOAMI_REG 0x00   // AKM ID addr
#define AK8963_WHOAMI_ID 0x48    // ID
#define AK8963_ST1_REG 0x02      // Data Status1
#define AK8963_ST2_REG 0x09      // Data reading end register & check Magnetic sensor overflow occurred
#define AK8963_ST1_DOR 0x02
#define AK8963_ST1_DRDY 0x01 // Data Ready
#define AK8963_ST2_BITM 0x10
#define AK8963_ST2_HOFL 0x08 // Magnetic sensor overflow
#define AK8963_CNTL1_REG 0x0A
#define AK8963_CNTL2_REG 0x0B
#define AK8963_CNTL2_SRST 0x01 // soft Reset
#define AK8963_ASAX 0x10       // X-axis sensitivity adjustment value
#define AK8963_ASAY 0x11       // Y-axis sensitivity adjustment value
#define AK8963_ASAZ 0x12       // Z-axis sensitivity adjustment value
//--------------------9axis  reg addr-----------------------//
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define TEMP_OUT_H 0x41 // temperture
#define TEMP_OUT_L 0x42

#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define MAG_XOUT_L 0x03
#define MAG_XOUT_H 0x04
#define MAG_YOUT_L 0x05
#define MAG_YOUT_H 0x06
#define MAG_ZOUT_L 0x07
#define MAG_ZOUT_H 0x08
//--------------------other reg addr-----------------------//
#define PWR_MGMT_1 0x6B //????,???:0x00(????)
#define WHO_AM_I 0x75   // ID?????(????0x71,??)

#define EXT_SENS_DATA_00 0x49 // MPU9250 IIC???????????00
#define EXT_SENS_DATA_01 0x4a // MPU9250 IIC???????????01
#define EXT_SENS_DATA_02 0x4b // MPU9250 IIC???????????02
#define EXT_SENS_DATA_03 0x4c // MPU9250 IIC???????????03

void my_spi_init(void);

#endif