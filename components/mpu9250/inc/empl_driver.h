/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-02-28 18:53:28
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-03-18 15:35:34
 * @FilePath: \esp32_positioning\components\mpu9250\inc\empl_driver.h
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */

#ifndef _EMPL_DRIVER_H_
#define _EMPL_DRIVER_H_

#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "./mpu9250_spi.h"

#ifdef __cplusplus
extern "C" {
#endif

extern QueueHandle_t gpio_evt_queue;
/* -------------------------------------------------------------------------- */
/*                                     I2C                                    */
/* -------------------------------------------------------------------------- */
#define WRITE_BIT I2C_MASTER_WRITE // I2C master write
#define READ_BIT I2C_MASTER_READ   // I2C master read
#define ACK_CHECK_EN 0x1           // I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0          // I2C master will not check ack from slave
#define ACK_VAL 0x0                // I2C ack value
#define NACK_VAL 0x1               // I2C nack value

/* -------------------------------------------------------------------------- */
/*                                    GPIO                                    */
/* -------------------------------------------------------------------------- */
#define GPIO_OUTPUT_IO_0 PIN_NUM_CS
#define GPIO_INPUT_IO_0 34
// #define GPIO_OUTPUT_IO_1 19
// #define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_OUTPUT_IO_0) | (1ULL << GPIO_OUTPUT_IO_1))
#define GPIO_OUTPUT_PIN_SEL (1ULL << GPIO_OUTPUT_IO_0)

int esp32_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);

int esp32_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);

int esp32_delay_ms(unsigned long num_ms);

int esp32_delay_us(unsigned long num_us);

int esp32_get_clock_ms(unsigned long *count);

void my_gpio_init(void);

void i2c_gpio_init(void);

void start_i2c_isr(void);

void gpio_intr_handle(void *arg);

#ifdef __cplusplus
}
#endif

#endif