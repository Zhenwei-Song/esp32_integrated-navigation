/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-02-28 18:53:28
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-03-18 15:27:34
 * @FilePath: \esp32_positioning\components\mpu9250\src\empl_driver.c
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */

#include "./../inc/empl_driver.h"


#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

#include "./../../../main/main.h"


/* -------------------------------------------------------------------------- */
/*                                   I2C读取模式                                  */
/* -------------------------------------------------------------------------- */
#define GPIO_MPU_INTR GPIO_INTR
QueueHandle_t gpio_evt_queue = NULL; // 用于接收 GPIO 中断的 queue

int esp32_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write(cmd, data, length, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    // printf("esp32_i2c_write ret:%d\n", ret);
    // printf("esp32_i2c_write err:%s\n", esp_err_to_name(ret));
    return ret;
}

int esp32_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
    if (length == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | READ_BIT, ACK_CHECK_EN);
    if (length > 1) {
        i2c_master_read(cmd, data, length - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data + length - 1, NACK_VAL);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

int esp32_delay_ms(unsigned long num_ms)
{
    vTaskDelay(num_ms / portTICK_PERIOD_MS);
    return 0;
}

int esp32_delay_us(unsigned long num_us)
{
    esp_rom_delay_us(num_us);
    return 0;
}

int esp32_get_clock_ms(unsigned long *count)
{
    *count = (unsigned long)esp_timer_get_time();
    return 0;
}

void my_gpio_init(void)
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    gpio_config(&io_conf);
    printf("gpio init finished\n");
}
// gpio_set_level(GPIO_OUTPUT_IO_0, 0);

void my_gpio2_init(void)
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_INPUT_IO_0;
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    gpio_config(&io_conf);
    printf("gpio init finished\n");
}

void i2c_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_MPU_INTR),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&io_conf);
}

void start_i2c_isr(void)
{
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_MPU_INTR, gpio_intr_handle, (void *)GPIO_MPU_INTR);
}

void gpio_intr_handle(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}