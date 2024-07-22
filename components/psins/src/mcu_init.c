/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-03-11 15:53:52
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-04-17 14:37:40
 * @FilePath: \esp32_positioning\components\psins\src\mcu_init.c
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
#include "./../inc/mcu_init.h"
#include "./../../main/main.h"
#include "./../../mpu9250/inc/empl_driver.h"
// #include "./../../mpu9250/inc/mpu9250_spi.h"
#include "./../../mpu_timer/inc/positioning_timer.h"

uint8_t mcu_init_gpscfg = 0;
uint8_t Usart1_out_DATA[200];
uint8_t Usart1_out_Length = 0;

uint8_t Rx2_data[120], Rx2_data1[120];
uint8_t Rx2_complete = 0;
uint16_t Length2 = 0;

uint8_t PPs_cnt = 0;
uint8_t GPS_exist = 0;
uint16_t GPS_break_cnt = 0;

uint32_t OUT_cnt = 0;
uint32_t GPS_Delay = 0;

uint8_t GAMT_OK_flag = 0;
uint8_t GPS_OK_flag = 0;
uint8_t Bar_OK_flag = 0;

MPU_AD_value mpu_AD_value;
MPU_Data_value mpu_Data_value;
GPS_Data_value gps_Data_value;
INS_Data_value ins_Data_value;
Out_Data out_data;

void mcu_init()
{
    printf("mcu_init\n");
    positioning_timer_init();
    my_gpio_init();
#ifdef USING_SPI
    my_spi_init();
    esp_timer_start_periodic(positioning_time1_timer, TIME1_TIMER_PERIOD);
#endif // USING_SPI
    Init_MPU9250();

    printf("mcu_init done\n");
}