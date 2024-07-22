/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-03-26 16:25:07
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-03-28 10:57:34
 * @FilePath: \esp32_positioning\components\psins\src\uart_out.cpp
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
#include "../inc/uart_out.h"
#include "../inc/mcu_init.h"
#include "../inc/psins.h"

void Data_updata(void)
{
    float Data_F;
    double Data_D;
    uint32_t Data_U, i, checksum;
    uint8_t *pcheck;
    //////////////////////////////////////////////////
    out_data.OUT_cnt = (float)((double)OUT_cnt / (double)1000);
    out_data.Gyro[0] = (float)mpu_Data_value.Gyro[0];
    out_data.Gyro[1] = (float)mpu_Data_value.Gyro[1];
    out_data.Gyro[2] = (float)mpu_Data_value.Gyro[2];

    // out_data.Accel[0] = (float)(mpu_Data_value.Accel[0] * (double)9.8);
    // out_data.Accel[1] = (float)(mpu_Data_value.Accel[1] * (double)9.8);
    // out_data.Accel[2] = (float)(mpu_Data_value.Accel[2] * (double)9.8);

    out_data.Accel[0] = (float)(mpu_Data_value.Accel[0] * (double)glv.g0);
    out_data.Accel[1] = (float)(mpu_Data_value.Accel[1] * (double)glv.g0);
    out_data.Accel[2] = (float)(mpu_Data_value.Accel[2] * (double)glv.g0);

    out_data.Magn[0] = (float)mpu_Data_value.Mag[0];
    out_data.Magn[1] = (float)mpu_Data_value.Mag[1];
    out_data.Magn[2] = (float)mpu_Data_value.Mag[2];

    out_data.mBar = (float)(mpu_Data_value.Pressure);

    out_data.GPS_Vn[0] = (float)(gps_Data_value.GPS_Vn[0]);
    out_data.GPS_Vn[1] = (float)(gps_Data_value.GPS_Vn[1]);
    out_data.GPS_Vn[2] = (float)(gps_Data_value.GPS_Vn[2]);

    Data_D = gps_Data_value.GPS_Pos[1] / DEG;
    Data_U = (uint32_t)Data_D;
    Data_F = (float)(Data_D - (double)Data_U);
    out_data.GPS_Pos[0] = (float)Data_U;
    out_data.GPS_Pos[1] = Data_F;

    Data_D = gps_Data_value.GPS_Pos[0] / DEG;
    Data_U = (uint32_t)Data_D;
    Data_F = (float)(Data_D - (double)Data_U);
    out_data.GPS_Pos[2] = (float)Data_U;
    out_data.GPS_Pos[3] = Data_F;

    out_data.GPS_Pos[4] = (float)gps_Data_value.GPS_Pos[2];

    if (gps_Data_value.GPS_pDOP > 99) {
        gps_Data_value.GPS_pDOP = 99;
    }
    out_data.GPS_status = (float)gps_Data_value.GPS_numSV + gps_Data_value.GPS_pDOP;

    out_data.GPS_delay = (float)GPS_Delay / (float)1000; // ms?

    out_data.Temp = mpu_Data_value.Temp;
    //////////////////////////////////////////////////////////////////////////////////
    *(uint32_t *)&Usart1_out_DATA[0 * 4] = 0x56aa55aa;
    *(float *)&Usart1_out_DATA[1 * 4] = out_data.OUT_cnt;
    *(float *)&Usart1_out_DATA[2 * 4] = out_data.Gyro[0];
    *(float *)&Usart1_out_DATA[3 * 4] = out_data.Gyro[1];
    *(float *)&Usart1_out_DATA[4 * 4] = out_data.Gyro[2];
    *(float *)&Usart1_out_DATA[5 * 4] = out_data.Accel[0];
    *(float *)&Usart1_out_DATA[6 * 4] = out_data.Accel[1];
    *(float *)&Usart1_out_DATA[7 * 4] = out_data.Accel[2];
    *(float *)&Usart1_out_DATA[8 * 4] = out_data.Magn[0];
    *(float *)&Usart1_out_DATA[9 * 4] = out_data.Magn[1];
    *(float *)&Usart1_out_DATA[10 * 4] = out_data.Magn[2];
    *(float *)&Usart1_out_DATA[11 * 4] = out_data.mBar;
    *(float *)&Usart1_out_DATA[12 * 4] = out_data.Att[0];
    *(float *)&Usart1_out_DATA[13 * 4] = out_data.Att[1];
    *(float *)&Usart1_out_DATA[14 * 4] = out_data.Att[2];
    *(float *)&Usart1_out_DATA[15 * 4] = out_data.Vn[0];
    *(float *)&Usart1_out_DATA[16 * 4] = out_data.Vn[1];
    *(float *)&Usart1_out_DATA[17 * 4] = out_data.Vn[2];
    *(float *)&Usart1_out_DATA[18 * 4] = out_data.Pos[0];
    *(float *)&Usart1_out_DATA[19 * 4] = out_data.Pos[1];
    *(float *)&Usart1_out_DATA[20 * 4] = out_data.Pos[2];
    *(float *)&Usart1_out_DATA[21 * 4] = out_data.Pos[3];
    *(float *)&Usart1_out_DATA[22 * 4] = out_data.Pos[4];
    *(float *)&Usart1_out_DATA[23 * 4] = out_data.GPS_Vn[0];
    *(float *)&Usart1_out_DATA[24 * 4] = out_data.GPS_Vn[1];
    *(float *)&Usart1_out_DATA[25 * 4] = out_data.GPS_Vn[2];
    *(float *)&Usart1_out_DATA[26 * 4] = out_data.GPS_Pos[0];
    *(float *)&Usart1_out_DATA[27 * 4] = out_data.GPS_Pos[1];
    *(float *)&Usart1_out_DATA[28 * 4] = out_data.GPS_Pos[2];
    *(float *)&Usart1_out_DATA[29 * 4] = out_data.GPS_Pos[3];
    *(float *)&Usart1_out_DATA[30 * 4] = out_data.GPS_Pos[4];
    *(float *)&Usart1_out_DATA[31 * 4] = out_data.GPS_status;
    *(float *)&Usart1_out_DATA[32 * 4] = out_data.GPS_delay;
    *(float *)&Usart1_out_DATA[33 * 4] = out_data.Temp;
    // printf("uartgyrp:%d\n", Usart1_out_DATA[2 * 4]);
    checksum = 0, pcheck = (uint8_t *)&Usart1_out_DATA[1 * 4];
    for (i = 4; i < 34 * 4; i++, pcheck++)
        checksum += *pcheck;
    *(uint32_t *)&Usart1_out_DATA[34 * 4] = checksum;
    //////////////////////////////////////////////////////////////////////////////////
    Usart1_out_Length = 35 * 4;
}
