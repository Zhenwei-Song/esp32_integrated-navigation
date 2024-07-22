/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-03-09 15:56:51
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-03-11 17:08:53
 * @FilePath: \esp32_positioning\components\niming\src\niming.c
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
#include "./../inc/ble_uart.h"

// 传送数据给匿名四轴上位机软件(V7.1版本)
// fun:功能字. 0X01~0X1C
// data:数据缓存区,最多28字节!!
// len:data区有效数据个数
void usart1_niming_report(uint8_t fun, uint8_t *data, uint8_t len)
{
    int i;
    uint8_t send_buf[32];
    if (len > 28)
        return;            // 最多28字节数据
    send_buf[len + 4] = 0; // 校验和数置零
    send_buf[len + 5] = 0; // 附加校验数置零
    send_buf[0] = 0xAA;    // 帧头
    send_buf[1] = 0xFF;    // 目标地址
    send_buf[2] = fun;     // 功能码
    send_buf[3] = len;     // 数据长度
    for (i = 0; i < len; i++)
        send_buf[4 + i] = data[i]; // 复制数据
    for (i = 0; i < len + 4; i++) {
        send_buf[len + 4] += send_buf[i];       // 计算校验和
        send_buf[len + 5] += send_buf[len + 4]; // 计算校验和
    }
    sendData_tx(TX_TAG, (const char *)send_buf);
}
// 发送加速度传感器数据+陀螺仪数据(传感器帧)
// aacx,aacy,aacz:x,y,z三个方向上面的加速度值
// gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
void send_sensorData(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz)
{
    uint8_t tbuf[12];
    tbuf[0] = aacx & 0XFF;
    tbuf[1] = (aacx >> 8) & 0XFF;
    tbuf[2] = aacy & 0XFF;
    tbuf[3] = (aacy >> 8) & 0XFF;
    tbuf[4] = aacz & 0XFF;
    tbuf[5] = (aacz >> 8) & 0XFF;
    tbuf[6] = gyrox & 0XFF;
    tbuf[7] = (gyrox >> 8) & 0XFF;
    tbuf[8] = gyroy & 0XFF;
    tbuf[9] = (gyroy >> 8) & 0XFF;
    tbuf[10] = gyroz & 0XFF;
    tbuf[11] = (gyroz >> 8) & 0XFF;
    usart1_niming_report(0XF1, tbuf, 12);
}

// 通过串口1上报结算后的姿态数据给电脑(状态帧)
// roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
// pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
// yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
// csb:超声波高度,单位:cm
// prs:气压计高度,单位:mm
void usart1_report_imu_V7_1(short roll, short pitch, short yaw, uint8_t state)
{
    uint8_t tbuf[7];
    tbuf[0] = roll & 0XFF;
    tbuf[1] = (roll >> 8) & 0XFF;
    tbuf[2] = pitch & 0XFF;
    tbuf[3] = (pitch >> 8) & 0XFF;
    tbuf[4] = yaw & 0XFF;
    tbuf[5] = (yaw >> 8) & 0XFF;
    tbuf[6] = state;
    usart1_niming_report(0X03, tbuf, 7); // 功能码ID，0X03，飞控姿态：欧拉角格式
}