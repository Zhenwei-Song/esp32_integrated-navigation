/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-03-26 16:11:33
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-07-17 16:41:48
 * @FilePath: \esp32_positioning\components\my_uart\src\my_uart.c
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
#include "my_uart.h"

int RX_BUF_SIZE = 1024;

void psins_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 460800,                   // 设置波特率    115200
        .data_bits = UART_DATA_8_BITS,         // 设置数据位    8位
        .parity = UART_PARITY_DISABLE,         // 设置奇偶校验  不校验
        .stop_bits = UART_STOP_BITS_1,         // 设置停止位    1
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // 设置硬件流控制 不使能
        .source_clk = UART_SCLK_DEFAULT,       // 设置时钟源
    };
    // 安装串口驱动 串口编号、接收buff、发送buff、事件队列、分配中断的标志
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0));
    // 串口参数配置 串口号、串口配置参数
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    // 设置串口引脚号 串口编号、tx引脚、rx引脚、rts引脚、cts引脚
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

}

int psins_sendData_tx(const char *logName, const char *data, int len)
{
    // const int len = strlen(data);                                // 获取数据长度  字符数据
    const int txBytes = uart_write_bytes(UART_NUM_1, data, (const int)len); // 发送数据
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);                           // log打印
    return txBytes;
}
