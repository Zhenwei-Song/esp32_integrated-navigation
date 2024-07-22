/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-02-28 18:53:28
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-03-19 16:40:02
 * @FilePath: \esp32_positioning\main\main.cpp
 * 驱动mpu9250，串口输出欧拉角，可用上位机进行串口连接查看图像
 * 利用官方dmp库输出欧拉角（使用I2C时）
 * 使用I2C连接,或者SPI(SPI存在陀螺仪z轴无数据情况，且速度慢，不推荐)
 * 引脚使用（I2C）：INT 23 SCL 22  SDA 21 供电3.3v
 * 引脚使用（SPI）: SCL 25 SDA 33 AD0 32 NCS 26
 * 采样率设置(I2C)：最快为200Hz(5ms采集一次)
 * 加速度量程设置：2到16g
 * 陀螺仪量程设置：250到2000deg/s
 * 读取mpu9250数据可采用dmp（i2c，磁力无法读），直接读寄存器（i2c），直接读寄存器（spi，存在数据读不全，速度慢问题）
 * 姿态解析可采用DMP（无磁力信息）、PSINS中解析，以及简化版捷联惯导sfann_sins(未验证)
 * psins可以使用串口上传数据，由电脑端运行psins进行解析
 *坐标广研院：23.305836,113.572256，59
 *添加了zupt(效果差)
 *添加了PDR(效果好)
 *添加了地磁测量，温度测量
 *添加了由地磁确定航向角的初始化方法
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
#include "./main.h"
#include "./all_tasks.h"

#include "./../components/mpu9250/inc/empl_driver.h"
#include "./../components/mpu_timer/inc/positioning_timer.h"

#ifdef USING_DMP
#include "./../components/mpu9250/inc/inv_mpu.h"
#include "./../components/mpu9250/inc/mpu_dmp_driver.h"
#endif // USING_DMP

#ifdef USING_RAW
#include "./../components/get_acc_without_g/inc/get_acc_without_g.h"
#include "./../components/mpu9250/inc/mpu9250_raw.h"
#endif // USING_RAW

#ifdef USING_SPI
#include "./../components/psins/inc/mcu_init.h"
#endif

// #ifdef USING_PSINS
// #include "./../components/mpu9250/inc/mpu_dmp_driver.h"
// #include "./../components/psins/inc/uart_out.h"
// #endif // USING_PSINS

#ifdef USING_PSINS
#include "./../components/mpu9250/inc/mpu_dmp_driver.h"
#include "./../components/psins/inc/KFApp.h"
#include "./../components/psins/inc/mcu_init.h"
#ifdef PSINS_UART
#include "./../components/psins/inc/uart_out.h"
#include "my_uart.h"
#endif // PSINS_UART
#ifdef YAW_INIT
#include "./../components/yaw_init_by_mag/inc/calibration.h"
#endif // YAW_INIT
#endif // USING_PSINS

#include "./../components/band_pass_filter/inc/band_pass_filter.h"
#include "./../components/shift_window/inc/shift_window.h"

#include <stdio.h>
#include <stdlib.h>

// static CKFApp kf(my_TS);

extern "C" void app_main(void)
{
/* -------------------------------------------------------------------------- */
/*                                   IMU数据获取                                  */
/* -------------------------------------------------------------------------- */
#ifdef USING_DMP
    positioning_timer_init();
    mpu_dmp_init();
    // initAK8963_2(magCalibration);
    i2c_gpio_init();
    ins_init();
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    start_i2c_isr();
    xTaskCreate(get_dmp_data, "get_dmp_data", 4096, NULL, 10, NULL);
#endif // USING_DMP

#ifdef USING_RAW
    uint8_t res = 0;
    mpu_init_i2c();
    res = RAW_MPU9250_Init();
    if (res != 0) {
        printf("raw_mPU9250_init() failed %d\n", res);
    }
    positioning_timer_init();
    esp_timer_start_periodic(positioning_time3_timer, TIME3_TIMER_PERIOD);
    printf("check point 0\n");
    xCountingSemaphore_timeout3 = xSemaphoreCreateCounting(200, 0);
    xCountingSemaphore_push_data = xSemaphoreCreateCounting(200, 0);
    xTaskCreatePinnedToCore(get_raw_data_i2c, "get_raw_data_i2c", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(push_raw_data, "push_raw_data", 4096, NULL, 8, NULL, 1);
#endif // USING_RAW

#ifdef USING_SPI
    xCountingSemaphore_timeout1 = xSemaphoreCreateCounting(200, 0);
    mcu_init();
    xTaskCreate(get_raw_data_spi, "get_raw_data_spi", 4096, NULL, 4, NULL);
#endif // USING_SPI
/* -------------------------------------------------------------------------- */
/*                                    数据解析                                    */
/* -------------------------------------------------------------------------- */
#ifdef PSINS_ATT
    xCountingSemaphore_data_update = xSemaphoreCreateCounting(200, 0);
    xTaskCreate(psins_att_data_update, "psins_att_data_update", 4096, NULL, 4, NULL);
#endif // PSINS_ATT

#ifdef PSINS_POS
#ifdef PSINS_UART
    esp_timer_start_periodic(positioning_time1_timer, TIME1_TIMER_PERIOD);
    xCountingSemaphore_timeout2 = xSemaphoreCreateCounting(200, 0);
    xTaskCreatePinnedToCore(psins_uart_pop_data, "psins_uart_pop_data", 8129, NULL, 7, NULL, 0);
    esp_timer_start_periodic(positioning_time2_timer, TIME2_TIMER_PERIOD);
#endif // PSINS_UART

    xCountingSemaphore_data_update = xSemaphoreCreateCounting(200, 0);
#if 0
    xTaskCreatePinnedToCore(psins_static_pos_data_update, "psins_static_pos_data_update", 15360, NULL, 9, NULL, 1);
#else

    psins_uart_init();
/* -------------------------------------------------------------------------- */
/*                             初始校准+卡尔曼滤波                            */
/* -------------------------------------------------------------------------- */
#if 1
    bool align_ok = false;
    bool zero_bias_cal_ok = false;
    bool yaw_init_ok = false;

    int bias_count = 0;
    CVect3 db = O31;
    CVect3 eb = O31;

    CVect3 v_temp = O31;
    CVect3 w_temp = O31;
    CVect3 wm = O31;
    CVect3 vm = O31;

    CVect3 my_att = O31;

    float temp_gyro_bias[3] = {0};
    float temp_acc_bias[3] = {0};
    float temp_mag[3] = {0};

    double yaw0 = C360CC180(0 * glv.deg); // 北偏东为正
    CVect3 pos0 = LLH(LATITUDE, LONGITUDE, ALTITUDE);
    CAligni0 align(pos0, O31, 0); // 静态对准
    CKFApp kf(my_TS);

    bool zupt_f_flag = false;
    bool zupt_w_flag = false;
    bool zupt_f_var_flag = false;
    int zupt_count = 0;

    float pdr_f_ave = 0;
    float pdr_f_final = 0;
    float pdr_filtered_signal = 0;
    float pdr_last_step_time = 0;
    float pdr_current_step_time = 0;
    float pdr_length_n = 0;
    float pdr_length_e = 0;
    float pdr_ay_max = 0; // 用于步长估计
    float pdr_ay_min = 0;
    bool pdr_after_first_step = false;
    float pdr_step_len_estimated = 0;
    int pdr_error_jump = 0;
    unsigned int pdr_count = 0;

    // 初始化带通滤波器
    band_pass_filter_s filter;
    init_band_pass_filter(&filter, 1.0, 0.5, SAMPLE_RATE); // 参数由人体步态特征给出

    float f_m2 = 0;
    float f_m = 0;
    float w_m2 = 0;
    float w_m = 0;

    printf("Initalizing!!\n");
    while (1) {
        if (xSemaphoreTake(xCountingSemaphore_data_update, portMAX_DELAY) == pdTRUE) {

            // mpu_Data_value.Accel[0] = 0;
            // mpu_Data_value.Accel[1] = 0;
            // mpu_Data_value.Accel[2] = 1;
            /* -------------------------------------------------------------------------- */
            /*                                累计测量，求平均值得到零偏                               */
            /* -------------------------------------------------------------------------- */
            if (zero_bias_cal_ok == false) {
                for (int i = 0; i < 3; i++) {
                    temp_gyro_bias[i] = temp_gyro_bias[i] + mpu_Data_value.Gyro[i];
                    temp_mag[i] = temp_mag[i] + mpu_Data_value.Mag[i];
                }
                temp_acc_bias[0] = temp_acc_bias[0] + mpu_Data_value.Accel[0];
                temp_acc_bias[1] = temp_acc_bias[1] + mpu_Data_value.Accel[1];
                temp_acc_bias[2] = temp_acc_bias[2] + (mpu_Data_value.Accel[2] - 1);
                bias_count = bias_count + 1;
                if (OUT_cnt > ZERO_BIAS_CAL_TIME + WAIT_TIME) {
                    zero_bias_cal_ok = true;
                    for (int i = 0; i < 3; i++) {
                        temp_gyro_bias[i] = temp_gyro_bias[i] / bias_count;
                        temp_acc_bias[i] = temp_acc_bias[i] / bias_count;
                        temp_mag[i] = temp_mag[i] / bias_count;
                        printf("temp_gyro_bias[%d] %f\n", i, temp_gyro_bias[i]);
                        printf("temp_acc_bias[%d] %f \n", i, temp_acc_bias[i]);
                    }
                    eb = CVect3(temp_gyro_bias[0], temp_gyro_bias[1], temp_gyro_bias[2]); // 陀螺零偏 deg/s
                    db = CVect3(temp_acc_bias[0], temp_acc_bias[1], temp_acc_bias[2]);
                    printf("Finish calculating bias!!\n");
                }
            }
            else {
                w_temp = (*(CVect3 *)mpu_Data_value.Gyro - eb) * glv.dps;
                v_temp = (*(CVect3 *)mpu_Data_value.Accel - db) * glv.g0;
                wm = w_temp * my_TS;
                vm = v_temp * my_TS;
                //  CVect3 vm = (*(CVect3 *)mpu_Data_value.Accel * glv.g0) * my_TS;
#if defined USING_ZUPT
                /* -------------------------------------------------------------------------- */
                /*                                    ZUPT                                    */
                /* -------------------------------------------------------------------------- */
                w_m2 = w_temp.i * w_temp.i + w_temp.j * w_temp.j + w_temp.k * w_temp.k;
                f_m2 = v_temp.i * v_temp.i + v_temp.j * v_temp.j + v_temp.k * v_temp.k;

                f_m = sqrt(f_m2); // 比力模值
                w_m = sqrt(w_m2); // 角速度模值
                // printf("zupt f:%f w:%f\n", f_m, w_m);

                zupt_shift_window(v_temp.i, v_temp.j, v_temp.k);
                zupt_f_var_flag = check_zupt_f_window_state(zupt_f_window_x, zupt_f_window_y, zupt_f_window_z, ZUPT_F_WINDOW_SIZE);
                if (f_m >= ZUPT_F_MIN && f_m <= ZUPT_F_MAX) {
                    zupt_f_flag = true;
                    // printf("zupt f true!!\n");
                    if (w_m <= ZUPT_W_THESH) {
                        zupt_w_flag = true;
                        // printf("zupt w true!!\n");
                        if (zupt_f_flag == true && zupt_w_flag == true && zupt_f_var_flag == true && align_ok == true) {
                            zupt_final_flag = true;
                            zupt_f_flag = false;
                            zupt_w_flag = false;
                            zupt_f_var_flag = false;
                            zupt_count = zupt_count + 1;
                            printf("main zupt  %d !!\n", zupt_count);
                            kf.SetMeasGNSS(O31, CVect3(0, 0, 0.00001));
                        }
                    }
                }

                if (align_ok == false) // 静态校准
                {
                    align.Update(&wm, &vm, 1, my_TS, O31);
                    if (OUT_cnt > (ALIGN_TIME + ZERO_BIAS_CAL_TIME + WAIT_TIME)) {
                        align_ok = true;
                        kf.Init(CSINS(a2qua(CVect3(0, 0, yaw0)), O31, pos0)); // 请正确初始化方位和位置
                        printf("Finish Aligning!!\n");
                    }
                }
                else // 后续更新
                {
                    kf.Update(&wm, &vm, 1, my_TS, 1);
                    AVPUartOut(kf);
                }

#elif defined USING_PDR
                /* -------------------------------------------------------------------------- */
                /*                                     PDR                                    */
                /* -------------------------------------------------------------------------- */
                w_m2 = w_temp.i * w_temp.i + w_temp.j * w_temp.j + w_temp.k * w_temp.k;
                f_m2 = v_temp.i * v_temp.i + v_temp.j * v_temp.j + v_temp.k * v_temp.k;

                f_m = sqrt(f_m2); // 比力模值
                w_m = sqrt(w_m2); // 角速度模值

                if (align_ok == false) // 静态校准
                {
                    align.Update(&wm, &vm, 1, my_TS, O31);
                    if (OUT_cnt > (ALIGN_TIME + ZERO_BIAS_CAL_TIME + WAIT_TIME)) {
                        align_ok = true;
                        kf.Init(CSINS(a2qua(CVect3(0, 0, yaw0)), O31, pos0)); // 请正确初始化方位和位置
                        printf("Finish Aligning!!\n");
                    }
                }
                else // 后续更新
                {
                    kf.Update(&wm, &vm, 1, my_TS, 1);
                    // AVPUartOut(kf);
                }

                if (align_ok == true) {
                    pdr_count = pdr_count + 1;
                    pdr_f_ave = (pdr_f_ave + f_m) / pdr_count; // 整个运动过程的平均加速度矢量
                    pdr_f_final = f_m - pdr_f_ave;             // 步数检测信号(消除人体运动过程中摆动及重力加速度带来的影响)
                    pdr_filtered_signal = process_step_detection(pdr_f_final, &filter);
                    if (pdr_after_first_step == true) {
                        if (pdr_ay_max < v_temp.j) // 获得单步y轴最大加速度
                            pdr_ay_max = v_temp.j;
                        if (pdr_ay_min > v_temp.j) // 获得单步y轴最小加速度
                            pdr_ay_min = v_temp.j;
                    }

                    if (pdr_filtered_signal > 0.55 && vm.k < 0.2) {
                        pdr_current_step_time = OUT_cnt;
                        if (pdr_current_step_time - pdr_last_step_time > 450) { // 0.45s

                            // printf("sinal before: %f, pdr_filtered:%f\n", pdr_f_final, pdr_filtered_signal);
                            // kf.SetMeasGNSS(O31, CVect3(0, 0, 0.00001));

#if 1  // 使用步长估计

                            if (pdr_error_jump >= 2) { // 开机后会误测两次
                                if (pdr_after_first_step == false) {
                                    pdr_length_e = PDR_STEP_LENGTH * sin(CC180C360(kf.sins.att.k)); // 应该先进性kf.Update更新姿态角
                                    pdr_length_n = PDR_STEP_LENGTH * cos(CC180C360(kf.sins.att.k)); // 应该先进性kf.Update更新姿态角
                                    pdr_after_first_step = true;
                                }
                                else { // 步长估计
                                    pdr_step_len_estimated = PDR_STEP_K * pow((pdr_ay_max - pdr_ay_min), 0.25);
                                    printf("pdr_step_len_estimated: %f\n", pdr_step_len_estimated);
                                    pdr_length_e = pdr_step_len_estimated * sin(CC180C360(kf.sins.att.k)); // 应该先进性kf.Update更新姿态角
                                    pdr_length_n = pdr_step_len_estimated * cos(CC180C360(kf.sins.att.k)); // 应该先进性kf.Update更新姿态角
                                    pdr_ay_max = pdr_ay_min = 0;
                                }
                                pdr_length[0] = pdr_length_e;
                                pdr_length[1] = pdr_length_n;
                            }
                            else {
                                pdr_error_jump = pdr_error_jump + 1;
                            }
#else  // 仅使用固定步长
                            pdr_length_e = PDR_STEP_LENGTH * sin(CC180C360(kf.sins.att.k)); // 应该先进性kf.Update更新姿态角
                            pdr_length_n = PDR_STEP_LENGTH * cos(CC180C360(kf.sins.att.k)); // 应该先进性kf.Update更新姿态角
                            pdr_length[0] = pdr_length_e;
                            pdr_length[1] = pdr_length_n;
#endif // 使用步长估计

                            // printf("yaw: %f\n", CC180C360(kf.sins.att.k) / DEG);
                            // printf("pdr_length_n: %f, pdr_length_e:%f\n", pdr_length_n, pdr_length_e);
                            printf("pdr start!\n");
                            pdr_last_step_time = pdr_current_step_time;
                        }
                    }
                    AVPUartOut(kf);
                    pdr_length_e = pdr_length_n = 0;
                }

#else // 纯惯导
                if (align_ok == false) // 静态校准
                {
                    align.Update(&wm, &vm, 1, my_TS, O31);
                    if (OUT_cnt > (ALIGN_TIME + ZERO_BIAS_CAL_TIME + WAIT_TIME)) {
                        align_ok = true;
                        my_att = q2att(align.qnb); // 获取对准结束后的姿态阵（四元数）,不用qnb
                        my_att.k = yaw0;
#ifndef YAW_INIT
                        kf.Init(CSINS(my_att, O31, pos0)); // 请正确初始化方位和位置
#endif // YAW_INIT
                        printf("Finish Align!!\n");
                    }
                }
                else // 后续更新
                {
#ifdef YAW_INIT
                    if (yaw_init_ok == false) {
                        CVect3 Mag = CVect3(temp_mag);
                        IMURFU(&Mag, 1, "FRD");
                        Mag = (Mag * 0.01 - b) * A;
                        Mag = a2qua(my_att) * Mag;
                        double yaw = atan2Ex(Mag.i, Mag.j) + yaw0;
                        if (yaw > PI)
                            yaw = yaw - 2 * PI;
                        else if (yaw < -PI)
                            yaw = yaw + 2 * PI;
                        my_att.k = yaw;
                        kf.Init(CSINS(my_att, O31, pos0)); // 请正确初始化方位和位置
                        yaw_init_ok = true;
                    }
                    else {
                        kf.Update(&wm, &vm, 1, my_TS, 1);
                        AVPUartOut(kf);
                    }
#else  //! def YAW_INIT
                    kf.Update(&wm, &vm, 1, my_TS, 1);
                    AVPUartOut(kf);
#endif // YAW_INIT
                }

#endif
            }

#ifdef USING_DMP
            data_updated = true;
#endif
#if defined USING_RAW && !defined DOWN_SAMPLING
            timer3_flag = false;
#endif
        }
    }

/* -------------------------------------------------------------------------- */
/*                                  初始校准+捷联惯导                                 */
/* -------------------------------------------------------------------------- */
#elif 1
    float my_vm[3] = {0};
    float temp_gyro_bias[3] = {0};
    float temp_acc_bias[3] = {0};
    double yaw0 = C360CC180(0 * glv.deg); // 北偏东为正
    CVect3 pos0 = LLH(LATITUDE, LONGITUDE, ALTITUDE);
    CVect3 v00 = O31;

    CVect3 v_temp = O31;
    CVect3 w_temp = O31;
    CVect3 wm = O31;
    CVect3 vm = O31;
    CVect3 db = O31;
    CVect3 eb = O31;
    // CSINS sins;
    CSINS sins((CVect3(0, 0, yaw0)), O31, pos0);
    CAlignkf aln(sins, my_TS);
    bool wait_ok = false;
    bool alnOK = false;
    bool aln_init_OK = false;
    bool zero_bias_cal_ok = false;
    int bias_count = 0;

    // bool zupt_final_flag = false;
    bool zupt_f_flag = false;
    bool zupt_w_flag = false;
    float f_m2 = 0;
    float f_m = 0;
    float w_m2 = 0;
    float w_m = 0;

    printf("Initalizing!!\n");
    while (1) {
        if (xSemaphoreTake(xCountingSemaphore_data_update, portMAX_DELAY) == pdTRUE) {
            if (zero_bias_cal_ok == false) {
                if (wait_ok == false) {
                    if (OUT_cnt > WAIT_TIME)
                        wait_ok = true;
                }
                else {
                    for (int i = 0; i < 3; i++) {
                        temp_gyro_bias[i] = temp_gyro_bias[i] + mpu_Data_value.Gyro[i];
                        // temp_acc_bias[i] = temp_acc_bias[i] + mpu_Data_value.Accel[i];
                    }
                    temp_acc_bias[0] = temp_acc_bias[0] + mpu_Data_value.Accel[0];
                    temp_acc_bias[1] = temp_acc_bias[1] + mpu_Data_value.Accel[1];
                    temp_acc_bias[2] = temp_acc_bias[2] + (mpu_Data_value.Accel[2] - 1);
                    bias_count = bias_count + 1;
                    if (OUT_cnt > ZERO_BIAS_CAL_TIME + WAIT_TIME) {
                        zero_bias_cal_ok = true;
                        for (int i = 0; i < 3; i++) {
                            temp_gyro_bias[i] = temp_gyro_bias[i] / bias_count;
                            temp_acc_bias[i] = temp_acc_bias[i] / bias_count;
                            printf("temp_gyro_bias[%d] %f\n", i, temp_gyro_bias[i]);
                            printf("temp_acc_bias[%d] %f \n", i, temp_acc_bias[i]);
                        }
                        // eb = CVect3(temp_gyro_bias[0], temp_gyro_bias[1], temp_gyro_bias[2]) * glv.dps; // 陀螺零偏 deg/s
                        eb = CVect3(temp_gyro_bias[0], temp_gyro_bias[1], temp_gyro_bias[2]); // 陀螺零偏 deg/s

                        // kf.Init(CSINS(a2qua(CVect3(0, 0, yaw0)), O31, pos0));                           // 请正确初始化方位和位置
                        // db = CVect3(temp_acc_bias[0], temp_acc_bias[1], temp_acc_bias[2]) * glv.g0;
                        db = CVect3(temp_acc_bias[0], temp_acc_bias[1], temp_acc_bias[2]);
                        printf("Finish calculating bias  SINS!!\n");
                    }
                }
            }
            else {
                w_temp = (*(CVect3 *)mpu_Data_value.Gyro - eb) * glv.dps;
                v_temp = (*(CVect3 *)mpu_Data_value.Accel - db) * glv.g0;
                wm = w_temp * my_TS;
                vm = v_temp * my_TS;

                /* -------------------------------------------------------------------------- */
                /*                                    ZUPT                                    */
                /* -------------------------------------------------------------------------- */
                // f_m2 = mpu_Data_value.Accel[0] * mpu_Data_value.Accel[0] + mpu_Data_value.Accel[1] * mpu_Data_value.Accel[1] + mpu_Data_value.Accel[2] * mpu_Data_value.Accel[2];
                // w_m2 = mpu_Data_value.Gyro[0] * mpu_Data_value.Gyro[0] + mpu_Data_value.Gyro[1] * mpu_Data_value.Gyro[1] + mpu_Data_value.Gyro[2] * mpu_Data_value.Gyro[2];

                w_m2 = w_temp.i * w_temp.i + w_temp.j * w_temp.j + w_temp.k * w_temp.k;
                f_m2 = v_temp.i * v_temp.i + v_temp.j * v_temp.j + v_temp.k * v_temp.k;

                f_m = sqrt(f_m2); // 比力模值
                w_m = sqrt(w_m2); // 角速度模值
                // printf("zupt f:%f w:%f\n", f_m, w_m);
                if (f_m >= ZUPT_F_MIN && f_m <= ZUPT_F_MAX) {
                    zupt_f_flag = true;
                    // printf("zupt f true!!\n");
                    if (w_m <= ZUPT_W_THESH) {
                        zupt_w_flag = true;
                        // printf("zupt w true!!\n");
                        if (zupt_f_flag == true && zupt_w_flag == true && alnOK == true) {
                            zupt_final_flag = true;
                            zupt_f_flag = false;
                            zupt_w_flag = false;
                            // printf("zupt!!\n");
                        }
                    }
                }

                if (alnOK == false) {
                    if (aln_init_OK == false) {
                        aln.Init(sins);
                        aln_init_OK = true;
                    }
                    aln.Update(&wm, &vm, 1, my_TS);
                    if (OUT_cnt > (ALIGN_TIME + ZERO_BIAS_CAL_TIME + WAIT_TIME)) {
                        alnOK = true;
                        v00 = CVect3(0, 0, 0);
                        sins.Init(aln.qnb, v00, pos0, aln.kftk);
                        // sins.db = db / glv.g0;
                        // sins.eb = eb / glv.dps;
                        printf("Finish Aligning!!\n");
                        printf("my_TS: %f\n", my_TS);
                    }
                }
                else {
                    sins.Update(&wm, &vm, 1, my_TS);
                    // AVPUartOut(sins);
#if 0
                    for (int i = 0; i < 3; i++) {
                        // my_vm[i] = (mpu_Data_value.Accel[i] * glv.g0 - temp_acc_bias[i] * glv.g0) * my_TS;
                        // my_vm[i] = (mpu_Data_value.Accel[i] * glv.g0) * my_TS;
                        my_v[i] = my_v[i] + my_vm[i];
                    }
                    my_vm[0] = (mpu_Data_value.Accel[0] * glv.g0 - temp_acc_bias[0] * glv.g0) * my_TS;
                    my_vm[1] = (mpu_Data_value.Accel[1] * glv.g0 - temp_acc_bias[1] * glv.g0) * my_TS;
                    my_vm[2] = (mpu_Data_value.Accel[2] * glv.g0 - (temp_acc_bias[2] + 1) * glv.g0) * my_TS;
                    // printf("my_vm: (%f,%f,%f)\n", my_vm[0], my_vm[1], my_vm[2]);
                    printf("my_v: (%f,%f,%f)\n", my_v[0], my_v[1], my_v[2]);
#endif
                    AVPUartOut(q2att(sins.qnb));
                }
            }
#ifdef USING_DMP
            data_updated = true;
#endif
#if defined USING_RAW && !defined DOWN_SAMPLING
            timer3_flag = false;
#endif
        }
    }

#endif

#endif

#endif // PSINS_POS && USING_RAW
}
