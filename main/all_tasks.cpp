#include "./all_tasks.h"

#include <stdio.h>
#include <stdlib.h>

#include "./../components/mpu9250/inc/empl_driver.h"
#include "./../components/mpu_timer/inc/positioning_timer.h"
#include "./../components/my_uart/inc/my_uart.h"

#ifdef USING_DMP
#include "./../components/mpu9250/inc/inv_mpu.h"
#include "./../components/mpu9250/inc/mpu_dmp_driver.h"
#endif // USING_DMP

#ifdef USING_RAW
#include "./../components/mpu9250/inc/mpu9250_raw.h"
#endif // USING_RAW

#ifdef USING_PSINS
#include "./../components/mpu9250/inc/mpu_dmp_driver.h"
#include "./../components/psins/inc/KFApp.h"
#ifdef PSINS_UART
#include "./../components/psins/inc/uart_out.h"
#endif
#endif // USING_PSINS

#ifdef USING_DMP
// float magCalibration[3];
bool data_updated = true;
static s_point new_point;

#endif // USING_DMP

#ifdef USING_RAW
SemaphoreHandle_t xCountingSemaphore_push_data;
// #ifdef YAW_INIT
// const double declination = 0.0;   // 磁偏角(rad)
// CVect3 wm, vm, eb, db, mag;
// CQuat qnb0;
// #endif // YAW_INIT

#ifdef GET_ACC_WITHOUT_G
#include "./../components/get_acc_without_g/inc/get_acc_without_g.h"
#endif // GET_ACC_WITHOUT_G

#endif // USING_RAW

#ifdef PSINS_ATT
SemaphoreHandle_t xCountingSemaphore_data_update;
#endif // PSINS_ATT

#ifdef PSINS_POS
// CKFApp kf(my_TS);
#if defined USING_RAW || defined USING_DMP
SemaphoreHandle_t xCountingSemaphore_data_update;
#endif
#endif // PSINS_POS

static char mag_read_flag = 2;

#ifdef USING_DMP
void ins_init(void)
{
    new_point.q[0] = 1;
    new_point.q[1] = 0;
    new_point.q[2] = 0;
    new_point.q[3] = 0;
    printf("ins_init finished\n");
}

/**
 * @description: 通过中断获取DMP中传来的数据
 * @param {void} *arg
 * @return {*}
 */
void get_dmp_data(void *arg)
{
    uint32_t io_num;
    // static int16_t magCount[3];
    s_point point_got;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
#if defined USING_PSINS
            if (data_updated == true) { // 上一个数据处理完
                data_updated = false;
                gyro_data_ready_cb();
                dmp_get_data(&point_got);
                for (int i = 0; i < 3; i++) {
                    new_point.acc[i] = point_got.acc[i];
                    new_point.gyr[i] = point_got.gyr[i];
                    new_point.linear_acc[i] = point_got.linear_acc[i];
                    new_point.acc_fifo[i] = point_got.acc_fifo[i];
                    new_point.gyr_fifo[i] = point_got.gyr_fifo[i];
#ifdef GET_RAW_INFO
                    new_point.acc_raw[i] = point_got.acc_raw[i];
                    new_point.gyr_raw[i] = point_got.gyr_raw[i];
#endif // GET_RAW_INFO
                }
                // mpu_AD_value.Gyro[0] = new_point.gyr_fifo[0]; // fifo读数和直接寄存器读数相差一个负号
                // mpu_AD_value.Gyro[1] = new_point.gyr_fifo[1]; // fifo读数和直接寄存器读数相差一个负号
                // mpu_AD_value.Gyro[2] = new_point.gyr_fifo[2];
                // mpu_AD_value.Gyro[0] = 0; // fifo读数和直接寄存器读数相差一个负号
                // mpu_AD_value.Gyro[1] = 0; // fifo读数和直接寄存器读数相差一个负号
                // mpu_AD_value.Gyro[2] = 0;
                for (int i = 0; i < 3; i++) {
                    mpu_AD_value.Accel[i] = new_point.acc_fifo[i];
                    mpu_AD_value.Gyro[i] = new_point.gyr_fifo[i];
                    mpu_Data_value.Accel[i] = (double)mpu_AD_value.Accel[i] / (double)A_RANGE_NUM;
                    mpu_Data_value.Gyro[i] = (double)mpu_AD_value.Gyro[i] / (double)65.5;
                }
                // readMagData(magCount);
                // printf("readMagData:(%d,%d,%d)\n", magCount[0], magCount[1], magCount[2]);
// printf("new_point.acc:(%.3f,%.3f,%.3f)\n", new_point.acc[0], new_point.acc[1], new_point.acc[2]);
// printf("new_point.linear_acc:(%.3f,%.3f,%.3f)\n", new_point.linear_acc[0], new_point.linear_acc[1], new_point.linear_acc[2]);
// printf("new_point.gyr(rad):(%.3f,%.3f,%.3f)\n", new_point.gyr[0], new_point.gyr[1], new_point.gyr[2]);
#ifdef PSINS_ATT
                xSemaphoreGive(xCountingSemaphore_data_update);
#endif
#ifdef PSINS_POS
                xSemaphoreGive(xCountingSemaphore_data_update);
#endif
            }
#endif // USING_PSINS
        }
    }
}

#endif // USING_DMP

#ifdef USING_RAW
/**
 * @description: 监听timer3超时，定时从寄存器读取数据（raw，i2c）
 * @param {void} *pvParameters
 * @return {*}
 */
void get_raw_data_i2c(void *pvParameters)
{
#ifdef DOWN_SAMPLING
    static int temp_count = 0;
    short temp_gyr[3] = {0};
    short temp_acc[3] = {0};
    short temp_mag[3] = {0};
    short temp_tem = 0;
    static float sum_gyr[3] = {0};
    static float sum_acc[3] = {0};
    static float sum_mag[3] = {0};
    static float sum_tem;
    int mag_count = 1;
    int mag_out_count = 0;
#endif
    while (1) {
        if (xSemaphoreTake(xCountingSemaphore_timeout3, portMAX_DELAY) == pdTRUE) // 得到了信号量
        {
            // printf("timer3_check_task\n");
#ifdef DOWN_SAMPLING

            /* -------------------------------------------------------------------------- */
            /*                              读取磁力，固定4ms(250hz)读取一次                              */
            /* -------------------------------------------------------------------------- */
            // if (mag_count < MAG_SAMPLE_FACTOR) {
            //     mag_count = mag_count + 1;
            //     //printf("mag_count %d\n", mag_count);
            // }
            // else {
            //     RAW_MPU_Get_Magnetometer(&temp_mag[0], &temp_mag[1], &temp_mag[2]); // 读取磁力计原始数据
            //     for (int j = 0; j < 3; j++) {
            //         sum_mag[j] = sum_mag[j] + temp_mag[j];
            //         //printf("sum_mag %f\n", sum_mag[j]);
            //     }
            //     //printf("RAW_MPU_Get_Magnetometer\n");
            //     mag_count = 1;
            //     mag_out_count = mag_out_count + 1;
            //     //printf("mag_out_count %d\n", mag_out_count);
            //     if (mag_out_count == (MAG_SAMPLE_RATE / OUT_SAMPING_RATE)) {
            //         for (int j = 0; j < 3; j++) {
            //             sum_mag[j] = sum_mag[j] / (MAG_SAMPLE_RATE / OUT_SAMPING_RATE);
            //             raw_mag[j] = (short)sum_mag[j];
            //             sum_mag[j] = 0;
            //             // printf("MAG_SAMPLE_FACTOR %d\n", MAG_SAMPLE_FACTOR);
            //         }
            //         //printf("mag_out_\n");
            //         mag_out_count = 0;
            //     }
            // }

            /* -------------------------------------------------------------------------- */
            /*                                 读取加计，陀螺仪和温度                                */
            /* -------------------------------------------------------------------------- */
            // printf("get_raw_data_i2c checkpoint 0\n");
            if (temp_count < DOWNSAMPLE_FACTOR) {
                RAW_MPU_Get_Gyroscope(&temp_gyr[0], &temp_gyr[1], &temp_gyr[2]);     // 读取角速度原始数据
                RAW_MPU_Get_Accelerometer(&temp_acc[0], &temp_acc[1], &temp_acc[2]); // 读取角加速度原始数据
                RAW_MPU_Get_Magnetometer(&temp_mag[0], &temp_mag[1], &temp_mag[2]);  // 读取磁力计原始数据
                temp_tem = RAW_MPU_Get_Temperature();
                // for (int j = 0; j < 3; j++) {
                //     temp_gyr[j] = 0;
                //     temp_acc[j] = 0;
                //     temp_mag[j] = 0;
                // }
                for (int j = 0; j < 3; j++) {
                    sum_gyr[j] = sum_gyr[j] + temp_gyr[j];
                    sum_acc[j] = sum_acc[j] + temp_acc[j];
                    sum_mag[j] = sum_mag[j] + temp_mag[j];
                }
                sum_tem = sum_tem + temp_tem;
                temp_count = temp_count + 1;
                timer3_flag = false;
                // printf("temp_count %d\n", temp_count);
            }
            else {
                for (int j = 0; j < 3; j++) {
                    sum_gyr[j] = sum_gyr[j] / DOWNSAMPLE_FACTOR;
                    sum_acc[j] = sum_acc[j] / DOWNSAMPLE_FACTOR;
                    sum_mag[j] = sum_mag[j] / DOWNSAMPLE_FACTOR;
                    raw_gyr[j] = (short)sum_gyr[j];
                    raw_acc[j] = (short)sum_acc[j];
                    raw_mag[j] = (short)sum_mag[j];
                    sum_gyr[j] = 0;
                    sum_acc[j] = 0;
                    sum_mag[j] = 0;
                }
                sum_tem = sum_tem / DOWNSAMPLE_FACTOR;
                raw_tem = (short)sum_tem;
                sum_tem = 0;
                temp_count = 0;
                // printf("down sampling\n");
                // printf("Accx:%d,Accy:%d,Accz:%d\nGyrox:%d,Gyroy:%d,Gyroz:%d\nMagx:%d,Magy:%d,Magz:%d\n\n",
                //        raw_acc[0], raw_acc[1], raw_acc[2], raw_gyr[0], raw_gyr[1], raw_gyr[2], raw_mag[0], raw_mag[1], raw_mag[2]); // 源数据串口输出
                xSemaphoreGive(xCountingSemaphore_push_data);
                timer3_flag = false;
            }
#else  // DOWN_SAMPLING
            RAW_MPU_Get_Gyroscope(&raw_gyr[0], &raw_gyr[1], &raw_gyr[2]);     // 读取角速度原始数据
            RAW_MPU_Get_Accelerometer(&raw_acc[0], &raw_acc[1], &raw_acc[2]); // 读取角加速度原始数据

            // RAW_MPU_Get_Magnetometer(&raw_mag[0], &raw_mag[1], &raw_mag[2]);  // 读取磁力计原始数据
            if (mag_read_flag % 2 == 0) {
                RAW_MPU_Get_Magnetometer(&raw_mag[0], &raw_mag[1], &raw_mag[2]); // 读取磁力计原始数据
                if (mag_read_flag == 200)
                 mag_read_flag = 0; 
            }
            mag_read_flag = mag_read_flag + 1;
            // printf("%d", mag_read_flag);
            raw_tem = RAW_MPU_Get_Temperature();
            //  printf("Accx:%d,Accy:%d,Accz:%d\nGyrox:%d,Gyroy:%d,Gyroz:%d\nMagx:%d,Magy:%d,Magz:%d\n\n",
            //         raw_acc[0], raw_acc[1], raw_acc[2], raw_gyr[0], raw_gyr[1], raw_gyr[2], raw_mag[0], raw_mag[1], raw_mag[2]); // 源数据串口输出
            xSemaphoreGive(xCountingSemaphore_push_data);
#endif // DOWN_SAMPLING
        }
    }
}

/**
 * @description: 将读取到的数据传给解算程序
 * @param {void} *pvParameters
 * @return {*}
 */
void push_raw_data(void *pvParameters)
{
    while (1) {
        if (xSemaphoreTake(xCountingSemaphore_push_data, portMAX_DELAY) == pdTRUE) // 得到了信号量
        {
#if defined PSINS_ATT || defined PSINS_POS
            for (int i = 0; i < 3; i++) {
                mpu_AD_value.Accel[i] = raw_acc[i];
                mpu_AD_value.Gyro[i] = raw_gyr[i];
                mpu_AD_value.Mag[i] = raw_mag[i];
                mpu_AD_value.Temp = raw_tem;
                mpu_Data_value.Accel[i] = (double)mpu_AD_value.Accel[i] / (double)A_RANGE_NUM;
                // mpu_Data_value.Gyro[i] = (double)mpu_AD_value.Gyro[i] / (double)65.536;
                mpu_Data_value.Gyro[i] = (double)mpu_AD_value.Gyro[i] / (double)131.072;
                mpu_Data_value.Mag[i] = mpu_AD_value.Mag[i] * (double)0.25 * ((double)1 + ((double)mag_sensitivity[i] - (double)128) / (double)256);
                mpu_Data_value.Temp = ((double)mpu_AD_value.Temp / (double)333.87) + (double)21;
            }

#ifdef GET_ACC_WITHOUT_G
            vector3D linearAcc = {0, 0, 0};
            vector3D gravity = {0.0, 0.0, glv.g0};
            eulerAngles angles = {0, 0, 0};
            linearAcc.x = mpu_Data_value.Accel[0] * glv.g0;
            linearAcc.y = mpu_Data_value.Accel[1] * glv.g0;
            linearAcc.z = mpu_Data_value.Accel[2] * glv.g0;
            angles.pitch =
                vector3D_G acc_result = get_acc_without_g(linearAcc, gravity, angles);

#endif // GET_ACC_WITHOUT_G

#endif // defined PSINS_ATT || defined PSINS_POS
       // printf("push_raw_data check\n");
            xSemaphoreGive(xCountingSemaphore_data_update);
        }
    }
}

#endif // USING_RAW

#ifdef PSINS_ATT
/**
 * @description: PSINS中仅进行姿态解算
 * @param {void} *pvParameters
 * @return {*}
 */
void psins_att_data_update(void *pvParameters)
{
    CMahony mahony(10.0);
    // CVect3 eb = CVect3(-4.0, 1.3, 0.0) * glv.dps; // 陀螺零偏 deg/s
    CVect3 eb = CVect3(0, 0, 0) * glv.dps; // 陀螺零偏 deg/s
    CVect3 db = O31;
    double tmp1[3];
    double tmp2[3];
    while (1) {
        if (xSemaphoreTake(xCountingSemaphore_data_update, portMAX_DELAY) == pdTRUE) // 得到了信号量
        {
            // printf("psins_att_data_update check\n");
            CVect3 wm = (*(CVect3 *)mpu_Data_value.Gyro * glv.dps - eb) * my_TS;
            CVect3 vm = (*(CVect3 *)mpu_Data_value.Accel * glv.g0 - db) * my_TS;
            for (int i = 0; i < 3; i++) {
                tmp1[i] = (mpu_Data_value.Gyro[i] * glv.dps - 0) * my_TS;
                tmp2[i] = (mpu_Data_value.Accel[i] * glv.dps - 0) * my_TS;
            }
            // printf("glv.dps:%f\n", glv.dps);
            printf("wm:%f,%f,%f\n", tmp1[0], tmp1[1], tmp1[2]);
            printf("vm:%f,%f,%f\n", tmp2[0], tmp2[1], tmp2[2]);
            // printf("mpu_AD_value.Accel:(%d,%d,%d)\n", mpu_AD_value.Accel[0], mpu_AD_value.Accel[1], mpu_AD_value.Accel[2]);
            // printf("mpu_AD_value.Gyro:(%d,%d,%d)\n", mpu_AD_value.Gyro[0], mpu_AD_value.Gyro[1], mpu_AD_value.Gyro[2]);
            printf("mpu_Data_value.Accel:(%f,%f,%f)\n", mpu_Data_value.Accel[0], mpu_Data_value.Accel[1], mpu_Data_value.Accel[2]);
            printf("mpu_Data_value.Gyro:(%f,%f,%f)\n", mpu_Data_value.Gyro[0], mpu_Data_value.Gyro[1], mpu_Data_value.Gyro[2]);
            printf("mpu_Data_value.Mag:(%f,%f,%f)\n", mpu_Data_value.Mag[0], mpu_Data_value.Mag[1], mpu_Data_value.Mag[2]);
            mahony.Update(wm, vm, my_TS);
            AVPUartOut(q2att(mahony.qnb));
#ifdef USING_DMP
            data_updated = true;
#endif
        }
    }
}
#endif // PSINS_ATT

#ifdef PSINS_POS
/**
 * @description: PSINS中进行速度，姿态角，位置解算（存在问题）
 * @param {void} *pvParameters
 * @return {*}
 */
void psins_static_pos_data_update(void *pvParameters)
{
    printf("check point 0 psins_static_pos_data_update\n");
    psins_uart_init();
    CKFApp kf(my_TS);
    printf("check point 1 psins_static_pos_data_update\n");
    double yaw0 = C360CC180(0 * glv.deg); // 北偏东为正
    CVect3 gpspos = LLH(LATITUDE, LONGITUDE, ALTITUDE);
    kf.Init(CSINS(a2qua(CVect3(0, 0, yaw0)), O31, gpspos)); // 请正确初始化方位和位置
#if 0
    CVect3 eb = CVect3(-1.023, 2.12, 0.59) * glv.dps; // 陀螺零偏 deg/s
    // CVect3 db = CVect3(0.0040283, 0.016958, 1.0010945) * glv.g0;
    CVect3 db = O31;
#else
    CVect3 db = O31;
    CVect3 eb = O31;
#endif
#ifdef DEBUG
    printf("check point psins_static_pos_data_update\n");
#endif
    while (1) {
        if (xSemaphoreTake(xCountingSemaphore_data_update, portMAX_DELAY) == pdTRUE) {
            printf("psins_static_pos_data_update check\n");
#ifdef PSINS_POS_ON_BOARD
            CVect3 wm = (*(CVect3 *)mpu_Data_value.Gyro * glv.dps - eb) * my_TS;
            CVect3 vm = (*(CVect3 *)mpu_Data_value.Accel * glv.g0 - db) * my_TS;
            kf.Update(&wm, &vm, 1, my_TS, 5);
            AVPUartOut(kf);
#endif // PSINS_POS_ON_BOARD
#ifdef USING_DMP
            data_updated = true;
#endif
#if defined USING_RAW && !defined DOWN_SAMPLING
            timer3_flag = false;
#endif
        }
    }
}
#endif // PSINS_POS && defined USING_DMP

#if 0
/**
 * @description: 通过地磁来进行初始的航向角校准（未完成）
 * @return {*}
 */
void SINSGPS_byYawPos_main(void)
{
    AlignInit(3);
    //Mcu_ConfigGps = 0;
    CVect3 gpspos = LLH(LATITUDE, LONGITUDE, ALTITUDE);
    kf.Init(CSINS(a2qua(CVect3(0, 0, yaw0)), O31, gpspos));
    while (1) {
        if (xSemaphoreTake(xCountingSemaphore_data_update, portMAX_DELAY) == pdTRUE) {
            CVect3 wm = (*(CVect3 *)mpu_Data_value.Gyro * glv.dps - eb) * my_TS;
            CVect3 vm = (*(CVect3 *)mpu_Data_value.Accel * glv.g0 - db) * my_TS;
            kf.Update(&wm, &vm, 1, my_TS, 5);
            AVPUartOut(kf);
            //NavCalcuDone();
        }
    }
#ifdef USING_DMP
    data_updated = true;
#endif
#ifdef USING_RAW && !defined DOWN_SAMPLING
    timer3_flag = false;
#endif
}
}
#endif

#ifdef PSINS_UART
/**
 * @description: 用于PSINS中，从串口向上位机发送数据
 * @param {void} *pvParameters
 * @return {*}
 */
void psins_uart_pop_data(void *pvParameters)
{
    while (1) {
        if (xSemaphoreTake(xCountingSemaphore_timeout2, portMAX_DELAY) == pdTRUE) // 得到了信号量
        {
            Data_updata();
            psins_sendData_tx(TX_TAG, (const char *)Usart1_out_DATA, 35 * 4);
            timer2_flag = false;
        }
    }
}

#endif // PSINS_UART

#ifdef USING_SPI
/**
 * @description: 使用SPI时，定时从寄存器读取数据（raw，spi）
 * @param {void} *pvParameters
 * @return {*}
 */
void get_raw_data_spi(void *pvParameters)
{
    while (1) {
        if (xSemaphoreTake(xCountingSemaphore_timeout1, portMAX_DELAY) == pdTRUE) // 得到了信号量
        {
            READ_MPU9250_A_T_G();
            // READ_MPU9250_MAG(); // 读MPU9250：IMU和磁
            //  printf("get_raw_data_spi\n");
            xSemaphoreGive(xCountingSemaphore_data_update);
            timer1_flag = false;
        }
    }
}

#endif // USING_SPI