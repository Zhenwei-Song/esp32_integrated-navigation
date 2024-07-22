#include "./../inc/mpu9250_raw.h"
#include "./../../../main/main.h"
#include "./../inc/empl_driver.h"

uint8_t mag_sensitivity[3];
short raw_gyr[3], raw_acc[3], raw_mag[3], raw_tem;

uint8_t RAW_MPU9250_Init(void)
{
    uint8_t res = 0;
    // IIC_Init();                                            // 初始化IIC总线
    MPU_Write_Byte(MPU9250_ADDR, MPU_PWR_MGMT1_REG, 0X80); // 复位MPU9250
    esp32_delay_ms(100);                                   // 延时100ms
    MPU_Write_Byte(MPU9250_ADDR, MPU_PWR_MGMT1_REG, 0x01); // 打开温度传感器  设置时钟
    MPU_Write_Byte(MPU9250_ADDR, MPU_PWR_MGMT1_REG, 0X00); // 唤醒MPU9250
    //MPU_Set_Gyro_Fsr(1);                                   // 陀螺仪传感器,±500dps
    //MPU_Set_Accel_Fsr(1);                                  // 加速度传感器,±4g
    MPU_Set_Gyro_Fsr(0);
    MPU_Set_Accel_Fsr(0);
    //MPU_Write_Byte(MPU9250_ADDR, ACCEL_CONFIG_2, 0x48);    // 加速度计滤波频率 1K OUT   200平滑(新增)
    //MPU_Write_Byte(MPU9250_ADDR, CONFIG, 0x43);            // 低通滤波时间（新增）
    MPU_Set_Rate(DEFAULT_HZ);                              // 设置陀螺仪采样率
    MPU_Write_Byte(MPU9250_ADDR, MPU_INT_EN_REG, 0X00);    // 关闭所有中断
    MPU_Write_Byte(MPU9250_ADDR, MPU_USER_CTRL_REG, 0X00); // I2C主模式关闭
    MPU_Write_Byte(MPU9250_ADDR, MPU_FIFO_EN_REG, 0X00);   // 关闭FIFO
    MPU_Write_Byte(MPU9250_ADDR, MPU_INTBP_CFG_REG, 0X82); // INT引脚低电平有效，开启bypass模式，可以直接读取磁力计
    res = MPU_Read_Byte(MPU9250_ADDR, MPU_DEVICE_ID_REG);  // 读取MPU6500的ID
    if (res == MPU6500_ID)                                 // 器件ID正确
    {
        MPU_Write_Byte(MPU9250_ADDR, MPU_PWR_MGMT1_REG, 0X01); // 设置CLKSEL,PLL X轴为参考
        MPU_Write_Byte(MPU9250_ADDR, MPU_PWR_MGMT2_REG, 0X00); // 加速度与陀螺仪都工作
        MPU_Set_Rate(DEFAULT_HZ);                              // 设置采样率为50Hz
    }
    else
        return 1;

    res = MPU_Read_Byte(AK8963_ADDR, MAG_WIA); // 读取AK8963 ID
    if (res == AK8963_ID) {
        MPU_Write_Byte(AK8963_ADDR, MAG_CNTL1, 0X11); // 设置AK8963为单次测量模式
    }
    else
        return 2;
    res = MPU_Read_Len(AK8963_ADDR, AK8963_ASAX, 3, mag_sensitivity);
    if (res != 0) {
        return 3;
    }
    printf("mag_sensitivity:%d,%d,%d\n", mag_sensitivity[0], mag_sensitivity[1], mag_sensitivity[2]);
    printf("RAW_MPU9250_Init finished\n");
    return 0;
}

// 设置MPU9250陀螺仪传感器满量程范围
// fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
// 返回值:0,设置成功
//     其他,设置失败
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU9250_ADDR, MPU_GYRO_CFG_REG, fsr << 3); // 设置陀螺仪满量程范围
}
// 设置MPU9250加速度传感器满量程范围
// fsr:0,±2g;1,±4g;2,±8g;3,±16g
// 返回值:0,设置成功
//     其他,设置失败
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU9250_ADDR, MPU_ACCEL_CFG_REG, fsr << 3); // 设置加速度传感器满量程范围
}

// 设置MPU9250的数字低通滤波器
// lpf:数字低通滤波频率(Hz)
// 返回值:0,设置成功
//     其他,设置失败
uint8_t MPU_Set_LPF(uint16_t lpf)
{
    uint8_t data = 0;
    if (lpf >= 188)
        data = 1;
    else if (lpf >= 98)
        data = 2;
    else if (lpf >= 42)
        data = 3;
    else if (lpf >= 20)
        data = 4;
    else if (lpf >= 10)
        data = 5;
    else
        data = 6;
    return MPU_Write_Byte(MPU9250_ADDR, MPU_CFG_REG, data); // 设置数字低通滤波器
}

// 设置MPU9250的陀螺仪采样率(假定Fs=1KHz)
// rate:4~1000(Hz)
// 返回值:0,设置成功
//     其他,设置失败
uint8_t MPU_Set_Rate(uint16_t rate)
{
    uint8_t data;
    if (rate > 1000)
        rate = 1000;
    if (rate < 4)
        rate = 4;
    data = 1000 / rate - 1;
    data = MPU_Write_Byte(MPU9250_ADDR, MPU_SAMPLE_RATE_REG, data); // 设置数字低通滤波器
    //return MPU_Set_LPF(rate / 5);                                   // 自动设置LPF为采样率的一半
    return MPU_Set_LPF(5);
    return 0;
}

// 得到温度值
// 返回值:温度值(扩大了100倍)
short RAW_MPU_Get_Temperature(void)
{
    uint8_t buf[2];
    short raw;
    float temp;
    MPU_Read_Len(MPU9250_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
    raw = ((uint16_t)buf[0] << 8) | buf[1];
    temp = 21 + ((double)raw) / 333.87;
    return temp * 100;
    ;
}
// 得到陀螺仪值(原始值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
// 返回值:0,成功
//     其他,错误代码
uint8_t RAW_MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
    uint8_t buf[6];
    MPU_Read_Len(MPU9250_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
    *gx = ((uint16_t)buf[0] << 8) | buf[1];
    *gy = ((uint16_t)buf[2] << 8) | buf[3];
    *gz = ((uint16_t)buf[4] << 8) | buf[5];
    return 0;
}
// 得到加速度值(原始值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
// 返回值:0,成功
//     其他,错误代码
uint8_t RAW_MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
    uint8_t buf[6];
    MPU_Read_Len(MPU9250_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
    *ax = ((uint16_t)buf[0] << 8) | buf[1];
    *ay = ((uint16_t)buf[2] << 8) | buf[3];
    *az = ((uint16_t)buf[4] << 8) | buf[5];
    return 0;
}

// 得到磁力计值(原始值)
// mx,my,mz:磁力计x,y,z轴的原始读数(带符号)
// 返回值:0,成功
//     其他,错误代码
uint8_t RAW_MPU_Get_Magnetometer(short *mx, short *my, short *mz)
{
    uint8_t res;
    uint8_t buf[6];
    MPU_Read_Len(AK8963_ADDR, MAG_XOUT_L, 6, buf);
    *mx = ((uint16_t)buf[1] << 8) | buf[0];
    *my = ((uint16_t)buf[3] << 8) | buf[2];
    *mz = ((uint16_t)buf[5] << 8) | buf[4];
    res= MPU_Write_Byte(AK8963_ADDR, MAG_CNTL1, 0X11); // AK8963每次读完以后都需要重新设置为单次测量模式
    if(res != 0){
        printf("Error mag write\n");
    }
    return 0;
    ;
}

uint8_t MPU_Write_Byte(uint8_t devaddr, uint8_t reg, uint8_t data)
{
    uint8_t res;
    res = esp32_i2c_write(devaddr, reg, 1, &data);
    return res;
}

uint8_t MPU_Read_Byte(uint8_t devaddr, uint8_t reg)
{
    unsigned char data;
    esp32_i2c_read(devaddr, reg, 1, &data);
    return data;
}

// IIC连续读
// addr:器件地址
// reg:要读取的寄存器地址
// len:要读取的长度
// buf:读取到的数据存储区
// 返回值:0,正常
//     其他,错误代码
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    esp32_i2c_read(addr, reg, len, buf);
    return 0;
}