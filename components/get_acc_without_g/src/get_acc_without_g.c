/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-03-26 16:11:33
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-05-08 11:26:50
 * @FilePath: \esp32_positioning\components\get_acc_without_g\src\get_acc_without_g.c
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
#include "./../inc/get_acc_without_g.h"

#define TEMP_PI 3.14159265358979

// vector3D linearAcc = {0, 0, 0};
// vector3D gravity = {0.0, 0.0, 9.8};
// eulerAngles angles = {0, 0, 0};

static float degrees_to_radians(float degrees) // 将角度转换为弧度
{
    return degrees * TEMP_PI / 180.0;
}

// 计算旋转矩阵
static void calculate_rotation_matrix(eulerAngles angles, float rotationMatrix[3][3])
{
    float cosPitch = cos(degrees_to_radians(angles.pitch));
    float sinPitch = sin(degrees_to_radians(angles.pitch));
    float cosRoll = cos(degrees_to_radians(angles.roll));
    float sinRoll = sin(degrees_to_radians(angles.roll));
    float cosYaw = cos(degrees_to_radians(angles.yaw));
    float sinYaw = sin(degrees_to_radians(angles.yaw));

    // rotationMatrix[0][0] = cosPitch * cosYaw;
    // rotationMatrix[0][1] = cosPitch * sinYaw;
    // rotationMatrix[0][2] = -sinPitch;

    // rotationMatrix[1][0] = sinRoll * sinPitch * cosYaw - cosRoll * sinYaw;
    // rotationMatrix[1][1] = sinRoll * sinPitch * sinYaw + cosRoll * cosYaw;
    // rotationMatrix[1][2] = sinRoll * cosPitch;

    // rotationMatrix[2][0] = cosRoll * sinPitch * cosYaw + sinRoll * sinYaw;
    // rotationMatrix[2][1] = cosRoll * sinPitch * sinYaw - sinRoll * cosYaw;
    // rotationMatrix[2][2] = cosRoll * cosPitch;

    rotationMatrix[0][0] = cosYaw * cosPitch + sinYaw * sinRoll * sinPitch;
    rotationMatrix[0][1] = -cosYaw * sinPitch + sinYaw * sinRoll * sinPitch;
    rotationMatrix[0][2] = -sinYaw * cosRoll;

    rotationMatrix[1][0] = sinPitch * cosRoll;
    rotationMatrix[1][1] = cosPitch * cosRoll;
    rotationMatrix[1][2] = sinRoll;

    rotationMatrix[2][0] = sinYaw * sinPitch - cosYaw * sinRoll * sinPitch;
    rotationMatrix[2][1] = -sinYaw * sinPitch - cosYaw * sinRoll * cosPitch;
    rotationMatrix[2][2] = cosYaw * cosRoll;
}

// 剔除重力加速度
vector3D_G get_acc_without_g(vector3D linearAcc, vector3D gravity, eulerAngles angles)
{
    float rotationMatrix[3][3];
    calculate_rotation_matrix(angles, rotationMatrix);

    // 将重力加速度旋转到局部坐标系
    float localGravity[3];
    localGravity[0] = rotationMatrix[0][0] * gravity.x + rotationMatrix[0][1] * gravity.y + rotationMatrix[0][2] * gravity.z;
    localGravity[1] = rotationMatrix[1][0] * gravity.x + rotationMatrix[1][1] * gravity.y + rotationMatrix[1][2] * gravity.z;
    localGravity[2] = rotationMatrix[2][0] * gravity.x + rotationMatrix[2][1] * gravity.y + rotationMatrix[2][2] * gravity.z;

    // 剔除重力加速度
    vector3D_G linearAccWithoutGravity;
    linearAccWithoutGravity.Gy = localGravity[0];
    linearAccWithoutGravity.Gx = localGravity[1];
    linearAccWithoutGravity.Gz = localGravity[2];
    linearAccWithoutGravity.x = linearAcc.x - localGravity[0];
    linearAccWithoutGravity.y = linearAcc.y - localGravity[1];
    linearAccWithoutGravity.z = linearAcc.z + localGravity[2];

    return linearAccWithoutGravity;
}