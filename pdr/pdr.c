#include <math.h>
#include <stdio.h>

#include "pdr.h"

static int data_points_num = 0;

void model_table_init(p_model_table q)
{
    q->head = NULL;
    q->tail = NULL;
}

int push_model(p_model_table q, float *liner, float *gravity)
{
    float g_y;
    float g_z;
    float theta;
    float linear_y;
    float linear_z;
    float a_vertical;
    p_model new_node = (p_model)malloc(sizeof(model));
    if (new_node == NULL)
        return -1;
    memcpy(new_node->linear, liner, 3);
    memcpy(new_node->gravity, gravity, 3);
    g_y = new_node->gravity[1];
    g_z = new_node->gravity[2];
    theta = atan(fabs(g_z / g_y));
    linear_y = new_node->linear[1];
    linear_z = new_node->linear[2];
    a_vertical = linear_y * cos(theta) + linear_z * sin(theta); // 得到垂直方向加速度（除去g）
    new_node->a_vertical = a_vertical;
    if (q->head == NULL) {
        q->head = q->tail = new_node;
        q->tail = NULL;
    }
    else {
        q->tail->next = new_node;
        q->tail = new_node;
        q->tail->next = NULL;
    }
    return 0;
}

int pop_model(p_model_table q)
{
    if (q->head == NULL)
        return -1;
    p_model temp = q->head;
    if (q->head->next == NULL)
        q->head = q->tail = NULL;
    else
        q->head = q->head->next;
    free(temp);
    return 0;
}

int get_data_points_num(p_model_table q)
{
    int num = 0;
    p_model temp = q->head;
    while (temp != NULL) {
        temp = temp->next;
        num++;
        data_points_num++;
    }
    return num;
}

void model_table_destory(p_model_table q)
{
    while (q->head != NULL)
        pop_model(q);
}

void step_table_init(p_step_table q)
{
    q->head = NULL;
    q->tail = NULL;
}

int push_step(p_step_table q, int index, float acceleration)
{
    p_step new_node = (p_step)malloc(sizeof(step));
    if (new_node == NULL)
        return -1;
    new_node->index = index;
    new_node->acceleration = acceleration;
    if (q->head == NULL) {
        q->head = q->tail = new_node;
        q->tail = NULL;
    }
    else {
        q->tail->next = new_node;
        q->tail = new_node;
        q->tail->next = NULL;
    }
    return 0;
}

int pop_step(p_step_table q)
{
    if (q->head == NULL)
        return -1;
    p_step temp = q->head;
    if (q->head->next == NULL)
        q->head = q->tail = NULL;
    else
        q->head = q->head->next;
    free(temp);
    return 0;
}

int pop_step_from_index(p_step_table q, int index)
{
    if (q->head == NULL)
        return -1;
    p_step temp = q->head;
    if (temp->next == NULL) {
        if (temp->index == index) {
            pop_step(q);
            return 0;
        }
        else {
            return -1;
        }
    }
    else {
        while (temp != NULL) {
            if (temp->next->index == index)
                break;
            else
                temp = temp->next;
        }
        temp->next = temp->next->next;
        return 0;
    }
    return 0;
}

int get_step_node_num(p_step_table q)
{
    int num = 0;
    p_step temp = q->head;
    while (temp != NULL) {
        temp = temp->next;
        num++;
    }
    return num;
}

void step_table_destory(p_step_table q)
{
    while (q->head != NULL)
        pop_step(q);
}

// // 四元数转换为欧拉角
// void quaternion2euler(p_model model, double *pitch, double *roll, double *yaw)
// {
//     double x = model->rotation[0];
//     double y = model->rotation[1];
//     double z = model->rotation[2];
//     double w = model->rotation[3];
//     *pitch = asin(2 * (w * y - z * x));
//     *roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
//     *yaw = atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y));
// }

// 计算坐标系之间的角度
// float *coordinate_conversion(p_model_table table)
// {
//     p_model temp = table->head;
//     // float g_y[data_points_num];
//     // float g_z[data_points_num];
//     // float theta[data_points_num];
//     // float linear_y[data_points_num];
//     // float linear_z[data_points_num];
//     // float a_vertical[data_points_num];
//     float *g_y = (float *)malloc(sizeof(float) * data_points_num);
//     float *g_z = (float *)malloc(sizeof(float) * data_points_num);
//     float *theta = (float *)malloc(sizeof(float) * data_points_num);
//     float *linear_y = (float *)malloc(sizeof(float) * data_points_num);
//     float *linear_z = (float *)malloc(sizeof(float) * data_points_num);
//     static float *a_vertical = (float *)malloc(sizeof(float) * data_points_num);
//     for (int i = 0; i < data_points_num; i++) {
//         g_y[i] = temp->gravity[1];
//         g_z[i] = temp->gravity[2];
//         theta[i] = atan(fabs(g_z[i] / g_y[i]));
//         linear_y[i] = temp->linear[1];
//         linear_z[i] = temp->linear[2];
//         a_vertical[i] = linear_y[i] * cos(theta) + linear_z[i] * sin(theta); // 得到垂直方向加速度（除去g）
//         temp = temp->next;
//     }
//     return a_vertical[data_points_num];
// }

// 步数检测
// void step_counter(p_model_table model_table, p_step_table step_table)
// {
//     step last_step;
//     int counter = 0;
//     float offset = FREQUENCY / 100;
//     // float a_vertical[data_points_num];
//     // 行人加速度阈值
//     float min_acceleration = 0.2 * G; // 0.2g
//     float max_acceleration = 2 * G;   // 2g
//     float slide = 40 * offset;        // 滑动窗口
//     float frequency = offset * 100;
//     // 峰值间隔(s)
// #ifdef NORMAL_MODE
//     float min_interval = 0.4;
// #else
//     float min_interval = 3;
// #endif
//     // 检测峰值
//     // 以40 *offset为滑动窗检测峰值
//     // 条件1：峰值在0 .2g ~2g之间
//     int step_count = 0;
//     step peak;
//     peak.index = PEAK_INDEX_INIT;
//     peak.acceleration = 0;
//     peak.next = NULL;
//     p_model temp_model = model_table->head;
//     for (int i = 0; i < data_points_num; i++) { // 遍历所有数据，存储峰值
//         if ((temp_model->a_vertical >= peak.acceleration) && (temp_model->a_vertical >= min_acceleration) && (temp_model->a_vertical <= max_acceleration)) {
//             peak.index = i;
//             peak.acceleration = temp_model->a_vertical;
//         }
//         // 当滑动窗口达到设定大小且存在峰值时，将峰值信息添加到步数列表 steps 中，并重新初始化 peak。
//         if ((i % slide == 0) && (peak.index != PEAK_INDEX_INIT)) {
//             push_step(step_table, peak.index, peak.acceleration);
//             peak.acceleration = 0;
//             peak.index = PEAK_INDEX_INIT;
//             step_count++; // 即step_table node个数
//         }
//         temp = temp->next;
//     }
//     // 条件2：两个峰值之前间隔至少大于0 .4s * offset
//     // del使用的时候，一般采用先记录再删除的原则
//     //  检测步数间隔
//     if (step_count > 0) {
//         last_step.index = step_table->head->index;
//         last_step.acceleration = step_table->head->acceleration;
//         last_step.next = NULL;
//         p_step temp = step_table->head->next;
//         // int dirty_points[step_count];
//         int *dirty_points = (int *)malloc(sizeof(int) * step_count);
//         int dirty_count = 0;
//         for (int i = 1; i < step_count; i++) {
//             if ((temp->index - last_step.index) < (min_interval * frequency)) {
//                 if (temp->acceleration <= last_step.acceleration) {
//                     dirty_points[dirty_count] = i;
//                     dirty_count++;
//                 }
//                 else {
//                     last_step.index = temp->index;
//                     last_step.acceleration = temp->acceleration;
//                     dirty_points[dirty_count] = i - 1;
//                     dirty_count++;
//                 }
//             }
//             else {
//                 last_step.index = temp->index;
//                 last_step.acceleration = temp->acceleration;
//             }
//             temp = temp->next;
//         }

//         // 删除无效步数
//         // 删除步数列表中间隔时间过短的峰值，以排除由于噪声或其他干扰引起的假步数
//         for (int i = 0; i < dirty_count; i++) {
//             int key = dirty_points[i];
//             p_step current = step_table->head;
//             p_step prev = NULL;
//             int j = 0;
//             // 遍历步数表，找到要删除的节点
//             while (current != NULL && j < key - counter) {
//                 prev = current;
//                 current = current->next;
//                 j++;
//             }
//             // 如果找到了要删除的节点
//             if (current != NULL && j == key - counter) {
//                 if (prev == NULL) { // 如果要删除的是头节点
//                     step_table->head = current->next;
//                 }
//                 else {
//                     prev->next = current->next;
//                 }
//                 free(current); // 释放要删除的节点
//                 counter++;     // 增加删除数量
//             }
//         }
//     }
// }

void get_step(p_model_table model_table, p_step_table step_table)
{
    float frequency = OFFSET * 100;
// 峰值间隔(s)
#ifdef NORMAL_MODE
    float min_interval = 0.4;
#else
    float min_interval = 3;
#endif
    // 检测峰值
    // 以40 *offset为滑动窗检测峰值
    // 条件1：峰值在0 .2g ~2g之间
    int step_count = 0;
    step peak;
    peak.index = PEAK_INDEX_INIT;
    peak.acceleration = 0;
    peak.next = NULL;
    p_model temp_model = model_table->head;
        for (int i = 0; i < data_points_num; i++) { // 遍历所有数据，存储峰值
            if ((temp_model->a_vertical >= peak.acceleration) && (temp_model->a_vertical >= min_acceleration) && (temp_model->a_vertical <= max_acceleration)) {
                peak.index = i;
                peak.acceleration = temp_model->a_vertical;
            }
            // 当滑动窗口达到设定大小且存在峰值时，将峰值信息添加到步数列表 steps 中，并重新初始化 peak。
            if ((i % slide == 0) && (peak.index != PEAK_INDEX_INIT)) {
                push_step(step_table, peak.index, peak.acceleration);
                peak.acceleration = 0;
                peak.index = PEAK_INDEX_INIT;
                step_count++; // 即step_table node个数
            }
            temp = temp->next;
        }
}

// 步长推算
float step_stride(float max_acceleration)
{
    return pow(max_acceleration, 0.25) * 0.5;
}

// // 航向角
// void step_heading(p_model model, float *yaw, float raw_yam)
// {
//     float pitch, roll, y;
//     quaternion2euler(model, &pitch, &roll, &y);
//     for (int i = 0; i < FREQUENCY; i++) {
//         yaw[i] = -y;
//     }
// }

// 步行轨迹的每一个相对坐标位置
void pdr_position(p_model model, int frequency, char *walkType, double offset, double initPosition[], double position_x[], double position_y[], double strides[], double angle[])
{
    double yaw[FREQUENCY];
    step_heading(model, yaw);
    // 步数检测
    // 假设最多100步
    // 这里应该根据实际情况进行修改
    // 此处省略了对步数的检测和记录
    int step_count = 0;
    // 计算步长
    double length = step_stride(2 * 9.794); // 假设最大加速度为 2g
    double x = initPosition[0];
    double y = initPosition[1];
    position_x[0] = x;
    position_y[0] = y;
    strides[0] = 0.0;
    angle[0] = offset;
    for (int i = 0; i < FREQUENCY; i++) {
        // 更新位置
        x = x + length * sin(yaw[i] + offset);
        y = y + length * cos(yaw[i] + offset);
        position_x[i + 1] = x;
        position_y[i + 1] = y;
        // 记录步长和角度
        strides[i + 1] = length;
        angle[i + 1] = yaw[i] + offset;
    }
    // 最后一步步长为0
    strides[FREQUENCY - 1] = 0.0;
}

int main()
{
    // 初始化模型数据
    double linear[] = {0.0, 0.0, 0.0};
    double gravity[] = {0.0, 0.0, 9.794};
    double rotation[] = {0.0, 0.0, 0.0, 0.0};
    model model;
    init_model(&model, linear, gravity, rotation);

    // 计算步数
    step steps[INIT_STEPS];
    int step_count = 0;
    step_counter(&model, steps, step_count);

    // 输出步数及其峰值位置和合加速度值
    printf("step count: %d\n", step_count);
    for (int i = 0; i < step_count; i++) {
        printf("step %d: index = %d, acceleration = %f\n", i + 1, steps[i].index, steps[i].acceleration);
    }

    return 0;
}
