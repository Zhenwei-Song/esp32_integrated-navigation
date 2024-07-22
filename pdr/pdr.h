#ifndef PDR_H__
#define PDR_H__

#define NORMAL_MODE
// #define ABNORMAL_MODE
#define DATA_POINTS_NUM 100

#define G 9.794
#define FREQUENCY 100
#define OFFSET FREQUENCY / 100
#define MIN_ACCELERATION 0.2 * G; // 0.2g          行人加速度阈值
#define MAX_ACCELERATION 2 * G;   // 2g
#define SLIDE 40 * OFFSET;        // 滑动窗口

#define PEAK_INDEX_INIT -1
#define MAX_STEPS_NUM 100

typedef struct model {
    float linear[3];
    float gravity[3];
    float a_vertical;
    p_model next;
} model, *p_model;

typedef struct model_table {
    p_model head;
    p_model tail;
} model_table, *p_model_table;

// 定义步态模型结构体
typedef struct step {
    int index;
    float acceleration;
    p_step next;
} step, *p_step;

typedef struct step_table {
    p_step head;
    p_step tail;
} step_table, *p_step_table;

void model_table_init(p_model_table q);

void model_table_destory(p_model_table q);

int push_model(p_model_table q, float *liner, float *gravity);

int pop_model(p_model_table q);

int get_model_node_num(p_model_table q);

void step_table_init(p_step_table q);

void step_table_destory(p_step_table q);

int push_step(p_step_table q, int index, float acceleration);

int pop_step(p_step_table q);

int pop_step_from_index(p_step_table q, int index);

int get_step_node_num(p_step_table q);

#endif