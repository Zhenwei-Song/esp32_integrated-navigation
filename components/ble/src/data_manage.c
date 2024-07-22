/*
 * @Author: Zhenwei-Song zhenwei.song@qq.com
 * @Date: 2023-11-13 16:00:10
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-07-22 15:18:44
 * @FilePath: \esp32_integrated navigation\components\ble\src\data_manage.c
 * @Description: 仅供学习交流使用
 * Copyright (c) 2023 by Zhenwei-Song, All Rights Reserved.
 */
#include "data_manage.h"
#include "ble_queue.h"
#include "esp_gap_ble_api.h"

SemaphoreHandle_t xCountingSemaphore_send;
SemaphoreHandle_t xCountingSemaphore_receive;

#ifdef PRINT_RSSI
bool print_rssi = false;
#endif

uint8_t id_774a[ID_LEN] = {119, 74};
uint8_t id_cae6[ID_LEN] = {202, 230};
uint8_t id_eb36[ID_LEN] = {235, 54};
uint8_t id_b50a[ID_LEN] = {181, 10};
uint8_t id_0936[ID_LEN] = {9, 54};

my_info my_information;

uint8_t threshold_high[QUALITY_LEN] = {THRESHOLD_HIGH_1, THRESHOLD_HIGH_2};

uint8_t threshold_low[QUALITY_LEN] = {THRESHOLD_LOW_1, THRESHOLD_LOW_2};

uint8_t phello_final[PHELLO_FINAL_DATA_LEN] = {PHELLO_FINAL_DATA_LEN - 1, ESP_BLE_AD_TYPE_PHELLO};

uint8_t message_final[MESSAGE_FINAL_DATA_LEN] = {MESSAGE_FINAL_DATA_LEN - 1, ESP_BLE_AD_TYPE_MESSAGE};

uint8_t temp_quality_of_mine[QUALITY_LEN] = {0};

uint8_t temp_data_31[FINAL_DATA_LEN] = {0};

uint8_t adv_data_final_for_hello[FINAL_DATA_LEN] = {0};
uint8_t adv_data_final_for_message[FINAL_DATA_LEN] = {0};

uint8_t adv_data_name_7[HEAD_DATA_LEN] = {
    HEAD_DATA_LEN - 1, ESP_BLE_AD_TYPE_NAME_CMPL, 'O', 'L', 'T', 'H', 'R'};

uint8_t adv_data_message_16[MESSAGE_DATA_LEN - 6] = {
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10};

uint8_t adv_data_response_16[MESSAGE_DATA_LEN - 6] = {
    0x10, 0x0f, 0x0e, 0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01};

uint8_t adv_data_31[31] = {
    /* device name */
    0x06, 0x09, 'O', 'L', 'T', 'H', 'R',
    /*自定义数据段2*/
    0x17,
    0xff, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16};

uint8_t adv_data_62[62] = {
    /* device name */
    0x06, 0x09, 'O', 'L', 'T', 'H', 'R',
    /*自定义数据段2*/
    0x36,
    0xff, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};

/**
 * @description:广播数据段拼接
 * @param {uint8_t} *data1  前段数据段
 * @param {uint8_t} *data2  后段数据段
 * @return {*}  返回拼接后的数据
 */
uint8_t *data_match(uint8_t *data1, uint8_t *data2, uint8_t data_1_len, uint8_t data_2_len)
{
    memset(temp_data_31, 0, sizeof(temp_data_31) / sizeof(temp_data_31[0]));
    if (data_1_len + data_2_len <= FINAL_DATA_LEN) {
        memcpy(temp_data_31, data1, data_1_len);
        memcpy(temp_data_31 + data_1_len, data2, data_2_len);
        return temp_data_31;
    }
    else {
        ESP_LOGE(DATA_TAG, "the length of data error");
        return NULL;
    }
}

/**
 * @description:my_infomation初始化
 * @param {p_my_info} my_information
 * @param {uint8_t} *my_mac
 * @return {*}
 */
void my_info_init(p_my_info my_information, uint8_t *my_mac)
{
    memcpy(my_information->my_id, my_mac + 4, ID_LEN);
    if (memcmp(my_information->my_id, id_b50a, 2) == 0) {
        my_information->x = X_B50A;
        my_information->y = Y_B50A;
    }
    else if (memcmp(my_information->my_id, id_cae6, 2) == 0) {
        my_information->x = X_CAE6;
        my_information->y = Y_CAE6;
    }
    else if (memcmp(my_information->my_id, id_eb36, 2) == 0) {
        my_information->x = X_EB36;
        my_information->y = Y_EB36;
    }
    else if (memcmp(my_information->my_id, id_774a, 2) == 0) {
        my_information->x = X_774A;
        my_information->y = Y_774A;
    }
#ifdef SELF_ROOT
    my_information->is_root = true;
    my_information->is_connected = true;
    my_information->distance = 0;
    // memset(my_information->quality, 0, QUALITY_LEN);
    my_information->quality_from_me[0] = 0xff;
    my_information->quality_from_me[1] = 0xff;
    my_information->quality_from_me_to_neighbor[0] = 0xff;
    my_information->quality_from_me_to_neighbor[1] = 0xff;
    my_information->update = 0;
    memcpy(my_information->root_id, my_mac + 4, ID_LEN);
    memcpy(my_information->next_id, my_mac + 4, ID_LEN);
#else
    my_information->is_root = false;
    my_information->is_connected = false;
    my_information->distance = 0;
    my_information->quality_from_me[0] = NOR_NODE_INIT_QUALITY;
    my_information->quality_from_me[1] = NOR_NODE_INIT_QUALITY;
    my_information->quality_from_me_to_neighbor[0] = NOR_NODE_INIT_QUALITY;
    my_information->quality_from_me_to_neighbor[1] = NOR_NODE_INIT_QUALITY;
    my_information->update = 0;
#endif
}

/**
 * @description: 生成phello包
 * @param {p_my_info} info
 * @return {*} 返回生成的phello包（带adtype）
 */
uint8_t *generate_phello(p_my_info info)
{
    uint8_t phello[PHELLO_DATA_LEN] = {0};
    uint8_t temp_my_id[ID_LEN];
    uint8_t temp_root_id[ID_LEN];
    uint8_t temp_next_id[ID_LEN];
    uint8_t temp_quality[QUALITY_LEN];
    memcpy(temp_my_id, info->my_id, ID_LEN);
    memcpy(temp_root_id, info->root_id, ID_LEN);
    memcpy(temp_next_id, info->next_id, ID_LEN);
    // hello

    memcpy(temp_quality, info->quality_from_me, QUALITY_LEN);

    if (info->is_root)     // 自己是根节点
        phello[1] |= 0x11; // 节点类型，入网标志
    else                   // 自己不是根节点
        phello[1] |= 0x00; // 节点类型为非root

    phello[1] |= info->is_connected; // 入网标志
    phello[0] |= info->moveable;     // 移动性
    phello[2] |= info->x;            // x坐标
    phello[3] |= info->y;            // y坐标
    phello[4] |= info->distance;     // 到root跳数
    phello[6] |= temp_quality[0];    // 链路质量
    phello[7] |= temp_quality[1];
    phello[8] |= temp_my_id[0]; // 节点ID
    phello[9] |= temp_my_id[1];
    phello[10] |= temp_root_id[0]; // root ID
    phello[11] |= temp_root_id[1];
    phello[12] |= temp_next_id[0]; // next ID
    phello[13] |= temp_next_id[1];
    phello[15] |= info->update; // 最新包号
    // printf("generating hello\n");
    memcpy(phello_final + 2, phello, PHELLO_DATA_LEN);
    return phello_final;
}

/**
 * @description: 解析phello包
 * @param {uint8_t} *phello_data
 * @param {p_my_info} info
 * @return {*}
 */
void resolve_phello(uint8_t *phello_data, p_my_info info, int rssi)
{
    p_phello_info temp_info = (p_phello_info)malloc(sizeof(phello_info));
    uint8_t temp[PHELLO_DATA_LEN];
    memcpy(temp, phello_data, PHELLO_DATA_LEN);
    temp_info->moveable = ((temp[0] & 0x01) == 0x01 ? true : false);
    temp_info->is_root = ((temp[1] & 0x10) == 0x10 ? true : false);
    temp_info->is_connected = ((temp[1] & 0x01) == 0x01 ? true : false);
    temp_info->x = temp[2];
    temp_info->y = temp[3];
    temp_info->distance = temp[4];
    // temp_info->quality_from_me = temp[7];4
    memcpy(temp_info->quality, temp + 6, QUALITY_LEN);
    memcpy(temp_info->node_id, temp + 8, ID_LEN);
    // memcpy(info->root_id, temp + 10, ID_LEN);
    memcpy(temp_info->next_id, temp + 12, ID_LEN);
    temp_info->update = temp[15];
    if (info->is_root == false && temp_info->is_connected == true) {
        memcpy(info->root_id, temp + 10, ID_LEN);
        if (memcmp(temp_info->next_id, info->my_id, ID_LEN) != 0) { // 邻居中不以我为父节点的节点均是我的多路径选择对象
                                                                    // insert_up_routing_node(&my_up_routing_table, info->root_id, temp_info->node_id, temp_info->distance);
        }
    }

#if 0
    if (temp_info->is_connected) {
        info->distance = temp_info->distance + 1;
        memcpy(temp_quality, quality_calculate(rssi, temp_info->quality_from_me, info), QUALITY_LEN);
    }
    else {
        memcpy(temp_quality, temp_info->quality_from_me, QUALITY_LEN);
    }
#endif
    /* -------------------------------------------------------------------------- */
    /*                                    更新邻居表                                   */
    /* -------------------------------------------------------------------------- */
    /*20240124 马兴旺 更改插入内容*/

    //   insert_neighbor_node(&my_neighbor_table, temp_info->node_id, temp_info->is_root,
    //                      temp_info->is_connected, temp_info->quality, temp_info->distance, rssi, temp_info->next_id);
                         //  update_quality_of_neighbor_table(&my_neighbor_table, &my_information);
#ifdef PRINT_HELLO_DETAIL
                         ESP_LOGI(DATA_TAG, "****************************Hello info:*****************************************");
                         ESP_LOGI(DATA_TAG, "the type of the packet:HELLO");
                         if (temp_info->moveable)
                             ESP_LOGI(DATA_TAG, "locomotivity:Yes");
                         else
                             ESP_LOGI(DATA_TAG, "locomotivity:No");
                         if (temp_info->is_root)
                             ESP_LOGI(DATA_TAG, "the identity of the node:Network Exchange Point");
                         else
                             ESP_LOGI(DATA_TAG, "the identity of the node:Normal Point");
                         if (temp_info->is_connected)
                             ESP_LOGI(DATA_TAG, "the network connection:Success");
                         else
                             ESP_LOGI(DATA_TAG, "the network connection:Fail");
                         ESP_LOGI(DATA_TAG, "the x-coordinate of the location:%d", temp_info->x);
                         ESP_LOGI(DATA_TAG, "the y-coordinate of the location:%d", temp_info->y);
                         ESP_LOGI(DATA_TAG, "the number of hops to the Network Exchange Point:%d", temp_info->distance);
                         ESP_LOGI(DATA_TAG, "link quality:");
                         esp_log_buffer_hex(DATA_TAG, temp_info->quality, QUALITY_LEN);
                         ESP_LOGI(DATA_TAG, "Node ID:");
                         esp_log_buffer_hex(DATA_TAG, temp_info->node_id, ID_LEN);
                         ESP_LOGI(DATA_TAG, "ID of the Network Exchange Point:");
                         esp_log_buffer_hex(DATA_TAG, info->root_id, ID_LEN);
                         ESP_LOGI(DATA_TAG, "ID of the next hop node:");
                         esp_log_buffer_hex(DATA_TAG, temp_info->next_id, ID_LEN);
                         ESP_LOGI(DATA_TAG, "*********************************************************************************");
#endif
                         // printf("resolve hello\n");
                         // uint8_t temp1[10];
                         // memcpy(temp1, temp_info->node_id, ID_LEN);
                         // temp1[2] = ';';
                         // //temp1[3] = rssi;
                         //  memcpy(temp1 + 3, rssi, sizeof(int));
                         // sendData_tx(TX_TAG, (const char *)temp1);
                         printf("%x%x : %d\n", temp_info->node_id[0], temp_info->node_id[1], rssi);

#if 0
    if (info->is_root == true) {                // 若root dead后重回网络
        if (temp_info->update > info->update) { // 自己重新上电
            if (temp_info->update == 255)       // 启动循环
                info->update = 2;
            else
                info->update = temp_info->update + 1;
            ESP_LOGE(DOWN_ROUTING_TAG, "!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        }
    }
    else {                                                      // 自己非root
        if (temp_info->distance == 100) {                       // 接收到root dead信息
            if (info->distance == 1 || info->distance == 100) { // 自己是根节点的邻居节点，或者已经更新过自己的info
            }
            else {
                info->is_connected |= 0;
                info->distance = 100;
                info->update = temp_info->update;
                memset(info->root_id, 0, ID_LEN);
                memset(info->next_id, 0, ID_LEN);
            }
        }
        else {                                                                 // 不是root dead信息
            if (info->distance > temp_info->distance || info->distance == 0) { // 仅从父节点更新自己状态
                // 自己不是root
                if (temp_info->update > info->update || (info->update - temp_info->update == 253 && temp_info->update == 2)) { // root is back，更新当前信息
                    info->is_connected |= 1;                                                                                   // 自己也已入网
                    info->distance = temp_info->distance + 1;                                                                  // 距离加1
                    info->update = temp_info->update;
                    memcpy(info->root_id, temp_info->root_id, ID_LEN); // 存储root id

                    // memcpy(info->next_id, temp_info->next_id, ID_LEN); // 把发送该hello包的节点作为自己的到root下一跳id
                }
                else if (temp_info->update == info->update) { // root is alive 正常接收
                    if (temp_info->is_connected == 1) {
                        if (info->distance == 0 || info->distance - temp_info->distance > 1)
                            info->distance = temp_info->distance + 1; // 距离加1
                        info->is_connected |= 1;                      // 自己也已入网
                    }
                    else {
                        info->is_connected |= 0;
                    }
                    memcpy(info->root_id, temp_info->root_id, ID_LEN); // 存储root id
                }
                else { // 旧update的包，丢弃
                }
            }
        }
    }
#endif
}
