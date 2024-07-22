#ifndef BLE_H_
#define BLE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/* Attributes State Machine */
enum {
    IDX_SVC,
    IDX_CHAR_A,
    IDX_CHAR_VAL_A,
    IDX_CHAR_CFG_A,

    IDX_CHAR_B,
    IDX_CHAR_VAL_B,

    IDX_CHAR_C,
    IDX_CHAR_VAL_C,

    HRS_IDX_NB,
};

#define TAG "BLE_BROADCAST"

// SemaphoreHandle_t xMutex1;

#define ADV_INTERVAL_MS 2000 // 广播间隔时间，单位：毫秒
#define SCAN_INTERVAL_MS 225
#define DATA_INTERVAL_MS 600

extern uint32_t duration;

const char remote_device_name[] = "OLTHR";

extern esp_ble_adv_params_t adv_params;

extern esp_ble_scan_params_t ble_scan_params;

void all_queue_init(void);
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void ble_rec_data_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif