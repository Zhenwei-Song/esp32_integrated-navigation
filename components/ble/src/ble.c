#include "./../inc/ble.h"
#include "./../inc/ble_queue.h"
#include "./../inc/data_manage.h"

uint32_t duration = 0;

// 配置广播参数
esp_ble_adv_params_t adv_params = {
#if 1
    .adv_int_min = (uint16_t)(ADV_INTERVAL_MS / 0.625),        // 广播间隔时间，单位：0.625毫秒
    .adv_int_max = (uint16_t)((ADV_INTERVAL_MS + 10) / 0.625), // 广播间隔时间，单位：0.625毫秒
#else
    .adv_int_min = 0x20,
    .adv_int_max = 0x30,
#endif
    .adv_type = ADV_TYPE_NONCONN_IND, // 不可连接
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL, // 在所有三个信道上广播
    //.channel_map = ADV_CHNL_37,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_WLST_CON_WLST,
};

// 配置扫描参数
esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_PASSIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
#if 0
    .scan_interval = 0xff, // 50 * 0.625ms = 31.25ms
    .scan_window = 0xff,
#else
    .scan_interval = (uint16_t)(SCAN_INTERVAL_MS / 0.625),
    .scan_window = (uint16_t)(SCAN_INTERVAL_MS / 0.625),
#endif
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE};



void all_queue_init(void)
{
    queue_init(&rec_queue);
}

/**
 * @description: 处理接收数据
 * @param {void} *pvParameters
 * @return {*}
 */
void ble_rec_data_task(void *pvParameters)
{
    uint8_t *phello = NULL;
    uint8_t *rec_data = NULL;
    uint8_t phello_len = 0;
    while (1) {
        if (xSemaphoreTake(xCountingSemaphore_receive, portMAX_DELAY) == pdTRUE) // 得到了信号量
        {
            if (!queue_is_empty(&rec_queue)) {
                rec_data = queue_pop(&rec_queue);
                if (rec_data != NULL) {
                    phello = esp_ble_resolve_adv_data(rec_data, ESP_BLE_AD_TYPE_PHELLO, &phello_len);
                    // ESP_LOGI(TAG, "ADV_DATA:");
                    // esp_log_buffer_hex(TAG, rec_data, 31);
                    if (phello != NULL) {
                        resolve_phello(phello, &my_information, temp_rssi);
#ifdef PRINT_CONTROL_PACKAGES_RECEIVED
                        ESP_LOGI(TAG, "PHELLO_DATA:");
                        esp_log_buffer_hex(TAG, phello, phello_len);
#endif
                        // ESP_LOGE(TAG, "rssi:%d", temp_rssi);
                    }

                    free(rec_data);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(0));
        }
    }
}

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    bool is_filtered = false;
#ifdef THROUGHPUT
    uint32_t bit_rate = 0;
#endif // THROUGHPUT
#ifdef GPIO
    static bool flag = false;
#endif // GPIO
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT: /*!< When raw advertising data set complete, the event comes */
        // ESP_LOGW(TAG, "ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT");
        // esp_ble_gap_stop_scanning();
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        /* advertising start complete event to indicate advertising start successfully or failed */
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
#ifdef PRINT_GAP_EVENT_STATES
            ESP_LOGE(TAG, "Advertising start failed");
#endif
        }
        else {
            // esp_ble_gap_start_scanning(duration);
            // ESP_LOGI(TAG, "Advertising start successfully");
#ifdef GPIO
            gpio_set_level(GPIO_OUTPUT_IO_0, 0);
            esp_ble_gap_stop_advertising();
#endif
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
#ifdef PRINT_GAP_EVENT_STATES
            ESP_LOGE(TAG, "Advertising stop failed");
#endif
        }
        else {
#ifdef PRINT_GAP_EVENT_STATES
            ESP_LOGI(TAG, "Stop adv successfully");
#endif
#ifdef GPIO
            gpio_set_level(GPIO_OUTPUT_IO_0, 1);
            esp_ble_gap_start_advertising(&adv_params);
#endif
        }
        break;

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        // scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
#ifdef PRINT_GAP_EVENT_STATES
            ESP_LOGE(TAG, "Scan start failed");
#endif
        }
        else {
#ifdef PRINT_GAP_EVENT_STATES
            ESP_LOGI(TAG, "Scan start successfully");
#endif
        }
        break;

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
#ifdef PRINT_GAP_EVENT_STATES
            ESP_LOGE(TAG, "Scan stop failed");
#endif
        }
        else {
            // esp_ble_gap_start_advertising(&adv_params);
#ifdef PRINT_GAP_EVENT_STATES
            ESP_LOGI(TAG, "Stop scan successfully\n");
#endif
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: { // 表示已经扫描到BLE设备的广播数据
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) { // 用于检查扫描事件类型
        case ESP_GAP_SEARCH_INQ_RES_EVT:            // 表示扫描到一个设备的广播数据
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len); // 解析广播设备名
            if (adv_name != NULL) {
                if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) { // 检查扫描到的设备名称是否与指定的 remote_device_name 匹配
#ifdef THROUGHPUT
#if 0
                    current_time = esp_timer_get_time();
                    bit_rate = 31 * SECOND_TO_USECOND / (current_time - start_time);
                    ESP_LOGI(TAG, " Bit rate = %" PRIu32 " Byte/s, = %" PRIu32 " bit/s, time = %ds",
                             bit_rate, bit_rate << 3, (int)((current_time - start_time) / SECOND_TO_USECOND));

                    start_time = esp_timer_get_time();
#else
                    cal_len += 31;
                    if (start == false) {
                        start_time = esp_timer_get_time();
                        start = true;
                    }
#endif
#endif // THROUGHPUT

#ifndef THROUGHPUT
                    // ESP_LOGW(TAG, "%s is found", remote_device_name);
#ifdef FILTER
                    if (memcmp(my_information.my_id, id_b50a, 2) == 0) {
                        if (memcmp(id_eb36, scan_result->scan_rst.bda + 4, 2) == 0 ||
                            memcmp(id_774a, scan_result->scan_rst.bda + 4, 2) == 0) {
                            is_filtered = true;
                        }
                    }
                    else if (memcmp(my_information.my_id, id_cae6, 2) == 0) {
                        if (memcmp(id_774a, scan_result->scan_rst.bda + 4, 2) == 0 ||
                            memcmp(id_0936, scan_result->scan_rst.bda + 4, 2) == 0) {
                            is_filtered = true;
                        }
                    }
                    else if (memcmp(my_information.my_id, id_eb36, 2) == 0) {
                        if (memcmp(id_b50a, scan_result->scan_rst.bda + 4, 2) == 0) {
                            is_filtered = true;
                        }
                    }
                    else if (memcmp(my_information.my_id, id_0936, 2) == 0) {
                        if (memcmp(id_cae6, scan_result->scan_rst.bda + 4, 2) == 0) {
                            is_filtered = true;
                        }
                    }
                    else if (memcmp(my_information.my_id, id_774a, 2) == 0) {
                        if (memcmp(id_b50a, scan_result->scan_rst.bda + 4, 2) == 0 ||
                            memcmp(id_cae6, scan_result->scan_rst.bda + 4, 2) == 0) {
                            is_filtered = true;
                        }
                    }
                    if (is_filtered) {
                        is_filtered = false;
                    }
                    else {
                        queue_push_with_check(&rec_queue, scan_result->scan_rst.ble_adv, scan_result->scan_rst.rssi);
                        xSemaphoreGive(xCountingSemaphore_receive);
                    }
#else
                    queue_push_with_check(&rec_queue, scan_result->scan_rst.ble_adv, scan_result->scan_rst.rssi);
                    xSemaphoreGive(xCountingSemaphore_receive);
#endif // FILTER
       //  queue_print(&rec_queue);
#endif // ndef THROUGHPUT

#ifdef GPIO
                    gpio_set_level(GPIO_OUTPUT_IO_1, flag);
                    flag = !flag;
                    printf("flag is %d\n", flag);
#endif // GPIO
                }
            }
            break;
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
}
