#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_defs.h"
#include "esp_gatt_common_api.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "sdkconfig.h"

#include "sdm_as7341.h"


static const char *TAG =                    "SDM";

#define GPIO_LED_CTRL                       GPIO_NUM_0
#define GPIO_NIR_CTRL                       GPIO_NUM_4

#define LEDC_MODE                           LEDC_LOW_SPEED_MODE
#define LEDC_TIMER                          LEDC_TIMER_0
#define LEDC_DUTY_RESOLUTION                LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY                      10000

// PWM for AS clock signal
#define LEDC_AS_OUTPUT_IO                   GPIO_NUM_10
#define LEDC_AS_CHANNEL                     LEDC_CHANNEL_0
#define LEDC_AS_DUTY                        127             // ((2 ** 8) - 1) * 50% = 127

// PWM for LED brightness control
#define LEDC_PWM_OUTPUT_IO                  GPIO_NUM_1
#define LEDC_PWM_CHANNEL                    LEDC_CHANNEL_1
#define LEDC_PWM_DUTY                       10               // ((2 ** 8) - 1) * 1% = 2

// I2C config
#define I2C_MASTER_SCL_IO                   GPIO_NUM_18
#define I2C_MASTER_SDA_IO                   GPIO_NUM_19
#define I2C_MASTER_FREQ_HZ                  1000000         // 1 MHz
#define I2C_MASTER_NUM                      I2C_NUM_0

// Timer for measurement and BLE
#define TIMER_ALARM_US                      10000
#define BUFFER_SIZE                         1
#define CHANNEL_COUNT                       12
#define CHAR_SIZE                           (CHANNEL_COUNT * BUFFER_SIZE * 6 + 1)

static bool use_f1f4_clear_nir_mode =       true;

// BLE
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define GATTS_SDM_SERVICE_UUID              0xACEF
#define GATTS_SDM_CHAR_UUID                 0xFF01
#define GATTS_DESCR_UUID                    0x3333
#define GATTS_NUM_HANDLE                    4

#define DEVICE_NAME                         "SpectraDerma"
#define TEST_MANUFACTURER_DATA_LEN          17

#define PREPARE_BUF_MAX_SIZE                1024

static uint8_t adv_config_done =            0;
#define ADV_CONFIG_FLAG                     (1 << 0)
#define SCAN_RSP_CONFIG_FLAG                (1 << 1)

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0x3f, 0xca, 0xb1, 0x31, 0x62, 0x0d, 0x48, 0x85, 0x81, 0xd7, 0x44, 0xac, 0xef, 0x35, 0x30, 0x96,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0x3f, 0xca, 0xb1, 0x31, 0x62, 0x0d, 0x48, 0x85, 0x81, 0xd7, 0x44, 0xad, 0xef, 0x35, 0x30, 0x96,
};

static bool is_ready_for_notif =            false;
static bool is_connected =                  false;

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    //.min_interval = 0x0006,
    //.max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

typedef struct {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t connection_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
} sdm_gatts_profile_t;

static sdm_gatts_profile_t gatts_profile = {
    .gatts_cb = gatts_event_handler,
    .gatts_if = ESP_GATT_IF_NONE,
};

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

void write_prepare_handler(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

sdm_as7341_t                                as7341_sensor;
uint16_t                                    notify_buffer[BUFFER_SIZE][CHANNEL_COUNT];
uint16_t                                    buffer_index = 0;

SemaphoreHandle_t                           sensor_sphr;
SemaphoreHandle_t                           ble_sphr;

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0)
                esp_ble_gap_start_advertising(&adv_params);
            break;

        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0)
                esp_ble_gap_start_advertising(&adv_params);
            break;

        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            // advertising start complete event to indicate advertising start successfully or failed
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
                ESP_LOGE(TAG, "advertising start failed");
            else
                ESP_LOGI(TAG, "advertising start success");
            break;

        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
                ESP_LOGE(TAG, "advertising stop failed");
            else
                ESP_LOGI(TAG, "advertising stop success");
            break;

        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "update connection params status = %d, min_int = %d, max_int = %d, conn_int = %d, latency = %d, timeout = %d",
                     param->update_conn_params.status,
                     param->update_conn_params.min_int,
                     param->update_conn_params.max_int,
                     param->update_conn_params.conn_int,
                     param->update_conn_params.latency,
                     param->update_conn_params.timeout);
            break;

        default:
            break;
    }
}

void write_prepare_handler(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp)
    {
        if (param->write.is_prep)
        {
            if (param->write.offset > PREPARE_BUF_MAX_SIZE)
            {
                status = ESP_GATT_INVALID_OFFSET;
            }
            else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE)
            {
                status = ESP_GATT_INVALID_ATTR_LEN;
            }

            if (status == ESP_GATT_OK && prepare_write_env->prepare_buf == NULL)
            {
                prepare_write_env->prepare_buf = (uint8_t *) malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL)
                {
                    ESP_LOGE(TAG, "Gatt_server prep no mem");
                    status = ESP_GATT_NO_RESOURCES;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *) malloc(sizeof(esp_gatt_rsp_t));
            if (gatt_rsp)
            {
                gatt_rsp->attr_value.len = param->write.len;
                gatt_rsp->attr_value.handle = param->write.handle;
                gatt_rsp->attr_value.offset = param->write.offset;
                gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
                memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
                esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
                if (response_err != ESP_OK)
                {
                    ESP_LOGE(TAG, "Send response error\n");
                }
                free(gatt_rsp);
            }
            else
            {
                ESP_LOGE(TAG, "malloc failed, no resource to send response error\n");
                status = ESP_GATT_NO_RESOURCES;
            }
            if (status != ESP_GATT_OK){
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;

        }
        else
        {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "REGISTER_APP_EVT, status = %d, app_id = %d",
                     param->reg.status, param->reg.app_id);

            gatts_profile.service_id.is_primary = true;
            gatts_profile.service_id.id.inst_id = 0x00;
            gatts_profile.service_id.id.uuid.len = ESP_UUID_LEN_16;
            gatts_profile.service_id.id.uuid.uuid.uuid16 = GATTS_SDM_SERVICE_UUID;

            esp_ble_gap_set_device_name(DEVICE_NAME);
            esp_ble_gap_config_adv_data(&adv_data);
            adv_config_done |= ADV_CONFIG_FLAG;
            esp_ble_gap_config_adv_data(&scan_rsp_data);
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;

            esp_ble_gatts_create_service(gatts_if, &gatts_profile.service_id, 4);
            break;

        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;

        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(TAG, "CREATE_SERVICE_EVT, status = %d, service handle = %d",
                     param->create.status, param->create.service_handle);
            
            gatts_profile.service_handle = param->create.service_handle;
            gatts_profile.char_uuid.len = ESP_UUID_LEN_16;
            gatts_profile.char_uuid.uuid.uuid16 = GATTS_SDM_CHAR_UUID;
            esp_ble_gatts_start_service(gatts_profile.service_handle);

            esp_attr_value_t gatt_char_value = {
                .attr_max_len = CHAR_SIZE,
                .attr_value = NULL,
            };

            esp_err_t add_char_ret = esp_ble_gatts_add_char(
                gatts_profile.service_handle,
                &gatts_profile.char_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                &gatt_char_value,
                NULL
            );

            if (add_char_ret != ESP_OK)
                ESP_LOGE(TAG, "Add char failed, error code %x", add_char_ret);
            break;

        case ESP_GATTS_READ_EVT:
            ESP_LOGI(TAG, "GATT_READ_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);

            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = 4;
            rsp.attr_value.value[0] = 0xda;
            rsp.attr_value.value[1] = 0xed;
            rsp.attr_value.value[2] = 0xbe;
            rsp.attr_value.value[3] = 0xef;
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                        ESP_GATT_OK, &rsp);
            break;

        case ESP_GATTS_ADD_CHAR_EVT:
        {
            ESP_LOGI(TAG, "ADD_CHAR_EVT, status = %d, attr_handle = %d, service_handle = %d",
                     param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
            gatts_profile.char_handle = param->add_char.attr_handle;
            gatts_profile.descr_uuid.len = ESP_UUID_LEN_16;
            gatts_profile.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
            esp_ble_gatts_add_char_descr(
                param->add_char.service_handle, &gatts_profile.descr_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                NULL, NULL);
            break;
        }
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            ESP_LOGI(TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d",
                     param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
            gatts_profile.descr_handle = param->add_char_descr.attr_handle;
            break;

        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d",
                     param->connect.conn_id);
            gatts_profile.connection_id = param->connect.conn_id;
            gatts_profile.gatts_if = gatts_if;
            is_connected = true;

            vTaskDelay(pdMS_TO_TICKS(100));
            is_ready_for_notif = true;
            break;

        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
            if (!param->write.is_prep)
            {
                ESP_LOGI(TAG, "GATT_WRITE_EVT, value len %d, value: ", param->write.len);
                esp_log_buffer_hex(TAG, param->write.value, param->write.len);
                uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];

                if (descr_value == 0x0001)
                {
                    ESP_LOGI(TAG, "Notify enabled.");
                    uint8_t notify_data[15];
                    for (int i = 0; i < sizeof(notify_data); ++i)
                    {
                        notify_data[i] = i%0xff;
                    }
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gatts_profile.char_handle,
                                                sizeof(notify_data), notify_data, false);
                }
                else if (descr_value == 0x0000)
                {
                    ESP_LOGI(TAG, "Notify disabled.");
                }
                else
                {
                    ESP_LOGE(TAG, "Unknown value written.");
                }
            }
            write_prepare_handler(gatts_if, &prepare_write_env, param);
            break;

        case ESP_GATTS_EXEC_WRITE_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            break;

        case ESP_GATTS_UNREG_EVT:
            break;

        case ESP_GATTS_ADD_INCL_SRVC_EVT:
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, reason = %d", param->disconnect.reason);
            is_connected = false;
            esp_ble_gap_start_advertising(&adv_params);
            break;

        default:
            break;
    }
}

static bool IRAM_ATTR sdm_intr_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(sensor_sphr, &higher_priority_task_woken);

    // Return true if a higher-priority task was woken
    return higher_priority_task_woken == pdTRUE;
}

static void sdm_sensor_task(void *arg)
{
    esp_err_t ret;
    uint16_t reading_buffer[6];

    while (1)
    {
        if (xSemaphoreTake(sensor_sphr, portMAX_DELAY) != pdTRUE) continue;
        
        gpio_set_level(GPIO_LED_CTRL, 1);
        gpio_set_level(GPIO_NIR_CTRL, 1);

        // Check the current mode and perform the corresponding measurement
        if (use_f1f4_clear_nir_mode)
            ret = sdm_as7341_start_measure(&as7341_sensor, AS7341_CH_F1F4_CLEAR_NIR);
        else
            ret = sdm_as7341_start_measure(&as7341_sensor, AS7341_CH_F5F8_CLEAR_NIR);

        if (ret == ESP_OK)
        {
            sdm_as7341_read_channel_data(&as7341_sensor, reading_buffer);

            if (use_f1f4_clear_nir_mode)
            {
                // Fill the first 6 channels when in F1F4 mode
                for (int i = 0; i < 6; i++)
                {
                    notify_buffer[buffer_index][i] = reading_buffer[i];
                }
            }
            else
            {
                // Fill the last 6 channels (7th location onward) when in F5F8 mode
                for (int i = 0; i < 6; i++)
                {
                    notify_buffer[buffer_index][i + 6] = reading_buffer[i];
                }

                // Only increase buffer_index after F5F8 data is filled
                buffer_index++;
            }

            // If buffer is full, signal BLE task
            if (buffer_index >= BUFFER_SIZE)
            {
                xSemaphoreGive(ble_sphr);  // Signal BLE task to send data
                buffer_index = 0;  // Reset buffer index
            }
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read sensor data");
        }

        use_f1f4_clear_nir_mode = !use_f1f4_clear_nir_mode;

        gpio_set_level(GPIO_LED_CTRL, 0);
        gpio_set_level(GPIO_NIR_CTRL, 0);
    }
}

static void sdm_ble_task(void *arg)
{
    while (1)
    {
        if (xSemaphoreTake(ble_sphr, portMAX_DELAY) != pdTRUE) continue;
        if (!is_connected || !is_ready_for_notif) continue;

        char data_string[CHAR_SIZE];
        int offset = 0;

        for (int i = 0; i < BUFFER_SIZE; i++)
        {
            for (int j = 0; j < CHANNEL_COUNT; j++)
            {
                offset += snprintf(&data_string[offset], CHAR_SIZE - offset, "%d,", notify_buffer[i][j]);
            }
        }

        if (offset > 0 && offset < CHAR_SIZE)
        {
            data_string[offset - 1] = '\0';  // Terminate the string properly
        }

        // Send notification
        esp_err_t ret = esp_ble_gatts_send_indicate(gatts_profile.gatts_if,
                                                    gatts_profile.connection_id,
                                                    gatts_profile.char_handle,
                                                    strlen(data_string),
                                                    (uint8_t *) data_string,
                                                    false);
        if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to send notification, error code: %x", ret);
    }
}

static void sdm_ble_init()
{
    esp_err_t ret;

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    esp_bt_controller_config_t bt_config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_config);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatt_set_local_mtu(ESP_GATT_MAX_MTU_SIZE - 1);  // 516 bytes
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "gatts mtu value set error, error code = %x", ret);
        return;
    }
}

static void sdm_gpio_init()
{
    // Configure GPIO
    gpio_config_t io_config = {
        .pin_bit_mask = (1ULL << GPIO_LED_CTRL | 1ULL << GPIO_NIR_CTRL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_config);
    gpio_set_level(GPIO_LED_CTRL, 1);
    gpio_set_level(GPIO_NIR_CTRL, 1);
}

static void sdm_ledc_init()
{
    // Prepare and set configuration of timer for LEDC
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RESOLUTION,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // Prepare and set configuration LEDC channel for AS7341 GPIO pin
    ledc_channel_config_t ledc_as_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_AS_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_AS_OUTPUT_IO,
        .duty = LEDC_AS_DUTY,
        .hpoint = 0,
    };
    ledc_channel_config(&ledc_as_channel);
    
    // Prepare and set configuration LEDC channel for LED PWM
    ledc_channel_config_t ledc_pwm_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_PWM_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_PWM_OUTPUT_IO,
        .duty = LEDC_PWM_DUTY,
        .hpoint = 0,
    };
    ledc_channel_config(&ledc_pwm_channel);
}

static void sdm_wire_init()
{
    // Initialize I2C
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ,
        },
    };
    i2c_param_config(I2C_MASTER_NUM, &i2c_config);
    i2c_driver_install(I2C_MASTER_NUM, i2c_config.mode, 0, 0, 0);

    esp_err_t ret = sdm_as7341_init(&as7341_sensor, I2C_MASTER_NUM, AS7341_I2CADDR_DEFAULT);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "AS7341 initialized.");
    }
    else
    {
        ESP_LOGE(TAG, "AS7341 failed to initialize.");
        while (1);
    }
}

static void sdm_gptimer_init()
{
    // Create semaphores
    sensor_sphr = xSemaphoreCreateBinary();
    ble_sphr = xSemaphoreCreateBinary();

    // Start tasks
    xTaskCreate(sdm_sensor_task, "sensor task", 4096, sensor_sphr, 5, NULL);
    xTaskCreate(sdm_ble_task, "ble task", 8192, ble_sphr, 4, NULL);

    // Create and the timer
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,  // 1 MHz
    };
    gptimer_new_timer(&timer_config, &gptimer);

    // Set up alarm for the timer
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = TIMER_ALARM_US,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true,
        }
    };
    gptimer_set_alarm_action(gptimer, &alarm_config);

    // Register the timer callback
    gptimer_event_callbacks_t cbs = {
        .on_alarm = sdm_intr_callback,
    };
    gptimer_register_event_callbacks(gptimer, &cbs, NULL);
    gptimer_enable(gptimer);
    gptimer_start(gptimer);
}

void app_main(void)
{
    sdm_gpio_init();
    sdm_ledc_init();
    sdm_wire_init();
    sdm_ble_init();
    sdm_gptimer_init();

    // Main loop
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}