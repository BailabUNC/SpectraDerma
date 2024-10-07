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
#include "esp_gatt_common_api.h"
#include "driver/i2c.h"
#include "driver/gptimer.h"
#include "sdkconfig.h"

#include "sdm_as7341.h"

#define LED_CTRL_GPIO                       0
#define LED_PWM_GPIO                        1
#define I2C_MASTER_SCL_IO                   18
#define I2C_MASTER_SDA_IO                   19
#define I2C_MASTER_FREQ_HZ                  1000000  // 1 MHz
#define I2C_MASTER_NUM                      I2C_NUM_0

#define TIMER_ALARM_US                      1000
#define BUFFER_SIZE                         1
#define CHANNEL_COUNT                       10
#define CHAR_SIZE                           (CHANNEL_COUNT * BUFFER_SIZE * 6 + 1)

// UUIDs
#define GATTS_SERVICE_UUID_TEST             0x00FF
#define GATTS_CHAR_UUID_TEST_A              0xFF01
#define GATTS_NUM_HANDLE_TEST               4

#define PROFILE_NUM                         1
#define PROFILE_APP_IDX                     0

#define CHAR_DECLARATION_SIZE               (sizeof(uint8_t))

// static uint8_t adv_config_done =            0;
uint16_t gatt_service_handle =              0;
esp_gatt_char_prop_t gatt_char_property =   0;
esp_attr_value_t gatt_char_value = {
    .attr_max_len = CHAR_SIZE,
    .attr_value = NULL,
};
static esp_gatt_srvc_id_t gatt_service_id;
static uint16_t connection_id =             0;
static esp_gatt_if_t gatt_if =              0;
static uint16_t gatt_char_handle =          0;
static bool is_connected =                  false;

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,                 // 7.5 ms
    .max_interval = 0x0010,                 // 20 ms
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static const char *TAG =                    "SDM";

sdm_as7341_t                                as7341_sensor;
uint16_t                                    notify_buffer[BUFFER_SIZE][CHANNEL_COUNT];
uint16_t                                    buffer_index = 0;

SemaphoreHandle_t                           sensor_sphr;
SemaphoreHandle_t                           ble_sphr;

static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "REGISTER_APP_EVT, status = %d, app_id = %d",
                     param->reg.status, param->reg.app_id);
            
            gatt_service_id.is_primary = true;
            gatt_service_id.id.inst_id = 0x00;
            gatt_service_id.id.uuid.len = ESP_UUID_LEN_16;
            gatt_service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST;

            esp_ble_gap_set_device_name("SpectraDerma");
            esp_ble_gap_config_adv_data(&adv_data);

            esp_ble_gatts_create_service(gatts_if, &gatt_service_id, GATTS_NUM_HANDLE_TEST);
            break;

        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(TAG, "CREATE_SERVICE_EVT, status = %d, service handle = %d",
                     param->create.status, param->create.service_handle);
            
            gatt_service_handle = param->create.service_handle;
            esp_ble_gatts_start_service(gatt_service_handle);

            esp_bt_uuid_t char_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid.uuid16 = GATTS_CHAR_UUID_TEST_A,
            };

            gatt_char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

            esp_err_t add_char_ret = esp_ble_gatts_add_char(gatt_service_handle,
                                                            &char_uuid,
                                                            ESP_GATT_PERM_READ,
                                                            gatt_char_property,
                                                            &gatt_char_value,
                                                            NULL);

            if (add_char_ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Add char failed, error code %x", add_char_ret);
            }
            break;
        
        case ESP_GATTS_ADD_CHAR_EVT:
            ESP_LOGI(TAG, "ADD_CHAR_EVT, status = %d, attr_handle = %d, service_handle = %d",
                     param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
            gatt_char_handle = param->add_char.attr_handle;
            break;
        
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            connection_id = param->connect.conn_id;
            gatt_if = gatts_if;
            is_connected = true;
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

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&adv_params);
            break;

        default:
            break;
    }
}

static bool IRAM_ATTR sdm_on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(sensor_sphr, &higher_priority_task_woken);

    // Return true if a higher-priority task was woken
    return higher_priority_task_woken == pdTRUE;
}

static void sdm_sensor_task(void *arg)
{
    uint16_t reading_buffer[12];

    while (1)
    {
        if (xSemaphoreTake(sensor_sphr, portMAX_DELAY) != pdTRUE) continue;
        
        gpio_set_level(LED_CTRL_GPIO, 1);

        esp_err_t ret = sdm_as7341_read_all_channels(&as7341_sensor, reading_buffer);
        if (ret == ESP_OK)
        {
            for (int i = 0; i < CHANNEL_COUNT; i++)
            {
                notify_buffer[buffer_index][i] = reading_buffer[i];  // fill the notify_buffer with new reading
            }
            buffer_index++;

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

        gpio_set_level(LED_CTRL_GPIO, 0);
    }
}

static void sdm_ble_task(void *arg)
{
    while (1)
    {
        if (xSemaphoreTake(ble_sphr, portMAX_DELAY) != pdTRUE) continue;
        if (!is_connected) continue;

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
        esp_err_t ret = esp_ble_gatts_send_indicate(gatt_if,
                                                    connection_id,
                                                    gatt_char_handle,
                                                    strlen(data_string),
                                                    (uint8_t *) data_string,
                                                    false);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to send notification, error code: %x", ret);
        }
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

    ret = esp_ble_gatts_app_register(PROFILE_APP_IDX);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_ble_gatt_set_local_mtu(517);
}

static void sdm_gpio_init()
{
    // Configure GPIO
    gpio_config_t io_config = {
        .pin_bit_mask = (1ULL << LED_CTRL_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_config);
    gpio_set_level(LED_CTRL_GPIO, 1);
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
    esp_err_t ret;

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
    ret = gptimer_new_timer(&timer_config, &gptimer);
    ESP_ERROR_CHECK(ret);

    // Set up alarm for the timer
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = TIMER_ALARM_US,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true,
        }
    };
    ret = gptimer_set_alarm_action(gptimer, &alarm_config);
    ESP_ERROR_CHECK(ret);

    // Register the timer callback
    gptimer_event_callbacks_t cbs = {
        .on_alarm = sdm_on_alarm,
    };
    ret = gptimer_register_event_callbacks(gptimer, &cbs, NULL);
    ESP_ERROR_CHECK(ret);

    ret = gptimer_enable(gptimer);
    ESP_ERROR_CHECK(ret);
    ret = gptimer_start(gptimer);
    ESP_ERROR_CHECK(ret);
}

void app_main(void)
{
    sdm_gpio_init();
    sdm_wire_init();
    sdm_gptimer_init();
    sdm_ble_init();

    // Main loop
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}