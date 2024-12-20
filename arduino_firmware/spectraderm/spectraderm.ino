#include "Wire.h"
#include "ArduinoBLE.h"
#include <Adafruit_AS7341.h>

#include "esp32-hal.h"

#define CORE_DEBUG_LEVEL    3
#define ESP_TAG             "SD"

#define LED_CTRL            0
#define LED_PWM             1

#define SCL                 18
#define SDA                 19

#define ALARM_MS            (uint64_t) 5
#define BUFFER_SIZE         1
#define CHANNEL_COUNT       10
#define CHAR_SIZE           (CHANNEL_COUNT * BUFFER_SIZE * 6 + 1)

hw_timer_t                  *timer = NULL;
SemaphoreHandle_t           sensorSemaphore;
SemaphoreHandle_t           bleSemaphore;

int channel_buffers[CHANNEL_COUNT][BUFFER_SIZE];
int buffer_index = 0;

#define SERVICE_UUID        "9f39d509-9ac9-49bd-9e73-ec8e02057535"
#define CHAR_UUID           "e8917cde-8d60-413d-b8ec-bd00fb949290"

BLEService sdService(SERVICE_UUID);
BLECharacteristic sdCharacteristic(CHAR_UUID, BLERead | BLENotify, CHAR_SIZE);

bool isConnected = false;
Adafruit_AS7341 as7341;

void IRAM_ATTR onTimer()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(sensorSemaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

void sdSensorTask(void *pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(sensorSemaphore, portMAX_DELAY) == pdTRUE)
        {
            digitalWrite(LED_CTRL, HIGH);

            if (as7341.readAllChannels())
            {
                // Store sensor data in the buffer
                channel_buffers[0][buffer_index] = as7341.getChannel(AS7341_CHANNEL_415nm_F1);
                channel_buffers[1][buffer_index] = as7341.getChannel(AS7341_CHANNEL_445nm_F2);
                channel_buffers[2][buffer_index] = as7341.getChannel(AS7341_CHANNEL_480nm_F3);
                channel_buffers[3][buffer_index] = as7341.getChannel(AS7341_CHANNEL_515nm_F4);
                channel_buffers[4][buffer_index] = as7341.getChannel(AS7341_CHANNEL_555nm_F5);
                channel_buffers[5][buffer_index] = as7341.getChannel(AS7341_CHANNEL_590nm_F6);
                channel_buffers[6][buffer_index] = as7341.getChannel(AS7341_CHANNEL_630nm_F7);
                channel_buffers[7][buffer_index] = as7341.getChannel(AS7341_CHANNEL_680nm_F8);
                channel_buffers[8][buffer_index] = as7341.getChannel(AS7341_CHANNEL_CLEAR);
                channel_buffers[9][buffer_index] = as7341.getChannel(AS7341_CHANNEL_NIR);
            }
            buffer_index++;
            digitalWrite(LED_CTRL, LOW);

            if (buffer_index >= BUFFER_SIZE)
            {
                xSemaphoreGive(bleSemaphore);
                buffer_index = 0;
            }
        }
    }
}

void sdBleTask(void *pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(bleSemaphore, portMAX_DELAY) == pdTRUE)
        {
            for (int i = 0; i < BUFFER_SIZE; i++)
            {
                char data[CHAR_SIZE];
                char *pData = data;
                for (int j = 0; j < CHANNEL_COUNT; j++)
                {
                    pData += sprintf(pData, "%d,", channel_buffers[j][i]);
                }
                *pData = '\0'; // Remove the last comma and add a null terminator

                sdCharacteristic.writeValue(data);  // Send each line individually via BLE
            }
        }
    }
}

void setup()
{
    pinMode(LED_CTRL, OUTPUT);
    digitalWrite(LED_CTRL, HIGH);

    ledcAttach(LED_PWM, 20000, 10);
    ledcWrite(LED_PWM, 1);

    Serial.begin(115200);
    Wire.setPins(SDA, SCL);
    Wire.begin();

    ESP_LOGI(ESP_TAG, "Info: Started.");

    // Setup timer
    timer = timerBegin(1000000);
    timerAttachInterrupt(timer, &onTimer);
    timerAlarm(timer, ALARM_MS * 1000, true, 0);

    // Setup AS7341
    if (!as7341.begin())
    {
        ESP_LOGE(ESP_TAG, "AS7341 sensor not found!");
        while (1);
    }
    ESP_LOGI(ESP_TAG, "AS7341 sensor initialized.");

    as7341.setATIME(29);
    as7341.setASTEP(59);
    as7341.setGain(AS7341_GAIN_512X);

    // Setup sensor task
    sensorSemaphore = xSemaphoreCreateBinary();
    xTaskCreate(sdSensorTask, "Sensor task", 4096, NULL, 2, NULL);

    Adafruit_I2CDevice i2c_dev = Adafruit_I2CDevice(AS7341_I2CADDR_DEFAULT, &Wire);
    Adafruit_BusIO_Register enable_reg = Adafruit_BusIO_Register(&i2c_dev, (uint16_t) AS7341_ENABLE);
    uint8_t enable_reg_value;
    enable_reg.read(&enable_reg_value);
    ESP_LOGI(ESP_TAG, "AS7341_ENABLE register value: 0x%02X", enable_reg_value);

    // Initialize BLE
    if (!BLE.begin())
    {
        ESP_LOGE(ESP_TAG, "Error starting BLE!");
        while (1);
    }

    BLE.setLocalName("SpectraDerma");
    BLE.setDeviceName("SpectraDerma");
    BLE.setAdvertisedService(sdService);
    sdService.addCharacteristic(sdCharacteristic);
    BLE.addService(sdService);

    sdCharacteristic.writeValue("Waiting for data.");

    // Setup BLE task
    bleSemaphore = xSemaphoreCreateBinary();
    xTaskCreate(sdBleTask, "BLE Task", 8192, NULL, 1, NULL);

    BLE.advertise();
    ESP_LOGI(ESP_TAG, "BLE Initialized.");
}

void loop()
{
    BLEDevice central = BLE.central();
    if (central)
    {
        ESP_LOGI(ESP_TAG, "Connected to central: %s", central.address().c_str());
        while (central.connected())
        {
            vTaskDelay(pdMS_TO_TICKS(1000));  // Non-blocking delay
        }
        ESP_LOGI(ESP_TAG, "Disconnected from central: %s", central.address().c_str());
    }
}
