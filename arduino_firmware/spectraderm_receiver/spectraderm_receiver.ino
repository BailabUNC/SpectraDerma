#include <ArduinoBLE.h>

#define SERVICE_UUID        "9f39d509-9ac9-49bd-9e73-ec8e02057535"
#define CHAR_UUID           "e8917cde-8d60-413d-b8ec-bd00fb949290"

BLEDevice spectraderm;
BLECharacteristic sdCharacteristic(CHAR_UUID, BLERead | BLENotify);

void setup()
{
    Serial.begin(115200);

    if (!BLE.begin())
    {
        Serial.println("Starting BLE failed!");
        while (1);
    }

    Serial.println("BLE Central - SpectraDerma Client");

    BLE.setLocalName("ESP32C3_Client");
    BLE.setAdvertisedServiceUuid(SERVICE_UUID);

    BLE.scanForUuid(SERVICE_UUID);

    Serial.println("Scanning for SpectraDerma...");

    while (true)
    {
        spectraderm = BLE.available();

        if (spectraderm)
        {
            Serial.println("SpectraDerma found!");
            if (spectraderm.hasService(SERVICE_UUID))
            {
                Serial.println("Found the service.");
                BLE.stopScan();
                break;
            }
        }
    }

    if (spectraderm.connect())
    {
        Serial.println("Connected to SpectraDerma");
    }
    else
    {
        Serial.println("Failed to connect to SpectraDerma");
        while (1);
    }

    // Discover the service and characteristic
    BLEService spectradermService = spectraderm.service(SERVICE_UUID);
    if (!spectradermService)
    {
        Serial.println("SpectraDerma service not found!");
        spectraderm.disconnect();
        while (1);
    }

    sdCharacteristic = spectradermService.characteristic(CHAR_UUID);
    if (!sdCharacteristic)
    {
        Serial.println("SpectraDerma characteristic not found!");
        spectraderm.disconnect();
        while (1);
    }

    if (sdCharacteristic.canNotify())
    {
        sdCharacteristic.subscribe();
        Serial.println("Subscribed to notifications.");
    }
    else
    {
        Serial.println("Cannot subscribe to notifications!");
        spectraderm.disconnect();
        while (1);
    }
}

void loop()
{
    // Wait for the device to send a notification
    BLE.poll();

    if (sdCharacteristic.valueUpdated())
    {
        const uint8_t* data = sdCharacteristic.value();
        int length = sdCharacteristic.valueLength();
        
        Serial.print("Received data: ");
        for (int i = 0; i < length; i++)
        {
            Serial.print((char)data[i]);
        }
        Serial.println();
    }
}
