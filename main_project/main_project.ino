#include <Arduino_HTS221.h>
#include <ArduinoBLE.h>
#include <PDM.h>
#include <Arduino_LSM9DS1.h>

void setup() {
    // Initialise Serial Output
    Serial.begin(9600);
    while (!Serial);

    // Initialise Temp/Humidity Sensor
    if (!HTS.begin()) {
        Serial.println("Failed to initialize humidity temperature sensor!");
        while (1);
    }

    // Initialise PDM
    if (!PDM.begin(1, 16000)) {
        Serial.println("Failed to start PDM!");
        while (1);
    }

    // Initialise IMU
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }

    // Initialise Bluetooth Module
    if (!BLE.begin()){
        Serial.println("starting BLE failed!");
        while (1);
    }

}

void loop() {



}
