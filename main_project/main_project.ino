#include <Arduino_HTS221.h>
#include <ArduinoBLE.h>
#include <PDM.h>
#include <Arduino_LSM9DS1.h>

// Global Variables
// Temperature & Humidity Values
float temp;
float tempH;
float humidity;
float humidityH

// Temperature & Humidity Task Control
int tempHumidityInterval = 500; // The task interval in ms
int tempHumidityIntervalH; // How long it took last time.
int tempHumidityLength; // Length of task this time.
int tempHumidityStartTime; // The start time of the task in ms.
int tempHumidityStartTimeH = millis(); // The clock when the task starts history.

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

    float humid = HTS.readHumidity();

    Serial.println(temp);
    Serial.println(humid);


}
