#include <Arduino_HTS221.h>
#include <ArduinoBLE.h>
#include <PDM.h>
#include <Arduino_LSM9DS1.h>

// Global Variables
// Debug Mode
bool testing = true;

// Error Constants
const int ERRORCLEAR = 0; // No fault.
const int ERRORTIMING = 1; // The task failed due to a timing error.
const int ERRORSENSOR = 2; // A task sensor has failed.

// Temperature & Humidity Values
float temp;
float tempH;
float humidity;
float humidityH;

// Temperature & Humidity Task Control
int tempHumidityInterval = 500; // The task interval in ms
int tempHumidityIntervalH; // How long it took last time.
int tempHumidityLength; // Length of task this time.
int tempHumidityStartTime; // The start time of the task in ms.
int tempHumidityStartTimeH = millis(); // The clock when the task starts history.
int tempHumidityFailInterval = 1500; // The task fail interval in ms.
int tempHumidityError = ERRORCLEAR; // Error Code container.

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
        Serial.println("Starting BLE failed!");
        while (1);
    }

}

// Method to Update Temperature and Humidity
void updateTempHumidity(){

    // Reading Temperature & Humidity
    tempHumidityStartTime = millis();

    // Decide if the interval for Updating Temperature & Humidity has passed.
    if(tempHumidityStartTime > (tempHumidityStartTimeH+tempHumidityInterval)){

        // Check that the we have not failed to read the sensors in the expected period.
        if(tempHumidityStartTime > (tempHumidityStartTimeH+tempHumidityFailInterval)){
            if(testing){
                Serial.println("Temperature & Humidity failed to read specified time.");
                delay(300);
            }

            tempHumidityError = ERRORTIMING;
            tempHumidityStartTimeH = tempHumidityStartTime;
        } else {
            // Update History
            humidityH = humidity;
            tempH = temp;

            // Read New Values
            humidity = HTS.readHumidity();
            temp = HTS.readTemperature();

            // Print Values if in Debug Mode
            if (testing) {
                Serial.print("Temperature: ");
                Serial.println(temp);
                Serial.print("Humidity: ");
                Serial.println(humidity);
            }

            tempHumidityLength = millis() - tempHumidityStartTime;
            tempHumidityStartTimeH = tempHumidityStartTime; //Copy history so we can use it to trigger the next shot

            if(testing){
                Serial.print("Temperature & Humidity Task Time Interval: ");
                Serial.print(tempHumidityLength);
                Serial.println(" Milliseconds");
            }
        }
    }
}

void loop() {

    updateTempHumidity();

}
