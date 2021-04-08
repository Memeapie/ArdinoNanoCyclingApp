#include <Arduino_HTS221.h>
#include <ArduinoBLE.h>
#include <PDM.h>
#include <Arduino_LSM9DS1.h>
#include <Arduino_APDS9960.h>

// Global Variables
// Debug Mode
bool testing = true;

// Error Constants
const int ERRORCLEAR = 0; // No fault.
const int ERRORTIMING = 1; // The task failed due to a timing error.
const int ERRORSENSOR = 2; // A task sensor has failed.

// BLE Service
BLEService bluetoothCycleService("180C");

// BLE Charactertistics
BLEFloatCharacteristic bluetoothTempLevel("2101", BLERead | BLENotify);
BLEFloatCharacteristic bluetoothHumidityLevel("2102", BLERead | BLENotify);
BLEFloatCharacteristic bluetoothLightLevelIntensity("2102", BLERead | BLENotify);
BLEFloatCharacteristic bluetoothLightLevelRed("2102", BLERead | BLENotify);
BLEFloatCharacteristic bluetoothLightLevelGreen("2102", BLERead | BLENotify);
BLEFloatCharacteristic bluetoothLightLevelBlue("2102", BLERead | BLENotify);

// BLE Descriptors
BLEDescriptor bluetoothTempLevelDescriptor("2901", "Temperature");
BLEDescriptor bluetoothHumidityLevelDescriptor("2901", "Humidity");
BLEDescriptor bluetoothLightLevelIntensityDescriptor("2901", "Light Level (Intensity)");
BLEDescriptor bluetoothLightLevelRedDescriptor("2901", "Light Level (Red)");
BLEDescriptor bluetoothLightLevelGreenDescriptor("2901", "Light Level (Green)");
BLEDescriptor bluetoothLightLevelBlueDescriptor("2901", "Light Level (Blue)");

// BLE Task Control
long bluetoothInterval = 200; // The task interval in ms.
long bluetoothStartTime; // The start time of the task in ms.
long bluetoothStartTimeH = 0; // The clock when the task starts history.

// Temperature & Humidity Values
float temp;
float tempH;
float humidity;
float humidityH;

// Temperature & Humidity Task Control
int tempHumidityInterval = 500; // The task interval in ms.
int tempHumidityIntervalH; // How long it took last time.
int tempHumidityLength; // Length of task this time.
int tempHumidityStartTime; // The start time of the task in ms.
int tempHumidityStartTimeH = millis(); // The clock when the task starts history.
int tempHumidityFailInterval = 1500; // The task fail interval in ms.
int tempHumidityError = ERRORCLEAR; // Error Code container.

// Light Level Values
int lightLevelRed, lightLevelBlue, lightLevelGreen;
int lightLevelRedH, lightLevelBlueH, lightLevelGreenH;
int lightLevelIntensity;
int lightLevelIntensityH;

// Light Level Task Control
int lightLevelInterval = 500; // The task interval in ms.
int lightLevelIntervalH; // How long it took last time.
int lightLevelLength; // Length of task this time.
int lightLevelStartTime; // The start time of the task in ms.
int lightLevelStartTimeH = millis(); // The clock when the task starts history.
int lightLevelFailInterval = 1500; // The task fail interval in ms.
int lightLevelError = ERRORCLEAR; // Error Code container.

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

    // Initialise APDS9960 Light Sensor
    if (!APDS.begin()) {
       Serial.println("Error initializing APDS9960 sensor!");
    }

    // Initialise Bluetooth Module
    if (!BLE.begin()){
        Serial.println("Starting BLE failed!");
        while (1);
    }

    // Initialise BLE Service
    BLE.setLocalName("CycleArduino");
    BLE.setAdvertisedService(bluetoothCycleService);

    // BLE Characteristic Descriptors
    bluetoothTempLevel.addDescriptor(bluetoothTempLevelDescriptor);
    bluetoothHumidityLevel.addDescriptor(bluetoothHumidityLevelDescriptor);
    bluetoothLightLevelIntensity.addDescriptor(bluetoothLightLevelIntensityDescriptor);
    bluetoothLightLevelRed.addDescriptor(bluetoothLightLevelRedDescriptor);
    bluetoothLightLevelGreen.addDescriptor(bluetoothLightLevelGreenDescriptor);
    bluetoothLightLevelBlue.addDescriptor(bluetoothLightLevelBlueDescriptor);

    // BLE Characteristics
    bluetoothCycleService.addCharacteristic(bluetoothTempLevel);
    bluetoothCycleService.addCharacteristic(bluetoothHumidityLevel);
    bluetoothCycleService.addCharacteristic(bluetoothLightLevelIntensity);
    bluetoothCycleService.addCharacteristic(bluetoothLightLevelRed);
    bluetoothCycleService.addCharacteristic(bluetoothLightLevelGreen);
    bluetoothCycleService.addCharacteristic(bluetoothLightLevelBlue);

    // Advertise BLE Service
    BLE.addService(bluetoothCycleService);
    BLE.advertise();

    if (testing) {
        Serial.print("Peripheral device MAC: ");
        Serial.println(BLE.address());
        Serial.println("Waiting for connections...");
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
                Serial.println("Temperature & Humidity failed to read in specified time.");
                delay(300);
            }

            // Log Error Code
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

// Method to Update Temperature and Humidity
void updateLightLevel(){

    // Reading Temperature & Humidity
    lightLevelStartTime = millis();

    // Decide if the interval for Updating Temperature & Humidity has passed.
    if(lightLevelStartTime > (lightLevelStartTimeH+lightLevelInterval)){

        // Check that the we have not failed to read the sensors in the expected period.
        if(lightLevelStartTime > (lightLevelStartTimeH+lightLevelFailInterval)){
            if(testing){
                Serial.println("Light Level failed to read in specified time.");
                delay(300);
            }

            // Log Error Code
            lightLevelError = ERRORTIMING;
            lightLevelStartTimeH = lightLevelStartTime;

        } else {

            // Check there is new Light Data Available
            if (APDS.colorAvailable()) {

                // Update the Light Information History
                lightLevelRedH = lightLevelRed;
                lightLevelBlueH = lightLevelBlue;
                lightLevelGreenH = lightLevelGreen;
                lightLevelIntensityH = lightLevelIntensity;

                // Update with new Light Data
                APDS.readColor(lightLevelRed, lightLevelBlue, lightLevelGreen);
                lightLevelIntensity = lightLevelRed + lightLevelBlue + lightLevelGreen;

                // Print Values if in Debug Mode
                if(testing){
                    Serial.print("Light Level Red: ");
                    Serial.println(lightLevelRed);
                    Serial.print("Light Level Green: ");
                    Serial.println(lightLevelBlue);
                    Serial.print("Light Level Blue: ");
                    Serial.println(lightLevelGreen);
                    Serial.print("Light Level Intensity: ");
                    Serial.println(lightLevelIntensity);                    
                }
            } else {
            
                // Log Error Code
                if(lightLevelError == ERRORCLEAR){
                    if(testing){
                        Serial.println("APDS9960 - Color Sensor Failed to Start.");
                    }
                    lightLevelError = ERRORSENSOR;
                }
            }

            // Copy history so we can use it to trigger the next shot
            lightLevelLength = millis() - lightLevelStartTime;
            lightLevelStartTimeH = lightLevelStartTime;

            // Print Values if in Debug Mode
            if(testing){
                Serial.print("Light Level Task Time Interval: ");
                Serial.print(lightLevelLength);
                Serial.println(" Milliseconds");
            }
        }
    }
}

void bluetoothServicing(){

    // Designate Central Task
    BLEDevice central = BLE.central();

    // If the BLE Service is defined, send some data
    if (central) {
        if (testing) {
            Serial.print("Connected to central: ");
            Serial.println(central.address());
        }

        // Make the LED Turn on to signify data transmission
        digitalWrite(LED_BUILTIN, HIGH);

        // If there is a device connected, begin sending data
        if (central.connected()) {

            // Write to the Characteristics of BLE (i.e characteristic.writeValue(value))
            bluetoothTempLevel.writeValue(temp);
            bluetoothHumidityLevel.writeValue(humidity);

        }
    } else {

        if (testing) {
            Serial.println("Central Disconnected");
        }

        // Make the LED Turn off to signify data transmission has finished
        digitalWrite(LED_BUILTIN, LOW);
    }

    //Now lets get the history set up.
    bluetoothStartTimeH = bluetoothStartTime;

}

void loop() {

    updateTempHumidity();

    updateLightLevel();

    bluetoothStartTime = millis();
    if(bluetoothStartTime > (bluetoothStartTimeH+bluetoothInterval)) {
         bluetoothServicing();
    }
}
