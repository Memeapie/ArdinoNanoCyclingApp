#include <Arduino_HTS221.h>
#include <ArduinoBLE.h>
#include <PDM.h>
#include <Arduino_LSM9DS1.h>
#include <Arduino_APDS9960.h>
#include <Arduino_LPS22HB.h>
#include <math.h> //library for maths pow function

// Global Variables
// Debug Mode
bool testing = true;

// Error Constants
const int ERRORCLEAR = 0; // No fault.
const int ERRORTIMING = 1; // The task failed due to a timing error.
const int ERRORSENSOR = 2; // A task sensor has failed.

// BLE Service
BLEService bluetoothCycleService("180C");

// BLE Characteristics
BLEFloatCharacteristic bluetoothTempLevel("2101", BLERead | BLENotify);
BLEFloatCharacteristic bluetoothHumidityLevel("2102", BLERead | BLENotify);
BLEFloatCharacteristic bluetoothLightLevelIntensity("2103", BLERead | BLENotify);
BLEFloatCharacteristic bluetoothLightLevelRed("2104", BLERead | BLENotify);
BLEFloatCharacteristic bluetoothLightLevelGreen("2105", BLERead | BLENotify);
BLEFloatCharacteristic bluetoothLightLevelBlue("2106", BLERead | BLENotify);
BLEFloatCharacteristic bluetoothElevationGainLevel("2107", BLERead | BLENotify);
BLEFloatCharacteristic bluetoothMaxElevationLevel("2108", BLERead | BLENotify);
BLEFloatCharacteristic bluetoothMaximumVelocity("2109", BLERead | BLENotify);
BLEFloatCharacteristic bluetoothMaximumAcceleration("2110", BLERead | BLENotify);
BLEFloatCharacteristic bluetoothCurrentAcceleration("2111", BLERead | BLENotify);
BLEFloatCharacteristic bluetoothImpact("2112", BLERead | BLENotify);
BLEStringCharacteristic bluetoothWeather("2113", BLERead | BLENotify, 20);

// BLE Descriptors
BLEDescriptor bluetoothTempLevelDescriptor("2901", "Temperature");
BLEDescriptor bluetoothHumidityLevelDescriptor("2902", "Humidity");
BLEDescriptor bluetoothLightLevelIntensityDescriptor("2903", "Light Level (Intensity)");
BLEDescriptor bluetoothLightLevelRedDescriptor("2904", "Light Level (Red)");
BLEDescriptor bluetoothLightLevelGreenDescriptor("2905", "Light Level (Green)");
BLEDescriptor bluetoothLightLevelBlueDescriptor("2906", "Light Level (Blue)");
BLEDescriptor bluetoothMaximumVelocityDescriptor("2907", "Max Velocity (M/s)");
BLEDescriptor bluetoothMaximumAccelerationDescriptor("2908", "Max Acceleration (M/S^2)");
BLEDescriptor bluetoothElevationGainLevelDescriptor("2909", "Elevation Gain");
BLEDescriptor bluetoothMaxElevationLevelDescriptor("2910", "Max Elevation");
BLEDescriptor bluetoothMaxCurrentAccelerationDescriptor("2911", "Current Acceleration");
BLEDescriptor bluetoothImpactDescriptor("2912", "Impact Detected?");
BLEDescriptor bluetoothWeatherDescriptor("2913", "Weather");

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

// Weather Values
String weather;
String weatherH;

// Weather Task Control
int weatherInterval = 500; // The task interval in ms.
int weatherIntervalH; // How long it took last time.
int weatherLength; // Length of task this time.
int weatherStartTime; // The start time of the task in ms.
int weatherStartTimeH = millis(); // The clock when the task starts history.
int weatherFailInterval = 1500; // The task fail interval in ms.
int weatherError = ERRORCLEAR; // Error Code container.

// Pressure Sensor Values
int readAttempts = 1; //counter to read how many attempts has taken place, because when the sensor reads the pressure for the first time something weird happens
bool secondRead = false; //turns to true after first read, prevents first reading affecting whole loop
double pressureAtSeaLevel = 1023.0; //atmpshpehric pressure at sea level, need to get a value for kent coast// currently using manston reading at 1800 on 07/04
double firstHalfOfElevationCalculation = 0; // part one of elevation calculation, makes use of <math> to do an x to the power of calculation
double elevation = 0; // the elevation the cyclist is cycling at
double elevationH = 0; // the cyclist elevation last time
double elevationChange = 0; // elevation change between current read and previous read
double elevationGain = 0; // the rider's total elevationGain or loss throughout the ride
double pressureKPA = 0; //sensor reads pressure in kPa
double pressureHPA = 0; // to turn pressure into elevation we need hPa
double maxElevationPoint = 0; // the highest elevation the cyclist is at

// Pressure Task Control
int pressureInterval = 500; // The task interval in ms.
int pressureLength; // Length of task this time.
int pressureStartTime; // The start time of the task in ms.
int pressureStartTimeH = millis(); // The clock when the task starts history.
int pressureFailInterval = 1500; // The task fail interval in ms.
int pressureError = ERRORCLEAR; // Error Code container.

// Accelerometer Values
double AccelerometerGravityMultiplier = 9.80665; // Multiplier to convert acceleration reading from G's to M/S^2.
float x, y, z; // Accelerometer readings in G's.
float AccelerometeraccellX; // Acceleration along X axis in M/S^2.
static float AccelerometerVelocityX = 0; // Initial Velocity of X axis.
float AccelerometerVelocityChangeX; // Change in velocity as a measure of accleration times time.
float AccelerometerVelocityXH; // Previous recorded velocity of X axis.
float AccelerometerMaxVelocityX; // Highest Velocity Recorded.
float AccelerometerMaxAccellX; // Great acceleration value recorded in M/S^2.
int AccelerometerImpact = -3; // Impact threshold in G's.
bool AccelerometerImpactDetected = false; // Impact Indicator.

// Accelerometer Task Control
int AccelerometerTaskInterval = 50; // Task interval in ms.
int AccelerometerTaskLength; // Length of task this time.
int AccelerometerStartTime = 0; // The start time of the task in ms.
int AccelerometerStartTimeH = millis(); // Start time of the last task run in ms.
double AccelerometerTimeChange = 0; //Time between each sensor measurement in seconds.
int AccelerometerTaskFailInterval = 1000; // The task fail interval in ms.
int AccelerometerTaskError = ERRORCLEAR; // Error Code Container.

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

    // Initialize PressureSensor
    if (!BARO.begin()) {
        Serial.println("Failed to initialize pressure sensor!");
        while (1);
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
    bluetoothElevationGainLevel.addDescriptor(bluetoothElevationGainLevelDescriptor);
    bluetoothMaxElevationLevel.addDescriptor(bluetoothMaxElevationLevelDescriptor);
    bluetoothMaximumVelocity.addDescriptor(bluetoothMaximumVelocityDescriptor);
    bluetoothMaximumAcceleration.addDescriptor(bluetoothMaximumAccelerationDescriptor);
    bluetoothCurrentAcceleration.addDescriptor(bluetoothMaxCurrentAccelerationDescriptor);
    bluetoothImpact.addDescriptor(bluetoothImpactDescriptor);
    bluetoothWeather.addDescriptor(bluetoothWeatherDescriptor);

    // BLE Characteristics
    bluetoothCycleService.addCharacteristic(bluetoothTempLevel);
    bluetoothCycleService.addCharacteristic(bluetoothHumidityLevel);
    bluetoothCycleService.addCharacteristic(bluetoothLightLevelIntensity);
    bluetoothCycleService.addCharacteristic(bluetoothLightLevelRed);
    bluetoothCycleService.addCharacteristic(bluetoothLightLevelGreen);
    bluetoothCycleService.addCharacteristic(bluetoothLightLevelBlue);
    bluetoothCycleService.addCharacteristic(bluetoothElevationGainLevel);
    bluetoothCycleService.addCharacteristic(bluetoothMaxElevationLevel);
    bluetoothCycleService.addCharacteristic(bluetoothMaximumVelocity);
    bluetoothCycleService.addCharacteristic(bluetoothMaximumAcceleration);
    bluetoothCycleService.addCharacteristic(bluetoothCurrentAcceleration);
    bluetoothCycleService.addCharacteristic(bluetoothImpact);
    bluetoothCycleService.addCharacteristic(bluetoothWeather);

    // Advertise BLE Service
    BLE.addService(bluetoothCycleService);
    BLE.advertise();

    // Print out that we are Waiting for a Connection
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

            // Update the Task Timing Values
            tempHumidityLength = millis() - tempHumidityStartTime;
            tempHumidityStartTimeH = tempHumidityStartTime;

            if(testing){
                Serial.print("Temperature & Humidity Task Time Interval: ");
                Serial.print(tempHumidityLength);
                Serial.println(" Milliseconds");
            }
        }
    }
}

// Method to Update Light Level
void updateLightLevel(){

    // Reading LightLevel
    lightLevelStartTime = millis();

    // Decide if the interval for Updating LightLevel has passed
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

// Bluetooth Servicing Function
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
            bluetoothLightLevelIntensity.writeValue(lightLevelIntensity);
            bluetoothLightLevelRed.writeValue(lightLevelRed);
            bluetoothLightLevelGreen.writeValue(lightLevelGreen);
            bluetoothLightLevelBlue.writeValue(lightLevelBlue);
            bluetoothElevationGainLevel.writeValue(elevationGain);
            bluetoothMaxElevationLevel.writeValue(maxElevationPoint);
            bluetoothMaximumVelocity.writeValue(AccelerometerMaxVelocityX);
            bluetoothMaximumAcceleration.writeValue(AccelerometerMaxAccellX);
            bluetoothCurrentAcceleration.writeValue(AccelerometeraccellX);
            bluetoothImpact.writeValue(AccelerometerImpactDetected);
            bluetoothWeather.writeValue(weather);

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

    if (testing) {
        Serial.print("Bluetooth Time: ");
        Serial.print(bluetoothStartTime);
        Serial.println(" ms");
    }

}

//Method to Update Pressure
void updatePressure () {

    // Reading Pressure
    pressureStartTime = millis();

    // Decide if the interval for Updating Pressure has passed
    if(pressureStartTime > (pressureStartTimeH+pressureInterval)) {

        if(pressureStartTime > (pressureStartTimeH+pressureFailInterval)){
            if(testing){
                Serial.println("Pressure failed to read in specified time.");
                delay(300);
            }

        // Log Error Code
        pressureError = ERRORTIMING;
        pressureStartTimeH = pressureStartTime;

        } else {

            // Print Out Elevation When Testing
            if (testing) {
                Serial.print("elevation History = ");
                Serial.print(elevationH);
                Serial.println(" m");
                Serial.print("elevation Gain = ");
                Serial.print(elevationGain);
                Serial.println(" m");
            }

            // Update Pressure Variables
            pressureKPA = BARO.readPressure(); // readPressure from sensor in kPa
            pressureHPA = pressureKPA * 10; // convert reading to hPa

            // Print Out Pressure When Testing
            if(testing) {
                Serial.print("Pressure = ");
                Serial.print(pressureKPA);
                Serial.println(" kPa");
                Serial.print("Pressure = ");
                Serial.print(pressureHPA);
                Serial.println(" hPa");
            }

            firstHalfOfElevationCalculation = pow((pressureHPA/pressureAtSeaLevel), (1/5.255)); // calculate the first half of the elevation calculation using Math library pow function
            elevation = 44340 * (1-firstHalfOfElevationCalculation); // perform second half of the calculation to get the elevation in m;

            // Print Out Elevation Calculation When Testing
            if (testing) {
                Serial.print("firstHalfOfElevationCalculation = ");
                Serial.print(firstHalfOfElevationCalculation);
                Serial.println();
                Serial.print("elevation = ");
                Serial.print(elevation);
                Serial.println(" m");
            }

            if (!secondRead) { // check to see what read we are in, if first read, basically ignore the read as its always weird
                readAttempts++;
                if (readAttempts == 1) {
                    secondRead = true;
                }
            } else {
                if (elevation > maxElevationPoint) { //update max elevation point if we go to a higher elevation point
                    maxElevationPoint = elevation;
                }

                if (elevationH == 0) { //if no change in elevation, update history, but basically do no nothing
                    elevationH = elevation;
                } else {
                    if (elevation != elevationH) { // if there is a change in elevation update elevationGain
                        elevationChange = (elevation - elevationH);
                        elevationGain += elevationChange;
                        elevationH = elevation; //reset history
                    }
                }
            }

            pressureLength = millis() - pressureStartTime; //7ms
            pressureStartTimeH = pressureStartTime; //Copy history so we can use it to trigger the next shot

            // Print Out Final Values When Testing
            if (testing) {
                Serial.print("elevation History = ");
                Serial.print(elevationH);
                Serial.println(" m");

                Serial.print("elevation Gain = ");
                Serial.print(elevationGain);
                Serial.println(" m");

                Serial.print("elevation Change = ");
                Serial.print(elevationChange);
                Serial.println(" m");

                Serial.print("Max Elevation Point = ");
                Serial.print(maxElevationPoint);
                Serial.println(" m");

                Serial.print("Pressure Length = ");
                Serial.print(pressureLength);
                Serial.println(" ms");


            }
        }
    }
}

// Method to Update Weather Prediction
void updateWeatherPrediction(){

    // Reading Weather Function Start Time
    weatherStartTime = millis();

    // Decide if the interval for Updating Weather Prediction has passed
    if(weatherStartTime > (weatherStartTimeH+weatherInterval)){

        // Check that the we have not failed to read the sensors in the expected period.
        if(weatherStartTime > (weatherStartTimeH+weatherFailInterval)){
            if(testing){
                Serial.println("Light Level failed to read in specified time.");
                delay(300);
            }

            // Log Error Code
            weatherError = ERRORTIMING;
            weatherStartTimeH = weatherStartTime;

        } else {

            // Update the Weather Information History
            weatherH = weather;

            // Update the Weather Prediction Value
            String tempWord = "";
            if ((temp < 20) && (temp < tempH)){
                tempWord = "Getting Colder";
            } else if (temp < 20){
                tempWord = "Cold";
            } else if ((temp > 34) && (temp > tempH)) {
                tempWord = "Getting Hotter";
            } else if (temp > 34) {
                tempWord = "Hot";
            } else {
                tempWord = "Mild";
            }

            String rainingWord = "";
            if ((humidity >= humidityH) && (pressureKPA < 100)){
                rainingWord = "Raining Harder";
            } else if ((humidity <= humidityH) && (pressureKPA > 100)){
                rainingWord = "Getting Dryer";
            } if (pressureKPA < 100){
                rainingWord = "Raining";
            } else if (pressureKPA > 100){
                rainingWord = "Dry";
            }

            String lightWord = "";
            if ((lightLevelIntensity < 150) && (lightLevelIntensity < lightLevelIntensityH)){
                lightWord = "Getting Dark";
            } else if (lightLevelIntensity < 150){
                lightWord = "Dark";
            } else if ((lightLevelIntensity > 1500) && (lightLevelIntensity < lightLevelIntensityH)){
                lightWord = "Getting Bright";
            } else if (lightLevelIntensity > 1500){
                lightWord = "Bright";
            } else {
                lightWord = "Mild Light";
            }

            weather = lightWord + " " + tempWord + " " + rainingWord;

            // Print Values if in Debug Mode
            if(testing){
                Serial.print("Weather: ");
                Serial.println(weather);
            }

            // Copy history so we can use it to trigger the next shot
            weatherLength = millis() - weatherStartTime;
            weatherStartTimeH = weatherStartTime;

            // Print Values if in Debug Mode
            if(testing){
                Serial.print("Weather Task Time Interval: ");
                Serial.print(weatherLength);
                Serial.println(" Milliseconds");
            }
        }
    }
}

//Method to Update Acceleration and Velocity
void updateAcceleration() {

    //Update Start Time and Calculate Interval between Readings
    AccelerometerStartTime = millis();
    AccelerometerTimeChange = ((AccelerometerStartTime - AccelerometerStartTimeH)*0.001);

    //Decide if interval for sensor update has passed
    if(AccelerometerStartTime > AccelerometerStartTimeH+AccelerometerTaskInterval){

        //Check to see whether we have failed to read the sensor within the designated Timeframe.
        if(AccelerometerStartTime > (AccelerometerStartTimeH+AccelerometerTaskFailInterval)){
            if(testing){
                Serial.println("Accelerometer Task Failed to happen in Time.");
                delay(300);
            }

            // Log Error Code
            AccelerometerTaskError = ERRORTIMING;
            AccelerometerStartTimeH = AccelerometerStartTime;
        } else{

            //Read Accelerometer Values
            if (IMU.accelerationAvailable()) {
                IMU.readAcceleration(x, y, z);

                //Update History
                AccelerometerVelocityXH = AccelerometerVelocityX;

                //Convert accelertion in G's into M/S^2
                float ax = (x*AccelerometerGravityMultiplier);
                AccelerometeraccellX = ax;

                //Calculate Change in Velocity
                AccelerometerVelocityChangeX = (ax*AccelerometerTimeChange);

                //Calculate total Velocity
                AccelerometerVelocityX = (AccelerometerVelocityXH + AccelerometerVelocityChangeX);


                //If negative deceleration exceeds threshold, update boolean expression
                if(x < AccelerometerImpact){
                    AccelerometerImpactDetected = true;
                }

            //If acceleration exceeds previous highest value, overwrite highest value
                if(ax > AccelerometerMaxAccellX){
                    AccelerometerMaxAccellX = ax;
                }

                //If velocity exceeds previous highest value, overwrite highest value
                if(AccelerometerVelocityX > AccelerometerMaxVelocityX){
                AccelerometerMaxVelocityX = AccelerometerVelocityX;
                }
            } else {
            // If IMU failed to start, log error code.
                if(AccelerometerTaskError == ERRORCLEAR){
                    if(testing) {
                    Serial.println("LSM9DS1 - IMU Failed to Start");
                    }
                    AccelerometerTaskError == ERRORSENSOR;
                }
            }

            AccelerometerTaskLength = millis() - AccelerometerStartTime;
            AccelerometerStartTimeH = AccelerometerStartTime; // Copy Start Time to history so we can use it to trigger the next reading

            if(testing) {
                Serial.print("Accelerometer Task Time Interval: ");
                Serial.print(AccelerometerTaskLength);
                Serial.println(" Milliseconds");
                Serial.print("Start Time: ");
                Serial.println(AccelerometerStartTime);
                Serial.print("Time Interval (S): ");
                Serial.println(AccelerometerTimeChange);
                Serial.print("Acceleration X (M/S^2): ");
                Serial.println(AccelerometeraccellX);
                Serial.print("Change in Velocity (M/S): ");
                Serial.println(AccelerometerVelocityChangeX);
                Serial.print("Velocity (M/S): ");
                Serial.println(AccelerometerVelocityX);
                Serial.print("Impact Detected: ");
                Serial.println(AccelerometerImpactDetected);
                Serial.print("Highest Recorded Acceleration (M/S^2): ");
                Serial.println(AccelerometerMaxAccellX);
                Serial.print("Highest Recorded Velocity (M/S): ");
                Serial.println(AccelerometerMaxVelocityX);
                Serial.println();
                Serial.println();
            }
        }
    }
}

void loop() {

    updateTempHumidity();

    updateLightLevel();

    updatePressure();

    updateAcceleration();

    updateWeatherPrediction();

    bluetoothStartTime = millis();
    if(bluetoothStartTime > (bluetoothStartTimeH+bluetoothInterval)) {
         bluetoothServicing();
    }
}
