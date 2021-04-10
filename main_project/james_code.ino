#include <Arduino_LSM9DS1.h>

// Enable Test Mode
bool testing = true;

const int ERRORCLEAR = 0; // No fault.
const int ERRORTIMING = 1; // The Task failed due to a timing error.
const int ERRORSENSOR = 2; // A Task sensor has failed.


//Accelerometer Values
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


//Accelerometer Task Control
int AccelerometerTaskInterval = 50; // Task interval in ms.
int AccelerometerTaskLength; // Length of task this time.
int AccelerometerStartTime = 0; // The start time of the task in ms.
int AccelerometerStartTimeH = millis(); // Start time of the last task run in ms.
double AccelerometerTimeChange = 0; //Time between each sensor measurement in seconds.
int AccelerometerTaskFailInterval = 1000; // The task fail interval in ms.
int AccelerometerTaskError = ERRORCLEAR; // Error Code Container.

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize Accelerometer!");
    while (1);
  }
}

void loop() {

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
  }

  AccelerometerTaskLength = millis() - AccelerometerStartTime;
  AccelerometerStartTimeH = AccelerometerStartTime; // Copy Start Time to history so we can use it to trigger the next reading

   if(testing){
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