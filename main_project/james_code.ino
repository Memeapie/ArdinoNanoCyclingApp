#include <Arduino_LSM9DS1.h>

// Enable Test Mode
bool testing = true;

const int ERRORCLEAR = 0; // No fault.
const int ERRORTIMING = 1; // The task failed due to a timing error.
const int ERRORSENSOR = 2; // A task sensor has failed.

double JW_gravityMultiplier = 9.80665;

long JW_startTime = 0;
long JW_startTimeH = 0;
double JW_timeChange = 0;

float x, y, z;

//Value History
float JW_accellX;
static float JW_velocityX = 0;
float JW_velocityChangeX;
float JW_velocityXH;
float JW_maxVelocityX;
float JW_maxAccellX;
int JW_impact = -3;
bool JW_impactDetected = false;


//Task Control
long JW_taskInterval = 50;
long JW_taskLength;
long JW_taskFailInterval = 1000;
int JW_taskError = ERRORCLEAR;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}

void loop() {

//Update Start Time and Calculate Interval between Readings
  JW_startTime = millis();
  JW_timeChange = ((JW_startTime - JW_startTimeH)*0.001);

//Decide if interval for sensor update has passed
if(JW_startTime > JW_startTimeH+JW_taskInterval){

//Check to see whether we have failed to read the sensor within the designated timeframe.
  if(JW_startTime > (JW_startTimeH+JW_taskFailInterval)){
    if(testing){
      Serial.println("JW Task Failed to happen in time.");
      delay(300);
    }

    // Log Error Code
    JW_taskError = ERRORTIMING;
    JW_startTimeH = JW_startTime;
  } else{

//Read IMU Values
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    //Update History
    JW_velocityXH = JW_velocityX;

    float ax = (x*JW_gravityMultiplier);
    JW_velocityChangeX = (ax*JW_timeChange);
    JW_velocityX = (JW_velocityXH + JW_velocityChangeX);
    JW_accellX = ax;

    if(ax < JW_impact){
      JW_impactDetected = true;
    }

    if(ax > JW_maxAccellX){
      JW_maxAccellX = ax;
    }

    if(JW_velocityX > JW_maxVelocityX){
      JW_maxVelocityX = JW_velocityX;
    }
  }

  JW_taskLength = millis() - JW_startTime;
  JW_startTimeH = JW_startTime; // Copy start time to history so we can use it to trigger the next reading

   if(testing){
    Serial.print("JW Task Time Interval: ");
    Serial.print(JW_taskLength);
    Serial.println(" Milliseconds");
    Serial.print("Start Time: ");
    Serial.println(JW_startTime);
    Serial.print("Time Interval (S): ");
    Serial.println(JW_timeChange);
    Serial.print("Acceleration X (M/S^2): ");
    Serial.println(JW_accellX);
    Serial.print("Change in Velocity (M/S): ");
    Serial.println(JW_velocityChangeX);
    Serial.print("Velocity (M/S): ");
    Serial.println(JW_velocityX);
    Serial.print("Impact Detected: ");
    Serial.println(JW_impactDetected);
    Serial.print("Maximum Acceleration (M/S^2): ");
    Serial.println(JW_maxAccellX);
    Serial.print("Maximum Velocity (M/S): ");
    Serial.println(JW_maxVelocityX);
    Serial.println();
    Serial.println();
     }
   }
  }
}