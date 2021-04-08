int readAttempts = 1; //counter to read how many attempts has taken place, because when the sensor reads the pressure for the first time something weird happens
boolean secondRead = false; //turns to true after first read, prevents first reading affecting whole loop
double pressureAtSeaLevel = 1023.0; //atmpshpehric pressure at sea level, need to get a value for kent coast// currently using manston reading at 1800 on 07/04
double firstHalfOfElevationCalculation = 0; // part one of elevation calculation, makes use of <math> to do an x to the power of calculation
double elevation = 0; // the elevation the cyclist is cycling at
double elevationH = 0; // the cyclist elevation last time
double elevationChange = 0; // elevation change between current read and previous read
double elevationGain = 0; // the rider's total elevationGain or loss throughout the ride
double pressureKPA = 0; //sensor reads pressure in kPa
double pressureHPA = 0; // to turn pressure into evelation we need hPa

// Pressure Task Control
int pressureInterval = 500; // The task interval in ms.
int pressureLength; // Length of task this time.
int pressureStartTime; // The start time of the task in ms.
int pressureStartTimeH = millis(); // The clock when the task starts history.
int pressureFailInterval = 1500; // The task fail interval in ms.
int pressureError = ERRORCLEAR; // Error Code container.

//double pow;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
  }
}

void loop() {
  updatePressure();
}


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

      if (testing) {
        Serial.print("elevation History = ");
        Serial.print(elevationH);
        Serial.println(" m");
        Serial.print("elevation Gain = ");
        Serial.print(elevationGain);
        Serial.println(" m");
      }

      pressureKPA = BARO.readPressure(); // readPressure from sensor in kPa
      pressureHPA = pressureKPA * 10; // convert reading to hPa

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
        if (readAttempts = 1) {
          secondRead = true;
        }
      } else {
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

      pressureLength = millis() - pressureStartTime;
      pressureStartTimeH = pressureStartTime; //Copy history so we can use it to trigger the next shot

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
      }
    }
  }

  Serial.println();

  // wait 1 second to print again
  delay(1000);

}