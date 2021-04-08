#include <Arduino_LPS22HB.h>
#include <math.h> //library for maths


boolean testing = true;
int readAttempts = 1;
boolean secondRead = false; //using second read at the minute because the first read is being weird
double pressureAtSeaLevel = 1023.0; //atmpshpehric pressure at sea level, need to get a value for kent coast// currently using manston reading at 1800 on 07/04
double firstHalfOfElevationCalculation = 0;
double elevation = 0;
double elevationH = 0;
double elevationChange = 0;
double elevationGain = 0;
double pressureKPA = 0;

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

  if (testing) {
    Serial.print("elevation History = "); //forTesting purposes
    Serial.print(elevationH);
    Serial.println(" m");
    Serial.print("elevation Gain = ");
    Serial.print(elevationGain);
    Serial.println(" m");
  }

  pressureKPA = BARO.readPressure();
  double pressureHPA = pressureKPA * 10;

  if(testing) {
    Serial.print("Pressure = ");
    Serial.print(pressureKPA);
    Serial.println(" kPa");
    Serial.print("Pressure = ");
    Serial.print(pressureHPA);
    Serial.println(" hPa");
  }

  firstHalfOfElevationCalculation = pow((pressureHPA/pressureAtSeaLevel), (1/5.255));
  elevation = 44340 * (1-firstHalfOfElevationCalculation);

  if (testing) {
    Serial.print("firstHalfOfElevationCalculation = ");
    Serial.print(firstHalfOfElevationCalculation);
    Serial.println();
    Serial.print("elevation = ");
    Serial.print(elevation);
    Serial.println(" m");

  }

  if (!secondRead) { //doesn't really work, need to fix this
    readAttempts++;
    if (readAttempts = 1) {
      secondRead = true;
    }
  } else {
    if (elevationH == 0) { //elevationH does something weird on first read therefore if we reach outlier, we reset history value
        elevationH = elevation;
    } else {
      if (elevation != elevationH) {
         elevationChange = (elevation - elevationH);
         elevationGain += elevationChange; //doesn't work as intended
         elevationH = elevation;
      }
    }
  }

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

   Serial.println();

  // wait 1 second to print again
  delay(1000);

}