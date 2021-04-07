#include <Arduino_LPS22HB.h>
#include <math.h> //library for maths


boolean testing = true;
int readAttempts = 0;
boolean secondRead = false; //using second read at the minute because the first read is being weird
double pressureAtSeaLevel = 1023.0; //atmpshpehric pressure at sea level, need to get a value for kent coast// currently using manston reading at 1800 on 07/04
double firstHalfofElevationCalculation = 0;
double elevation = 0;
double elevationH = 0;
double elevationGain = 0;

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
 double pressureKPA = BARO.readPressure();

  if (testing) {
    Serial.print("elevation History = "); //forTesting purposes
    Serial.print(elevationH);
    Serial.println(" m");
    Serial.print("elevation Gain = "); //something weird happens, elevation gain automatically becomes like a value greater than 2800;
    Serial.print(elevationGain);
    Serial.println(" m");

    Serial.print("readAttempts");
    Serial.print(readAttempts);
    Serial.println();
    Serial.print("secondRead = ");
    Serial.print(secondRead);
    Serial.println();

    Serial.print("Pressure = ");
    Serial.print(pressureKPA);
    Serial.println(" kPa");

  }

  double pressureHPA = pressureKPA * 10;
  if(testing) {
    Serial.print("Pressure = ");
    Serial.print(pressureHPA);
    Serial.println(" hPa");
  }

  firstHalfofElevationCalculation = pow((pressureHPA/pressureAtSeaLevel), (1/5.255));
  elevation = 44340 * (1-firstHalfofElevationCalculation);

  if (testing) {
    Serial.print("firstHalfofElevationCalculation = ");
    Serial.print(firstHalfofElevationCalculation);
    Serial.println();
    Serial.print("elevation = ");
    Serial.print(elevation);
    Serial.println(" m");

  }

  if (!secondRead) { //doesn't really work, need to fix this
    readAttempts++;
    if (readAttempts=1) {
      secondRead = true;
    }
  } else {
    if (elevationH = 0 || elevationH > 2000) { //elevationH does something weird on first read therefore if we reach outlier, we reset history value
      elevationH = elevation;
    } else {
      if (elevation != elevationH) {

         elevationGain = elevationGain + (elevation - elevationH); //doesn't work as intended
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
  }



   Serial.println();

  // wait 1 second to print again
  delay(1000);

}