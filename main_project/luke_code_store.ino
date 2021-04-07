//place for luke to store code to show progress, will move into main_project.ino when complete


#include <Arduino_LPS22HB.h>
#include <math.h> //library for maths


boolean testing = true;
double pressureAtSeaLevel = 1023.0; //atmpshpehric pressure at sea level, need to get a value for kent coast// currently using manston reading at 1800 on 07/04
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

  double firstHalfofElevationCalculation = pow((pressureHPA/pressureAtSeaLevel), (1/5.255));
  double elevation = 44340 * (1-firstHalfofElevationCalculation);

  if (testing) {
    Serial.print("firstHalfofElevationCalculation = ");
    Serial.print(firstHalfofElevationCalculation);
    Serial.println();
    Serial.print("elevation = ");
    Serial.print(elevation);
    Serial.println(" m");

  }

   Serial.println();

  // wait 1 second to print again
  delay(1000);

}