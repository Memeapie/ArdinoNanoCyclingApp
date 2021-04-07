//place for luke to store code to show progress, will move into main_project.ino when complete


#include <Arduino_LPS22HB.h>
#include <math.h> //library for maths



double Po = 1013.0; //atmpshpehric pressure at sea level, need to get a value for kent coast
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
  // read the sensor value
  float pressure = BARO.readPressure();

  // print the sensor value
  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" kPa");


  double pressureHPA = pressure * 10;
  Serial.print("Pressure = ");
  Serial.print(pressureHPA);
  Serial.println(" hPa");


  double test = 1/5.255;
  double test2 = pressureHPA / Po;
  double test3 = pow(test2, test);
  double test4 = 1 - test3;
  double test5 = 44340 * test4; // this outputs a result in hectopascal

  Serial.print(test); //obviously will tidy this up when i've got it working properly, currently says i'm -36m below seaLevel
  Serial.println();
  Serial.print(test2);
  Serial.println();
  Serial.print(test3);
  Serial.println();
  Serial.print(test4);
  Serial.println();
  Serial.print(test5);

  //pow = ((pressure/Po), test);
  //Serial.print(pow);
  //float altitude;

  // print an empty line
  Serial.println();

  // wait 1 second to print again
  delay(1000);
}