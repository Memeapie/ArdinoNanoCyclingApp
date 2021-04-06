#include <Arduino_HTS221.h>
#include <ArduinoBLE.h>
#include <PDM.h>
#include <Arduino_LSM9DS1.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);

  PDM.onReceive(onPDMdata);

  if (!HTS.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1);
  }

  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

}

void loop() {


}
