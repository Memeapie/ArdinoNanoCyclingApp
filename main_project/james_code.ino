#include <Arduino_LSM9DS1.h>

long JW_time = 0;
long JW_timeH = 0;
double JW_timechange = 0;

long JW_velocity = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in M/S^2's");
  Serial.println("X\tY\tZ\tTime\tInitialTime\tTimeChange");
}

void loop() {
  float x, y, z;

  JW_time = millis();
  JW_timechange = ((JW_time - JW_timeH)*0.001);

if(JW_time > JW_timeH+200){
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    Serial.print((x*9.80665));
    Serial.print('\t');
    Serial.print((y*9.80665));
    Serial.print('\t');
    Serial.print((z*9.80665));
    Serial.print('\t');
    Serial.print(JW_time);
    Serial.print('\t');
    Serial.print(JW_timeH);
    Serial.print('\t');
    Serial.println(JW_timechange);
  }
  JW_timeH = JW_time;
  }
}