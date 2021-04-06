#include <Arduino_HTS221.h>
#include <ArduinoBLE.h>
#include <PDM.h>
#include <Arduino_LSM9DS1.h>

// buffer to read samples into, each sample is 16-bits
short sampleBuffer[256];
  
// number of samples read
volatile int samplesRead;
  
void onPDMdata() {
    // query the number of bytes available
    int bytesAvailable = PDM.available();
  
    // read into the sample buffer
    int bytesRead = PDM.read(sampleBuffer, bytesAvailable);
  
    // 16-bit, 2 bytes per sample
    samplesRead = bytesRead / 2;
}

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
  /*float temp = HTS.readTemperature();
  float humid = HTS.readHumidity();

  Serial.println(temp);
  Serial.println(humid);
  */

  /*
  if(samplesRead){

      for(int i = 0; i < sampleBuffer[i]; i++){
        if(sampleBuffer[i] > -40 && sampleBuffer[i] < 40){
           sampleBuffer[i] = 0;
        }
        Serial.println(sampleBuffer[i]); 
      }

      samplesRead = 0;
  }
  */

  float x, y, z;

  if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(x, y, z);

        Serial.print(x);
        Serial.print('\t');
        Serial.print(y);
        Serial.print('\t');
        Serial.println(z);
  }

  delay(250);
}


//this is a test comment.
