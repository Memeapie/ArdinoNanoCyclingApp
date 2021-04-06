#include <Arduino_HTS221.h>
#include <ArduinoBLE.h>
#include <PDM.h>
#include <Arduino_LSM9DS1.h>

// buffer to read samples into, each sample is 16-bits
short sampleBuffer[256];
  
// number of samples read
volatile int samplesRead;

// BLE Characteristics
BLEService testService("180C");
BLECharCharacteristic accelLevelX("2101", BLERead | BLENotify);
BLECharCharacteristic accelLevelY("2102", BLERead | BLENotify);
BLECharCharacteristic accelLevelZ("2103", BLERead | BLENotify);
BLECharCharacteristic compassLevelX("2104", BLERead | BLENotify);
BLECharCharacteristic compassLevelY("2105", BLERead | BLENotify);
BLECharCharacteristic compassLevelZ("2106", BLERead | BLENotify);
BLECharCharacteristic compassHeading("2107", BLERead | BLENotify);
  
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

  if(!BLE.begin()){
    Serial.println("Failed to initialize BLE!");
    while (1);
  }

  BLE.setLocalName("Multitasker");
  BLE.setAdvertisedService(testService);
  testService.addCharacteristic(accelLevelX);
  testService.addCharacteristic(accelLevelY);
  testService.addCharacteristic(accelLevelZ);
  testService.addCharacteristic(compassLevelX);
  testService.addCharacteristic(compassLevelY);
  testService.addCharacteristic(compassLevelZ);
  BLE.addService(testService);

  BLE.advertise();
  Serial.print("Peripheral device MAC: ");
  Serial.println(BLE.address());
  Serial.println("Waiting for connections...");
  
}

void loop() { 
  // Temp/Humidity Example
  /*float temp = HTS.readTemperature();
  float humid = HTS.readHumidity();

  Serial.println(temp);
  Serial.println(humid);
  */

  // Sound Example
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
  
  // Gyroscope Example
  float gyroX, gyroY, gyroZ;

  if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gyroX, gyroY, gyroZ);

        /*
        Serial.print(gyroX);
        Serial.print('\t');
        Serial.print(gyroY);
        Serial.print('\t');
        Serial.println(gyroZ);
        */
        
  }

  // Mag Field Example
  float magX, magY, magZ;
  
  if(IMU.magneticFieldAvailable()){
        IMU.readMagneticField(magX, magY, magZ);

        /*
        Serial.print(magX);
        Serial.print('\t');
        Serial.print(magY);
        Serial.print('\t');
        Serial.println(magZ);
        */
  }

  // Wait for a BLE central to connect
  BLEDevice central = BLE.central();
  
  // if a central is connected to the peripheral:
  if (central.connected()) {
    accelLevelX.writeValue(gyroX);
    accelLevelY.writeValue(gyroY);
    accelLevelZ.writeValue(gyroZ);
    compassLevelX.writeValue(magX);
    compassLevelY.writeValue(magY);
    compassLevelZ.writeValue(magZ);
  }

  delay(50);
}
