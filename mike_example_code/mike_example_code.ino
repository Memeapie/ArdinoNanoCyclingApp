#include <ArduinoBLE.h>
#include <Arduino_HTS221.h>
#include <PDM.h>
#include <Arduino_LSM9DS1.h>



//Main globals for our real time system.
bool testing = true; //Used to control serial and other reporting during testing.
//Set to false to turn off reporting if timing starts to get tricky



//First lets set up the BLE_Handling variables:
long BLE_interval = 1000; //This is the interval in milliseconds at which I want to update the BLE
long BLE_StartTime;
long BLE_StartTimeH = 0;



long nowTime; //Holding place for current time when needed.



//Now for any constants.
const int ERRORCLEAR = 0; //No fault.
const int ERRORTIMING = 1; //The task failed due to a timing error.
const int ERRORSENSOR = 2; //A task sensor has failed.




// Now for some of the microphone handling variables.
short sampleBuffer[256]; // buffer to read samples into, each sample is 16-bits
volatile int samplesRead; // number of samples read



BLEService testService("1101"); //Get our BLE system ready.



//**************** TASK CONTROL ******************************************************
//Now lets set up some vars for each of our group members tasks, just one here for now
//We could do this with an array (pointers)but I will use discrete variables to control
//the group of tasks we will be using in our device to keep it easy to follow.
long task1_Interval = 500; //This is the task one interval in ms.
long task1_IntervalH; //This is the history of tasks interval how long it took last time.
long task1_Length; //This is the holding place for length of task this time.
long task1_StartTime; //The clock when the task starts.
long task1_StartTimeH = millis(); //The clock when the task starts history - force to NOW.
long task1_FailInterval = 1500; //This is the tasks fail interval if it
//is not serviced in this time the system has failed!
int task1_Error = ERRORCLEAR; //If we have and error



//Now any other variables we will need to be global of this particular tasks:
float x, y, z, cx, cy, cz; //The three accelerometer values and three magnatometer values.
float compassHeadingValue; //A calculated compass heading



//Now anything we want to send out on BLE lets get some vars ready.
BLECharCharacteristic accelLevelX("2101", BLERead | BLENotify);
BLECharCharacteristic accelLevelY("2102", BLERead | BLENotify);
BLECharCharacteristic accelLevelZ("2103", BLERead | BLENotify);
BLECharCharacteristic compassLevelX("2104", BLERead | BLENotify);
BLECharCharacteristic compassLevelY("2105", BLERead | BLENotify);
BLECharCharacteristic compassLevelZ("2106", BLERead | BLENotify);
BLECharCharacteristic compassHeading("2107", BLERead | BLENotify);



void setup(){
Serial.begin(9600); //Start the serial port.
while (!Serial); //Wait for serial to start.
if (!HTS.begin()) {
Serial.println("Failed to initialize humidity temperature sensor!");
while (1);
}
PDM.onReceive(audioCallBack);

if (!PDM.begin(1, 16000)) {
Serial.println("Failed to start PDM!");
while (1);
}



if (!IMU.begin()) {
Serial.println("Failed to initialize IMU!");
while (1);
}
if (!BLE.begin())
{
Serial.println("starting BLE failed!");
while (1);
}
BLE.setLocalName("MultiTasker");
BLE.setAdvertisedService(testService);
//Now for everything we want to send over our BLE service we need to set the characteristics up
//First those we want for Task 1 the accelerometer.
testService.addCharacteristic(accelLevelX);
testService.addCharacteristic(accelLevelY);
testService.addCharacteristic(accelLevelZ);
//Now for compass
testService.addCharacteristic(compassLevelX);
testService.addCharacteristic(compassLevelY);
testService.addCharacteristic(compassLevelZ);
testService.addCharacteristic(compassHeading);

BLE.addService(testService);



//Forgot this pair of lines
BLE.advertise();
Serial.println("Bluetooth device active, waiting for connections...");



}



void loop(){



float temperature = HTS.readTemperature();
//Serial.print("Temperature = ");
//Serial.print(temperature);
//Serial.println(" °C");
// wait for samples to be read
if (samplesRead) {



// print samples to the serial monitor or plotter
for (int i = 0; i < samplesRead; i++) {
if(sampleBuffer[i] > -40 && sampleBuffer[i] < 40){
sampleBuffer[i] = 0;
}
//Serial.println(sampleBuffer[i]);
}



// clear the read count
samplesRead = 0;

}



//********************************** The TASK 1 management *****************************
//Now lets set up our task processings system. There are many ways we could do this
//but for now we will keep it simple. Each task will be handled in the same way.
//First some preliminaries for the task - we will get set up
task1_StartTime = millis(); //First grab the time now, we will need this as a fixed point
//so getting it now is valuable and freezes it for us.
if(task1_StartTime > (task1_StartTimeH+task1_Interval)){ // We want to do task1 otherwise not.
//First lets check that things have not gone very badly.
if(task1_StartTime > (task1_StartTimeH+task1_FailInterval)){ //We have failed and we must report
if(testing){
Serial.println("Task 1 failed to happen in time.");
delay(300);
}
task1_Error = ERRORTIMING;
task1_StartTimeH = task1_StartTime;
}
else //We can carry on to do our task.
{
//Put all our task stuff here.
//In ths case just grab the magnatometor data.



if (IMU.magneticFieldAvailable()) {
IMU.readMagneticField(cx, cy, cz);
//Now we can find out how long our task has taken and update our histories.



//Now work out heading
float h = (atan2(cy,cx)*180)/PI;
//Copy calced value to variable for BLE handling.
compassHeadingValue = h;

if(testing){
Serial.print(cx);
Serial.print('\t');
Serial.print(cy);
Serial.print('\t');
Serial.print(cz);
Serial.print('\t');
Serial.println(h);
}
}
else {
//The magnatometer did not start - won't hold up but will report.
if(task1_Error == ERRORCLEAR){
if(testing){
Serial.println(" Magnatometer did not start.");
}
task1_Error = ERRORSENSOR;
}
}
//Now lets add a delay to check our timing system - we can remove this later.
//delay(50);
//Now we can find out how long our task has taken and update our histories.
task1_Length = millis() - task1_StartTime;
task1_StartTimeH = task1_StartTime; //Copy history so we can use it to trigger the next shot
if(testing){
Serial.print("Task 1 time interval: ");
Serial.print(task1_Length);
Serial.println(" Milliseconds");
}
}
}



BLE_StartTime = millis();
if(BLE_StartTime > (BLE_StartTimeH+BLE_interval)) {
BLE_servicing();
}



// Now longer used delay(1000);
if(testing){
Serial.println("Loop");
delay(500);
}
}



void BLE_servicing(){
BLEDevice central = BLE.central();

if (central)
{
if(testing) {
Serial.print("Connected to central: ");
Serial.println(central.address());
}
digitalWrite(LED_BUILTIN, HIGH);

if (central.connected()) { // We can do our sending to the BLE system
if(testing) {
Serial.print("AccelX % is now: ");
Serial.println(x);
}
//Now task by task send out the values.
//Remember these may need timed handling as well so task by task
//depending on timings needed for the client device the other end. (Watch / phone /...)
accelLevelX.writeValue(x);
accelLevelY.writeValue(y);
accelLevelZ.writeValue(z);
compassLevelX.writeValue(cx);
compassLevelY.writeValue(cy);
compassLevelZ.writeValue(cz);
compassHeading.writeValue(compassHeadingValue);
}
}
else {
Serial.print("Central disconnected: ");
Serial.println(" - ");
digitalWrite(LED_BUILTIN, LOW);
}
//Now lets get the history set up.
BLE_StartTimeH = BLE_StartTime;

}



void audioCallBack(){
// query the number of bytes available
int bytesAvailable = PDM.available();



// read into the sample buffer
PDM.read(sampleBuffer, bytesAvailable);



// 16-bit, 2 bytes per sample
samplesRead = bytesAvailable / 2;
}
