#include "xsens_dot_esp32_ble.h"

// Class instances 
Scanner scanner;

// Some declarations; 
unsigned long startTime;
bool measurementStarted_yes = false;
int time_duration = 150000; //Change this value to alter the duration of sensor operation (it is in ms)
int time_sec = 150; //this is just for serial output in seconds ; need to change both for optimal functionality
uint8_t outPutRate = 60; //Allowed values: 1,4,10,12,15,20,30,60,120 (Hz) {120Hz is reserved for recording only, no real-time streaming}
extern bool isInit;
bool dataPrinted = false;

// Setup code
void setup() {
  Serial.begin(9600);
  delay(2000);
  if(isInit == false){
      scanner.init();
      delay(3000);
  }
  scanner.connect_to_sensor();
  delay(1000);
  scanner.getBatteryInfo();
  delay(3000);
  scanner.getDeviceTag();
  delay(3000);
  // scanner.identifySensor();
  // delay(5000);
  scanner.headingReset();
  delay(5000);
  scanner.PayloadType = PayloadMode::CustomMode5; //Change this to alter the payload data ; Refer BLE guide for more information
  scanner.setOutputRate(outPutRate);
  delay(5000);

  scanner.startMeasurement();
  startTime = millis();
  Serial.print("Reading the data for: ");
  Serial.print(time_sec);
  Serial.println(" Seconds");
  measurementStarted_yes = true;

}

void loop(){
  if (measurementStarted_yes && (millis() - startTime >= time_duration)) {
    // Serial.println(esp_get_free_heap_size());
    scanner.stopMeasurement();
    Serial.print("Stopping the measurement after ");
    Serial.print(time_sec);
    Serial.println(" Seconds.");
    measurementStarted_yes = false;
    // scanner.powerOffSensor(); // Uncomment this to power off the sensor after measurement
  }
}
