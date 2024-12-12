#include <Arduino.h>
#include <NimBLEDevice.h>
#include "xsens_dot_esp32_ble.h"
#include <functional>



// UUID definitions; Defined in Xsens DOT BLE guide
NimBLEUUID DOT_Configuration_ServiceUUID("15171000-4947-11E9-8646-D663BD873D93");
NimBLEUUID DOT_Configuration_Control_CharacteristicUUID("15171002-4947-11E9-8646-D663BD873D93");
NimBLEUUID DOT_Measure_ServiceUUID("15172000-4947-11E9-8646-D663BD873D93");
NimBLEUUID Heading_Reset_Control_CharacteristicUUID("15172006-4947-11E9-8646-D663BD873D93");
NimBLEUUID DOT_ShortPayload_CharacteristicUUID("15172004-4947-11E9-8646-D663BD873D93");
NimBLEUUID DOT_MediumPayload_CharacteristicUUID("15172003-4947-11E9-8646-D663BD873D93");
NimBLEUUID DOT_Battery_CharacteristicUUID("15173001-4947-11E9-8646-D663BD873D93");
NimBLEUUID DOT_LongPayload_CharacteristicUUID("15172002-4947-11E9-8646-D663BD873D93");
NimBLEUUID DOT_Battery_ServiceUUID("15173000-4947-11E9-8646-D663BD873D93");
NimBLEUUID DOT_Control_Measure_CharacteristicUUID("15172001-4947-11E9-8646-D663BD873D93");

// Control command arrays
const byte Select_Orientation_Euler[] = {1, 1, 4};
const byte Deselect_Orientation_Euler[] = {1, 0, 4};
const byte Select_Orientation_Quaternion[] = {1, 1, 5};
const byte Deselect_Orientation_Quaternion[] = {1, 0, 5};
const byte Heading_Reset_Buffer[] = {1, 0};
const byte Select_Custom_Mode[] = {1, 1, 22};
const byte Deselect_Custom_Mode[] = {1, 0, 22};
const byte Select_Custom_Mode5[] = {1, 1 , 26};
const byte Deselect_Custom_Mode5[] = {1, 0 , 26};

// Pointer definitions
static NimBLEAdvertisedDevice* dotSensor = nullptr; 
NimBLEClient* pClient = nullptr;
NimBLERemoteService* pConfig_Service = nullptr;
NimBLERemoteCharacteristic* pBattery = nullptr;
NimBLERemoteService* pService_battery = nullptr;
NimBLERemoteCharacteristic* pGetTag = nullptr;
NimBLERemoteCharacteristic* pIdentifySensor = nullptr;
NimBLERemoteCharacteristic* pPowerOff = nullptr;
NimBLERemoteCharacteristic* pHeadingReset = nullptr;
NimBLERemoteService* pMeasurement = nullptr;
NimBLERemoteCharacteristic* pShortPayload = nullptr;
NimBLERemoteCharacteristic* pOutputRate = nullptr;
NimBLERemoteCharacteristic* pControl_measure = nullptr;
NimBLERemoteCharacteristic* pMediumPayload = nullptr;
NimBLERemoteCharacteristic* pLongPayload = nullptr;

// std::vector<std::vector<float>> saved_values_customMode1;
// std::vector<std::vector<float>> saved_values_customMode5;

bool isConnected = false;
bool isInit = false;
extern unsigned long startTime;
extern int time_duration;
extern bool measurementStarted_yes;

bool Scanner::init() {
    if (isConnected == false){
        Serial.println("Initializing BLE...");
        delay(5000);
        NimBLEDevice::init("");
        NimBLEDevice::setPower(ESP_PWR_LVL_P9);
        NimBLEDevice::getScan()->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
        NimBLEDevice::getScan()->setInterval(100);
        NimBLEDevice::getScan()->setWindow(50);
        NimBLEDevice::getScan()->setActiveScan(true);
        NimBLEDevice::getScan()->start(10, scanEndedCallback);
        isInit = true;
        return true;
        delay(1000);
    }else{
        Serial.println("Cannot initialise BLE..");
        isInit = false;
        return false;
    }
}

// Callback definitions
void MyClientCallback::onConnect(NimBLEClient* pClient) {
    Serial.println("Connected to sensor");
}

void MyClientCallback::onDisconnect(NimBLEClient* pClient) {
    Serial.println("Disconnected from sensor.");
}


void AdvertisedDeviceCallbacks::onResult(NimBLEAdvertisedDevice* advertisedDevice) {
    if (advertisedDevice->getName() == "Movella DOT") {
        Serial.println("Found Xsens DOT sensor");
        Serial.print("Address: ");
        Serial.println(advertisedDevice->getAddress().toString().c_str());
        int rssi = advertisedDevice->getRSSI();
        Serial.print("Device Signal strength: ");
        Serial.print(rssi);
        Serial.println(" dBm");
        dotSensor = advertisedDevice;
        NimBLEDevice::getScan()->stop();
        delay(2000);
    }
}

void scanEndedCallback(NimBLEScanResults results) {
    Serial.println("Scan ended");
}

bool Scanner::connect_to_sensor(){
    if (pClient == nullptr) {
        pClient = NimBLEDevice::createClient();
        pClient->setClientCallbacks(new MyClientCallback());
    }
    delay(1000);
    if (pClient->connect(dotSensor)&& isConnected == false) {
        isConnected = true;
        Serial.println(pClient->getPeerAddress().toString().c_str());
        delay(1500);  
        pService_battery = pClient->getService(DOT_Battery_ServiceUUID);
        pConfig_Service = pClient->getService(DOT_Configuration_ServiceUUID);
        pMeasurement = pClient->getService(DOT_Measure_ServiceUUID);
        return true;
        // if (pService_battery == nullptr) {
        //     Serial.println("Battery Service not found.");
        // } else {
        //     Serial.println("Battery Service found.");
        //     delay(500);
        // }
    } else {
        Serial.println("Failed to connect to DOT sensor, retrying..");
        dotSensor = nullptr;
        delay(1000);
        NimBLEDevice::getScan()->start(10, scanEndedCallback);
        return false;
    }

    
}

void Scanner::disconnect_from_sensor() {
    if (pClient != nullptr && pClient->isConnected()) {
        pClient->disconnect();
    }
    NimBLEDevice::deinit();
    isConnected = false;
}

void Scanner::getBatteryInfo() {
    if (!pClient->isConnected()) {
        Serial.println("Sensor is not connected!");
        return;
    }

    if (pService_battery == nullptr) {
        Serial.println("Battery Service not available.");
        return;
    }

    pBattery = pService_battery->getCharacteristic(DOT_Battery_CharacteristicUUID);
    if (pBattery == nullptr) {
        Serial.println("Battery characteristic not found.");
        return;
    }

        NimBLEAttValue batteryValue = pBattery->readValue();

        uint8_t batteryPer = batteryValue[0];
        uint8_t status = batteryValue[1];
        // Serial.print("Battery Data: ");
        // Serial.print(batteryValue[0], HEX);
        // Serial.print(", ");
        // Serial.println(batteryValue[1], HEX);

        // Print battery info
        Serial.print("Battery Level: ");
        Serial.print(batteryPer);
        Serial.print("%, Charging Status: ");
        Serial.println(status == 0 ? "Not Charging" : "Charging");
}

String Scanner::getDeviceTag(){
    pGetTag = pConfig_Service->getCharacteristic(DOT_Configuration_Control_CharacteristicUUID);
    if(pGetTag != nullptr){
        String tag_device_bytes = "";
        String tag_test_raw = "";
        NimBLEAttValue Tag = pGetTag->readValue();
        // for (int i = 8; i < 16; i++){
        //     tag_test_raw += String(Tag[i], HEX);
        // }
        // Serial.print("Raw device tag for testing..:");
        // Serial.println(tag_test_raw);

        for (int i = 8 ; i < 20 ; i++){
            String hexByte = String(Tag[i], HEX);
            if(hexByte.length()==1) hexByte = "0" + hexByte;
            tag_device_bytes += hexByte;
        }
        String name = hexToAscii(tag_device_bytes);
        Serial.print("Read device tag: ");
        Serial.println(name);
        return name;
    }else{
        Serial.println("Device tag not found");
        return "Not found";
    }
}

String Scanner::hexToAscii(String &name){
    String ascii = ""; 
    for (size_t i = 0; i < name.length(); i += 2) {
        String part = name.substring(i, i + 2);
        char ch = strtoul(part.c_str(), nullptr, 16);
        if (ch != '\0') {
            ascii += ch;
        }
    }
    return ascii;
}


void Scanner::identifySensor(){
    if(pIdentifySensor = pConfig_Service->getCharacteristic(DOT_Configuration_Control_CharacteristicUUID)){
        String UUID = pIdentifySensor->getUUID().toString().c_str();
        // Serial.println(UUID);
        delay(1000);
        uint8_t bArr[32];
        bArr[0] = 1;
        bArr[1] = 1;
        pIdentifySensor->writeValue(bArr, sizeof(bArr));
        Serial.println("Identifying, check the LED on your sensor");
    }else{
        Serial.println("Cannot identify sensor, please check the code");
    }
}

void Scanner::powerOffSensor(){
    if(pPowerOff = pConfig_Service->getCharacteristic(DOT_Configuration_Control_CharacteristicUUID)){
        delay(1000);
        uint8_t bArr[32];
        bArr[0] = 2;
        bArr[1] = 0;
        bArr[2] = 1;
        pPowerOff->writeValue(bArr, sizeof(bArr));
        Serial.println("Sensor powering off.. Goodbye.");
    }else{
        Serial.println("Cannot switch off the sensor");
    }
}

void Scanner::headingReset(){
    if(pHeadingReset = pMeasurement->getCharacteristic(Heading_Reset_Control_CharacteristicUUID)){
        Serial.println("Heading reset complete.");
    }else{
        Serial.println("Heading reset failed");
    }
}

bool Scanner::setOutputRate(uint8_t outPutRate){
    const int validRates[] = {1,4,10,12,15,20,30,60,120};
    uint8_t bArr[32];
    bool isValid = false;

    for (size_t i = 0; i < sizeof(validRates); i++){
        if(outPutRate == validRates[i]){
            isValid = true;
            break;
        }
    }
    if(isValid == true){
        bArr[0] = 16;
        bArr[24] = outPutRate & 0xff;
        bArr[25] = (outPutRate >>8) & 0xff;
        Serial.println("Setting output rate");
        pOutputRate = pConfig_Service->getCharacteristic(DOT_Configuration_Control_CharacteristicUUID);
        if(pOutputRate->writeValue(bArr, sizeof(bArr))){
            Serial.print("Successfully set output rate to: ");
            Serial.print(outPutRate);
            Serial.println(" Hz");
            delay(100);
            return true;
        }else{
            Serial.println("Error setting output rate");
        }
    }
    return false;
}

// Enable the notification on payload characteristic before start action 
// Which means : subcribe to nofitications first and then use enable_sensor
// Global flag to track if measurement has started
bool measurementStarted = false;

void Scanner::startMeasurement() {
    if (PayloadType == PayloadMode::OrientationEuler) {
        // Serial.println("Payload mode: Euler Angles");
        if (!measurementStarted) {
            pShortPayload = pMeasurement->getCharacteristic(DOT_ShortPayload_CharacteristicUUID);
            if (pShortPayload == nullptr) {
                Serial.println("Short Payload characteristic not found!");
                return;
            }
            enableSensor(Select_Orientation_Euler, sizeof(Select_Orientation_Euler));
            if (pShortPayload->subscribe(true, 
                std::bind(&Scanner::ShortPayload_OrientationEuler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4))) {
                Serial.println("Successfully subscribed to notifications for Short payload");
            } else {
                Serial.println("Failed to subscribe to notifications");
            }
            measurementStarted = true;
        } else {
            // Serial.println("Measurement already started, continuing data streaming...");
        }
    }else if (PayloadType == PayloadMode::OrientationQuaternion){
        if(!measurementStarted){
            pShortPayload = pMeasurement->getCharacteristic(DOT_ShortPayload_CharacteristicUUID);
            if(pShortPayload == nullptr){
                Serial.println("Short payload characteristic not found");
                return;
            }
            enableSensor(Select_Orientation_Quaternion, sizeof(Select_Orientation_Quaternion));
            if(pShortPayload->subscribe(true, 
            std::bind(&Scanner::ShortPayload_OrientationQuaternion, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4))){
                Serial.println("Successfully subscribed to short payload notifications");
            }else{
                Serial.println("Failed to subscribe to notifications");
            }
            measurementStarted = true;
        }else{
            Serial.println("Measurement already started..");
        }
    }else if(PayloadType == PayloadMode::CustomMode1){
        if(!measurementStarted){
            pMediumPayload = pMeasurement->getCharacteristic(DOT_MediumPayload_CharacteristicUUID);
            if(pMediumPayload == nullptr){
                Serial.println("Characteristic not found");
                return;
            }
        }   enableSensor(Select_Custom_Mode, sizeof(Select_Custom_Mode));
            if(pMediumPayload->subscribe(true,
            std::bind(&Scanner::MediumPayload_CustomMode1, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4))){
                Serial.println("Successfully subscribed to medium payload notifications");
            }else{
                Serial.println("Failed to subscribe to medium payload notifications");
            }
            measurementStarted = true;
    }else if(PayloadType == PayloadMode::CustomMode5){
        if(!measurementStarted){
            pLongPayload = pMeasurement->getCharacteristic(DOT_LongPayload_CharacteristicUUID);
            if(pLongPayload == nullptr){
                Serial.println("Characteristic not found");
                return;
            }
            enableSensor(Select_Custom_Mode5, sizeof(Select_Custom_Mode5));
            if(pLongPayload->subscribe(true,
            std::bind(&Scanner::LongPayload_CustomMode5, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4))){
                Serial.println("Successfully subscribed to long payload notifications");
            }else{
                Serial.println("Could not subscribe to long payload notifications");
            }
            measurementStarted = true;
        }
    }
}


void Scanner::stopMeasurement(){
    if (PayloadType == PayloadMode::OrientationEuler){
        disableSensor(Deselect_Orientation_Euler, sizeof(Deselect_Orientation_Euler));
        pShortPayload->unsubscribe();
        Serial.println();
        Serial.println("Unsubscribed from notifications.. Stopping measurement");
    }else if (PayloadType == PayloadMode::OrientationQuaternion){
        disableSensor(Deselect_Orientation_Quaternion, sizeof(Deselect_Orientation_Quaternion));
        pShortPayload->unsubscribe();
        Serial.println();
        Serial.println("Unsubscribed from notifications.. Stopping measurement");
    }else if(PayloadType == PayloadMode::CustomMode1){
        disableSensor(Deselect_Custom_Mode, sizeof(Deselect_Custom_Mode));
        pMediumPayload->unsubscribe();
        Serial.println();
        Serial.println("Unsubscribed from notifications.. Stopping measurement");
    }else if(PayloadType == PayloadMode::CustomMode5){
        disableSensor(Deselect_Custom_Mode5, sizeof(Deselect_Custom_Mode5));
        pLongPayload->unsubscribe();
        Serial.println();
        Serial.println("Unsubscribed from notifications.. Stopping measurement");
    }
}

void Scanner::ShortPayload_OrientationEuler(NimBLERemoteCharacteristic* characteristic, uint8_t* data, size_t length, bool isNotify) {
    this->OrientationEuler_notification_handler(data, length);
}

void Scanner::ShortPayload_OrientationQuaternion(NimBLERemoteCharacteristic* characteristic, uint8_t* data, size_t length, bool isNotify){
    this->OrientationQuaternion_notification_handler(data, length);
}

void Scanner::MediumPayload_CustomMode1(NimBLERemoteCharacteristic* characteristic, uint8_t* data, size_t length, bool isNotify){
    this->CustomMode1_notification_handler(data, length);
}

void Scanner::LongPayload_CustomMode5(NimBLERemoteCharacteristic* characteristic, uint8_t* data, size_t length, bool isNotify){
    this->CustomMode5_notification_handler(data, length);
}


void Scanner::enableSensor(const uint8_t* payload, size_t length){
    pControl_measure = pMeasurement->getCharacteristic(DOT_Control_Measure_CharacteristicUUID);
    if(pControl_measure->writeValue(payload, length)){
        delay(100);
        Serial.println("Sensor is enabled");
    }else{
        Serial.println("Could not write values to enable sensor");
    }
}

void Scanner::disableSensor(const uint8_t* payload, size_t length){
    pControl_measure = pMeasurement->getCharacteristic(DOT_Control_Measure_CharacteristicUUID);
    if(pControl_measure->writeValue(payload, length)){
        delay(100);
        Serial.println();
        Serial.println("Sensor is disabled");
    }else{
        Serial.println("Could not write values to disable sensor");
    }
}

void Scanner::OrientationEuler_notification_handler(const uint8_t* byteData, size_t length) {
    uint32_t timestamp = *reinterpret_cast<const uint32_t*>(byteData);
    float roll = *reinterpret_cast<const float*>(byteData + 4);
    float pitch = *reinterpret_cast<const float*>(byteData + 8);
    float yaw = *reinterpret_cast<const float*>(byteData + 12);
    String StringToPrint = "Time: " + String(timestamp) + ", Roll: " + String(roll,6) + ", Pitch: " + String(pitch,6) + ", Yaw: " + String(yaw,6);
    Serial.println(StringToPrint);
}

void Scanner::OrientationQuaternion_notification_handler(const uint8_t* byteData, size_t length){
    uint32_t timestamp = *reinterpret_cast<const uint32_t*>(byteData);
    float w = *reinterpret_cast<const float*>(byteData + 4);
    float x = *reinterpret_cast<const float*>(byteData + 8);
    float y = *reinterpret_cast<const float*>(byteData + 12);
    float z = *reinterpret_cast<const float*>(byteData + 16);
    String StringToPrint = "Time: " + String(timestamp) + " , q0: "  + String(w,6) + " , q1: " + String(x,6) + ", q2: " + String(y,6) + ", q3: " + String(z,6);
    Serial.println(StringToPrint);
}


void Scanner::CustomMode1_notification_handler(const uint8_t* byteData, size_t length){
    uint32_t timestamp = *reinterpret_cast<const uint32_t*>(byteData);
    float roll = *reinterpret_cast<const float*>(byteData + 4);
    float pitch = *reinterpret_cast<const float*>(byteData + 8);
    float yaw = *reinterpret_cast<const float*>(byteData + 12);
    float acc_x = *reinterpret_cast<const float*>(byteData + 16);
    float acc_y = *reinterpret_cast<const float*>(byteData + 20);
    float acc_z = *reinterpret_cast<const float*>(byteData + 24);
    float gyro_x = *reinterpret_cast<const float*>(byteData + 28);
    float gyro_y = *reinterpret_cast<const float*>(byteData + 32);
    float gyro_z = *reinterpret_cast<const float*>(byteData + 36);
    String StringToPrint = "Time: " + String(timestamp) + ", roll: " + String(roll, 6) + ", pitch: " + String(pitch, 6) + ", yaw: " + String(yaw,6) + ", Acc_X: " + String(acc_x,6) 
    + ", Acc_Y: " + String(acc_y,6) + ", Acc_Z: " + String(acc_z,6) + ", Gyro_X: " + String(gyro_x,6) + ", Gyro_Y: " + String(gyro_y,6) + ", Gyro_Z: " + String(gyro_z,6);
    Serial.println(StringToPrint);
}

extern bool calibration_mode;


void Scanner::CustomMode5_notification_handler(const uint8_t* byteData, size_t length){
    uint32_t timestamp = *reinterpret_cast<const uint32_t*>(byteData);
    float w = *reinterpret_cast<const float*>(byteData + 4);
    float x = *reinterpret_cast<const float*>(byteData + 8);
    float y = *reinterpret_cast<const float*>(byteData + 12);
    float z = *reinterpret_cast<const float*>(byteData + 16);
    float acc_x = *reinterpret_cast<const float*>(byteData + 20);
    float acc_y = *reinterpret_cast<const float*>(byteData + 24);
    float acc_z = *reinterpret_cast<const float*>(byteData + 28);
    float gyro_x = *reinterpret_cast<const float*>(byteData + 32);
    float gyro_y = *reinterpret_cast<const float*>(byteData + 36);
    float gyro_z = *reinterpret_cast<const float*>(byteData + 40);
    String StringToPrint = "Time: " + String(timestamp) + " , q0: " + String(w,6) + " , q1: " + String(x,6) + " , q2: " + String(y,6) + " , q3: " + String(z,6) 
    + " , Acc_X: " + String(acc_x,6) + " , Acc_Y: " + String(acc_y,6) + " , Acc_Z: " + String(acc_z,6) + " , Gyro_X: " + String(gyro_x,6) + " , Gyro_Y: " + String(gyro_y,6) + 
    " , Gyro_Z: " + String(gyro_z,6);
    Serial.println(StringToPrint);
}

