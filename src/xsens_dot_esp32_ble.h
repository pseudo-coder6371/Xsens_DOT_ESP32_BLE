#ifndef XSENS_DOT_ESP32_BLE_H
#define XSENS_DOT_ESP32_BLE_H

#include <NimBLEDevice.h>


// Callback functions
class MyClientCallback : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) override;
    void onDisconnect(NimBLEClient* pClient) override;
};

class AdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) override;
};

void scanEndedCallback(NimBLEScanResults results);

// Enum class for selecting payload
enum class PayloadMode{
    OrientationEuler = 0,
    OrientationQuaternion = 1,
    CustomMode1 = 2,
    CustomMode5 = 3
};

//Dot data class for saving it as 2D vectors (not used)
class DotData{
public:    
    String name;
    String address;
    String timestamp;
    std::vector<std::vector<float>> Euler;
    std::vector<std::vector<float>> Quaternion;
};

// Class for sensor functions
class Scanner{
public:
    PayloadMode PayloadType;
    DotData dotdata;
    bool init();
    bool connect_to_sensor();
    void disconnect_from_sensor();
    void getBatteryInfo();
    String getDeviceTag();
    String hexToAscii(String &name);
    void identifySensor();
    void powerOffSensor();
    void headingReset();
    void startMeasurement();
    void stopMeasurement();
    bool setOutputRate(uint8_t outPutRate);
    void enableSensor(const uint8_t* payload, size_t length);
    void disableSensor(const uint8_t* payload, size_t length);
    void OrientationEuler_notification_handler(const uint8_t* byteData, size_t length);
    float hexToFloat(const char* hexString);
    void ShortPayload_OrientationEuler(NimBLERemoteCharacteristic* characteristic, uint8_t* data, size_t length, bool isNotify);
    void ShortPayload_OrientationQuaternion(NimBLERemoteCharacteristic* characteristic, uint8_t* data, size_t length, bool isNotify);
    void MediumPayload_CustomMode1(NimBLERemoteCharacteristic* characteristic, uint8_t* data, size_t length, bool isNotify);
    void LongPayload_CustomMode5(NimBLERemoteCharacteristic* characteristic, uint8_t* data, size_t length, bool isNotify);
    void LongPayload_CustomMode5_calibration(NimBLERemoteCharacteristic* characteristic, uint8_t* data, size_t length, bool isNotify);
    void OrientationQuaternion_notification_handler(const uint8_t* byteData, size_t length);
    void CustomMode1_notification_handler(const uint8_t* byteData, size_t length);
    void CustomMode5_notification_handler(const uint8_t* byteData, size_t length);
    void CustomMode5_notification_handler_calibration(const uint8_t* byteData, size_t length);
    void timestampConvert(const uint8_t* byteData, size_t length);
};

#endif
