#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <Adafruit_NeoPixel.h>

#define LED_PIN    2
#define LED_COUNT 1
#define PI_SERIAL Serial

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

#define SERVICE_UUID        "fe77e1f2-1e06-11ee-be56-0242ac120002"
#define CHARACTERISTIC_UUID "060254ca-1e07-11ee-be56-0242ac120002"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        for (int i = 0; i < rxValue.length(); i++)
          Serial.println(rxValue[i]);
      }
    }
};

void setup() {

  // PI_SERIAL.begin(9600);
  Serial.begin(115200);


  // Create the BLE Device
  BLEDevice::init("DEV1");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {

  // Serial.println("running!!!!!");


  // while (!PI_SERIAL.available()) {
  //   delay(10);
  // }

  // while (PI_SERIAL.available()) {
  //   char ch = PI_SERIAL.read();
  //   Serial.println(ch);
  // }

  if (deviceConnected) {
    strip.setPixelColor(0, 0, 100, 255);
    strip.show();
    if (Serial.available() != 0) {
      char sendData = Serial.read();
      pCharacteristic->setValue((uint8_t*)&sendData, 1);
      pCharacteristic->notify();
    }
    delay(2);
  } else {
    strip.setPixelColor(0, 255, 0, 0);
    strip.show();
  }

  if (!deviceConnected && oldDeviceConnected) {
    delay(100);
    pServer->startAdvertising();
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    Serial.println("Connected");
    oldDeviceConnected = deviceConnected;
  }
}