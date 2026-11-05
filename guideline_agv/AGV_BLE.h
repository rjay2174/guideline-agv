#pragma once
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "AGV_Config.h"
#include "AGV_Motors.h"

#define BLE_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHAR_DRIVE_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"

extern AGVMotors motors;
extern RunState state;
extern uint32_t lastManualCommandTime;
extern bool isMotionBlocked();

static BLEServer* pServer = nullptr;
static BLECharacteristic* pDriveCharacteristic = nullptr;
static bool bleConnected = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleConnected = true;
    Serial.println("[BLE] Remote connected via Web Bluetooth");
  }
  void onDisconnect(BLEServer* pServer) override {
    bleConnected = false;
    if (state == MANUAL) {
      state = IDLE;
      motors.stop();
    }
    Serial.println("[BLE] Remote disconnected");
    BLEDevice::startAdvertising();
  }
};

class DriveCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    uint8_t* data = pCharacteristic->getData();
    size_t len = pCharacteristic->getLength();
    if (len >= 2) {
      int8_t throttleRaw = (int8_t)data[0];
      int8_t turnRaw     = (int8_t)data[1];

      float y = (float)throttleRaw / 100.0f;
      float x = (float)turnRaw / 100.0f;
      if (y < 0.0f) y = 0.0f;

      int p = motors.maxSpeed;
      int l = lroundf(p * y * (x < 0 ? 1.0f + x : 1.0f));
      int r = lroundf(p * y * (x > 0 ? 1.0f - x : 1.0f));

      if (isMotionBlocked() || (state != IDLE && state != MANUAL)) {
        motors.stop();
        return;
      }
      state = MANUAL;
      lastManualCommandTime = millis();
      motors.drive(l, r);
    }
  }
};

inline void initBLE(const char* deviceName) {
  BLEDevice::init(deviceName);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  
  BLEService *pService = pServer->createService(BLE_SERVICE_UUID);
  pDriveCharacteristic = pService->createCharacteristic(
      BLE_CHAR_DRIVE_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pDriveCharacteristic->setCallbacks(new DriveCallbacks());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
}