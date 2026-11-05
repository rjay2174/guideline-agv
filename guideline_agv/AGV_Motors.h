#pragma once
#include "AGV_Config.h"

// Implemented by the main sketch; protects BLE and HTTP motor commands.
extern bool isMotionBlocked();

class AGVMotors {
private:
  uint8_t pinL_RPWM, pinL_LPWM, pinR_RPWM, pinR_LPWM, pinMOT_EN;
  SemaphoreHandle_t motorMutex = nullptr;
  volatile int appliedL = 0, appliedR = 0;
  volatile uint32_t motionHeartbeat = 0;

public:
  int maxSpeed = 255;
  int turnPercent = 100;
  float steeringGain = 1.20f;

  void begin(uint8_t l_rpwm, uint8_t l_lpwm, uint8_t r_rpwm, uint8_t r_lpwm, uint8_t en, int speed, int turnPct, float gain) {
    pinL_RPWM = l_rpwm; // GPIO 21
    pinL_LPWM = l_lpwm; // GPIO 38
    pinR_RPWM = r_rpwm; // GPIO 39
    pinR_LPWM = r_lpwm; // GPIO 40
    pinMOT_EN = en;     // GPIO 41

    maxSpeed = speed;
    turnPercent = turnPct;
    steeringGain = gain;
    motorMutex = xSemaphoreCreateMutex();

    const uint8_t mp[4] = {pinL_RPWM, pinL_LPWM, pinR_RPWM, pinR_LPWM};
    for (uint8_t p : mp) {
      pinMode(p, OUTPUT);
      digitalWrite(p, LOW);
      ledcAttach(p, PWM_FREQ_HZ, 8);
    }
    pinMode(pinMOT_EN, OUTPUT);
    stop();
  }

  void stop() {
    if (motorMutex) xSemaphoreTake(motorMutex, portMAX_DELAY);
    digitalWrite(pinMOT_EN, LOW);
    ledcWrite(pinL_RPWM, 0); ledcWrite(pinL_LPWM, 0);
    ledcWrite(pinR_RPWM, 0); ledcWrite(pinR_LPWM, 0);
    appliedL = 0; appliedR = 0;
    motionHeartbeat = millis();
    if (motorMutex) xSemaphoreGive(motorMutex);
  }

  void drive(int l, int r) {
    if (!motorMutex) return;
    // Re-check obstacle safety at the actuator boundary, including BLE tasks.
    if (isMotionBlocked()) { stop(); return; }
    xSemaphoreTake(motorMutex, portMAX_DELAY);
    l = constrain(l, 0, maxSpeed);
    r = constrain(r, 0, maxSpeed);

    int peak = max(l, r);
    int minimumInside = (peak * (100 - turnPercent) + 99) / 100;
    if (l < minimumInside) l = minimumInside;
    if (r < minimumInside) r = minimumInside;

    // Restore proven v1.9.19 direction: LEFT LPWM GPIO38 = forward.
    // GPIO21 and GPIO40 are inactive reverse channels (always zero).
    ledcWrite(pinL_RPWM, 0);
    ledcWrite(pinR_LPWM, 0);
    ledcWrite(pinL_LPWM, l);
    ledcWrite(pinR_RPWM, r);

    if (isMotionBlocked()) {
      digitalWrite(pinMOT_EN, LOW);
      ledcWrite(pinL_LPWM, 0); ledcWrite(pinR_RPWM, 0);
      appliedL = 0; appliedR = 0;
      xSemaphoreGive(motorMutex);
      return;
    }
    digitalWrite(pinMOT_EN, (l || r) ? HIGH : LOW);
    appliedL = l; appliedR = r;
    motionHeartbeat = millis();
    xSemaphoreGive(motorMutex);
  }

  int getAppliedL() { return appliedL; }
  int getAppliedR() { return appliedR; }
};