#pragma once
#include <Arduino.h>
#include <math.h>
#include "AGV_Config.h"
#include "AGV_Sensors.h"
#include "AGV_Motors.h"
#include "AGV_RFID.h"
#include "AGV_StationConfig.h"
extern AGVStationConfig stations;
extern int latchedStation;

// ESP32-S3 / Arduino-ESP32 3.x: GPIO48 WS2812 RGB LED and event-based Serial diagnostics.
class AGVDiagnostics {
  uint8_t ledPin = 48;
  RunState lastState = IDLE;
  bool first = true;
  bool ledFlashOn = false;
  uint32_t lastFlashMs = 0;
  uint8_t ledR = 255, ledG = 255, ledB = 255;
  uint8_t lastIR = 0xff;
  int lastDist = -999, lastLPwm = -999, lastRPwm = -999;
  int lastPot = -999;
  int lastStation = -999;
  bool lastClear = false;
  Reading lastColor = {NAN,NAN,NAN,NAN};
  String loggedUID;
  bool cardShown = false;
  uint32_t lastSonarLog = 0, lastMotorLog = 0, lastColorLog = 0, lastPotLog = 0;
  int colorCandidate = -999;
  uint32_t colorCandidateSince = 0;

  static bool significant(float a, float b) {
    if (isnan(a) && isnan(b)) return false;
    if (isnan(a) || isnan(b)) return true;
    return fabsf(a-b) >= fmaxf(50.0f, fabsf(b)*0.08f);
  }

  static void fmtReading(char* dst, size_t cap, float v) {
    if (isnan(v)) snprintf(dst, cap, "NA");
    else snprintf(dst, cap, "%.0f", v);
  }

  void setRGB(uint8_t r,uint8_t g,uint8_t b) {
    if (r==ledR && g==ledG && b==ledB) return;
    ledR=r;ledG=g;ledB=b;
    // Unlike digitalWrite, neopixelWrite sends WS2812 serial color data.
    neopixelWrite(ledPin, r,g,b);
  }

  void updateLED(RunState st, uint32_t now, int colorIdx) {
    if (st == FAULT_STOP || st == RECOVERY_SEARCH) {
      if (first || st != lastState || now-lastFlashMs >= 450) {
        lastFlashMs = now;
        ledFlashOn = first || st != lastState ? true : !ledFlashOn;
        if (st == FAULT_STOP) setRGB(ledFlashOn ? 56 : 0,0,0);
        else setRGB(ledFlashOn ? 48 : 0,ledFlashOn ? 23 : 0,0);
      }
      return;
    }
    // Fault/obstacle indication is higher priority than station color.
    if(st==STATION_WAIT || st==IDLE || st==FOLLOW){
      if(colorIdx>=0){
        switch(colorIdx){
          case 0: setRGB(0,44,0);return;   // GREEN marker
          case 1: setRGB(52,0,0);return;   // RED marker
          case 2: setRGB(0,0,52);return;   // BLUE marker
          case 3: setRGB(44,28,0);return;  // YELLOW marker
        }
      }
    }
    switch(st) {
      case IDLE:          setRGB(0,0,36); break;      // Blue: idle
      case FOLLOW:        setRGB(0,44,0); break;      // Green: following
      case STATION_WAIT:  setRGB(48,28,0); break;     // Amber: station
      case OBSTACLE_STOP: setRGB(60,0,0); break;      // Red: obstacle
      case MANUAL:        setRGB(0,34,34); break;     // Cyan: manual
      case FINISHED:      setRGB(30,0,28); break;     // Purple: finished
      case LOAD_TEST_RUN: setRGB(28,28,28); break;
      default:            setRGB(0,0,24); break;
    }
  }

public:
  void beginRGB(uint8_t pin) {
    ledPin = pin;
    // This ESP32 core function uses the addressable-LED waveform (no external library).
    neopixelWrite(ledPin, 0, 0, 14);
    ledR=0;ledG=0;ledB=14;
    Serial.printf("[RGB] WS2812 active on GPIO%u.\n", ledPin);
  }

  void update(RunState st, AGVSensors& sensors, AGVRFID& rfid,
              AGVMotors& motors, uint8_t speedPotPin) {
    const uint32_t now=millis();
    int rgbStationColor=sensors.getCurrentStationIndex();
    if(st==STATION_WAIT && latchedStation>=1 && latchedStation<=3)
      rgbStationColor=stations.colors[latchedStation];
    updateLED(st, now, rgbStationColor);
    if (first || st != lastState) {
      Serial.printf("[%lu][STATE] %s%s\n", (unsigned long)now,
          getStateName(st),first ? " (boot)" : "");
      lastState=st;
    }

    const uint8_t ir=sensors.getIRBits();
    if (first || ir!=lastIR) {
      Serial.printf("[%lu][IR] S1..S5=%u%u%u%u%u (1=black) lineErr=%.2f\n",
        (unsigned long)now,(ir>>0)&1,(ir>>1)&1,(ir>>2)&1,(ir>>3)&1,(ir>>4)&1,
        sensors.getLineError());
      lastIR=ir;
    }

    const int dist=sensors.getDistanceCm();
    const bool clear=sensors.isPathClear();
    // No echo (distance=-1) is reported as NO ECHO, not a fictitious distance.
    if (first || clear!=lastClear || ((dist<0)!=(lastDist<0))) {
      if(dist < 0) Serial.printf("[%lu][SONAR] NO ECHO | path=%s (configured threshold=%dcm)\n",
        (unsigned long)now,clear?"CLEAR":"BLOCKED", sensors.getSafetyDistance());
      else Serial.printf("[%lu][SONAR] %dcm | path=%s (threshold=%dcm)\n",
        (unsigned long)now,dist,clear?"CLEAR":"BLOCKED", sensors.getSafetyDistance());
      lastDist=dist;lastClear=clear;lastSonarLog=now;
    }

    const int station=sensors.getCurrentStationIndex();
    const Reading c=sensors.getColorReading();
    if(station!=colorCandidate){colorCandidate=station;colorCandidateSince=now;}
    // A rapidly alternating floor/blue result is a calibration issue, not a
    // stable state transition. Report only classifications stable for 500ms.
    if (first || (station!=lastStation && (uint32_t)(now-colorCandidateSince)>=500)) {
      char r[20],g[20],b[20],cl[20];
      fmtReading(r,sizeof(r),c.r); fmtReading(g,sizeof(g),c.g);
      fmtReading(b,sizeof(b),c.b); fmtReading(cl,sizeof(cl),c.c);
      Serial.printf("[%lu][COLOR] R=%s G=%s B=%s Clear=%s Hz | classified=%s\n",
        (unsigned long)now,r,g,b,cl,sensors.getCurrentStationName());
      lastColor=c;lastStation=station;lastColorLog=now;
    }

    const String& uid=rfid.getLastSeenUID();
    const uint32_t seenAt=rfid.getLastSeenMs();
    const bool cardPresent=uid.length()>0 && (uint32_t)(now-seenAt)<800;
    if (cardPresent && (!cardShown || uid!=loggedUID)) {
      Serial.printf("[%lu][RFID] UID=%s\n",(unsigned long)now,uid.c_str());
      cardShown=true;loggedUID=uid;
    } else if(!cardPresent && cardShown) {
      Serial.printf("[%lu][RFID] Card no longer detected\n",(unsigned long)now);
      cardShown=false;
    }

    extern uint16_t speedPotRaw;
    extern int speedPotPwm;
    const int pot=int(speedPotRaw);
    if (first || (now-lastPotLog>=300 && abs(pot-lastPot)>=205)) {
      Serial.printf("[%lu][POT] GPIO%u raw=%d/4095 | potPWM=%d/255 | effectiveMaxPWM=%d/255\n",
        (unsigned long)now,speedPotPin,pot,speedPotPwm,motors.maxSpeed);
      lastPot=pot;lastPotLog=now;
    }

    const int l=motors.getAppliedL(),r=motors.getAppliedR();
    if (first || (now-lastMotorLog>=200 &&
        (abs(l-lastLPwm)>=4 || abs(r-lastRPwm)>=4 || ((l==0)!=(lastLPwm==0)) || ((r==0)!=(lastRPwm==0))))) {
      Serial.printf("[%lu][MOTOR] Left=%d/255 Right=%d/255\n",(unsigned long)now,l,r);
      lastLPwm=l;lastRPwm=r;lastMotorLog=now;
    }
    first=false;
  }
};