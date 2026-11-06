// =========================================================================
//                   GUIDELine AGV - MAIN CONTROLLER SKETCH
// =========================================================================

// -------------------------------------------------------------------------
// 1. HARDWARE PIN ASSIGNMENTS (Matched to Physical Wiring Table)
// -------------------------------------------------------------------------
// HC-SR04 Ultrasonic
const uint8_t PIN_US_TRIG     = 1;
const uint8_t PIN_US_ECHO     = 2;

// Potentiometer Analog Input
const uint8_t PIN_SPEED_POT   = 3;

// TCRT5000 5-Channel Line Tracker (S1 Leftmost -> S5 Rightmost)
const uint8_t PIN_IR_S1       = 4;
const uint8_t PIN_IR_S2       = 5;
const uint8_t PIN_IR_S3       = 6;
const uint8_t PIN_IR_S4       = 7;
const uint8_t PIN_IR_S5       = 8;

// TCS3200 Color Sensor
const uint8_t PIN_COLOR_S0    = 9;
const uint8_t PIN_COLOR_S1    = 10;
const uint8_t PIN_COLOR_S2    = 11;
const uint8_t PIN_COLOR_S3    = 14;
const uint8_t PIN_COLOR_OUT   = 15;

// MFRC522 RFID SPI Reader
const uint8_t PIN_RFID_SCK    = 12;
const uint8_t PIN_RFID_MISO   = 13;
const uint8_t PIN_RFID_MOSI   = 16;
const uint8_t PIN_RFID_SS     = 17;
const uint8_t PIN_RFID_RST    = 18;

// BTS7960 Motor Driver
const uint8_t PIN_L_RPWM      = 21; // Left reverse input: held at PWM=0
const uint8_t PIN_L_LPWM      = 38; // LEFT FORWARD: verified in working v1.9.19
const uint8_t PIN_R_RPWM      = 39; // Right Forward
const uint8_t PIN_R_LPWM      = 40; // Right Reverse
const uint8_t PIN_MOT_EN      = 41; // Common Enable (Both Drivers)

// Indicators & Board Controls
const uint8_t PIN_WS2812_RGB  = 48; // Onboard RGB Status
const uint8_t PIN_BOOT_BTN    = 0;  // Reset / Mode Button

// -------------------------------------------------------------------------
// 2. SPEED & MOTION CONTROL PARAMETERS
// -------------------------------------------------------------------------
int   maxMotorSpeed           = 255;  // 0 to 255 PWM ceiling
int   innerWheelReductionPct  = 100;  // 0% to 100% inside-wheel reduction
float steeringSensitivity     = 1.20; // 0.70 Slow, 1.20 Default, 1.80 Fast
int   safetyStopDistanceCm    = 20;   // Ultrasonic obstacle zone (cm)
bool  enableManualBypass      = false; // Safety guard cannot be bypassed

// -------------------------------------------------------------------------
// 3. COLOR DETECTION PRESETS (Chromaticity Scaled)
// -------------------------------------------------------------------------
#include "AGV_Config.h"
Reading CAL_GREEN_START       = { 768.28f,  910.89f,  946.80f, 2768.75f};
Reading CAL_RED_STATION       = {1491.26f,  819.48f, 1007.61f, 3357.40f};
Reading CAL_BLUE_STATION      = { 781.22f,  924.22f, 1396.21f, 3242.95f};
Reading CAL_YELLOW_STATION    = {1860.25f, 1473.57f, 1212.82f, 4675.39f};

// -------------------------------------------------------------------------
// 4. OPERATOR WI-FI ACCESS POINT
// -------------------------------------------------------------------------
const char* AP_HOTSPOT_SSID   = "GUIDELine-AGV";
const char* AP_HOTSPOT_PASS   = "GUIDELineBenchOnly"; // KNOWN DEMO PASS - NOT FOR PUBLIC OPERATION


// =========================================================================
// MODULAR SUBSYSTEMS & ORCHESTRATION
// =========================================================================
#include "AGV_WebPage.h"
#include "AGV_Motors.h"
#include "AGV_Sensors.h"
#include "AGV_RFID.h"
#include "AGV_StationConfig.h"
#include "AGV_LoadTest.h"
#include "AGV_Comm.h"
#include "AGV_OTA.h"
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include "AGV_Diagnostics.h"
#include "AGV_SerialConsole.h"

AGVMotors  motors;
AGVSensors sensors;
AGVRFID    rfid;
AGVStationConfig stations;
AGVLoadTest loadTest;
AGVComm    comm;
AGVDiagnostics diagnostics;
AGVSerialConsole console;

// Physical POT is the primary PWM ceiling; Web/Load Test cannot exceed it.
// No phone is required. ADC 0..4095 maps to 0..255, without a forced
// minimum PWM (zero MUST stop). Update every 40 ms with noise filtering.
// Saved two-endpoint calibration; only effective after explicit Save.
// Calibration remains in ESP32 NVS when power is removed.
Preferences potCalPrefs;
uint16_t speedPotCalMin = 0;
uint16_t speedPotCalMax = 4095;
bool speedPotCalEnabled = false;
uint16_t speedPotCaptureMin = 0;
uint16_t speedPotCaptureMax = 4095;
bool speedPotMinCaptured = false;
bool speedPotMaxCaptured = false;
uint16_t speedPotRaw = 0;
uint16_t speedPotFiltered = 0;
int speedPotPwm = 0;
int loadTestPwmCeiling = 255;
uint32_t lastSpeedPotReadMs = 0;
uint32_t dynamicBridgeMs = STRAIGHT_BRIDGE_MIN_MS;
uint32_t dynamicSearchEndMs = STRAIGHT_BRIDGE_MAX_MS;
int bridgePWMAtLineLoss = 0;

void updatePhysicalSpeedPot(bool force=false) {
  const uint32_t now = millis();
  if(!force && uint32_t(now-lastSpeedPotReadMs)<40) return;
  lastSpeedPotReadMs = now;
  uint32_t sum=0;
  for(int n=0;n<8;n++) sum += analogRead(PIN_SPEED_POT);
  const uint16_t sample=uint16_t(sum/8);
  speedPotRaw=sample;
  if(force) speedPotFiltered=sample;
  else speedPotFiltered=uint16_t((3UL*speedPotFiltered + sample + 2UL)/4UL);
  // Calibrated span maps the physical end stops to full 0..255 range.
  // A small deadband prevents low-end ADC noise from creeping the AGV.
  const int lo = speedPotCalEnabled ? int(speedPotCalMin) : 0;
  const int hi = speedPotCalEnabled ? int(speedPotCalMax) : 4095;
  const int range = max(1, hi-lo);
  const int adjusted = constrain(int(speedPotFiltered)-lo,0,range);
  speedPotPwm = (adjusted*255 + range/2)/range;
  if(adjusted <= max(4,range/100)) speedPotPwm = 0;
  if(adjusted >= range-max(4,range/100)) speedPotPwm = 255;
  motors.maxSpeed = min(speedPotPwm, loadTest.active ? loadTestPwmCeiling : 255);
  if(motors.maxSpeed == 0) motors.stop();
  else if(motors.getAppliedL()>motors.maxSpeed || motors.getAppliedR()>motors.maxSpeed)
    motors.drive(min(motors.getAppliedL(),motors.maxSpeed),
                 min(motors.getAppliedR(),motors.maxSpeed));
}

void configureLineLossRecoverySpeed() {
  // PWM-based travel approximation. Keep roughly constant bridge distance
  // across POT settings; actual distance must be checked on the floor.
  const int cap=motors.maxSpeed;
  bridgePWMAtLineLoss = (cap>0) ? min(cap, max(1,int(lroundf(float(cap)*45.0f/255.0f)))) : 0;
  dynamicBridgeMs = bridgePWMAtLineLoss>0 ?
      uint32_t(constrain(lroundf(2000.0f*45.0f/float(bridgePWMAtLineLoss)), 600L, 4000L)) : 2000UL;
  const int searchPWM=(cap>0)?min(cap,max(1,cap/2)):0;
  const uint32_t searchMs=searchPWM>0 ?
      uint32_t(constrain(lroundf(5000.0f*90.0f/float(searchPWM)), 2500L, 7000L)) : 5000UL;
  dynamicSearchEndMs=dynamicBridgeMs+searchMs;
}

RunState state = IDLE;
uint32_t lastManualCommandTime = 0;
RunState resumeAfterObstacle = FOLLOW;
uint32_t obstacleHoldStartedAt = 0;
uint32_t lineLostAt = 0;
float lastKnownLineError = 0.0f;
uint32_t departHoldUntil = 0;
bool routeStarted = false;
bool departedGreen = false;
bool visitedRedStation = false;
bool lastStationWasRed = false;
int latchedStation = -1;
uint8_t nextStationSlot=1;  // 1..3, reset on new route
uint32_t stationWaitStartMs=0;
uint32_t lastStationTimerLog=0;
bool loadTestAuthorized=false;
uint32_t lastRFIDPollAt = 0;
// All-black lift/wide-black safety: evaluated against calibrated IR polarity.
uint32_t allIRBlackSinceMs = 0;
bool allIRBlackSafetyLatched = false;

bool isAllIRBlack() { return (sensors.getIRBits() & ALL_IR_BLACK_MASK) == ALL_IR_BLACK_MASK; }
bool isAllIRBlackSafetyLatched() { return allIRBlackSafetyLatched; }
uint32_t getAllIRBlackElapsedMs() {
  if (!allIRBlackSinceMs) return 0;
  return (uint32_t)(millis() - allIRBlackSinceMs);
}

void checkAllIRBlackSafety() {
  if (allIRBlackSafetyLatched) { motors.stop(); return; }
  // Do not latch a fault just because the AGV is parked on a broad black marker.
  const bool wheelsRunning = motors.getAppliedL() > 0 || motors.getAppliedR() > 0;
  if (!wheelsRunning || !isAllIRBlack()) {
    allIRBlackSinceMs = 0;
    return;
  }
  const uint32_t now = millis();
  if (!allIRBlackSinceMs) { allIRBlackSinceMs = now; return; }
  if ((uint32_t)(now - allIRBlackSinceMs) < ALL_IR_BLACK_STOP_MS) return;

  // Persistent safety fault: only deliberate Web START/RESET or Master Card
  // authorization can recover, and only after the 11111 pattern clears.
  allIRBlackSafetyLatched = true;
  motors.stop();
  if (loadTest.active) {
    loadTest.finish("ALL IR BLACK - 3s SAFETY STOP");
    loadTestPwmCeiling=255; updatePhysicalSpeedPot(true);
  }
  state = FAULT_STOP;
  lineLostAt = 0;
  Serial.printf("[%lu][SAFETY] ALL FIVE IR BLACK continuously for %lu ms (11111): MOTORS DISABLED; latched FAULT_STOP. Clear IR pattern, then Master RFID or Web START/RESET.\n",
    (unsigned long)now, (unsigned long)ALL_IR_BLACK_STOP_MS);
}

// Arduino IDE network OTA: initialization once STA has an IP address.
// Keep the captive AP and GitHub pull-update functionality unchanged.
bool arduinoOTAStarted = false;
bool arduinoOTAUpdating = false;
bool otaHttpMdnsPublished = false;
uint32_t lastOTAConnectionCheck = 0;

void requestSafeStop();
void initArduinoNetworkOTA() {
  if (arduinoOTAStarted) return;
  ArduinoOTA.setHostname(DEVICE_HOSTNAME);
  ArduinoOTA.setPort(3232);
  ArduinoOTA.setPassword(AP_HOTSPOT_PASS);
  ArduinoOTA.setRebootOnSuccess(true);
  ArduinoOTA.onStart([]() {
    // OTA can start only while safely stopped. Disable all motor commands
    // until the transfer finishes or the controller is reset.
    motors.stop();
    requestSafeStop();
    arduinoOTAUpdating = true;
    state = SERVICE_TEST; // Explicit motor-locked maintenance state.
    Serial.println("[OTA] Arduino IDE update started. Motors locked OFF.");
  });
  ArduinoOTA.onProgress([](unsigned int, unsigned int) {
    motors.stop();
  });
  ArduinoOTA.onEnd([]() {
    motors.stop();
    Serial.println("[OTA] Firmware received; rebooting with motors disabled.");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    motors.stop();
    arduinoOTAUpdating = false;
    state = FAULT_STOP;
    Serial.printf("[OTA] Failure %u. Motors remain stopped; use RESET.\n", unsigned(error));
  });
  ArduinoOTA.begin();
  arduinoOTAStarted = true;
  Serial.printf("[OTA] ArduinoOTA initialized | UDP 3232 | host=%s | STA connected=%s\n",
    DEVICE_HOSTNAME,WiFi.status()==WL_CONNECTED?"YES":"NO");
}


bool isMotionBlocked() {
  // An IR=11111 pattern prevents any *new* motor command from rest, including BLE.
  // If already moving, let the existing continuous-3-second lift/wide-black
  // safeguard handle the stop, to avoid an immediate stop over a wide marker.
  const bool stationary = motors.getAppliedL() == 0 && motors.getAppliedR() == 0;
  return allIRBlackSafetyLatched || !sensors.isPathClear() ||
         rfid.isDiagnosticActive() || (stationary && isAllIRBlack());
}

void requestSafeStop() {
  motors.stop();
  if(loadTest.active){loadTest.finish("STOPPED BY USER");loadTestPwmCeiling=255; updatePhysicalSpeedPot(true);}
  // An all-black fault may only be cleared after the 11111 pattern disappears.
  // A RESET does not start the motors; a subsequent START is still required.
  if (allIRBlackSafetyLatched && !isAllIRBlack()) {
    allIRBlackSafetyLatched = false;
    allIRBlackSinceMs = 0;
    Serial.println("[SAFETY] All-black fault cleared by STOP/RESET; new START required.");
  }
  state = allIRBlackSafetyLatched ? FAULT_STOP : IDLE;
  lineLostAt = 0;
  latchedStation = -1;
  // An explicit STOP / RESET cancels the route and requires a new START.
  routeStarted = false;
  departedGreen = false;
  visitedRedStation = false;
  lastStationWasRed = false;
  departHoldUntil = 0;nextStationSlot=1;stationWaitStartMs=0;
}

bool requestAutonomousStart(bool resetFault) {
  if(rfid.isDiagnosticActive()) return false;
  if(state==LOAD_TEST_RUN || state==SERVICE_TEST || state==MANUAL) return false;
  motors.stop();
  if (!sensors.isPathClear() || sensors.captureBusy()) return false;
  // INSTANT START INHIBIT: reject Master, Web START, station acknowledgement,
  // and timer departure whenever all 5 sensors currently see BLACK. This is
  // independent of the 3-second all-black fault latch used while driving.
  if (isAllIRBlack()) {
    Serial.println("[SAFETY] START rejected: IR=11111 (vehicle not confirmed on guide line).");
    return false;
  }
  if (state == FAULT_STOP && !resetFault) return false;
  // The pattern has cleared. Master Card / Web START is a deliberate recovery.
  if (allIRBlackSafetyLatched) {
    allIRBlackSafetyLatched = false;
    allIRBlackSinceMs = 0;
    Serial.println("[SAFETY] All-black fault cleared by authorized START.");
  }
  if (state == STATION_WAIT && latchedStation>=1 && latchedStation<=3) {
    visitedRedStation=true;
    if(nextStationSlot<=3) nextStationSlot=uint8_t(latchedStation+1);
  }
  // Starting a new journey resets route progress, while station ACK keeps it.
  if (state == IDLE || state == FINISHED) {
    routeStarted = true;
    departedGreen = false;
    visitedRedStation = false;
    lastStationWasRed = false;nextStationSlot=1;
  }
  if (state == FAULT_STOP) routeStarted = true;
  state = FOLLOW;
  lineLostAt = 0;
  // Allow the vehicle to move off its current station marker before rereading.
  departHoldUntil = millis() + STATION_DEPARTURE_MS;
  sensors.clearStationLatch();
  latchedStation = -1;stationWaitStartMs=0;
  return true;
}

void handleAutonomousFollow();
void handleStationScan();
void handleManualWatchdog();
void handleObstacleHold();
void handleRecoverySearch();
void handleCardAuthorization();
void handleStationDetection();
void enterObstacleStop(RunState from);
void logEvent(const char* msg);

void setup() {
  Serial.begin(115200);
  diagnostics.beginRGB(PIN_WS2812_RGB);

  // Initialize hardware modules with verified pins
  motors.begin(PIN_L_RPWM, PIN_L_LPWM, PIN_R_RPWM, PIN_R_LPWM, PIN_MOT_EN, maxMotorSpeed, innerWheelReductionPct, steeringSensitivity);
  analogReadResolution(12);
  // GPIO3 must receive ONLY 0..3.3 V from the wiper of a 10k pot.
  analogSetPinAttenuation(PIN_SPEED_POT, ADC_11db);
  pinMode(PIN_SPEED_POT, INPUT);
  if(potCalPrefs.begin("agv-pot",false)) {
    const uint16_t storedMin=potCalPrefs.getUShort("min",0);
    const uint16_t storedMax=potCalPrefs.getUShort("max",4095);
    const bool saved=potCalPrefs.getBool("valid",false);
    if(saved && storedMax>storedMin+100 && storedMax<=4095) {
      speedPotCalMin=storedMin;speedPotCalMax=storedMax;speedPotCalEnabled=true;
    }
  }
  speedPotCaptureMin=speedPotCalMin;
  speedPotCaptureMax=speedPotCalMax;
  updatePhysicalSpeedPot(true); // Apply physical speed ceiling before RFID start
  sensors.beginIR(PIN_IR_S1, PIN_IR_S2, PIN_IR_S3, PIN_IR_S4, PIN_IR_S5);
  sensors.beginSonar(PIN_US_TRIG, PIN_US_ECHO, safetyStopDistanceCm);
  sensors.beginColor(PIN_COLOR_S0, PIN_COLOR_S1, PIN_COLOR_S2, PIN_COLOR_S3, PIN_COLOR_OUT);
  sensors.loadColorPresets(CAL_GREEN_START, CAL_RED_STATION, CAL_BLUE_STATION, CAL_YELLOW_STATION);
  rfid.begin(PIN_RFID_SCK, PIN_RFID_MISO, PIN_RFID_MOSI, PIN_RFID_SS, PIN_RFID_RST);
  stations.begin();
  comm.begin(AP_HOTSPOT_SSID, AP_HOTSPOT_PASS);
  console.begin();
  // Same sequencing as the old working v1.9.15: initialize ArduinoOTA in setup,
  // not conditionally only after STA has already connected.
  initArduinoNetworkOTA();
  Serial.println("[OTA] UDP 3232 initialized. mDNS network discovery requires router connection.");

  Serial.println("==================================================");
  Serial.println("       GUIDELine AGV Physical Core Online         ");
  Serial.printf("[FW] Build %s | UI %s\n",AGV_FW_BUILD,AGV_UI_BUILD);
  Serial.printf ("       Connect AP: %s (Key: %s)\n", AP_HOTSPOT_SSID, AP_HOTSPOT_PASS);
  Serial.println("==================================================");
}

void logEvent(const char* msg) {
  Serial.printf("[%lu][EVENT] %s\n", (unsigned long)millis(), msg);
}

void enterObstacleStop(RunState from) {
  resumeAfterObstacle = from;
  obstacleHoldStartedAt = millis();
  state = OBSTACLE_STOP;
  motors.stop();
  logEvent("Obstacle detected: motors stopped");
}

void handleCardAuthorization() {
  const String card = rfid.readUID();
  if (!card.length()) return;
  // Cards may still be scanned or registered while parked, but they cannot
  // authorize ANY departure while the IR pattern is 11111.
  if (isAllIRBlack()) {
    Serial.printf("[SAFETY] RFID %s rejected: IR=11111. Position AGV on guide line first.\n",card.c_str());
    return;
  }
  if (state == OBSTACLE_STOP) {
    logEvent("Card ignored: obstacle stop automatically resumes after clearance");
    return;
  }
  if (!sensors.isPathClear()) {
    logEvent("Card rejected: obstacle in path");
    return;
  }
  const bool master = rfid.isMaster(card);
  if (state == IDLE || state == FINISHED || state == FAULT_STOP || state == RECOVERY_SEARCH) {
    if (master) {
      bool recovering = state == FAULT_STOP || state == RECOVERY_SEARCH;
      if (requestAutonomousStart(true))
        logEvent(recovering ? "Master: authorized line recovery" : "Master: started journey");
    } else logEvent("Station card cannot start/recover vehicle");
  } else if (state == STATION_WAIT) {
    if (rfid.isAuthorized(card, latchedStation)) {
      if (requestAutonomousStart(true)) logEvent("RFID: station departure authorized");
    } else logEvent("RFID: card not authorized at this station");
  }
}

void handleStationDetection() {
  const int detected=sensors.getCurrentStationIndex();
  if(routeStarted && detected!=stations.colors[0] && detected>=0) departedGreen=true;
  // Sensor also may see an unclassified floor between markers.
  if(routeStarted && detected<0) departedGreen=true;
  if((int32_t)(millis()-departHoldUntil)<0) return;
  if(!sensors.detectsStationMarker())return;
  // GREEN (configurable start/finish) only ends a journey after at least one station.
  if(detected==stations.colors[0] && routeStarted && departedGreen && visitedRedStation) {
    motors.stop();state=FINISHED;routeStarted=false;
    logEvent("Return marker reached: trip complete"); return;
  }
  // Physical stations 1..3 are ordered. Repeated marker colors are allowed.
  if(nextStationSlot<=3 && stations.colors[nextStationSlot]>=0 &&
     detected==stations.colors[nextStationSlot] && routeStarted && departedGreen){
    motors.stop();state=STATION_WAIT;latchedStation=nextStationSlot;
    stationWaitStartMs=millis();
    Serial.printf("[STATION] Slot %u: detected color %d; waiting for assigned RFID or web\n",nextStationSlot,detected);
  }
}

void handleStationTimeout(){
  motors.stop();
  if(!stations.autoContinue || latchedStation<1 || latchedStation>3 ||
     !stations.timeoutSec[latchedStation] || !stationWaitStartMs)return;
  uint32_t waitMs=(uint32_t)(millis()-stationWaitStartMs);
  if(waitMs < stations.timeoutSec[latchedStation]*1000UL)return;
  // Timed departure obeys the SAME instant 11111 interlock as RFID/Web START.
  // Do not repeatedly log failed starts every loop tick.
  if(isAllIRBlack() || !sensors.isPathClear() || sensors.captureBusy())return;
  if(requestAutonomousStart(true))logEvent("Station timer expired: bounded departure authorized");
}

// Center-first branch selection: if the middle sensor still sees the
// intended line while S1/S5 also sees a junction, ignore the side branch.
// Do not classify 11111 as ordinary track; retain the all-black safeguard.
static inline bool hasCenterWithOuterBranch(uint8_t irMask) {
  return (irMask & 0x04u) != 0 && (irMask & 0x11u) != 0 && irMask != 0x1Fu;
}

void handleLoadTestRun(){
  // The end-of-line obstacle is the STOP signal. NEVER auto-resume the test.
  if(!loadTest.active){motors.stop();state=IDLE;return;}
  if(!sensors.isPathClear()){
    motors.stop();loadTest.finish("OBSTACLE END POINT");
    loadTestPwmCeiling=255; updatePhysicalSpeedPot(true);state=IDLE;
    logEvent("Load test stopped at obstacle; result recorded");return;
  }
  if((uint32_t)(millis()-loadTest.startMs)>=90000UL){
    motors.stop();loadTest.finish("TIMEOUT 90s");loadTestPwmCeiling=255; updatePhysicalSpeedPot(true);state=FAULT_STOP;
    logEvent("Load test timeout");return;
  }
  if(!sensors.hasLine()){
    motors.stop();loadTest.finish("LINE LOST");loadTestPwmCeiling=255; updatePhysicalSpeedPot(true);state=FAULT_STOP;
    logEvent("Load test stopped: line lost");return;
  }
  // When S3 and S1/S5 see black simultaneously at a junction, stay on S3.
  const uint8_t irMask=sensors.getIRBits();
  const int requested=motors.maxSpeed;
  if (hasCenterWithOuterBranch(irMask)) {
    motors.drive(requested,requested);
    return;
  }
  const float err=sensors.getLineError();
  if(fabsf(err)<0.16f){motors.drive(requested,requested);return;}
  const float severity=constrain(fabsf(err)/2.0f,0.0f,1.0f);
  int outer=constrain(int(lroundf(requested*(1.0f-0.12f*severity))),0,255);
  float reduction=constrain(0.40f*motors.steeringGain*fabsf(err),0.0f,0.72f);
  int inner=constrain(int(lroundf(outer*(1.0f-reduction))),0,outer);
  if(err<0)motors.drive(inner,outer);else motors.drive(outer,inner);
}

void loop() {
  // Keep the OTA listener alive exactly as in the previously working sketch.
  if(arduinoOTAStarted) ArduinoOTA.handle();
  // Publish Web UI hostname once router STA has an address. ArduinoOTA already
  // uses this mDNS responder; do not stop/restart it while OTA is active.
  if(!otaHttpMdnsPublished && WiFi.status()==WL_CONNECTED) {
    if(MDNS.begin(DEVICE_HOSTNAME)) {
      MDNS.addService("http","tcp",80);
      otaHttpMdnsPublished=true;
      Serial.printf("[MDNS] Web: http://%s.local/ | OTA UDP 3232 | IP %s\n",
        DEVICE_HOSTNAME,WiFi.localIP().toString().c_str());
    } else {
      Serial.println("[MDNS] Hostname advertising failed; use router IP for browser");
    }
  }
  if (arduinoOTAUpdating) { motors.stop(); delay(1); return; }
  // Fresh sensor readings precede any web/BLE motion requests.
  updatePhysicalSpeedPot();
  sensors.updateIR();
  sensors.updateSonar();
  sensors.updateColor();
  checkAllIRBlackSafety();
  // Diagnostic scans work in stopped states, independent of station marker.
  // Normal authorization still only polls in relevant states.
  if ((rfid.isDiagnosticActive() || state == IDLE || state == FINISHED || state == STATION_WAIT ||
       state == FAULT_STOP || state == RECOVERY_SEARCH) &&
      (uint32_t)(millis() - lastRFIDPollAt) >= 25) {
    lastRFIDPollAt = millis();
    rfid.update();
  }

  // Global hard software interlock: stops MANUAL as well as FOLLOW.
  if (!sensors.isPathClear() && (state == FOLLOW || state == RECOVERY_SEARCH || state == MANUAL)) {
    enterObstacleStop(state);
  }
  if (!sensors.isPathClear()) motors.stop();

  comm.update();
  console.update();
  handleCardAuthorization();

  switch (state) {
    case FOLLOW: handleAutonomousFollow(); break;
    case STATION_WAIT: handleStationTimeout(); break;
    case MANUAL: handleManualWatchdog(); break;
    case OBSTACLE_STOP: handleObstacleHold(); break;
    case RECOVERY_SEARCH: handleRecoverySearch(); break;
    case LOAD_TEST_RUN: handleLoadTestRun(); break;
    case SERVICE_TEST: break; // Serial service console owns guarded pulse and timeout
    case IDLE:
    case FAULT_STOP:
    case FINISHED:
    default: motors.stop(); break;
  }

  // Check again after communication callbacks and driving commands.
  if (!sensors.isPathClear() || allIRBlackSafetyLatched) motors.stop();
  diagnostics.update(state, sensors, rfid, motors, PIN_SPEED_POT);
  delay(1);
}

void handleAutonomousFollow() {
  if (!sensors.isPathClear()) { enterObstacleStop(FOLLOW); return; }
  handleStationDetection();
  if (state != FOLLOW) return;

  if (!sensors.hasLine()) {
    if(motors.maxSpeed==0) { motors.stop(); lineLostAt=0; return; }
    if (!lineLostAt) {
      configureLineLossRecoverySpeed();
      lineLostAt = millis();
      logEvent("Line lost: straight bridge started");
    }
    uint32_t elapsed = millis() - lineLostAt;
    if (elapsed < dynamicBridgeMs) {
      // Pot sets both speed and approximate bridge travel distance.
      const int bridgePWM = min(motors.maxSpeed, bridgePWMAtLineLoss);
      motors.drive(bridgePWM, bridgePWM);
    } else {
      state = RECOVERY_SEARCH;
      logEvent("Bridge expired: begin bounded line search");
      handleRecoverySearch();
    }
    return;
  }
  if (lineLostAt) logEvent("Guide line reacquired");
  lineLostAt = 0;
  const uint8_t mask = sensors.getIRBits();
  const bool centerPriority = hasCenterWithOuterBranch(mask);
  // S3 and an outer junction branch active: continue along center.
  const float err = centerPriority ? 0.0f : sensors.getLineError();
  lastKnownLineError = err;
  const int requested = motors.maxSpeed;

  // Fast forward-only PD steering for the installed DIGITAL 5-channel array.
  // IR2/IR4 (|error|=1) now produce substantial correction, BEFORE IR1/IR5.
  // Differentiate at ~25ms to avoid amplifying 1ms digital threshold noise.
  static float previousError = 0.0f;
  static float filteredRate = 0.0f;
  static uint32_t previousSampleMs = 0;
  const uint32_t now = millis();
  if (previousSampleMs == 0 || uint32_t(now - previousSampleMs) >= 25) {
    if (previousSampleMs != 0 && uint32_t(now - previousSampleMs) <= 250) {
      const float dt = float(now - previousSampleMs) * 0.001f;
      const float rate = constrain((err - previousError) / dt, -50.0f, 50.0f);
      filteredRate = 0.65f * filteredRate + 0.35f * rate;
    } else filteredRate = 0.0f;
    previousError = err;
    previousSampleMs = now;
  }

  // A centered line is full-speed; wide/symmetric markers are treated more
  // cautiously to avoid racing across intersections or dark floor patches.
  if (fabsf(err) < 0.16f) {
    const int pwm = (mask == 0x04u || centerPriority) ? requested : min(requested, 170);
    motors.drive(pwm, pwm);
    return;
  }

  // Keep the OUTER wheel near the requested PWM through curves. Most turning
  // comes from slowing the INSIDE wheel, not slowing BOTH motors.
  // Coefficients are provisional bench-tuning defaults, NOT proven values.
  const float gainFactor = constrain(motors.steeringGain / 1.20f, 0.60f, 1.50f);
  // Scale PWM correction in proportion to the physical POT speed ceiling.
  // This preserves a similar left/right PWM ratio through slow/fast bends.
  const float speedScale = float(requested)/255.0f;
  const float kp = 135.0f * gainFactor * speedScale;
  const float kd = 0.30f * gainFactor * speedScale;
  const int outer = requested;
  const int correction = constrain(int(lroundf(kp * fabsf(err) +
      kd * (err < 0 ? -filteredRate : filteredRate))),
      0, int(lroundf(float(outer) * 0.78f)));
  const int inner = max(0, outer - correction);
  if (err < 0) motors.drive(inner, outer); // line to LEFT: slow left wheel
  else         motors.drive(outer, inner); // line to RIGHT: slow right wheel
}

void handleRecoverySearch() {
  if (!sensors.isPathClear()) { enterObstacleStop(RECOVERY_SEARCH); return; }
  if (sensors.hasLine()) {
    state = FOLLOW;
    lineLostAt = 0;
    logEvent("Line found during recovery");
    handleAutonomousFollow();
    return;
  }
  if (motors.maxSpeed==0) { motors.stop(); lineLostAt=0; return; }
  if (!lineLostAt) { lineLostAt = millis(); configureLineLossRecoverySpeed(); }
  uint32_t elapsed = millis() - lineLostAt;
  if (elapsed >= dynamicSearchEndMs) {
    motors.stop();
    state = FAULT_STOP;
    logEvent("Line recovery timed out: Master RFID or Web START needed");
    return;
  }
  // Search slowly in the last known correction direction; stay within the time bound.
  const int p = min(motors.maxSpeed, max(1,motors.maxSpeed / 2));
  const int inner = p / 3;
  if (lastKnownLineError < 0) motors.drive(inner, p);
  else if (lastKnownLineError > 0) motors.drive(p, inner);
  else motors.drive(p, p);
}

void handleManualWatchdog() {
  if (!sensors.isPathClear()) { enterObstacleStop(MANUAL); return; }
  if (millis() - lastManualCommandTime > RC_TIMEOUT_MS) {
    state = IDLE;
    motors.stop();
  }
}

void handleObstacleHold() {
  motors.stop();
  // Never restart after obstacle clearance if all five IR channels still
  // report BLACK. No automatic departure from an unconfirmed guide line.
  if (!sensors.isPathClear() || isAllIRBlack()) return;
  // Paused obstacle time does not consume the 2s bridge / 5s search budget.
  if (lineLostAt) lineLostAt += (uint32_t)(millis() - obstacleHoldStartedAt);
  if (resumeAfterObstacle == MANUAL) {
    // The joystick must still be actively held; never replay an old command.
    state = IDLE;
    logEvent("Obstacle clear: manual control ready for new joystick command");
  } else {
    state = resumeAfterObstacle;
    logEvent("Obstacle clear: automatic navigation resumed");
  }
}