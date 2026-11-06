#pragma once
#include "AGV_Config.h"
#include <Preferences.h>

static portMUX_TYPE sensorMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool echoArmed = false, echoHigh = false, echoDone = false;
static volatile uint32_t echoRise = 0, echoWidth = 0;
static volatile bool colorCounting = false;
static volatile uint32_t colorEdges = 0, colorFirst = 0, colorLast = 0;
static uint8_t globalEchoPin = 2; // Matched to GPIO 2

void IRAM_ATTR echoISR() {
  uint32_t now = micros();
  bool high = digitalRead(globalEchoPin) == HIGH; // Reads ECHO on GPIO 2
  portENTER_CRITICAL_ISR(&sensorMux);
  if (echoArmed) {
    if (high) {
      echoRise = now;
      echoHigh = true;
    } else if (echoHigh) {
      echoWidth = now - echoRise;
      echoDone = true;
      echoArmed = false;
      echoHigh = false;
    }
  }
  portEXIT_CRITICAL_ISR(&sensorMux);
}

void IRAM_ATTR colorISR() {
  uint32_t now = micros();
  portENTER_CRITICAL_ISR(&sensorMux);
  if (colorCounting) {
    if (colorEdges == 0) colorFirst = now;
    colorLast = now;
    colorEdges++;
  }
  portEXIT_CRITICAL_ISR(&sensorMux);
}

class AGVSensors {
private:
  uint8_t irPins[5];
  uint8_t usTrig, usEcho;
  uint8_t cs0, cs1, cs2, cs3, cout;
  int safetyCm = 20;

  Reading calPresets[2][4];
  Reading factoryPresets[2][4];
  Reading floorRefs[2] = {{NAN,NAN,NAN,NAN},{NAN,NAN,NAN,NAN}};
  bool floorValid[2]={false,false};
  bool calibrated[2][4] = {};
  // IR inversion can be changed from the web after checking actual sensor polarity.
  bool irBlackLow = true;
  Preferences irPrefs;
  uint8_t irRawBits = 0;
  uint32_t colorCycleId = 0, lastStationCountCycle = 0;
  float tolerances[2] = {0.050f, 0.045f};
  // mode: 0=indoor, 1=outdoor, 2=automatic (best passing profile).
  uint8_t colorMode = 2;
  int matchedProfile = -1;
  float colorMatchDistance = NAN;
  bool colorFresh = false;
  uint32_t lastColorCycleMs = 0;
  Preferences prefs;
  bool prefsOpen = false;
  struct CaptureJob {
    bool active = false;
    uint8_t profile = 0, color = 0;
    uint16_t count = 0;
    Reading sum = {0,0,0,0};
    uint32_t begun = 0;
  } capture;
  String captureMessage = "Idle";
  static bool validRGB(const Reading& x) {
    return isfinite(x.r) && isfinite(x.g) && isfinite(x.b) && x.r>0 && x.g>0 && x.b>0;
  }
  static bool validReading(const Reading& x) {
    return isfinite(x.r) && isfinite(x.g) && isfinite(x.b) && isfinite(x.c) &&
      x.r>0 && x.g>0 && x.b>0 && x.c>0;
  }
  static float chromaDist(const Reading& a,const Reading& b) {
    float aa=a.r+a.g+a.b, bb=b.r+b.g+b.b;
    if(!validRGB(a)||!validRGB(b)||aa<=0||bb<=0) return 999.0f;
    float dr=a.r/aa-b.r/bb, dg=a.g/aa-b.g/bb, db=a.b/aa-b.b/bb;
    return sqrtf(dr*dr+dg*dg+db*db);
  }
  void saveCalibration() {
    if(!prefsOpen) return;
    prefs.putBytes("cal",calPresets,sizeof(calPresets));
    prefs.putBytes("valid",calibrated,sizeof(calibrated));
    prefs.putBytes("tol",tolerances,sizeof(tolerances));
    prefs.putUChar("mode",colorMode);
    prefs.putBytes("floor",floorRefs,sizeof(floorRefs));
    prefs.putBytes("floorV",floorValid,sizeof(floorValid));
  }
  void classifyColor() {
    matchedProfile = -1; colorMatchDistance = NAN;
    if (!validRGB(colorSample)) {currentStationIdx=-1; return;}
    // Reject a calibrated ordinary floor before testing any station color.
    for(int p=0;p<2;p++)if(floorValid[p] && (colorMode==2 || colorMode==p)) {
      if(chromaDist(colorSample,floorRefs[p])<0.030f){currentStationIdx=-1;return;}
    }
    int winner=-1; float best=999.0f, nextOtherColor=999.0f;
    for(int profile=0;profile<2;profile++) {
      if(colorMode!=2 && profile!=colorMode) continue;
      for(int i=0;i<4;i++) {
        if(!calibrated[profile][i]) continue;
        const Reading& ref=calPresets[profile][i];
        // Reject weak/dark samples before chromaticity matching.
        if(isfinite(colorSample.c) && isfinite(ref.c) && ref.c>0 &&
           colorSample.c < ref.c*0.12f) continue;
        float d=chromaDist(colorSample,ref);
        if(d>=tolerances[profile]) continue;
        if(d<best) {
          if(winner>=0 && i!=winner) nextOtherColor=fminf(nextOtherColor,best);
          best=d;winner=i;matchedProfile=profile;
        } else if(i!=winner) nextOtherColor=fminf(nextOtherColor,d);
      }
    }
    colorMatchDistance=(best<999)?best:NAN;
    // Similar distances to different color classes are ambiguous, not station markers.
    // A second color nearly as plausible is ambiguous, especially outdoor
    // RED versus YELLOW. Reject rather than stop on a false station marker.
    if(winner>=0 && nextOtherColor-best < 0.020f) winner=-1;
    currentStationIdx=winner;
  }
  void processCapture() {
    if(!capture.active || !validReading(colorSample)) return;
    // Skip initial sensor settling and ignore invalid / non-finite samples.
    if(millis()-capture.begun < 150) return;
    capture.sum.r+=colorSample.r; capture.sum.g+=colorSample.g;
    capture.sum.b+=colorSample.b; capture.sum.c+=colorSample.c;
    capture.count++;
    if(capture.count<24) return; // about one second of complete RGB/Clear cycles
    float n=float(capture.count);
    Reading sample={capture.sum.r/n,capture.sum.g/n,capture.sum.b/n,capture.sum.c/n};
    if(capture.color==4){
      floorRefs[capture.profile]=sample;floorValid[capture.profile]=true;
    } else {
      calPresets[capture.profile][capture.color]=sample;
      calibrated[capture.profile][capture.color]=true;
    }
    saveCalibration();
    capture.active=false;
    captureMessage=String("Saved ")+(capture.profile==0?"INDOOR ":"OUTDOOR ")+
      String(capture.color==4?"FLOOR":capture.color==0?"GREEN":capture.color==1?"RED":capture.color==2?"BLUE":"YELLOW")+
      String(" using ")+String(capture.count)+" samples";
    Serial.println("[COLOR CAL] "+captureMessage);
    classifyColor();
  }
  uint8_t colorPhase = 0;
  Reading colorSample = {NAN, NAN, NAN, NAN};
  uint32_t filterSinceUs = 0;
  bool colorGate = false;
  int currentStationIdx = -1;
  int stableHits = 0;
  int previousStationCandidate = -1;
  uint32_t candidateSinceMs = 0;
  uint8_t awayFrames = 0;
  int lastConfirmedColor = -1;
  bool sawAway = true;

  uint8_t irBlackBits = 0;
  float lineError = 0.0f;
  int distanceCm = -1;
  bool pathClear = true;
  uint32_t lastUS = 0;
  bool sonarPending = false;
  uint32_t sonarTriggerUs = 0;

public:
  void beginIR(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4, uint8_t s5) {
    irPins[0] = s1; irPins[1] = s2; irPins[2] = s3; irPins[3] = s4; irPins[4] = s5;
    for (int i = 0; i < 5; i++) pinMode(irPins[i], INPUT);
    if(irPrefs.begin("agv-line",false)) irBlackLow=irPrefs.getBool("blackLow",true);
  }

  void beginSonar(uint8_t trig, uint8_t echo, int safetyDist) {
    usTrig = trig;
    usEcho = echo;
    globalEchoPin = echo;
    safetyCm = safetyDist;

    pinMode(usTrig, OUTPUT);
    digitalWrite(usTrig, LOW);
    pinMode(usEcho, INPUT);
    attachInterrupt(digitalPinToInterrupt(usEcho), echoISR, CHANGE);
  }

  void beginColor(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3, uint8_t out) {
    cs0 = s0; cs1 = s1; cs2 = s2; cs3 = s3; cout = out;
    pinMode(cs0, OUTPUT); pinMode(cs1, OUTPUT);
    pinMode(cs2, OUTPUT); pinMode(cs3, OUTPUT);
    pinMode(cout, INPUT);
    digitalWrite(cs0, HIGH); digitalWrite(cs1, LOW); // 20% frequency scaling
    digitalWrite(cs2, LOW); digitalWrite(cs3, LOW); // phase 0 must start with RED filter
    filterSinceUs=micros();
    attachInterrupt(digitalPinToInterrupt(cout), colorISR, RISING);
  }

  void loadColorPresets(const Reading (&indoor)[4], const Reading (&outdoor)[4]) {
    for(int i=0;i<4;i++) {
      factoryPresets[0][i]=indoor[i];
      factoryPresets[1][i]=outdoor[i];
    }
    // These are defaults ONLY. NVS-saved calibration takes precedence on boot.
    for(int p=0;p<2;p++) for(int i=0;i<4;i++) {
      calPresets[p][i]=factoryPresets[p][i]; calibrated[p][i]=true;
    }
    prefsOpen=prefs.begin("agv-colors",false);
    if(!prefsOpen) { Serial.println("[COLOR CAL] NVS unavailable; defaults only"); return; }
    // Only load a complete saved set; older layouts are not assumed compatible.
    if(prefs.getBytesLength("cal")==sizeof(calPresets) &&
       prefs.getBytesLength("valid")==sizeof(calibrated) &&
       prefs.getBytesLength("tol")==sizeof(tolerances)) {
      prefs.getBytes("cal",calPresets,sizeof(calPresets));
      prefs.getBytes("valid",calibrated,sizeof(calibrated));
      prefs.getBytes("tol",tolerances,sizeof(tolerances));
      for(int p=0;p<2;p++) {
        if(!isfinite(tolerances[p])||tolerances[p]<0.025f||tolerances[p]>0.30f)
          tolerances[p]=(p==0?0.085f:0.115f);
        for(int i=0;i<4;i++) if(calibrated[p][i]&&!validReading(calPresets[p][i])) calibrated[p][i]=false;
      }
      colorMode=constrain(int(prefs.getUChar("mode",2)),0,2);
    }
    if(prefs.getBytesLength("floor")==sizeof(floorRefs) &&
       prefs.getBytesLength("floorV")==sizeof(floorValid)){
      prefs.getBytes("floor",floorRefs,sizeof(floorRefs));
      prefs.getBytes("floorV",floorValid,sizeof(floorValid));
      for(int p=0;p<2;p++)if(floorValid[p]&&!validReading(floorRefs[p]))floorValid[p]=false;
    }
  }
  bool applyMeasuredColorReferences() {
    // Explicit operator action, when stationary. Preserve floor references.
    if(!prefsOpen || capture.active) return false;
    for(int p=0;p<2;p++) for(int i=0;i<4;i++) {
      if(!validReading(factoryPresets[p][i])) return false;
    }
    for(int p=0;p<2;p++) for(int i=0;i<4;i++) {
      calPresets[p][i]=factoryPresets[p][i];
      calibrated[p][i]=true;
    }
    tolerances[0]=0.050f;
    tolerances[1]=0.045f;
    colorMode=2; // Evaluate both profiles; do not average their raw frequencies.
    saveCalibration();
    clearStationLatch();
    classifyColor();
    captureMessage="Measured indoor/outdoor references saved to NVS";
    return true;
  }
  bool startCapture(int profile,int color) {
    if(profile<0||profile>1||color<0||color>4||capture.active) return false;
    capture=CaptureJob{}; capture.active=true;capture.profile=profile;
    capture.color=color;capture.begun=millis();
    captureMessage="Sampling... keep sensor stationary over marker";
    return true;
  }
  bool captureBusy() const {return capture.active;}
  bool floorCalibrated(int p) const {return p>=0&&p<2&&floorValid[p];}
  String captureStatus() const {return captureMessage;}
  bool setColorSettings(int mode,float indoorTolerance,float outdoorTolerance) {
    if(mode<0||mode>2||!isfinite(indoorTolerance)||!isfinite(outdoorTolerance)||
       indoorTolerance<0.025f||indoorTolerance>0.30f||
       outdoorTolerance<0.025f||outdoorTolerance>0.30f) return false;
    colorMode=mode;tolerances[0]=indoorTolerance;tolerances[1]=outdoorTolerance;
    saveCalibration(); classifyColor(); return true;
  }
  void resetColorCalibration() {
    if(prefsOpen) {prefs.remove("cal");prefs.remove("valid");prefs.remove("tol");prefs.remove("mode");prefs.remove("floor");prefs.remove("floorV");}
    // Do not delete unrelated NVS settings.
    capture.active=false; floorValid[0]=floorValid[1]=false;
    for(int p=0;p<2;p++) for(int i=0;i<4;i++) {
      calPresets[p][i]=factoryPresets[p][i];
      calibrated[p][i]=true;
    }
    tolerances[0]=0.050f;tolerances[1]=0.045f;colorMode=2;
    captureMessage="Saved calibration cleared; indoor defaults restored";
    classifyColor();
  }
  int getColorMode() const {return colorMode;}
  int getMatchedProfile() const {return matchedProfile;}
  float getColorDistance() const {return colorMatchDistance;}
  float getColorTolerance(int profile) const {return tolerances[profile==1?1:0];}
  bool isColorCalibrated(int profile,int color) const {return profile>=0&&profile<2&&color>=0&&color<4&&calibrated[profile][color];}
  Reading getColorPreset(int profile,int color) const {return calPresets[profile==1?1:0][constrain(color,0,3)];}
  uint32_t getColorSampleAgeMs() const {return millis()-lastColorCycleMs;}
  bool hasFreshColor() const {return colorFresh && getColorSampleAgeMs()<500;}


  void setSafetyDistance(int cm) { safetyCm = cm; }
  int getSafetyDistance() { return safetyCm; }

  void updateIR() {
    irBlackBits = 0; irRawBits = 0;
    int weightedSum = 0, count = 0;
    const int weights[5] = {-2, -1, 0, 1, 2};
    for (int i = 0; i < 5; i++) {
      bool hi = digitalRead(irPins[i]) == HIGH;
      if(hi) irRawBits |= (1 << i);
      bool active = irBlackLow ? !hi : hi;
      if (active) {
        irBlackBits |= (1 << i);
        weightedSum += weights[i];
        count++;
      }
    }
    lineError = (count > 0) ? (float)weightedSum / (float)count : 0.0f;
  }

  void updateSonar() {
    uint32_t nowUs = micros();
    if (sonarPending) {
      bool done; uint32_t width;
      portENTER_CRITICAL(&sensorMux);
      done = echoDone; width = echoWidth;
      if (done || uint32_t(nowUs - sonarTriggerUs) >= 25000) {
        echoArmed = false; echoDone = false; echoHigh = false;
      }
      portEXIT_CRITICAL(&sensorMux);

      if (done) {
        sonarPending = false;
        int d = (width >= 116 && width <= 23200) ? int(width / 58) : -1;
        if (d < 2 || d > 400) {
          distanceCm = -1;
          pathClear = true; // Open space evaluates as clear
        } else {
          distanceCm = d;
          pathClear = (d > safetyCm);
        }
      } else if (uint32_t(nowUs - sonarTriggerUs) >= 25000) {
        sonarPending = false;
        distanceCm = -1;
        pathClear = true;
      }
    }

    if (!sonarPending && uint32_t(millis() - lastUS) >= 85) {
      lastUS = millis();
      portENTER_CRITICAL(&sensorMux);
      echoDone = false; echoHigh = false; echoArmed = true;
      portEXIT_CRITICAL(&sensorMux);
      sonarPending = true;
      sonarTriggerUs = micros();
      digitalWrite(usTrig, HIGH);
      delayMicroseconds(10);
      digitalWrite(usTrig, LOW);
    }
  }

  void updateColor() {
    uint32_t now = micros();
    if (!colorGate) {
      if (uint32_t(now - filterSinceUs) < 1000) return;
      portENTER_CRITICAL(&sensorMux);
      colorEdges = 0; colorFirst = 0; colorLast = 0; colorCounting = true;
      portEXIT_CRITICAL(&sensorMux);
      colorGate = true; filterSinceUs = now; return;
    }
    if (uint32_t(now - filterSinceUs) < 9000) return;

    uint32_t n, first, last;
    portENTER_CRITICAL(&sensorMux);
    colorCounting = false; n = colorEdges; first = colorFirst; last = colorLast;
    portEXIT_CRITICAL(&sensorMux);
    float hz = (n >= 3 && uint32_t(last - first) > 0) ? 1000000.0f * float(n - 1) / float(uint32_t(last - first)) : NAN;

    switch (colorPhase) {
      case 0: colorSample.r = hz; break;
      case 1: colorSample.b = hz; break;
      case 2: colorSample.c = hz; break;
      default: colorSample.g = hz; break;
    }
    colorPhase = (colorPhase + 1) % 4;
    digitalWrite(cs2, colorPhase >= 2 ? HIGH : LOW);
    digitalWrite(cs3, (colorPhase == 1 || colorPhase == 3) ? HIGH : LOW);
    filterSinceUs = micros(); colorGate = false;
    if (colorPhase != 0) return;

    colorFresh=validRGB(colorSample);
    lastColorCycleMs=millis();
    ++colorCycleId;
    if(capture.active) processCapture();
    classifyColor();
  }

  bool isPathClear() const { return pathClear; }
  bool hasLine() const { return irBlackBits != 0; }
  float getLineError() { return lineError; }
  uint8_t getIRBits() { return irBlackBits; }
  uint8_t getRawIRBits() const { return irRawBits; }
  bool getIRBlackLow() const { return irBlackLow; }
  void setIRBlackLow(bool v) {irBlackLow=v; irPrefs.putBool("blackLow",v); updateIR();}
  int getDistanceCm() { return distanceCm; }
  Reading getColorReading() const { return colorSample; }
  int getCurrentStationIndex() { return currentStationIdx; }
  uint32_t getColorCycleId() const { return colorCycleId; }

  const char* getCurrentStationName() {
    switch (currentStationIdx) {
      case 0: return "Green Start/Lap";
      case 1: return "Red Station";
      case 2: return "Blue Station";
      case 3: return "Yellow Station";
      default: return "Track Floor";
    }
  }

  bool detectsStationMarker() {
    // Only complete color frames can increase confidence. A changed candidate
    // must restart the 3-frame counter rather than inheriting a prior color.
    if (lastStationCountCycle == colorCycleId) return false;
    lastStationCountCycle = colorCycleId;
    const uint32_t now = millis();
    if (currentStationIdx != previousStationCandidate) {
      previousStationCandidate = currentStationIdx;
      stableHits = 0;
      candidateSinceMs = now;
    }
    if (currentStationIdx < 0) {
      if (awayFrames < 3) ++awayFrames;
      if (awayFrames >= 2) { sawAway = true; lastConfirmedColor = -1; }
      return false;
    }
    awayFrames = 0;
    if (!sawAway || !colorFresh) return false;
    // GREEN returns at speed: two strong full RGBC frames can confirm it.
    // All other colors require three consistent frames (approx. 120 ms).
    // Ambiguous, distant GREEN-like floor readings are not confirmed.
    const bool strongGreen = currentStationIdx == 0 &&
      isfinite(colorMatchDistance) && colorMatchDistance < 0.040f;
    const uint8_t required = strongGreen ? 2 : 3;
    if (++stableHits < required || (now-candidateSinceMs) < 35) return false;
    sawAway = false; stableHits = 0;
    lastConfirmedColor = currentStationIdx;
    return true;
  }
  int getLastConfirmedColor() const { return lastConfirmedColor; }
  uint8_t getStationCandidateHits() const { return uint8_t(stableHits); }
  void clearStationLatch() {
    sawAway = false; stableHits = 0; awayFrames = 0;
    previousStationCandidate = -1; candidateSinceMs = 0;
    lastConfirmedColor = -1;
  }
};