#pragma once
#include <Arduino.h>
#include "AGV_Config.h"
#include "AGV_RFID.h"
#include "AGV_Sensors.h"
#include "AGV_Motors.h"
#include "AGV_StationConfig.h"
#include "AGV_LoadTest.h"

extern AGVRFID rfid;
extern AGVSensors sensors;
extern AGVMotors motors;
extern AGVStationConfig stations;
extern AGVLoadTest loadTest;
extern RunState state;
extern bool requestAutonomousStart(bool);
extern void requestSafeStop();
extern uint8_t nextStationSlot;
extern bool isAllIRBlackSafetyLatched();
extern uint32_t getAllIRBlackElapsedMs();

class AGVSerialConsole {
  String buffer;
  uint32_t motorDeadline=0, armedUntil=0;
  bool motorTest=false;
  bool monitoring=false;
  uint32_t lastMonitor=0;
  static String nextWord(String &s){
    s.trim();int n=s.indexOf(' ');
    if(n<0){String a=s;s="";return a;}
    String a=s.substring(0,n);s=s.substring(n+1);s.trim();return a;
  }
  bool stationary(){return state==IDLE||state==FAULT_STOP||state==FINISHED||state==STATION_WAIT;}
  void printStatus(){
    Reading c=sensors.getColorReading();
    Serial.printf("[SERVICE] state=%s | IR=%02X raw=%02X error=%.2f | SONAR=%d clear=%d | RGBC=%.0f/%.0f/%.0f/%.0f | color=%d | lastUID=%s | PWM=%d/%d\n",
      getStateName(state),sensors.getIRBits(),sensors.getRawIRBits(),sensors.getLineError(),
      sensors.getDistanceCm(),sensors.isPathClear(),c.r,c.g,c.b,c.c,sensors.getCurrentStationIndex(),
      rfid.getLastSeenUID().c_str(),motors.getAppliedL(),motors.getAppliedR());
    Serial.printf("[SAFETY] allIRBlack=%s elapsed=%lu/%lu ms | latched=%s (1=black; 11111 triggers stop)\n",
      sensors.getIRBits()==ALL_IR_BLACK_MASK?"YES":"NO",
      (unsigned long)getAllIRBlackElapsedMs(),(unsigned long)ALL_IR_BLACK_STOP_MS,
      isAllIRBlackSafetyLatched()?"YES":"NO");
  }
  void execute(String cmd){
    cmd.trim();cmd.toUpperCase();
    if(cmd.isEmpty())return;
    if(cmd=="HELP" || cmd=="?"){
      Serial.println("[SERVICE] HELP STATUS PINS IR COLOR SONAR POT RFID TEST RFID STATUS RFID REINIT RFID LIST");
      Serial.println("[SERVICE] IR POL LOW|HIGH | CAL IN|OUT GREEN|RED|BLUE|YELLOW|FLOOR");
      Serial.println("[SERVICE] STATION LIST | STATION <0..3> <GREEN|RED|BLUE|YELLOW|OFF> <SECONDS>");
      Serial.println("[SERVICE] AUTO ON|OFF | KEY <UID> <MASK 0..15> | MASTER <NEW UID> (scan old master first)");
      Serial.println("[SERVICE] MOTOR ARM | FORWARD <PWM> | LEFT <PWM> | RIGHT <PWM> | MOTOR L|R <PWM> | STOP");
      Serial.println("[SERVICE] RGB TEST | WIFI STATUS | WIFI DEFAULTS | WIFI AP DEFAULTS | WIFI RETRY | BLE STATUS | LOG ON|OFF");return;
    }
    if(cmd=="RFID TEST"){
      if(!stationary()){Serial.println("[RFID] Stop AGV before diagnostic scan");return;}
      motors.stop();rfid.startDiagnostic(30000);rfid.printDiagnostic();return;
    }
    if(cmd=="RFID STATUS"){rfid.printDiagnostic();return;}
    if(cmd=="RFID REINIT"){
      if(!stationary()){Serial.println("[RFID] Stop AGV before reader reset");return;}
      motors.stop();rfid.reinitialize();rfid.printDiagnostic();return;
    }
    if(cmd=="STATUS"||cmd=="IR"||cmd=="COLOR"||cmd=="SONAR"||cmd=="POT"){
      printStatus();return;
    }
    if(cmd=="PINS"){
      Serial.println("[PINS] IR 4..8 | TCS 9,10,11,14,15 | RFID 12,13,16,17,18 | SONAR 1,2 | POT3 | LEFT FWD38 | RIGHT FWD39 | ENABLE41 | RGB48");return;
    }
    if(cmd=="RFID LIST") {Serial.println(rfid.listJSON());return;}
    if(cmd=="STATION LIST") {Serial.println(stations.json());return;}
    if(cmd=="STOP") {motorTest=false;armedUntil=0;requestSafeStop();Serial.println("[SERVICE] STOPPED");return;}
    if(cmd=="LOG ON"){monitoring=true;Serial.println("[SERVICE] Monitor enabled");return;}
    if(cmd=="LOG OFF"){monitoring=false;Serial.println("[SERVICE] Monitor disabled");return;}
    if(cmd=="WIFI STATUS"){
      Serial.printf("[WIFI] mode=%d | AP SSID=%s IP=%s stations=%d | STA=%s status=%d IP=%s RSSI=%d | hostname=%s.local\n",
        int(WiFi.getMode()),WiFi.softAPSSID().c_str(),WiFi.softAPIP().toString().c_str(),
        WiFi.softAPgetStationNum(),WiFi.SSID().c_str(),int(WiFi.status()),
        WiFi.localIP().toString().c_str(),WiFi.RSSI(),DEVICE_HOSTNAME);
      Serial.println("[WIFI] If previous credentials are incorrect, use WIFI DEFAULTS (stopped only).");return;
    }
    if(cmd=="WIFI DEFAULTS") {
      if(!stationary()) {Serial.println("[WIFI] Stop AGV before changing credentials");return;}
      Preferences p;if(!p.begin("agv-network",false)) {Serial.println("[WIFI] Cannot open NVS");return;}
      bool hadSsid=p.isKey("ssid"),hadPass=p.isKey("pass");
      bool okSsid=!hadSsid||p.remove("ssid");
      bool okPass=!hadPass||p.remove("pass");
      bool ok=okSsid&&okPass;p.end();
      Serial.printf("[WIFI] Router overrides cleared: %s. REBOOT ESP32 to use compiled default router credentials.\n",ok?"YES":"PARTIAL/NOT PRESENT");return;
    }
    if(cmd=="WIFI AP DEFAULTS") {
      if(!stationary()){Serial.println("[WIFI] Stop the AGV first");return;}
      Preferences p;
      if(!p.begin("agv-network",false)){Serial.println("[WIFI] NVS inaccessible");return;}
      bool a=!p.isKey("apSsid")||p.remove("apSsid");
      bool b=!p.isKey("apPass")||p.remove("apPass");
      p.end();
      Serial.printf("[WIFI] Restore factory AP SSID/pass: %s. Reboot required.\n",a&&b?"OK":"FAILED");return;
    }
    if(cmd=="WIFI RETRY") {
      if(!stationary()) {Serial.println("[WIFI] Stop AGV before manual reconnect");return;}
      WiFi.reconnect();Serial.println("[WIFI] Reconnect requested. AP remains enabled.");return;
    }
    if(cmd=="BLE STATUS") {Serial.println("[BLE] BLE module initialized; use Web UI for connection status");return;}
    if(cmd=="RGB TEST"){
      if(!stationary()) {Serial.println("[SERVICE] Stop first");return;}
      neopixelWrite(48,48,0,0);delay(160);neopixelWrite(48,0,48,0);delay(160);
      neopixelWrite(48,0,0,48);delay(160);neopixelWrite(48,0,0,0);
      Serial.println("[SERVICE] RGB test complete");return;
    }
    if(cmd.startsWith("IR POL ")){
      if(!stationary()){Serial.println("[SERVICE] Stop first");return;}
      if(cmd.endsWith(" LOW")){sensors.setIRBlackLow(true);Serial.println("[IR] LOW=BLACK saved");}
      else if(cmd.endsWith(" HIGH")){sensors.setIRBlackLow(false);Serial.println("[IR] HIGH=BLACK saved");}
      return;
    }
    if(cmd.startsWith("CAL ")){
      if(!stationary()){Serial.println("[CAL] Stop first");return;}
      String v=cmd.substring(4),profile=nextWord(v),color=nextWord(v);
      int p=(profile=="IN"?0:profile=="OUT"?1:-1);
      int c=(color=="GREEN"?0:color=="RED"?1:color=="BLUE"?2:color=="YELLOW"?3:color=="FLOOR"?4:-1);
      if(p<0||c<0||!sensors.startCapture(p,c))Serial.println("[CAL] Invalid/busy");
      else Serial.println("[CAL] Capture started; keep sensor stationary until saved");return;
    }
    if(cmd.startsWith("AUTO ")){
      if(!stationary()){Serial.println("[STATION] Stop first");return;}
      String v=cmd.substring(5);
      if(v=="ON"||v=="OFF")Serial.println(stations.setAuto(v=="ON")?"[STATION] Saved":"[STATION] NVS error");
      return;
    }
    if(cmd.startsWith("STATION ")){
      if(!stationary()){Serial.println("[STATION] Stop first");return;}
      String v=cmd.substring(8);int slot=nextWord(v).toInt();String color=nextWord(v);long sec=nextWord(v).toInt();
      int c=(color=="GREEN"?0:color=="RED"?1:color=="BLUE"?2:color=="YELLOW"?3:color=="OFF"?-1:-99);
      if(c==-99 || sec<0 || sec>3600 || slot<0 || slot>3){Serial.println("[STATION] Invalid config");return;}
      Serial.println(stations.saveSlot(slot,c,uint32_t(sec))?"[STATION] Saved":"[STATION] Save failed");return;
    }
    if(cmd.startsWith("KEY ")){
      if(!stationary()){Serial.println("[RFID] Stop first");return;}
      int cut=cmd.lastIndexOf(' ');if(cut<5)return;
      String uid=cmd.substring(4,cut),maskStr=cmd.substring(cut+1);
      int mask=maskStr.toInt();
      if(mask<0||mask>15){Serial.println("[RFID] Invalid mask: enter 0..15");return;}
      auto result=rfid.saveKeyMask(uid,uint8_t(mask));
      Serial.printf("[RFID] %s\n",AGVRFID::keySaveMessage(result));return;
    }
    if(cmd.startsWith("MASTER ")){
      if(!stationary()){Serial.println("[RFID] Stop first");return;}
      if(!rfid.isMaster(rfid.getLastSeenUID()) || millis()-rfid.getLastSeenMs()>30000){Serial.println("[RFID] Scan present Master first");return;}
      Serial.println(rfid.setMasterUID(cmd.substring(7))?"[RFID] New Master saved":"[RFID] Save failed");return;
    }
    if(cmd=="MOTOR ARM"){
      if(state!=IDLE||rfid.isDiagnosticActive()||!sensors.isPathClear()||!sensors.hasLine()||sensors.getIRBits()==ALL_IR_BLACK_MASK){
        Serial.println("[MOTOR] Required: IDLE, path clear, black line. Raise wheels before testing");return;
      }
      armedUntil=millis()+30000;Serial.println("[MOTOR] ARMED for 30s, each drive pulse max 400ms. Raise wheels");return;
    }
    if(cmd.startsWith("FORWARD ")||cmd.startsWith("LEFT ")||cmd.startsWith("RIGHT ")||cmd.startsWith("MOTOR ")){
      if(state!=IDLE||!armedUntil||(int32_t)(millis()-armedUntil)>=0||!sensors.isPathClear()||sensors.getIRBits()==ALL_IR_BLACK_MASK){
        Serial.println("[MOTOR] Not armed or blocked");return;
      }
      String v=cmd,action=nextWord(v),side="";
      if(action=="MOTOR")side=nextWord(v);
      int pwm=v.toInt();if(pwm<1||pwm>100){Serial.println("[MOTOR] PWM must be 1..100");return;}
      int l=pwm,r=pwm;
      if(action=="LEFT"){l=pwm/2;r=pwm;}
      if(action=="RIGHT"){l=pwm;r=pwm/2;}
      if(action=="MOTOR"){l=(side=="L"?pwm:0);r=(side=="R"?pwm:0);}
      state=SERVICE_TEST;motors.drive(l,r);motorTest=true;motorDeadline=millis()+400;
      Serial.println("[MOTOR] 400ms diagnostic pulse started");return;
    }
    Serial.println("[SERVICE] Unknown command; type HELP");
  }
public:
  void begin(){buffer.reserve(100);Serial.println("[SERVICE] Console online. Type HELP");}
  void update(){
    if(motorTest && ((int32_t)(millis()-motorDeadline)>=0 || !sensors.isPathClear())){
      motors.stop();motorTest=false;
      if(state==SERVICE_TEST) state=IDLE; // Never overwrite a latched safety FAULT_STOP.
      Serial.println("[MOTOR] Diagnostic pulse stopped");
    }
    for(int n=0;n<32 && Serial.available();n++){
      char ch=char(Serial.read());
      if(ch=='\n'||ch=='\r'){
        if(buffer.length()){String cmd=buffer;buffer="";execute(cmd);}
      } else if(ch>=32 && ch<=126 && buffer.length()<95)buffer+=ch;
    }
    if(monitoring && millis()-lastMonitor>=1000){lastMonitor=millis();printStatus();}
  }
};