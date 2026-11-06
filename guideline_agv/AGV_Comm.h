#pragma once
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h> // Links2004 WebSockets library
#include <DNSServer.h>
#include <LittleFS.h>
#include <Preferences.h>
#include "AGV_StationConfig.h"
#include "AGV_RFID.h"
#include "AGV_LoadTest.h"
#include "AGV_Config.h"
#include "AGV_Motors.h"
#include "AGV_Sensors.h"
#include "AGV_BLE.h"
#include "AGV_OTA.h"
#include "AGV_WebPage.h" // Embedded fallback if LittleFS is missing

extern AGVMotors motors;
extern AGVSensors sensors;
extern AGVRFID rfid;
extern AGVStationConfig stations;
extern AGVLoadTest loadTest;
extern int latchedStation;
extern uint8_t nextStationSlot;
extern uint32_t stationWaitStartMs;
extern RunState state;
extern uint32_t lastManualCommandTime;
extern bool requestAutonomousStart(bool resetFault);
extern void requestSafeStop();
extern bool isMotionBlocked();
extern bool isAllIRBlack();
extern bool isAllIRBlackSafetyLatched();
extern uint32_t getAllIRBlackElapsedMs();
extern uint16_t speedPotRaw;
extern uint16_t speedPotFiltered;
extern int speedPotPwm;
extern int loadTestPwmCeiling;
extern void updatePhysicalSpeedPot(bool force);

class AGVComm {
private:
  WebServer server{80};
  WebSocketsServer wsServer{81};  // Same AP + router interfaces, separate TCP port.
  int wsOwner = -1;              // One active joystick client at a time.
  bool wsArmed = false;
  uint32_t wsLastMotionMs = 0;
  DNSServer dnsServer;
  bool fsMounted = false;
  bool indexReady = false;
  Preferences wifiPrefs;
  bool wifiPrefsOK=false;
  String apName;
  String apKey;
  String routerName;
  String routerKey;
  bool previousRouterConnected = false;
  uint32_t lastRouterRetryMs = 0;
  uint32_t lastAPCheckMs = 0;
  uint32_t lastRouterStatusMs = 0;
  uint8_t routerAttempts = 0;
  bool routerConnectPending = false;
  uint32_t routerConnectStartedMs = 0;
  String routerConnectError;

  bool routerHasAddress() const {
    return WiFi.status() == WL_CONNECTED &&
      WiFi.SSID() == routerName && (uint32_t)WiFi.localIP() != 0;
  }

  void updateRouterJoinResult() {
    if (!routerConnectPending) return;
    if (routerHasAddress()) {
      routerConnectPending = false;
      routerConnectError = "";
      Serial.printf("[WIFI] Router connected successfully | IP=%s | SSID=%s\n",
        WiFi.localIP().toString().c_str(), routerName.c_str());
    } else if (uint32_t(millis()-routerConnectStartedMs) >= 20000) {
      routerConnectPending = false;
      routerConnectError = String("Router not connected within 20 s (") +
        wifiStatusText(WiFi.status()) + "). Check 2.4 GHz SSID/password and router signal.";
      Serial.printf("[WIFI] %s\n", routerConnectError.c_str());
    }
  }

  // Only A (fresh finger press) can arm this connection. WebSocket reconnection
  // by itself must never restart a stopped wheel. S releases immediately.
  void handleWebSocket(uint8_t client, WStype_t type, uint8_t* data, size_t size) {
    if (type == WStype_CONNECTED) {
      Serial.printf("[WS] Client %u connected on port 81; manual control disarmed\n", unsigned(client));
      wsServer.sendTXT(client, "READY");
      return;
    }
    if (type == WStype_DISCONNECTED) {
      if (wsOwner == int(client)) {
        if (state == MANUAL) { motors.stop(); state = IDLE; }
        wsOwner = -1;
        wsArmed = false;
        Serial.println("[WS] Active joystick disconnected; motors stopped");
      }
      return;
    }
    if (type == WStype_TEXT && size == 1) {
      if (data[0] == 'S') {
        if (wsOwner == int(client)) {
          motors.stop();
          if (state == MANUAL) state = IDLE;
          wsOwner = -1;
          wsArmed = false;
          wsServer.sendTXT(client, "STOPPED");
        }
        return;
      }
      if (data[0] == 'A') {
        if (wsOwner >= 0 && wsOwner != int(client)) {
          wsServer.sendTXT(client, "BUSY");
          return;
        }
        if (isMotionBlocked() || (state != IDLE && state != MANUAL)) {
          wsServer.sendTXT(client, "BLOCKED");
          return;
        }
        wsOwner = int(client);
        wsArmed = true;
        wsLastMotionMs = millis();
        wsServer.sendTXT(client, "ARMED");
        return;
      }
    }
    // Binary frame: signed int8 throttle (0..100), signed int8 turn (-100..100).
    if (type != WStype_BIN || size != 2 || wsOwner != int(client) || !wsArmed) return;
    const int throttle = int(int8_t(data[0]));
    const int turn = int(int8_t(data[1]));
    if (throttle < 0 || throttle > 100 || turn < -100 || turn > 100) return;
    if (isMotionBlocked() || (state != IDLE && state != MANUAL)) {
      if (state == MANUAL) { motors.stop(); state = IDLE; }
      wsArmed = false;
      wsOwner = -1;
      wsServer.sendTXT(client, "BLOCKED");
      return;
    }
    wsLastMotionMs = millis();
    lastManualCommandTime = wsLastMotionMs;
    const float y = float(throttle) / 100.0f;
    const float x = float(turn) / 100.0f;
    const int p = motors.maxSpeed;
    const int l = lroundf(p * y * (x < 0.0f ? 1.0f + x : 1.0f));
    const int r = lroundf(p * y * (x > 0.0f ? 1.0f - x : 1.0f));
    if (l == 0 && r == 0) {
      motors.stop();
      if (state == MANUAL) state = IDLE;
    } else {
      state = MANUAL;
      motors.drive(l, r); // Enforces ultrasonic, all-IR and diagnostic interlocks.
    }
  }

  static const char* wifiStatusText(wl_status_t st) {
    switch(st) {
      case WL_CONNECTED: return "CONNECTED";
      case WL_NO_SSID_AVAIL: return "SSID NOT FOUND";
      case WL_CONNECT_FAILED: return "AUTH/CONNECTION FAILED";
      case WL_CONNECTION_LOST: return "CONNECTION LOST";
      case WL_DISCONNECTED: return "DISCONNECTED";
      case WL_IDLE_STATUS: return "CONNECTING/IDLE";
      default: return "OTHER";
    }
  }

  bool startAccessPoint() {
    // In AP+STA mode both interfaces MUST share one radio channel. If the
    // router is linked, put AP on that channel; otherwise use predictable 1.
    const int staChannel = WiFi.status()==WL_CONNECTED ? WiFi.channel() : 1;
    const int channel = (staChannel>=1 && staChannel<=11) ? staChannel : 1;
    bool ok=WiFi.softAP(apName.c_str(),apKey.c_str(),channel,false,4);
    if(ok) {
      dnsServer.start(53,"*",WiFi.softAPIP());
      Serial.printf("[PORTAL] AP ACTIVE | SSID=%s | IP=%s | channel=%d | clients=%d\n",
          apName.c_str(),WiFi.softAPIP().toString().c_str(),channel,WiFi.softAPgetStationNum());
    } else Serial.println("[PORTAL] ERROR: softAP() returned false; check Wi-Fi mode/board reset");
    return ok;
  }

  void connectRouter(bool reconnect=false) {
    if(routerName.isEmpty()) {
      Serial.println("[WIFI] No STA SSID set, AP remains available.");
      return;
    }
    // A router retry can force AP to change channel and disconnect an iPhone.
    // Preserve the direct AP session over automatic router reconnection.
    if(reconnect && WiFi.softAPgetStationNum()>0) {
      lastRouterRetryMs=millis();
      return;
    }
    lastRouterRetryMs=millis();
    routerConnectStartedMs = lastRouterRetryMs;
    routerConnectPending = true;
    routerConnectError = "";
    ++routerAttempts;
    Serial.printf("[WIFI] %s router %s (attempt %u; AP stays enabled)\n",
                  reconnect?"Retrying":"Connecting to",routerName.c_str(),unsigned(routerAttempts));
    // WiFi.begin on explicit saved credentials; do not disable AP.
    WiFi.begin(routerName.c_str(),routerKey.c_str());
  }


  void sendController() {
    server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    if (fsMounted && LittleFS.exists("/index.html")) {
      File f = LittleFS.open("/index.html", "r");
      if (f && !f.isDirectory()) {
        // GitHub OTA flashes firmware, NOT LittleFS. Prefer the newer built-in
        // page when the existing LittleFS HTML belongs to an earlier release.
        String head=f.readStringUntil('\n');
        for(int i=0;i<10 && f.available();i++) head+=f.readStringUntil('\n');
        const String wanted=String("agv-ui-version\" content=\"")+String(AGV_UI_BUILD);
        if(head.indexOf(wanted)>=0) {
          f.seek(0);
          server.streamFile(f,"text/html; charset=utf-8");
          f.close();return;
        }
        Serial.println("[WEB] LittleFS UI version old/missing; serving current firmware bundled UI");
      }
      if (f) f.close();
    }
    // Do not format flash or leave the operator without the web controller.
    server.send_P(200, "text/html; charset=utf-8", PAGE);
  }

  void redirectToController() {
    server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
    server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    server.send(302, "text/plain", "Open GUIDELine AGV controller");
  }

public:
  void begin(const char* apSsid, const char* apPass) {
    // 1. Mount LittleFS (the filesystem just uploaded via the IDE tool)
    fsMounted = LittleFS.begin(false); // Never auto-format an uploaded filesystem.
    indexReady = fsMounted && LittleFS.exists("/index.html");
    Serial.println(fsMounted ? "[FS] LittleFS mounted." : "[FS] LittleFS not mounted; flash preserved.");
    Serial.println(indexReady ? "[FS] /index.html present: web page served from LittleFS." :
                               "[FS] /index.html absent: embedded controller page will be used.");

    // 2. Start captive AP FIRST and preserve it independently of router status.
    wifiPrefsOK=wifiPrefs.begin("agv-network",false);
    apName=apSsid;
    apKey=apPass;
    routerName=wifiPrefsOK?wifiPrefs.getString("ssid",DEFAULT_STA_SSID):String(DEFAULT_STA_SSID);
    routerKey=wifiPrefsOK?wifiPrefs.getString("pass",DEFAULT_STA_PASS):String(DEFAULT_STA_PASS);
    routerName.trim();
    WiFi.mode(WIFI_AP_STA);
    WiFi.setHostname(DEVICE_HOSTNAME); // Before softAP()/WiFi.begin(), as in working v1.9.15.
    WiFi.setAutoReconnect(false); // AP clients take priority; controlled STA retries below.
    WiFi.setSleep(false); // Helps on routers prone to ESP32 STA sleep/reconnect delays.
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    bool apConfigured=WiFi.softAPConfig(IPAddress(192,168,4,1),IPAddress(192,168,4,1),IPAddress(255,255,255,0));
    Serial.printf("[PORTAL] softAPConfig=%s | WiFi mode=%d\n",apConfigured?"OK":"FAILED",int(WiFi.getMode()));
    startAccessPoint();
    if (wifiPrefsOK && routerName != String(DEFAULT_STA_SSID))
      Serial.printf("[WIFI] Stored NVS SSID overrides compiled default: %s\n",routerName.c_str());
    connectRouter();
    initBLE("GUIDELine-AGV");

    // 3. Controller web page: LittleFS first, embedded HTML as fallback.
    server.on("/", HTTP_GET, [this]() { sendController(); });
    server.on("/index.html", HTTP_GET, [this]() { sendController(); });

    // Android / iOS / macOS / Windows use probe URLs to detect captive networks.
    // Explicit redirect (rather than "204" or "Success") prompts their login UI.
    const char* captiveProbes[] = {
      "/generate_204", "/gen_204", "/hotspot-detect.html",
      "/library/test/success.html", "/success.txt",
      "/connecttest.txt", "/ncsi.txt", "/redirect", "/canonical.html",
      "/fwlink"
    };
    for (const char* path : captiveProbes) {
      server.on(path, HTTP_GET, [this]() { redirectToController(); });
    }

    server.on("/version", HTTP_GET, [this]() {
      server.sendHeader("Cache-Control","no-store");
      server.send(200,"application/json",String("{\"fw\":\"")+AGV_FW_BUILD+"\",\"ui\":\""+AGV_UI_BUILD+"\"}");
    });
    server.on("/fs/status", HTTP_GET, [this]() {
      String json = "{\"mounted\":";
      json += fsMounted ? "true" : "false";
      json += ",\"index\":";
      json += (fsMounted && LittleFS.exists("/index.html")) ? "true" : "false";
      json += ",\"fallback\":true,\"bytes\":";
      if (fsMounted && LittleFS.exists("/index.html")) {
        File f = LittleFS.open("/index.html", "r");
        json += String(f ? f.size() : 0);
        if (f) f.close();
      } else json += "0";
      json += "}";
      server.send(200, "application/json", json);
    });

    // Color sensor: live measurements and persistent indoor/outdoor calibration.
    server.on("/color/status",HTTP_GET,[this]() {
      Reading rd=sensors.getColorReading();
      auto num=[](float x)->String {return isfinite(x)?String(x,1):String("null");};
      String j="{\"r\":"+num(rd.r)+",\"g\":"+num(rd.g)+",\"b\":"+num(rd.b)+",\"c\":"+num(rd.c);
      j+=",\"fresh\":"+String(sensors.hasFreshColor()?"true":"false");
      j+=",\"ageMs\":"+String(sensors.getColorSampleAgeMs());
      j+=",\"station\":"+String(sensors.getCurrentStationIndex());
      j+=",\"matchedProfile\":"+String(sensors.getMatchedProfile());
      j+=",\"distance\":"+num(sensors.getColorDistance());
      j+=",\"mode\":"+String(sensors.getColorMode());
      j+=",\"indoorTolerance\":"+String(sensors.getColorTolerance(0),3);
      j+=",\"outdoorTolerance\":"+String(sensors.getColorTolerance(1),3);
      j+=",\"floorIndoor\":"+String(sensors.floorCalibrated(0)?"true":"false");
      j+=",\"floorOutdoor\":"+String(sensors.floorCalibrated(1)?"true":"false");
      j+=",\"sampling\":"+String(sensors.captureBusy()?"true":"false");
      j+=",\"message\":\""+sensors.captureStatus()+"\"";
      j+=",\"profiles\":[";
      for(int p=0;p<2;p++) {
        if(p) j+=",";
        j+="[";
        for(int i=0;i<4;i++) {
          if(i) j+=",";
          j+=sensors.isColorCalibrated(p,i)?"true":"false";
        }
        j+="]";
      }
      j+="]}";
      server.sendHeader("Cache-Control","no-store");
      server.send(200,"application/json",j);
    });
    server.on("/color/capture",HTTP_POST,[this]() {
      // Prevent moving the robot while sampling a fixed floor marker.
      if(state!=IDLE && state!=STATION_WAIT && state!=FINISHED && state!=FAULT_STOP) {
        server.send(409,"text/plain","STOP the vehicle before calibrating");return;
      }
      if(!sensors.hasFreshColor()) {server.send(409,"text/plain","Color sensor has no valid fresh reading");return;}
      if(!sensors.startCapture(server.arg("profile").toInt(),server.arg("color").toInt())) {
        server.send(409,"text/plain","Invalid calibration or already sampling");return;
      }
      server.send(202,"text/plain","Sampling started. Keep sensor over marker until saved.");
    });
    server.on("/color/settings",HTTP_POST,[this]() {
      if(!sensors.setColorSettings(server.arg("mode").toInt(),
          server.arg("indoorTolerance").toFloat(),server.arg("outdoorTolerance").toFloat())) {
        server.send(400,"text/plain","Invalid color settings");return;
      }
      server.send(200,"text/plain","Calibration settings saved to NVS");
    });
    server.on("/color/reset",HTTP_POST,[this]() {
      if(state!=IDLE && state!=FINISHED && state!=FAULT_STOP) {
        server.send(409,"text/plain","Stop and reset AGV first");return;
      }
      sensors.resetColorCalibration();
      server.send(200,"text/plain","Color calibration cleared. Restart device to reload default references.");
    });


    // Station assignment: repeated colors and many-to-many RFID permissions.
    server.on("/stations/status", HTTP_GET, [this]() {
      String json=stations.json();
      json.remove(json.length()-1);
      json+=",\"nextSlot\":"+String(nextStationSlot)+",\"latched\":"+String(latchedStation);
      uint32_t remain=0;
      if(state==STATION_WAIT && latchedStation>=1 && latchedStation<=3 && stationWaitStartMs && stations.autoContinue){
        uint32_t limit=stations.timeoutSec[latchedStation]*1000UL;
        uint32_t elapsed=uint32_t(millis()-stationWaitStartMs);
        remain=limit>elapsed?(limit-elapsed+999)/1000:0;
      }
      json+=",\"remainingSec\":"+String(remain)+",\"rfid\":"+rfid.listJSON()+"}";
      server.send(200,"application/json",json);
    });
    server.on("/stations/config",HTTP_POST,[this](){
      if(state!=IDLE && state!=FINISHED && state!=FAULT_STOP){
        server.send(409,"text/plain","Stop and reset route before editing station mapping");return;
      }
      int slot=server.arg("slot").toInt();
      int color=server.arg("color").toInt();
      long sec=server.arg("seconds").toInt();
      if(!server.hasArg("slot")||!server.hasArg("color")||!server.hasArg("seconds")||
         slot<0||slot>3||color<-1||color>3||sec<0||sec>3600){
        server.send(400,"text/plain","Invalid slot/color/seconds");return;
      }
      if((slot>0 && color==stations.colors[0]) || (slot==0 &&
         (color==stations.colors[1]||color==stations.colors[2]||color==stations.colors[3]))){
        server.send(409,"text/plain","Start/finish marker color must differ from workstations");return;
      }
      bool ok=stations.saveSlot(slot,color,uint32_t(sec));
      server.send(ok?200:500,"text/plain",ok?"Station settings saved":"NVS save failed");
    });
    server.on("/stations/auto",HTTP_POST,[this](){
      if(state!=IDLE && state!=FINISHED && state!=FAULT_STOP){server.send(409,"text/plain","Stop before changing auto-continue");return;}
      String x=server.arg("enabled");
      if(x!="0" && x!="1"){server.send(400,"text/plain","enabled=0 or 1");return;}
      bool ok=stations.setAuto(x=="1");
      server.send(ok?200:500,"text/plain",ok?"Station auto-continue saved":"NVS save failed");
    });
    // uid can be obtained by tapping a key and viewing lastSeenUID in the RFID test panel.
    server.on("/rfid/assign",HTTP_POST,[this](){
      if(state!=IDLE && state!=STATION_WAIT && state!=FINISHED && state!=FAULT_STOP){
        server.send(409,"text/plain","Stop before registering a key");return;
      }
      String uid=server.arg("uid");
      if(!server.hasArg("mask")){server.send(400,"text/plain","Missing station permission mask");return;}
      String maskText=server.arg("mask");
      for(size_t i=0;i<maskText.length();++i) if(maskText[i]<'0'||maskText[i]>'9') {
        server.send(400,"text/plain","Invalid mask: enter 0..15");return;
      }
      int mask=maskText.toInt();
      if(mask<0||mask>15){server.send(400,"text/plain","Invalid mask: enter 0..15");return;}
      AGVRFID::KeySaveResult result=rfid.saveKeyMask(uid,uint8_t(mask));
      bool ok=result==AGVRFID::KeySaveResult::SAVED || result==AGVRFID::KeySaveResult::MASTER_ALREADY_AUTHORIZED;
      Serial.printf("[RFID] Key registration: %s | UID=%s | mask=0x%02X\n",
        AGVRFID::keySaveMessage(result),AGVRFID::canonicalUID(uid).c_str(),unsigned(mask));
      server.send(ok?200:(result==AGVRFID::KeySaveResult::STORAGE_FULL?409:400),
        "text/plain",AGVRFID::keySaveMessage(result));
    });
    server.on("/rfid/master",HTTP_POST,[this](){
      if(state!=IDLE && state!=FINISHED){server.send(409,"text/plain","Stop and reset before changing master");return;}
      String uid=server.arg("uid");uid.trim();uid.toUpperCase();
      // Updating the Master requires an existing Master tap within the last 30 seconds.
      if(!rfid.isMaster(rfid.getLastSeenUID()) || millis()-rfid.getLastSeenMs()>30000){
        server.send(403,"text/plain","Present current Master Card first");return;
      }
      bool ok=rfid.setMasterUID(uid);
      server.send(ok?200:400,"text/plain",ok?"New Master Card UID saved":"Invalid Master UID");
    });
    server.on("/rfid/health",HTTP_GET,[this](){
      server.sendHeader("Cache-Control","no-store");
      server.send(200,"application/json",rfid.diagnosticJSON());
    });
    server.on("/rfid/test",HTTP_POST,[this](){
      if(state!=IDLE && state!=FINISHED && state!=FAULT_STOP && state!=STATION_WAIT){
        server.send(409,"text/plain","Stop the AGV before testing RFID");return;
      }
      motors.stop();rfid.startDiagnostic(30000);
      server.send(200,"text/plain","RFID diagnostic scan active for 30s; motor authorization disabled");
    });
    server.on("/rfid/reinit",HTTP_POST,[this](){
      if(state!=IDLE && state!=FINISHED && state!=FAULT_STOP && state!=STATION_WAIT){
        server.send(409,"text/plain","Stop the AGV before reinitializing RFID");return;
      }
      motors.stop();rfid.reinitialize();
      server.send(200,"application/json",rfid.diagnosticJSON());
    });
    server.on("/rfid/latest",HTTP_GET,[this](){
      // A card read is informational, not station authorization.
      server.send(200,"application/json",String("{\"uid\":\"")+rfid.getLastSeenUID()+"\",\"ageMs\":"+String(millis()-rfid.getLastSeenMs())+"}");
    });
    // Live router and captive AP information; never return stored passwords.
    server.on("/wifi/status", HTTP_GET, [this]() {
      auto escapeJson=[](String x)->String {
        x.replace("\\", "\\\\");
        x.replace("\"", "\\\"");
        x.replace("\n", " ");x.replace("\r", " ");
        return x;
      };
      updateRouterJoinResult();
      bool linked = routerHasAddress();
      String configured = wifiPrefsOK ? wifiPrefs.getString("ssid", DEFAULT_STA_SSID) : String(DEFAULT_STA_SSID);
      String j = "{\"connected\":" + String(linked ? "true" : "false");
      j += ",\"ssid\":\"" + escapeJson(linked ? WiFi.SSID() : configured) + "\"";
      j += ",\"ip\":\"" + String(linked ? WiFi.localIP().toString() : String("")) + "\"";
      j += ",\"rssi\":" + String(linked ? WiFi.RSSI() : 0);
      j += ",\"hostname\":\"" + escapeJson(String(DEVICE_HOSTNAME)) + "\"";
      j += ",\"apActive\":" + String(WiFi.softAPSSID().length() ? "true" : "false");
      j += ",\"apSsid\":\"" + escapeJson(WiFi.softAPSSID()) + "\"";
      j += ",\"apIp\":\"" + WiFi.softAPIP().toString() + "\"";
      j += ",\"wifiStatus\":" + String(int(WiFi.status()));
      const char* phase = linked ? "connected" :
        (routerConnectPending ? "connecting" :
          (routerConnectError.length() ? "failed" : "disconnected"));
      j += ",\"routerPhase\":\"" + String(phase) + "\"";
      j += ",\"routerMessage\":\"" + escapeJson(routerConnectError) + "\"";
      j += ",\"routerAttempt\":" + String(routerAttempts);
      j += ",\"routerElapsedMs\":" + String(routerConnectPending ? uint32_t(millis()-routerConnectStartedMs) : 0);
      j += ",\"wifiStatusText\":\"" + String(wifiStatusText(WiFi.status())) + "\"}";
      server.sendHeader("Cache-Control", "no-store");
      server.send(200, "application/json", j);
    });
    server.on("/wifi/sta",HTTP_POST,[this](){
      if(state!=IDLE && state!=FINISHED){server.send(409,"text/plain","Stop/Reset before router changes");return;}
      String ssid=server.arg("ssid"),pass=server.arg("pass");
      if(ssid.length()<1||ssid.length()>32||pass.length()>63||!wifiPrefsOK){
        server.send(400,"text/plain","Invalid credentials or NVS unavailable");return;
      }
      if(wifiPrefs.putString("ssid",ssid)!=ssid.length() || wifiPrefs.putString("pass",pass)!=pass.length()){
        server.send(500,"text/plain","NVS save failed");return;
      }
      routerName=ssid;routerKey=pass;
      connectRouter();
      server.sendHeader("Cache-Control", "no-store");
      server.send(202,"text/plain","Router connection requested. Check /wifi/status for the assigned IP.");
    });
    // Straight-line load test. Only authorized Web START runs it; no RFID required.
    server.on("/loadtest/start",HTTP_POST,[this](){
      if(state!=IDLE || loadTest.active || sensors.captureBusy() || rfid.isDiagnosticActive()){
        server.send(409,"text/plain","STOP/RESET before load test");return;
      }
      if(isAllIRBlack()) {
        server.send(409,"text/plain","Load test blocked: all five IR sensors read BLACK (11111)");return;
      }
      if(!sensors.isPathClear() || !sensors.hasLine()){
        server.send(409,"text/plain","Place robot on guide line with clear path");return;
      }
      int n=server.arg("number").toInt();
      float kg=server.arg("kg").toFloat();
      int pct=server.arg("speedPct").toInt();
      if(n<1||n>9999||!isfinite(kg)||kg<0||kg>20||pct<1||pct>100){
        server.send(400,"text/plain","Test no. 1..9999; load 0..20 kg; speed 1..100 percent");return;
      }
      motors.stop();
      loadTest.begin(uint16_t(n),kg,uint8_t(pct),motors.maxSpeed);
      loadTestPwmCeiling=constrain(int(lroundf(255.0f*pct/100.0f)),1,255);
      updatePhysicalSpeedPot(true);
      state=LOAD_TEST_RUN;
      server.send(200,"application/json",loadTest.json());
    });
    server.on("/loadtest/stop",HTTP_POST,[this](){
      if(loadTest.active){motors.stop();loadTest.finish("MANUAL STOP");loadTestPwmCeiling=255;updatePhysicalSpeedPot(true);state=IDLE;}
      server.send(200,"application/json",loadTest.json());
    });
    server.on("/loadtest/status",HTTP_GET,[this](){
      server.sendHeader("Cache-Control","no-store");
      server.send(200,"application/json",loadTest.json());
    });

    // 4. API Endpoints
    server.on("/status", HTTP_GET, [this]() {
      String s = "{";
      s += "\"state\":\"" + String(getStateName(state)) + "\",";
      s += "\"station\":\"" + String(sensors.getCurrentStationName()) + "\",";
      s += "\"distCm\":" + String(sensors.getDistanceCm()) + ",";
      s += "\"speed\":" + String(int((motors.maxSpeed / 255.0) * 100)) + ",";
      s += "\"potRaw\":"+String(speedPotRaw)+",";
      s += "\"potFiltered\":"+String(speedPotFiltered)+",";
      s += "\"potPwm\":"+String(speedPotPwm)+",";
      s += "\"maxPwm\":"+String(motors.maxSpeed)+",";
      s += "\"loadTestPwmCeiling\":"+String(loadTest.active ? loadTestPwmCeiling : 255)+",";
      s += "\"event\":\"" + String(isAllIRBlackSafetyLatched() ? "ALL FIVE IR BLACK - SAFETY STOP" : "Operational") + "\",";
      s += "\"stationRemainingSec\":"+String((state==STATION_WAIT && latchedStation>0 && stationWaitStartMs && stations.autoContinue)?
        max(0L,long(stations.timeoutSec[latchedStation])-long((millis()-stationWaitStartMs)/1000UL)):0L)+",";
      // Always send 5 bits in physical S1..S5 order. String(bits,BIN) loses leading zeros.
      String irBlack,irRaw;
      for(int i=0;i<5;i++) {
        irBlack += ((sensors.getIRBits() >> i)&1) ? '1':'0';
        irRaw += ((sensors.getRawIRBits() >> i)&1) ? '1':'0';
      }
      s += "\"black\":\""+irBlack+"\",";
      s += "\"rawIR\":\""+irRaw+"\",";
      s += "\"blackLow\":"+String(sensors.getIRBlackLow()?"true":"false")+",";
      s += "\"irAllBlack\":"+String((sensors.getIRBits() & ALL_IR_BLACK_MASK)==ALL_IR_BLACK_MASK?"true":"false")+",";
      s += "\"irAllBlackLatched\":"+String(isAllIRBlackSafetyLatched()?"true":"false")+",";
      s += "\"irAllBlackElapsedMs\":"+String(getAllIRBlackElapsedMs())+",";
      s += "\"irAllBlackStopMs\":"+String(ALL_IR_BLACK_STOP_MS);
      s += "}";
      server.send(200, "application/json", s);
    });

    // Change sensor output polarity from the web, while stationary only.
    server.on("/ir/polarity", HTTP_POST, [this]() {
      if(state!=IDLE && state!=FAULT_STOP && state!=FINISHED) {
        server.send(409,"text/plain","Stop/Reset AGV before changing IR polarity");return;
      }
      const String v=server.arg("blackLow");
      if(v!="0" && v!="1") {server.send(400,"text/plain","Expected blackLow=0 or 1");return;}
      sensors.setIRBlackLow(v=="1");
      server.send(200,"text/plain",v=="1"?"BLACK = LOW saved":"BLACK = HIGH saved");
    });

    server.on("/start", HTTP_POST, [this]() {
      bool ok = requestAutonomousStart(true); // Web START may recover faults.
      server.send(ok ? 200 : 409, "text/plain", ok ? "OK" :
        (isAllIRBlack() ? "START BLOCKED: all 5 IR sensors read BLACK (11111); position AGV on guide line" :
        (isAllIRBlackSafetyLatched() ? "All-black fault latched: clear sensors then START" : "Start blocked by safety or operating state")));
    });
    server.on("/reset", HTTP_POST, [this]() {
      // A Web reset disarms any active WebSocket controller so a held finger
      // cannot immediately send another motor command.
      if(wsOwner>=0) wsServer.sendTXT(uint8_t(wsOwner), "STOPPED");
      wsOwner=-1; wsArmed=false;
      requestSafeStop();
      server.send(200, "text/plain", isAllIRBlackSafetyLatched() ?
        "RESET: all-black fault remains while IR=11111" : "RESET: Ready for START");
    });
    server.on("/stop", HTTP_POST, [this]() {
      if(wsOwner>=0) wsServer.sendTXT(uint8_t(wsOwner), "STOPPED");
      wsOwner=-1; wsArmed=false;
      requestSafeStop();
      server.send(200, "text/plain", "OK");
    });
    
    server.on("/drive", HTTP_POST, [this]() {
      if(rfid.isDiagnosticActive()) { motors.stop();server.send(409,"text/plain","RFID diagnostic scan active");return; }
      float x = server.arg("turn").toFloat();
      float y = server.arg("throttle").toFloat();
      int p = motors.maxSpeed;
      int l = lroundf(p * y * (x < 0 ? 1 + x : 1));
      int r = lroundf(p * y * (x > 0 ? 1 - x : 1));
      if (isMotionBlocked()) {
        motors.stop();
        server.send(409, "text/plain", isAllIRBlackSafetyLatched() ? "All-black safety fault" : "Obstacle/RFID diagnostic active");
        return;
      }
      if (state == FAULT_STOP || state == FINISHED || state == RECOVERY_SEARCH || state == LOAD_TEST_RUN || state == SERVICE_TEST) {
        motors.stop();
        server.send(409, "text/plain", "Use Web START or Master Card to authorize recovery");
        return;
      }
      // If parked on 11111, do not initiate a manual movement either.
      if(isAllIRBlack() && motors.getAppliedL()==0 && motors.getAppliedR()==0) {
        motors.stop();server.send(409,"text/plain","Cannot start manual drive: IR=11111");return;
      }
      state = MANUAL;
      lastManualCommandTime = millis();
      motors.drive(l, r);
      server.send(200, "text/plain", "OK");
    });

    server.on("/safety", HTTP_POST, [this]() {
      int d = server.arg("dist").toInt();
      sensors.setSafetyDistance(d);
      server.send(200, "text/plain", "OK");
    });

    server.on("/ota/github", HTTP_POST, [this]() {
      // Firmware replacement requires the AGV to be stationary; enforce this
      // server-side even if a client bypasses the UI confirmation dialog.
      if(state!=IDLE && state!=FINISHED && state!=FAULT_STOP && state!=STATION_WAIT){
        server.send(409,"text/plain","STOP/RESET AGV before a GitHub firmware update");return;
      }
      if(WiFi.status()!=WL_CONNECTED){
        server.send(503,"text/plain","GitHub OTA unavailable: router Wi-Fi is not connected");return;
      }
      motors.stop();
      state=SERVICE_TEST; // BLE and HTTP motion commands must be rejected.
      server.send(202,"text/plain","GitHub update started; motors locked OFF. Monitor Serial for HTTP/update status.");
      String msg;
      bool complete=updateFromGitHub(msg); // On success, Update reboots ESP32.
      if(!complete){
        motors.stop();state=FAULT_STOP;
        Serial.printf("[OTA] GitHub update failed, safety fault latched: %s\n",msg.c_str());
      }
    });

    // Redirect unknown browser GET URLs, including additional OS connectivity probes.
    // Unknown POST commands must not turn into page navigation or control actions.
    server.onNotFound([this]() {
      if (server.method() == HTTP_GET) redirectToController();
      else server.send(404, "text/plain", "Unknown AGV endpoint");
    });

    server.begin();
    Serial.println("[HTTP] Web server started on port 80.");
    wsServer.begin();
    wsServer.onEvent([this](uint8_t n, WStype_t t, uint8_t* p, size_t len) {
      handleWebSocket(n,t,p,len);
    });
    wsServer.enableHeartbeat(10000,3000,2);
    Serial.println("[WS] Low-latency joystick on TCP 81 (AP 192.168.4.1 + router IP)");
  }

  void update() {
    // Non-blocking AP/STA health manager: no waits that pause navigation or RFID.
    updateRouterJoinResult();
    bool linked=WiFi.status()==WL_CONNECTED;
    if(linked!=previousRouterConnected) {
      previousRouterConnected=linked;
      if(linked) {
        routerAttempts=0;
        Serial.printf("[WIFI] ROUTER CONNECTED | SSID=%s | IP=%s | RSSI=%d dBm | hostname=%s.local\n",
          WiFi.SSID().c_str(),WiFi.localIP().toString().c_str(),WiFi.RSSI(),DEVICE_HOSTNAME);
      } else Serial.printf("[WIFI] ROUTER DISCONNECTED | reason/status=%s (%d) | captive AP retained\n",
         wifiStatusText(WiFi.status()),int(WiFi.status()));
    }
    if(!linked && !routerConnectPending && uint32_t(millis()-lastRouterRetryMs)>=30000) {
      // Avoid disrupting captive-portal clients with channel hopping.
      if(WiFi.softAPgetStationNum()==0) connectRouter(true);
      else lastRouterRetryMs=millis();
    }
    if(!linked && uint32_t(millis()-lastRouterStatusMs)>=10000) {
      lastRouterStatusMs=millis();
      Serial.printf("[WIFI] STA=%s (%d), target=%s | AP SSID=%s IP=%s\n",
        wifiStatusText(WiFi.status()),int(WiFi.status()),routerName.c_str(),
        WiFi.softAPSSID().c_str(),WiFi.softAPIP().toString().c_str());
    }
    if(uint32_t(millis()-lastAPCheckMs)>=15000) {
      lastAPCheckMs=millis();
      Serial.printf("[PORTAL] health SSID=%s | IP=%s | APclients=%d | STA=%s | radioChannel=%d\n",
          WiFi.softAPSSID().c_str(),WiFi.softAPIP().toString().c_str(),
          WiFi.softAPgetStationNum(),linked?"CONNECTED":"OFFLINE",WiFi.channel());
      if(WiFi.softAPSSID().isEmpty()) {
        Serial.println("[PORTAL] AP unexpectedly absent; restoring AP+STA mode");
        WiFi.mode(WIFI_AP_STA);
        WiFi.softAPConfig(IPAddress(192,168,4,1),IPAddress(192,168,4,1),IPAddress(255,255,255,0));
        startAccessPoint();
      }
    }
    // Service the latency-critical joystick before the HTTP page/status requests.
    wsServer.loop();
    // Lost packets must NOT leave the AGV running, or resume on a late frame.
    if (wsOwner >= 0 && wsArmed &&
        uint32_t(millis()-wsLastMotionMs) > WS_RC_TIMEOUT_MS) {
      const int expiredClient = wsOwner;
      if (state == MANUAL) { motors.stop(); state = IDLE; }
      wsOwner = -1;
      wsArmed = false; // Must receive a new A after joystick release.
      wsServer.sendTXT(uint8_t(expiredClient), "TIMEOUT");
      Serial.println("[WS] Joystick watchdog timeout; stopped and disarmed");
    }
    dnsServer.processNextRequest();
    server.handleClient();
  }

};