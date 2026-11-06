#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include "AGV_Config.h"
#include <Preferences.h>
#include <ctype.h>

class AGVRFID {
private:
  MFRC522* pReader = nullptr;
  String lastCardUID = "";
  String lastSeenUID = "";
  String lastDeliveredUID = "";
  uint32_t lastDeliveredMs = 0;
  uint32_t lastSeenMs = 0;
  Preferences store;
  bool storeOK=false;
  // Station 0=start/finish, 1..3=workstations. A key's permissions are a bit mask.
  String masterUID=""; // stored in ESP32 NVS; never publish an authorized UID
  struct KeyEntry { String uid; uint8_t mask=0; };
  KeyEntry entries[12];
  uint8_t pinSS = 17;
  uint8_t pinRST = 18;
  uint8_t lastHardwareVersion = 0;
  uint32_t versionReadFailures = 0;
  uint32_t wakeupFailures = 0;
  uint32_t selectFailures = 0;
  uint32_t successfulReads = 0;
  bool hardwareErrorPrinted = false;
  uint8_t consecutiveSpiFaults = 0;
  uint32_t lastRecoveryAttemptMs = 0;
  uint32_t scanPollCount = 0;
  uint32_t noCardTimeouts = 0;
  uint32_t diagnosticUntilMs = 0;
  bool diagnosticScanEnabled = false;
  bool requireCardRemoval = false;
  uint32_t cardAbsentSinceMs = 0;


public:
  // UID format: 4, 7 or 10 hexadecimal bytes, with optional spaces,
  // colons or dashes. Canonical stored format is "AA BB CC DD".
  static String canonicalUID(const String& entered) {
    String compact;
    for (size_t i=0;i<entered.length();++i) {
      char ch=entered[i];
      if(ch==' ' || ch==':' || ch=='-' || ch=='\t') continue;
      if(!isxdigit(static_cast<unsigned char>(ch))) return "";
      compact+=char(toupper(static_cast<unsigned char>(ch)));
    }
    if(compact.length()!=8 && compact.length()!=14 && compact.length()!=20) return "";
    String output;
    for(size_t i=0;i<compact.length();i+=2) {
      if(output.length())output+=' ';
      output+=compact.substring(i,i+2);
    }
    return output;
  }
  enum class KeySaveResult : uint8_t {
    SAVED, MASTER_ALREADY_AUTHORIZED, INVALID_UID, INVALID_MASK,
    STORAGE_UNAVAILABLE, STORAGE_FULL, NVS_WRITE_FAILED
  };
  static const char* keySaveMessage(KeySaveResult result) {
    switch(result) {
      case KeySaveResult::SAVED: return "RFID permissions saved to NVS";
      case KeySaveResult::MASTER_ALREADY_AUTHORIZED: return "Master Card already authorizes every station; no duplicate registration required";
      case KeySaveResult::INVALID_UID: return "Invalid UID: enter 4, 7 or 10 hex bytes (example: AA BB CC DD)";
      case KeySaveResult::INVALID_MASK: return "Invalid station mask: use 0..15";
      case KeySaveResult::STORAGE_UNAVAILABLE: return "RFID NVS storage unavailable";
      case KeySaveResult::STORAGE_FULL: return "RFID key list full (12 keys); remove an unused key first";
      default: return "RFID NVS write failed; card not saved";
    }
  }
  void begin(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t ss, uint8_t rst) {
    storeOK=store.begin("agv-rfidmap",false);
    if(storeOK){
      String storedMaster=canonicalUID(store.getString("master",""));
      masterUID=storedMaster;
      for(int i=0;i<12;i++){
        entries[i].uid=canonicalUID(store.getString((String("u")+i).c_str(),""));
        entries[i].mask=store.getUChar((String("m")+i).c_str(),0);
      }
    }
    pinSS = ss;
    pinRST = rst;

    pinMode(pinSS, OUTPUT);
    digitalWrite(pinSS, HIGH);

    // Explicit pin assignment: SCK=12, MISO=13, MOSI=16, SS=17
    SPI.begin(sck, miso, mosi, pinSS);

    pReader = new MFRC522(pinSS, pinRST);
    pReader->PCD_Init();
    pReader->PCD_AntennaOn();
    pReader->PCD_SetAntennaGain(MFRC522::RxGain_max);

    uint8_t v = pReader->PCD_ReadRegister(MFRC522::VersionReg);
    lastHardwareVersion=v;
    Serial.printf("[RFID] MFRC522 Hardware Version: 0x%02X\n", v);
    if (v == 0x00 || v == 0xFF) {
      Serial.println("[RFID] ERROR: Check SPI jumper lines on SCK(12), MISO(13), MOSI(16), SS(17).");
    } else {
      Serial.println("[RFID] Reader active and scanning.");
    }
  }

  // A diagnostic scan observes UIDs but must NEVER authorize movement.
  bool isDiagnosticActive() const {
    return diagnosticScanEnabled && (int32_t)(millis()-diagnosticUntilMs)<0;
  }
  void startDiagnostic(uint32_t durationMs=30000) {
    lastCardUID="";
    diagnosticUntilMs=millis()+durationMs;
    diagnosticScanEnabled=true;
    Serial.printf("[RFID] Diagnostic scan active for %lu seconds: no motion authorization\n",(unsigned long)(durationMs/1000));
  }
  void cancelDiagnostic() {
    diagnosticScanEnabled=false;
    lastCardUID="";
  }
  // Only demand removal when a card was actually observed just before stop.
  // Otherwise the first new station RFID tap must be accepted immediately.
  void requireFreshTap() {
    const bool cardStillRecent = lastSeenUID.length() &&
      (uint32_t)(millis()-lastSeenMs) < 750UL;
    lastCardUID="";
    requireCardRemoval=cardStillRecent;
    cardAbsentSinceMs=0;
    lastDeliveredUID="";
    lastDeliveredMs=0;
    if (!cardStillRecent) {
      lastSeenUID="";
      lastSeenMs=0;
    }
    Serial.printf("[RFID] Fresh-tap ready: %s\n",
      requireCardRemoval ? "remove held card, then tap again" : "new tap accepted immediately");
  }
  void reinitialize() {
    if (!pReader) return;
    cancelDiagnostic();
    digitalWrite(pinSS,HIGH);
    pReader->PCD_Init();
    pReader->PCD_AntennaOn();
    pReader->PCD_SetAntennaGain(MFRC522::RxGain_max);
    lastHardwareVersion=pReader->PCD_ReadRegister(MFRC522::VersionReg);
    consecutiveSpiFaults=0;
    hardwareErrorPrinted=(lastHardwareVersion==0x00||lastHardwareVersion==0xFF);
    lastCardUID="";
    Serial.printf("[RFID] Reader reinitialized | VersionReg=0x%02X | %s\n", lastHardwareVersion,
      hardwareErrorPrinted?"SPI INVALID":"SPI RESPONDING");
  }
  void update() {
    if (!pReader) return;
    // Diagnostic expiry is logged once; pending diagnostic taps remain discarded.
    if(diagnosticScanEnabled && !isDiagnosticActive()) {
      diagnosticScanEnabled=false;
      lastCardUID="";
      Serial.println("[RFID] Diagnostic scan completed; remove card before normal authorization");
    }
    scanPollCount++;
    uint8_t v = pReader->PCD_ReadRegister(MFRC522::VersionReg);
    if (v == 0x00 || v == 0xFF) {
      versionReadFailures++;
      if(consecutiveSpiFaults<255)consecutiveSpiFaults++;
      if (!hardwareErrorPrinted) {
        Serial.printf("[RFID] SPI COMMUNICATION FAILURE: VersionReg=0x%02X. Check 3.3V, GND, SS17, SCK12, MISO13, MOSI16, RST18.\n",v);
        hardwareErrorPrinted=true;
      }
      // Restore the proven v1.9.15 SPI recovery: reinitialize only after
      // repeated invalid register values; never on ordinary no-card timeouts.
      if(consecutiveSpiFaults>=5 && (uint32_t)(millis()-lastRecoveryAttemptMs)>=1500) {
        lastRecoveryAttemptMs=millis();
        digitalWrite(pinSS,HIGH);
        pReader->PCD_Init();
        pReader->PCD_AntennaOn();
        pReader->PCD_SetAntennaGain(MFRC522::RxGain_max);
        uint8_t recovered=pReader->PCD_ReadRegister(MFRC522::VersionReg);
        if(recovered!=0x00 && recovered!=0xFF) {
          lastHardwareVersion=recovered;
          consecutiveSpiFaults=0;
          hardwareErrorPrinted=false;
          Serial.printf("[RFID] SPI RECOVERED after reader reinit: VersionReg=0x%02X\n",recovered);
        }
      }
      return;
    }
    if (hardwareErrorPrinted) Serial.printf("[RFID] SPI RECOVERED: VersionReg=0x%02X\n",v);
    hardwareErrorPrinted=false;
    consecutiveSpiFaults=0;
    lastHardwareVersion=v;

    byte atqa[2], size = sizeof(atqa);
    MFRC522::StatusCode wake = pReader->PICC_WakeupA(atqa, &size);
    if (wake != MFRC522::STATUS_OK) {
      if (wake == MFRC522::STATUS_TIMEOUT) {
        noCardTimeouts++;
        if (requireCardRemoval) {
          if (!cardAbsentSinceMs) cardAbsentSinceMs=millis();
          else if (uint32_t(millis()-cardAbsentSinceMs)>=350UL) {
            requireCardRemoval=false;
            lastDeliveredUID="";
            lastSeenUID="";
            lastSeenMs=0;
          }
        }
      } else {
        wakeupFailures++;
        cardAbsentSinceMs=0;
      }
      return;
    }
    cardAbsentSinceMs=0;
    MFRC522::StatusCode sel = pReader->PICC_Select(&pReader->uid);
    if (sel != MFRC522::STATUS_OK) { selectFailures++; return; }
    successfulReads++;
    if (requireCardRemoval) {
      // A still-present card is observed but cannot trigger authorization.
      pReader->PICC_HaltA();
      pReader->PCD_StopCrypto1();
      return;
    }

    String uid = "";
    for (byte i = 0; i < pReader->uid.size; i++) {
      if (i) uid += ' ';
      if (pReader->uid.uidByte[i] < 16) uid += '0';
      uid += String(pReader->uid.uidByte[i], HEX);
    }
    uid.toUpperCase();
    uint32_t now = millis();
    // Prevent a held card from repeatedly authorizing multiple states.
    if (!isDiagnosticActive() && (uid != lastDeliveredUID || (uint32_t)(now - lastSeenMs) >= RFID_REARM_MS)) {
      lastCardUID = uid;
      lastDeliveredUID = uid;
      lastDeliveredMs = now;
    }
    if (uid != lastSeenUID)
      Serial.printf("[RFID] SCANNED UID=%s | %s%s\n",uid.c_str(),uid==masterUID?"MASTER":"CARD",
        isDiagnosticActive()?" | DIAGNOSTIC ONLY":"");
    lastSeenUID = uid;
    lastSeenMs = now;

    pReader->PICC_HaltA();
    pReader->PCD_StopCrypto1();
  }

  void printDiagnostic() {
    if (!pReader) { Serial.println("[RFID] Reader object not initialized"); return; }
    uint8_t version=pReader->PCD_ReadRegister(MFRC522::VersionReg);
    Serial.printf("[RFID TEST] SPI SCK=12 MISO=13 MOSI=16 SS=17 RST=18 | VersionReg=0x%02X | %s\n",version,(version==0x00||version==0xFF)?"SPI FAILED":"SPI RESPONDING");
    Serial.printf("[RFID TEST] polls=%lu SPIbad=%lu noCard=%lu wakeError=%lu selectFail=%lu UIDread=%lu | master=%s | lastUID=%s | DIAG=%s | waitingRemoval=%s\n",
      (unsigned long)scanPollCount,(unsigned long)versionReadFailures,(unsigned long)noCardTimeouts,
      (unsigned long)wakeupFailures,(unsigned long)selectFailures,(unsigned long)successfulReads,
      masterUID.c_str(),lastSeenUID.length()?lastSeenUID.c_str():"NONE",isDiagnosticActive()?"ON":"OFF",
      requireCardRemoval?"YES":"NO");
    Serial.println("[RFID TEST] Remove card, then tap the Master Card; watch for [RFID] SCANNED UID.");
  }
  String diagnosticJSON() const {
    String j="{\"version\":\"0x";
    if(lastHardwareVersion<16)j+="0";
    j+=String(lastHardwareVersion,HEX);
    j+="\",\"spiResponding\":"+String((lastHardwareVersion!=0x00&&lastHardwareVersion!=0xFF)?"true":"false");
    j+=",\"scanActive\":"+String(isDiagnosticActive()?"true":"false");
    j+=",\"remainingSec\":"+String(isDiagnosticActive()?(diagnosticUntilMs-millis()+999)/1000UL:0);
    j+=",\"polls\":"+String(scanPollCount);
    j+=",\"spiFailures\":"+String(versionReadFailures);
    j+=",\"noCardTimeouts\":"+String(noCardTimeouts);
    j+=",\"wakeupErrors\":"+String(wakeupFailures);
    j+=",\"selectFailures\":"+String(selectFailures);
    j+=",\"successfulReads\":"+String(successfulReads);
    j+=",\"lastUID\":\""+lastSeenUID+"\"";
    j+=",\"waitingCardRemoval\":"+String(requireCardRemoval?"true":"false");
    j+=",\"masterConfigured\":\""+masterUID+"\"}";
    return j;
  }
  const String& getLastSeenUID() const { return lastSeenUID; }
  uint32_t getLastSeenMs() const { return lastSeenMs; }

  String readUID() {
    if(isDiagnosticActive()) {lastCardUID="";return "";}
    String card = lastCardUID;
    lastCardUID = "";
    return card;
  }

  bool isMaster(const String& uid) const { return uid.length()>0 && uid == masterUID; }
  String getMasterUID() const {return masterUID;}
  bool setMasterUID(const String &uid) {
    String clean=canonicalUID(uid);
    if(!storeOK || clean.isEmpty()) return false;
    if(store.putString("master",clean)!=clean.length())return false;
    masterUID=clean; return true;
  }
  String listJSON() const {
    String j="{\"master\":\""+masterUID+"\",\"keys\":[";
    bool first=true;
    for(int i=0;i<12;i++)if(entries[i].uid.length()){
      if(!first)j+=",";first=false;
      j+="{\"uid\":\""+entries[i].uid+"\",\"mask\":"+String(entries[i].mask)+"}";
    }
    return j+"]}";
  }
  KeySaveResult saveKeyMask(const String& submittedUID, uint8_t mask) {
    String uid=canonicalUID(submittedUID);
    if(uid.isEmpty()) return KeySaveResult::INVALID_UID;
    if(mask>15) return KeySaveResult::INVALID_MASK;
    if(uid==masterUID) return KeySaveResult::MASTER_ALREADY_AUTHORIZED;
    if(!storeOK) return KeySaveResult::STORAGE_UNAVAILABLE;
    int slot=-1;
    for(int i=0;i<12;i++)if(entries[i].uid==uid){slot=i;break;}
    if(mask==0) { // Clear an existing key so its slot can be reused.
      if(slot<0) return KeySaveResult::SAVED;
      String key=String("u")+slot, m=String("m")+slot;
      if(!store.remove(key.c_str()) || !store.remove(m.c_str()))
        return KeySaveResult::NVS_WRITE_FAILED;
      entries[slot].uid="";entries[slot].mask=0;
      return KeySaveResult::SAVED;
    }
    if(slot<0)for(int i=0;i<12;i++)if(entries[i].uid.isEmpty()){slot=i;break;}
    if(slot<0)return KeySaveResult::STORAGE_FULL;
    String key=String("u")+slot, m=String("m")+slot;
    if(store.putString(key.c_str(),uid)!=uid.length())return KeySaveResult::NVS_WRITE_FAILED;
    if(store.putUChar(m.c_str(),mask)!=1)return KeySaveResult::NVS_WRITE_FAILED;
    entries[slot].uid=uid; entries[slot].mask=mask;
    return KeySaveResult::SAVED;
  }
  bool setKeyMask(const String& uid, uint8_t mask) {
    KeySaveResult result=saveKeyMask(uid,mask);
    return result==KeySaveResult::SAVED || result==KeySaveResult::MASTER_ALREADY_AUTHORIZED;
  }
  bool keyAuthorized(const String& uid, int slot) const {
    if(isMaster(uid))return true;
    if(slot<0||slot>3)return false;
    for(const KeyEntry& e:entries)if(e.uid==uid && e.uid.length() && (e.mask & (1<<slot)))return true;
    // Historic keys stay valid for their earlier stations, unless reassigned.
    bool registered=false;
    for(const KeyEntry& e:entries)if(e.uid==uid && e.uid.length())registered=true;
    if(registered)return false;
    if(slot==1 && uid=="0D 7C A6 05")return true;
    if(slot==2 && uid=="E9 2D 00 D8")return true;
    return false;
  }

  bool isAuthorized(const String& uid, int stationIdx) {return keyAuthorized(uid,stationIdx);}
};