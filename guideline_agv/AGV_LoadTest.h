#pragma once
#include <Arduino.h>
class AGVLoadTest {
public:
  bool active=false, recorded=false;
  uint16_t testNumber=0;
  float loadKg=0;
  uint8_t speedPct=0;
  uint32_t startMs=0, stopMs=0, elapsedMs=0;
  int savedSpeed=255;
  String reason="Not started";
  void begin(uint16_t n,float kg,uint8_t pct,int priorSpeed){
    testNumber=n;loadKg=kg;speedPct=pct;savedSpeed=priorSpeed;
    startMs=millis();stopMs=0;elapsedMs=0;reason="Running";
    active=true;recorded=false;
  }
  void finish(const String& why){
    if(!active)return;
    stopMs=millis();elapsedMs=uint32_t(stopMs-startMs);
    reason=why;active=false;recorded=true;
  }
  String json() const {
    String j="{\"active\":"+String(active?"true":"false")+",\"recorded\":"+String(recorded?"true":"false")+
      ",\"testNumber\":"+String(testNumber)+",\"loadKg\":"+String(loadKg,2)+
      ",\"speedPct\":"+String(speedPct)+",\"startMs\":"+String(startMs)+
      ",\"stopMs\":"+String(stopMs)+",\"elapsedMs\":"+String(active?uint32_t(millis()-startMs):elapsedMs)+
      ",\"reason\":\""+reason+"\"}";
    return j;
  }
};