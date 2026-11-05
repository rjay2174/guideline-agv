#pragma once
#include <Arduino.h>
#include <Preferences.h>
class AGVStationConfig {
  Preferences prefs;
public:
  // Sensor colors: GREEN=0 RED=1 BLUE=2 YELLOW=3, or -1 disabled.
  int colors[4]={0,1,1,1};
  uint32_t timeoutSec[4]={0,30,30,30};
  bool autoContinue=false; // RFID / Web required by default.
  void begin(){
    if(!prefs.begin("agv-stations",false))return;
    autoContinue=prefs.getBool("auto",false);
    for(int i=0;i<4;i++){
      colors[i]=prefs.getInt((String("c")+i).c_str(),i==0?0:1);
      if(colors[i]<-1||colors[i]>3)colors[i]=(i==0?0:1);
      timeoutSec[i]=constrain(prefs.getUInt((String("t")+i).c_str(),i==0?0:30),0UL,3600UL);
    }
  }
  bool saveSlot(int i,int color,uint32_t sec){
    if(i<0||i>3||color<-1||color>3||sec>3600)return false;
    colors[i]=color;timeoutSec[i]=sec;
    return prefs.putInt((String("c")+i).c_str(),color)==sizeof(int) &&
      prefs.putUInt((String("t")+i).c_str(),sec)==sizeof(uint32_t);
  }
  bool setAuto(bool enabled){autoContinue=enabled;return prefs.putBool("auto",enabled)==1;}
  String json() const {
    String s="{\"auto\":"+String(autoContinue?"true":"false")+",\"slots\":[";
    for(int i=0;i<4;i++){
      if(i)s+=",";
      s+="{\"slot\":"+String(i)+",\"color\":"+String(colors[i])+
         ",\"seconds\":"+String(timeoutSec[i])+"}";
    }
    return s+"]}";
  }
};