
#ifndef WH24CP_h
#define WH24CP_h

#include <Arduino.h>

#include "hexToBinary.h"

#define LUX_TO_WM2 0.0079

class WH24CP {
 public:
  WH24CP();
  void init(byte* orig, int length);
  int setWindDir();
  int setWindSpeed();
  int setWindBurst();
  int setUv();
  int setLux();
  int setTemperature();
  int setHumidity();
  int setRain();
  void addBattery(int batV);
  void setWindSpeed(int swindSpeed);
  void setWindBurst(int swindBurst);
  int getWindSpeed();
  int getWindBurst();
  String originalToDec();
  String originalToHex();
  String humanReadable();
  int getOutput(uint8_t* appDataArray);

 private:
  String strInHex = "";
  byte originalPayload[22];
  int originalPayloadLen;
  int windDir;
  int windSpeed;
  int windBurst;
  int uv;
  int lux;
  int temperature;
  int humidity;
  int rain;
  int battery;
};
#endif