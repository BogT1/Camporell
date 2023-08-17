#include "WH24CP.h"

#include <Arduino.h>

#define DEBUG 3

WH24CP::WH24CP() {}

void WH24CP::init(byte* original, int len) {
  Decoder decoder;
  strInHex = decoder.bytesToHex(original, len);
  originalPayloadLen = len;
  for (int i = 0; i < len; i++)
    originalPayload[i] = original[i];
  setWindDir();
  setWindSpeed();
  setWindBurst();
  setUv();
  setLux();
  setTemperature();
  setHumidity();
  setRain();
}

int WH24CP::setWindDir() {
  Decoder decoder;
  String dir = strInHex.substring(4, 7);  // Extraccion de la cadena
  int dirInt = decoder.hexToInt(dir);     // Conversion a binario
  String dirBinary = decoder.intToBin(dirInt, 12);
  String dirBinary2 = dirBinary.substring(8, 9) + dirBinary.substring(0, 8);
  windDir = decoder.binToInt(dirBinary2);
  return windDir;
}

int WH24CP::setWindSpeed() {
  Decoder decoder;
  String speed = strInHex.substring(12, 14);
  windSpeed = (decoder.hexToInt(speed) / 8.0) * 112.0;
  return windSpeed;
}

int WH24CP::setWindBurst() {
  Decoder decoder;
  String speedR = strInHex.substring(14, 16);
  windBurst = decoder.hexToInt(speedR) * 112.0;
  return windBurst;
}

int WH24CP::setUv() {
  Decoder decoder;
  String strUV = strInHex.substring(20, 24);
  uv = decoder.hexToInt(strUV);
  return uv;
}

int WH24CP::setLux() {
  Decoder decoder;
  String strLux = strInHex.substring(24, 30);
  float fLux = (decoder.hexToInt(strLux) / 10.0) * LUX_TO_WM2;
  lux = fLux * 10;
  return lux;
}

int WH24CP::setTemperature() {
  Decoder decoder;
  String strTemp = strInHex.substring(7, 10);
  temperature = (decoder.hexToInt(strTemp) - 400);
  return temperature;
}

int WH24CP::setHumidity() {
  Decoder decoder;
  String strHumidity = strInHex.substring(10, 12);
  humidity = decoder.hexToInt(strHumidity);
  return humidity;
}

int WH24CP::setRain() {
  Decoder decoder;
  String strRain = strInHex.substring(16, 20);
  if (decoder.hexToInt(strRain) < 1000) {
    rain = decoder.hexToInt(strRain) * 3;
  } else {
    rain = decoder.hexToInt(strRain) * 10;
  }
  return rain;
}

void WH24CP::addBattery(int batV) {
  battery = batV;
}

void WH24CP::setWindSpeed(int swindSpeed) {
  windSpeed = swindSpeed;
}

void WH24CP::setWindBurst(int swindBurst) {
  windBurst = swindBurst;
}

int WH24CP::getWindSpeed() {
  return windSpeed;
}

int WH24CP::getWindBurst() {
  return windBurst;
}

String WH24CP::originalToDec() {
  String str = "";
  for (int i = 0; i < originalPayloadLen; i++) {
    if (i > 0)
      str += ",";
    str += String(originalPayload[i]);
  }
  return str;
}

String WH24CP::originalToHex() {
  Decoder decoder;
  return decoder.bytesToHex(originalPayload, originalPayloadLen);
}

String WH24CP::humanReadable() {
  String strResult = "";
  strResult += "\tWind Dir: " + String(windDir) + "º\n";
  strResult += "\tWind Speed: " + String(windSpeed / 100.0) + " m/s\n";
  strResult += "\tWind Burst: " + String(windBurst / 100.0) + " m/s\n";
  strResult += "\tUV: " + String(uv) + " uW/cm2\n";
  strResult += "\tLUX: " + String(lux / 10.0) + " w/m2\n";
  strResult += "\tTemp: " + String(temperature / 10.0) + " ºC\n";
  strResult += "\tHumidity: " + String(humidity) + "% RH\n";
  strResult += "\tRain: " + String(rain / 10.0) + " mL\n";
  strResult += "\tBattery: " + String(battery / 100.0) + " V\n";
  return strResult;
}

int WH24CP::getOutput(uint8_t* appDataArray) {
  int outputWinSpeed = (windSpeed * 10);  // Convert from m/s to Km/h
  int outputWinBurst = (windBurst * 10);  // Convert from m/s to Km/h

  appDataArray[0] = (uint8_t)(windDir >> 8);
  appDataArray[1] = (uint8_t)windDir;
  appDataArray[2] = (uint8_t)(temperature >> 8);
  appDataArray[3] = (uint8_t)temperature;
  appDataArray[4] = (uint8_t)(humidity >> 8);
  appDataArray[5] = (uint8_t)humidity;
  appDataArray[6] = (uint8_t)(uv >> 8);
  appDataArray[7] = (uint8_t)uv;
  appDataArray[8] = (uint8_t)(rain >> 8);
  appDataArray[9] = (uint8_t)rain;
  appDataArray[10] = (uint8_t)(battery >> 8);
  appDataArray[11] = (uint8_t)battery;
  appDataArray[12] = (uint8_t)(lux >> 8);
  appDataArray[13] = (uint8_t)lux;
  appDataArray[14] = (uint8_t)(outputWinSpeed >> 8);
  appDataArray[15] = (uint8_t)outputWinSpeed;
  appDataArray[16] = (uint8_t)(outputWinBurst >> 8);
  appDataArray[17] = (uint8_t)outputWinBurst;

  return 18;
}