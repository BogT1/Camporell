#include "hexToBinary.h"

#include <Arduino.h>

Decoder::Decoder() {
}

String Decoder::bytesToHex(byte Str[], int length) {
  String cad = "";
  for (int i = 0; i < length; i++) {
    cad = cad + calcHEX(Str[i]);
  }
  return cad;
}

String Decoder::calcHEX(byte b) {
  char ret[3];
  sprintf(ret, "%02X", b);
  return String(ret);
}

unsigned int Decoder::hexToInt(String str) {
  unsigned int i;
  sscanf(str.c_str(), "%X", &i);
  return i;
}

String Decoder::intToBin(int i, int minsize) {
  String ret = "";
  ret = String(i, BIN);
  while (ret.length() < minsize) {
    ret = "0" + ret;
  }
  return ret;
}

unsigned int Decoder::binToInt(String str) {
  unsigned int ret = 0;
  for (int i = 0; i < str.length(); i++) {
    ret = (ret << 1);
    if (str.charAt(i) == '1')
      ret++;
  }
  return ret;
}