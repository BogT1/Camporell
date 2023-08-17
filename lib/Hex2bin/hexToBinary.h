
#ifndef hex_binary_decoder_h
#define hex_binary_decoder_h

#include <Arduino.h>

class Decoder {
 public:
  Decoder();
  String bytesToHex(byte Str[], int length);
  unsigned int hexToInt(String str);
  String intToBin(int i, int min_size);
  unsigned int binToInt(String str);

 private:
  String calcHEX(byte i);
};

#endif