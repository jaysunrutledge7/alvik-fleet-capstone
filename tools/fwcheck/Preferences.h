#pragma once
#include "Arduino.h"
struct Preferences { bool begin(const char*, bool){return true;} void end(){} String getString(const char*, const char* d){return String(d);} size_t putString(const char*, const String&){return 0;}
 float getFloat(const char*, float d){return d;} size_t putFloat(const char*, float){return 0;}
 bool isKey(const char*){return false;} bool clear(){return true;} };   // both in the ESP32 core's Preferences (2026-11-05a uses them)
