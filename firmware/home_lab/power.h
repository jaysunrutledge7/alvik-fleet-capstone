// Power and the motor board, as the Nano sees them (firmware 2026-10-29b; 2 Nov 2026: all three robots went silent
// within 24 s at 21-24 %, both lights blinking red when Nick reached them; report e02-nov02-low-battery-doe.md).
//
// No battery voltage: the Alvik library (Arduino_Alvik 1.1.1) gives the Nano none. The motor board (STM32) reads the
// battery chip (MAX17332, voltage included) and sends the Nano only the charge %, signed for charging (its firmware
// sends 'p' once a second); get_battery_charge() / is_battery_charging() are that number. The library reads the battery
// chip itself only while the motor board is OFF (its idle() wait). So this reports what there is:
//   - charge %, charging (0 / 1)                         (in `status`)
//   - the motor board's power pin: alvik.is_on()         (CHECK_STM32; low = the motor board is off)
//   - how old the newest sensor reading is               (the motor board sends the IMU every ~10 ms; six raw floats
//                                                         never repeat exactly while it runs, so readings that stop
//                                                         changing mean it stopped sending, or the library's update
//                                                         thread is stuck in its idle() wait with the motor board off)
// When the readings stop for stm_stale_ms, the robot says once "M,<name>: motor board not answering (...)" (if its WiFi
// still works), and "M,<name>: motor board answering again (...)" when they come back.
// Not seen from here: the motor board's own battery alert (Arduino_AlvikCarrier 1.1.1, switched on by the library's
// begin()): not charging and under 20 % it blinks both lights red; under 10 % it stops the motors and ignores every
// command while still sending sensor data. The Nano's status says battery %, so the Pi can tell from that.
#pragma once

#include "Arduino_Alvik.h"
#include "params.h"
#include "net.h"

extern Arduino_Alvik alvik;

namespace power {

float last[6] = {0, 0, 0, 0, 0, 0};
unsigned long lastChange = 0;   // millis() of the newest reading that differed from the one before
unsigned long lastLook = 0;
bool said = false;              // "not answering" sent, "answering again" not yet

unsigned long ageMs() { return millis() - lastChange; }

const char* boardWords() { return alvik.is_on() ? "on" : "off"; }

// Called from loop() and every motion loop (shouldAbort): looks 10 times a second.
void tick() {
  if (millis() - lastLook < 100) return;
  lastLook = millis();
  float v[6];
  alvik.get_imu(v[0], v[1], v[2], v[3], v[4], v[5]);
  bool changed = false;
  for (int i = 0; i < 6; i++) {
    if (v[i] != last[i]) { changed = true; last[i] = v[i]; }
  }
  if (changed) {
    if (said) {
      net::logf("M,%s: motor board answering again (sensor readings back after %lu ms; motor board power %s), "
                "battery %d%%", net::name, ageMs(), boardWords(), alvik.get_battery_charge());
      said = false;
    }
    lastChange = millis();
    return;
  }
  float lim = P("stm_stale_ms");
  if (lim > 0 && !said && ageMs() >= (unsigned long)lim) {
    said = true;
    net::logf("M,%s: motor board not answering (no new sensor reading for %lu ms; motor board power %s; battery %d%% "
              "and charging %d are its last readings)", net::name, ageMs(), boardWords(),
              alvik.get_battery_charge(), alvik.is_battery_charging() ? 1 : 0);
  }
}

}  // namespace power
