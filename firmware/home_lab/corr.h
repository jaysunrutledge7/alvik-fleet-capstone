// Per-robot corrections: gyro_scale, gyro_drift_dps and wheel_scale (params.h),
// saved on the robot (ESP32 Preferences, namespace "corr") so they survive
// power-off, and applied by heading(), libraryMove(), rollStraight() and the
// own pose (motion.h, pose.h).
//
//   corr                       show the corrections and where they came from
//   corr reset                 back to out of the box (1, 0, 1): a self-calibration test starts here
//   corr tape                  load the tape-measured numbers for this robot (TAPE below)
//   corr save <name> <value>   set one and save it
//   save                       (2026-11-05a) save the values in use now (after the calibration run's `set`s) so they
//                              survive power-off and deep sleep; refused whole if one looks wrong (same limits as corr save)
//   unsave  (or defaults)      (2026-11-05a) forget the saved values: back to 1, 0, 1 now and at every power-on
//   saved                      (2026-11-05a) each value and whether it came from saved storage or is the default
//   run gyro                   stand still gyro_check_s, report how fast the heading creeps
//                              (then: corr save gyro_drift_dps <value it reports>)
//
// `set gyro_scale ...` still works for a quick try, but lasts until power-off.
// Every change is logged:  H,CORR,ms,name,old,new,source   (source: tape, save, reset, dock later)
// and every run's PARAM records carry the values in use.
#pragma once

#include <Preferences.h>
#include "params.h"
#include "net.h"
#include "motion.h"
#include "rawlog.h"

namespace corr {

// (2026-11-05c) arc_k_scale and arc_d_scale: the curves' calibration (calib_run.py arc phase), saved with the others
const char* NAMES[] = {"gyro_scale", "gyro_drift_dps", "wheel_scale", "arc_k_scale", "arc_d_scale"};
const float FACTORY[] = {1.0f, 0.0f, 1.0f, 1.0f, 1.0f};
const int N = 5;

// Tape-measured, T1 on 2026-10-18/19 (docs/session-notes/2026-10-19.md). Straights: 100 cm asked,
// Left 97.6 cm, Center 98.1 cm travelled. Right has no runs yet: the average of the two.
// Turns are NOT here: the squares' ~80 deg over-turn fits a gyro scale (x1.075) and a gyro drift
// (~0.6 deg/s) equally well, and the L route (+9 deg with no net turning) points to drift.
// `run gyro` tells them apart; the turn correction is saved from its result.
struct Tape { const char* robot; float wheelScale; };
const Tape TAPE[] = {{"Left", 0.976f}, {"Center", 0.981f}, {"Right", 0.9785f}};

String source = "out of the box";

void logChange(const char* name, float oldV, float newV, const char* src) {
  net::sendLine("H,CORR,ms,name,old,new,source");
  net::logf("D,CORR,%lu,%s,%.5f,%.5f,%s", millis(), name, oldV, newV, src);
}

bool isCorr(const String& name) {
  for (int i = 0; i < N; i++) if (name.equalsIgnoreCase(NAMES[i])) return true;
  return false;
}

void store(const char* name, float v) {
  Preferences prefs;
  prefs.begin("corr", false);
  prefs.putFloat(name, v);
  prefs.putString("src", source);
  prefs.end();
}

void setOne(const char* name, float v, const char* src) {
  Param* p = findParam(name);
  if (!p) return;
  float old = p->value;
  p->value = v;
  store(p->name, v);
  logChange(p->name, old, v, src);
}

void show() {
  net::logf("M,%s corrections (%s): gyro_scale %.4f, gyro_drift_dps %.4f, wheel_scale %.4f", net::name,
            source.c_str(), P("gyro_scale"), P("gyro_drift_dps"), P("wheel_scale"));
}

bool fromSaved[N] = {false, false, false, false, false};   // (2026-11-05a) this value came from saved storage at power-on / save

// (2026-11-05a) "M,<name> calibration: wheel_scale 0.9700 (saved), gyro_scale 1.0300 (saved), gyro_drift_dps
// 0.3000 (default)": each value and where it came from. At power-on and on `saved`.
void showSaved(const char* when) {
  char buf[300];
  int k = 0;
  for (int i = 0; i < N; i++) {
    k += snprintf(buf + k, sizeof(buf) - k, "%s%s %.4f (%s)", i ? ", " : "", NAMES[i], P(NAMES[i]),
                  fromSaved[i] ? "saved" : "default");
  }
  net::logf("M,%s calibration %s: %s", net::name, when, buf);
}

// At power-on, after the name is known.
void load() {
  Preferences prefs;
  prefs.begin("corr", true);
  for (int i = 0; i < N; i++) {
    Param* p = findParam(NAMES[i]);
    fromSaved[i] = prefs.isKey(NAMES[i]);
    if (p) p->value = prefs.getFloat(NAMES[i], FACTORY[i]);
  }
  source = prefs.getString("src", "out of the box");
  prefs.end();
  show();
  showSaved("at power-on");   // (2026-11-05a) which values came from saved storage
}

// The sanity limits of corr save: a typo (0, 75 instead of 0.75) would send the robot off the table.
bool looksWrong(const char* name, float v) {
  bool drift = strcasecmp(name, "gyro_drift_dps") == 0;
  return drift ? fabsf(v) > 3.0f : (v < 0.8f || v > 1.25f);
}

// (2026-11-05a) `save`: every value in use now, saved. All or nothing.
void saveAll() {
  for (int i = 0; i < N; i++) {
    float v = P(NAMES[i]);
    if (looksWrong(NAMES[i], v)) {
      net::logf("M,save refused: %s %.4f looks wrong (allowed: %s): nothing saved", NAMES[i], v,
                i == 1 ? "-3 to 3" : "0.8 to 1.25");   // (11-05c) the arc scales: 0.8 to 1.25 too
      return;
    }
  }
  source = "saved with save";
  for (int i = 0; i < N; i++) {
    Param* p = findParam(NAMES[i]);
    store(p->name, p->value);
    logChange(p->name, p->value, p->value, "save");
    fromSaved[i] = true;
  }
  net::logf("M,%s saved on the robot (kept after power-off): wheel_scale %.4f, gyro_scale %.4f, gyro_drift_dps %.4f, "
            "arc_k_scale %.4f, arc_d_scale %.4f", net::name, P("wheel_scale"), P("gyro_scale"), P("gyro_drift_dps"),
            P("arc_k_scale"), P("arc_d_scale"));
}

// (2026-11-05a) `unsave` / `defaults`: the saved values forgotten, back to out of the box now and at power-on.
void unsaveAll() {
  Preferences prefs;
  prefs.begin("corr", false);
  prefs.clear();
  prefs.end();
  source = "out of the box";
  for (int i = 0; i < N; i++) {
    Param* p = findParam(NAMES[i]);
    if (!p) continue;
    float old = p->value;
    p->value = FACTORY[i];
    logChange(p->name, old, FACTORY[i], "unsave");
    fromSaved[i] = false;
  }
  net::logf("M,%s saved calibration cleared: back to defaults wheel_scale %.4f, gyro_scale %.4f, gyro_drift_dps %.4f",
            net::name, P("wheel_scale"), P("gyro_scale"), P("gyro_drift_dps"));
}

// `corr ...` from the laptop. Not while a run is going (the heading would jump mid-run).
void command(String rest, bool running) {
  rest.trim();
  if (rest.length() == 0) { show(); return; }
  if (running) { net::sendLine("M,busy: send stop first"); return; }
  int sp = rest.indexOf(' ');
  String verb = sp < 0 ? rest : rest.substring(0, sp);
  String args = sp < 0 ? String("") : rest.substring(sp + 1);
  args.trim();
  if (verb.equalsIgnoreCase("reset")) {
    source = "out of the box";
    for (int i = 0; i < N; i++) { setOne(NAMES[i], FACTORY[i], "reset"); fromSaved[i] = true; }   // saved as 1, 0, 1
    show();
    return;
  }
  if (verb.equalsIgnoreCase("tape")) {
    for (const Tape& t : TAPE) {
      if (strcasecmp(t.robot, net::name) == 0) {
        source = "tape, T1 2026-10-19";
        setOne("wheel_scale", t.wheelScale, "tape");
        fromSaved[2] = true;
        show();
        return;
      }
    }
    net::logf("M,no tape numbers for a robot called %s (Left, Center or Right: send name first)", net::name);
    return;
  }
  if (verb.equalsIgnoreCase("save")) {
    int sp2 = args.indexOf(' ');
    String name = sp2 < 0 ? args : args.substring(0, sp2);
    String val = sp2 < 0 ? String("") : args.substring(sp2 + 1);
    val.trim();
    if (!isCorr(name)) { net::sendLine("M,usage: corr save gyro_scale|gyro_drift_dps|wheel_scale <value>"); return; }
    char* end = nullptr;
    float v = strtof(val.c_str(), &end);
    if (end == val.c_str()) { net::logf("M,'%s' is not a number", val.c_str()); return; }
    // sanity limits: a typo (0, 75 instead of 0.75) would send the robot off the board
    bool drift = name.equalsIgnoreCase("gyro_drift_dps");
    if (looksWrong(name.c_str(), v)) {
      net::logf("M,%s %.4f looks wrong (allowed: %s): not saved", name.c_str(), v, drift ? "-3 to 3" : "0.8 to 1.25");
      return;
    }
    if (source == "out of the box") source = "saved by hand";
    setOne(findParam(name)->name, v, "save");
    for (int i = 0; i < N; i++) if (name.equalsIgnoreCase(NAMES[i])) fromSaved[i] = true;
    show();
    return;
  }
  net::sendLine("M,usage: corr | corr reset | corr tape | corr save <name> <value>");
}

}  // namespace corr

// ===================================================================
// GYRO (`run gyro`): stand still gyro_check_s and measure how fast the
// gyro's own heading creeps (no scale or drift correction in it). Put the
// robot down anywhere flat and don't touch it or the table.
//   H,GYRO,run,secs,raw_change_deg,raw_drift_dps,corrected_change_deg,gyro_scale,gyro_drift_dps,battery
// raw_drift_dps is the number to save (corr save gyro_drift_dps ...);
// corrected_change_deg should be near 0 once it is saved.
// ===================================================================
void runGyro() {
  if (!calibrateYawSign()) return;
  waitMs(1000);                          // settle after the compass wiggle
  float s = P("gyro_check_s");
  if (s < 5) s = 5;
  float r0 = rawHeadingAcc(), h0 = heading();
  unsigned long t0 = millis();
  bool ok = waitMs((unsigned long)(s * 1000));
  float secs = (millis() - t0) / 1000.0f;
  float rawChange = rawHeadingAcc() - r0;
  float corrChange = wrap180(heading() - h0);
  float drift = secs > 0 ? rawChange / secs : 0;
  net::sendLine("H,GYRO,run,secs,raw_change_deg,raw_drift_dps,corrected_change_deg,gyro_scale,gyro_drift_dps,battery");
  net::logf("D,GYRO,%s,%.1f,%.3f,%.4f,%.3f,%.4f,%.4f,%d", gRunId.c_str(), secs, rawChange, drift, corrChange,
            P("gyro_scale"), P("gyro_drift_dps"), alvik.get_battery_charge());
  if (!ok) { net::logf("M,gyro check stopped after %.0f s", secs); return; }
  net::logf("M,gyro check: standing still %.0f s, the gyro's heading crept %.2f deg = %.4f deg/s "
            "(with the saved correction: %.2f deg). Over a 100 s square that is %.0f deg.",
            secs, rawChange, drift, corrChange, drift * 100.0f);
}
