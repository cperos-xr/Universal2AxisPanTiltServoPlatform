// ------------------------------------------------------------
// PanTilt_JSON: 2-axis pan/tilt servo controller (ESP32-C3 tested)
// Protocol: JSONL (one JSON object per line, newline-terminated)
// No ArduinoJson dependency.
//
// Features:
// - set/adjust/center with dur or speed
// - stop/stopAll/resetAll
// - invert axis without jump
// - queue: off|on|step + qAdd/qClear/qAbort/qStatus/qList (use {"cmd":"queue","mode":"step"} for macros)
// - sweep macro (enqueues steps)
// - position favorites: save/recall (1..5)
// - command favorites (macros): favSave/favRun/favList/favClear (1..5), supports \\n multi-line scripts
// - persistence: persist + factoryReset using Preferences (NVS wear-leveled), versioned + CRC
//
// Notes:
// - "id" is optional command correlation id (for bots), NOT servo id
// - axis: x, y, xy
// - degrees are logical -90..+90
// ------------------------------------------------------------

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Preferences.h>

// ------------------- Pins -------------------
// Use the pins that you already validated on your board.
// If 3/4 worked for you, keep them.
#define SERVO1_PIN 3  // X (pan)
#define SERVO2_PIN 4  // Y (tilt)

Servo s1, s2;

// ------------------- Calibrated safe ranges (microseconds) -------------------
static const int S1_MIN_US = 500;
static const int S1_MAX_US = 2400;

static const int S2_MIN_US = 800;
static const int S2_MAX_US = 2050;

// ------------------- Limits -------------------
static const float POS_MIN = -90.0f;
static const float POS_MAX =  90.0f;

// Avoid macro collisions with libc headers:
static const uint16_t CMD_LINE_MAX = 3600;

// Queue sizing (fixed for predictability)
static const uint8_t QMAX = 20;

// Timeout guard for queued steps
static const uint32_t STEP_TIMEOUT_GRACE_MS = 2000;

// Favorites
static const int POS_FAV_SLOTS = 5;
static const int CMD_FAV_SLOTS = 5;
static const uint16_t FAV_SCRIPT_MAX = 3600; // max stored macro length per slot

// ------------------- Runtime State -------------------
static float v1 = 0.0f;  // X degrees (logical)
static float v2 = 0.0f;  // Y degrees (logical)

static bool invX = false;
static bool invY = false;

// default speed in deg/sec when dur not specified
static float defaultSpeed = 90.0f;

// Position favorites
static bool posFavValid[POS_FAV_SLOTS] = {false,false,false,false,false};
static float posFavX[POS_FAV_SLOTS] = {0,0,0,0,0};
static float posFavY[POS_FAV_SLOTS] = {0,0,0,0,0};

// Command favorites (macros/scripts)
static bool cmdFavValid[CMD_FAV_SLOTS] = {false,false,false,false,false};
static String cmdFavScript[CMD_FAV_SLOTS] = {"","","","",""};

// Optional routing fields for master-script demux (echoed)
static String lastSubsystem = "";
static String lastRoute = "";

// Dirty flag for persistence (only written when persist is called)
static bool cfgDirty = false;

// ------------------- Motion Profiles -------------------
struct MoveProfile {
  bool active = false;
  float start = 0;
  float target = 0;
  uint32_t t0 = 0;
  uint32_t durMs = 0;
  uint32_t cmdRef = 0; // used for done events
  char axis = '?';
};

static MoveProfile mx, my;

// ------------------- Queue -------------------
enum QueueMode : uint8_t { Q_OFF=0, Q_ON=1, Q_STEP=2 };
static QueueMode qMode = Q_STEP;

struct QueueItem {
  bool used = false;
  uint32_t id = 0;

  String subsystem;
  String route;
  String kind;       // set/adjust/center/recall/sweep legs/etc.

  bool useX = false;
  bool useY = false;

  float tx = 0;
  float ty = 0;
  uint32_t dx = 0;
  uint32_t dy = 0;

  uint32_t expectedEnd = 0;
};

static QueueItem q[QMAX];
static uint8_t qHead = 0;
static uint8_t qTail = 0;
static uint8_t qCount = 0;

static bool qActive = false;
static QueueItem qCurrent;
static bool qCurXDone = true;
static bool qCurYDone = true;
static uint32_t qStartedAt = 0;

static uint32_t autoId = 0;

// ------------------- Preferences / Persistence -------------------
static Preferences prefs;
static const char* PREF_NS = "pantilt";
static const uint32_t CFG_MAGIC = 0x50544A31; // 'PTJ1'
static const uint16_t CFG_VERSION = 1;

struct PersistedConfig {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  float defaultSpeed;
  uint8_t invX;
  uint8_t invY;
  uint8_t posValidMask; // 5 bits
  uint8_t reserved2;
  float posX[POS_FAV_SLOTS];
  float posY[POS_FAV_SLOTS];
  uint32_t crc32;
};

// ------------------- Utilities -------------------
static float clampf(float x, float lo, float hi) { return (x<lo)?lo:((x>hi)?hi:x); }
static int clampInt(int x, int lo, int hi) { return (x<lo)?lo:((x>hi)?hi:x); }

static int mapSignedToUs(float v, int minUs, int maxUs) {
  v = clampf(v, POS_MIN, POS_MAX);
  int center = (minUs + maxUs) / 2;
  int halfRange = (maxUs - minUs) / 2;
  int us = center + (int)((v * halfRange) / 90.0f);
  return clampInt(us, minUs, maxUs);
}

static void applyOutputs() {
  float px = invX ? -v1 : v1;
  float py = invY ? -v2 : v2;

  int us1 = mapSignedToUs(px, S1_MIN_US, S1_MAX_US);
  int us2 = mapSignedToUs(py, S2_MIN_US, S2_MAX_US);

  s1.writeMicroseconds(us1);
  s2.writeMicroseconds(us2);
}

static uint32_t durationFromSpeed(float start, float target, float speedDegPerSec) {
  float sp = speedDegPerSec;
  if (sp < 0.1f) sp = 0.1f;
  float delta = fabsf(target - start);
  float sec = delta / sp;
  if (sec < 0.0f) sec = 0.0f;
  return (uint32_t)(sec * 1000.0f + 0.5f);
}

// Toggle invert with no physical jump:
static void toggleInvertX() { invX = !invX; v1 = -v1; cfgDirty = true; }
static void toggleInvertY() { invY = !invY; v2 = -v2; cfgDirty = true; }

// ------------------- CRC32 (simple, deterministic) -------------------
static uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
  crc = ~crc;
  for (size_t i=0;i<len;i++) {
    crc ^= data[i];
    for (int b=0;b<8;b++) {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

// ------------------- Minimal JSON Helpers -------------------
static bool findKey(const String& s, const char* key, int& keyPos) {
  String pat = "\""; pat += key; pat += "\"";
  keyPos = s.indexOf(pat);
  return keyPos >= 0;
}

static bool getStringField(const String& s, const char* key, String& out) {
  int kp;
  if (!findKey(s, key, kp)) return false;

  int colon = s.indexOf(':', kp);
  if (colon < 0) return false;

  int q1 = s.indexOf('"', colon + 1);
  if (q1 < 0) return false;

  String tmp;
  tmp.reserve(128);

  bool esc = false;
  for (int i = q1 + 1; i < (int)s.length(); i++) {
    char c = s[i];

    if (esc) {
      // keep escaped char exactly once
      tmp += c;
      esc = false;
      continue;
    }

    if (c == '\\') {
      esc = true;
      continue;
    }

    if (c == '"') {
      out = tmp;
      return true;
    }

    tmp += c;
  }
  return false;
}

static void flushOut() {
  Serial.flush();
}

static bool getNumberField(const String& s, const char* key, float& out) {
  int kp;
  if (!findKey(s, key, kp)) return false;
  int colon = s.indexOf(':', kp);
  if (colon < 0) return false;

  int i = colon + 1;
  while (i < (int)s.length() && (s[i] == ' ' || s[i] == '\t')) i++;

  int j = i;
  while (j < (int)s.length()) {
    char c = s[j];
    if (c == ',' || c == '}' || c == ' ' || c == '\t' || c == '\r' || c == '\n') break;
    j++;
  }
  if (j <= i) return false;
  out = s.substring(i, j).toFloat();
  return true;
}

static bool getIntField(const String& s, const char* key, int& out) {
  float tmp;
  if (!getNumberField(s, key, tmp)) return false;
  out = (int)tmp;
  return true;
}

static bool getBoolField(const String& s, const char* key, bool& out) {
  int kp;
  if (!findKey(s, key, kp)) return false;
  int colon = s.indexOf(':', kp);
  if (colon < 0) return false;

  int i = colon + 1;
  while (i < (int)s.length() && (s[i] == ' ' || s[i] == '\t')) i++;

  if (s.startsWith("true", i))  { out = true;  return true; }
  if (s.startsWith("false", i)) { out = false; return true; }
  return false;
}

// Unescape only what we need for scripts: \\n \\r \\t \\\\ \\/ \\"
static String unescapeScript(const String& in) {
  String out; out.reserve(in.length());
  for (int i=0;i<(int)in.length();i++) {
    char c = in[i];

    // Handle \n, \r, \t
    if (c == '\\' && i+1 < (int)in.length()) {
      char n = in[i+1];

      // Normal escapes
      if (n == 'n') { out += '\n'; i++; continue; }
      if (n == 'r') { out += '\r'; i++; continue; }
      if (n == 't') { out += '\t'; i++; continue; }

      // If user sent \\n (double backslash form), treat it as newline too.
      if (n == '\\' && i+2 < (int)in.length()) {
        char n2 = in[i+2];
        if (n2 == 'n') { out += '\n'; i += 2; continue; }
        if (n2 == 'r') { out += '\r'; i += 2; continue; }
        if (n2 == 't') { out += '\t'; i += 2; continue; }
        if (n2 == '\\') { out += '\\'; i += 2; continue; }
      }

      // Other escapes we allow
      if (n == '\\') { out += '\\'; i++; continue; }
      if (n == '/')  { out += '/';  i++; continue; }
      if (n == '\"') { out += '\"'; i++; continue; }
    }

    out += c;
  }
  return out;
}


// ------------------- Reply Helpers -------------------
static void printRoutingFields(const String& subsystem, const String& route) {
  if (subsystem.length()) { Serial.print(",\"subsystem\":\""); Serial.print(subsystem); Serial.print("\""); }
  if (route.length())     { Serial.print(",\"route\":\"");     Serial.print(route);     Serial.print("\""); }
}

static void sendOk(uint32_t id, const String& subsystem, const String& route, const char* msg) {
  Serial.print("{\"ok\":true,\"id\":"); Serial.print(id);
  printRoutingFields(subsystem, route);
  Serial.print(",\"msg\":\""); Serial.print(msg); Serial.println("\"}");
  flushOut();
}

static void sendErr(uint32_t id, const String& subsystem, const String& route, const char* code, const char* msg) {
  Serial.print("{\"ok\":false,\"id\":"); Serial.print(id);
  printRoutingFields(subsystem, route);
  Serial.print(",\"error\":\""); Serial.print(code);
  Serial.print("\",\"msg\":\""); Serial.print(msg); Serial.println("\"}");
  flushOut();
}

static void sendState(const char* eventName, uint32_t ref, const String& subsystem, const String& route) {
  Serial.print("{\"ok\":true");
  if (eventName) { Serial.print(",\"event\":\""); Serial.print(eventName); Serial.print("\""); }
  if (ref) { Serial.print(",\"ref\":"); Serial.print(ref); }
  printRoutingFields(subsystem, route);

  Serial.print(",\"state\":{");
  Serial.print("\"x\":"); Serial.print(v1, 2);
  Serial.print(",\"y\":"); Serial.print(v2, 2);
  Serial.print(",\"invX\":"); Serial.print(invX ? "true":"false");
  Serial.print(",\"invY\":"); Serial.print(invY ? "true":"false");
  Serial.print(",\"speed\":"); Serial.print(defaultSpeed, 2);
  Serial.print(",\"moving\":{");
  Serial.print("\"x\":"); Serial.print(mx.active ? "true":"false");
  Serial.print(",\"y\":"); Serial.print(my.active ? "true":"false");
  Serial.print("}");

  Serial.print(",\"queue\":{");
  Serial.print("\"mode\":\"");
  Serial.print(qMode==Q_OFF?"off":(qMode==Q_ON?"on":"step"));
  Serial.print("\",\"count\":"); Serial.print(qCount);
  Serial.print(",\"active\":"); Serial.print(qActive ? "true":"false");
  Serial.print("}");

  Serial.print(",\"cfgDirty\":"); Serial.print(cfgDirty ? "true":"false");
  Serial.print("}}");
  Serial.println("}");
  flushOut();
}

static void sendEventDoneAxis(char axis, uint32_t ref, const String& subsystem, const String& route) {
  Serial.print("{\"ok\":true,\"event\":\"done\",\"axis\":\""); Serial.print(axis);
  Serial.print("\",\"ref\":"); Serial.print(ref);
  printRoutingFields(subsystem, route);
  Serial.println("}");
  flushOut();
}

static void sendEventStarted(const QueueItem& it) {
  Serial.print("{\"ok\":true,\"event\":\"started\",\"ref\":"); Serial.print(it.id);
  printRoutingFields(it.subsystem, it.route);
  Serial.print(",\"step\":{");
  Serial.print("\"kind\":\""); Serial.print(it.kind); Serial.print("\"");
  Serial.print(",\"axis\":\"");
  if (it.useX && it.useY) Serial.print("xy");
  else if (it.useX) Serial.print("x");
  else Serial.print("y");
  Serial.print("\"");
  if (it.useX) { Serial.print(",\"x\":"); Serial.print(it.tx, 2); }
  if (it.useY) { Serial.print(",\"y\":"); Serial.print(it.ty, 2); }
  Serial.print(",\"dx\":"); Serial.print(it.dx);
  Serial.print(",\"dy\":"); Serial.print(it.dy);
  Serial.print("}}");
  Serial.println("}");
  flushOut();
}

static void sendEventStepDone(const QueueItem& it) {
  Serial.print("{\"ok\":true,\"event\":\"stepDone\",\"ref\":"); Serial.print(it.id);
  printRoutingFields(it.subsystem, it.route);
  Serial.println("}");
  flushOut();
}

static void sendEventFault(const String& subsystem, const String& route, const char* code, uint32_t ref, const char* msg) {
  Serial.print("{\"ok\":false,\"event\":\"fault\",\"error\":\""); Serial.print(code);
  Serial.print("\",\"ref\":"); Serial.print(ref);
  printRoutingFields(subsystem, route);
  Serial.print(",\"msg\":\""); Serial.print(msg); Serial.println("\"}");
  flushOut();
}

// ------------------- Queue Ops -------------------
static bool qIsFull() { return qCount >= QMAX; }
static bool qIsEmpty() { return qCount == 0; }

static bool qEnqueue(const QueueItem& it) {
  if (qIsFull()) return false;
  q[qTail] = it;
  q[qTail].used = true;
  qTail = (uint8_t)((qTail + 1) % QMAX);
  qCount++;
  return true;
}

static bool qDequeue(QueueItem& out) {
  if (qIsEmpty()) return false;
  out = q[qHead];
  q[qHead].used = false;
  qHead = (uint8_t)((qHead + 1) % QMAX);
  qCount--;
  return true;
}

static void qClearAll() {
  for (uint8_t i=0;i<QMAX;i++) q[i].used = false;
  qHead = qTail = qCount = 0;
}

static void stopX() { mx.active = false; mx.durMs = 0; }
static void stopY() { my.active = false; my.durMs = 0; }
static void stopAllMotion() { stopX(); stopY(); }

static void abortQueueAndMotion() {
  stopAllMotion();
  qClearAll();
  qActive = false;
  qCurXDone = qCurYDone = true;
}

// ------------------- Motion Start -------------------
static void startMoveX(float target, uint32_t durMs, uint32_t ref) {
  target = clampf(target, POS_MIN, POS_MAX);
  if (durMs == 0) { v1 = target; mx.active = false; mx.durMs = 0; return; }
  mx.active = true;
  mx.start = v1;
  mx.target = target;
  mx.t0 = millis();
  mx.durMs = durMs;
  mx.cmdRef = ref;
  mx.axis = 'x';
}

static void startMoveY(float target, uint32_t durMs, uint32_t ref) {
  target = clampf(target, POS_MIN, POS_MAX);
  if (durMs == 0) { v2 = target; my.active = false; my.durMs = 0; return; }
  my.active = true;
  my.start = v2;
  my.target = target;
  my.t0 = millis();
  my.durMs = durMs;
  my.cmdRef = ref;
  my.axis = 'y';
}

static void executeStep(const QueueItem& it) {
  stopAllMotion();

  qCurXDone = !it.useX || (it.dx == 0);
  qCurYDone = !it.useY || (it.dy == 0);

  if (it.useX) startMoveX(it.tx, it.dx, it.id);
  if (it.useY) startMoveY(it.ty, it.dy, it.id);

  applyOutputs();
}

// ------------------- Parsing Helpers -------------------
static bool parseAxisMask(const String& axis, bool& useX, bool& useY) {
  String a = axis; a.toLowerCase();
  if (a == "x")  { useX = true;  useY = false; return true; }
  if (a == "y")  { useX = false; useY = true;  return true; }
  if (a == "xy") { useX = true;  useY = true;  return true; }
  return false;
}

static bool computeDurations(
  bool useX, bool useY,
  float tx, float ty,
  bool hasDur, float durSec,
  bool hasSpeed, float speedDegPerSec,
  uint32_t& dx, uint32_t& dy
) {
  if (hasDur) {
    if (durSec < 0.0f || durSec > 3600.0f) return false;
    uint32_t durMs = (uint32_t)(durSec * 1000.0f + 0.5f);
    dx = useX ? durMs : 0;
    dy = useY ? durMs : 0;
    return true;
  }

  float sp = hasSpeed ? speedDegPerSec : defaultSpeed;
  if (sp < 0.1f || sp > 1000.0f) return false;

  dx = useX ? durationFromSpeed(v1, tx, sp) : 0;
  dy = useY ? durationFromSpeed(v2, ty, sp) : 0;
  return true;
}

static QueueItem buildStepFromCommand(
  uint32_t id,
  const String& subsystem,
  const String& route,
  const String& cmd,
  const String& axis,
  const String& line,
  bool& ok,
  String& errCode,
  String& errMsg
) {
  ok = false;
  QueueItem it;
  it.id = id;
  it.subsystem = subsystem;
  it.route = route;
  it.kind = cmd;

  bool useX=false, useY=false;
  if (!parseAxisMask(axis, useX, useY)) {
    errCode = "bad_axis"; errMsg = "axis must be x, y, or xy";
    return it;
  }
  it.useX = useX; it.useY = useY;

  float durSec = -1.0f;
  float speed = -1.0f;
  bool hasDur = getNumberField(line, "dur", durSec);
  bool hasSpeed = getNumberField(line, "speed", speed);

  float tx=v1, ty=v2;

  if (cmd == "center") {
    tx = 0.0f; ty = 0.0f;
  } else if (cmd == "set" || cmd == "adjust") {
    float val=0, xVal=0, yVal=0;
    bool hasValue = getNumberField(line, "value", val);
    bool hasX = getNumberField(line, "x", xVal);
    bool hasY = getNumberField(line, "y", yVal);

    String a = axis; a.toLowerCase();
    if (a == "xy") {
      if (!hasX && !hasY && !hasValue) {
        errCode="missing_value"; errMsg="For axis=xy provide x and/or y (or value for both)";
        return it;
      }
      if (hasValue) { xVal = val; yVal = val; hasX = true; hasY = true; }
      if (cmd == "set") {
        if (hasX) tx = xVal;
        if (hasY) ty = yVal;
      } else {
        if (hasX) tx = v1 + xVal;
        if (hasY) ty = v2 + yVal;
      }
    } else {
      if (!hasValue) { errCode="missing_value"; errMsg="Provide: value (degrees)"; return it; }
      if (a == "x") tx = (cmd=="set") ? val : (v1 + val);
      else          ty = (cmd=="set") ? val : (v2 + val);
    }
  } else {
    errCode="unknown_cmd"; errMsg="Unknown motion cmd";
    return it;
  }

  tx = clampf(tx, POS_MIN, POS_MAX);
  ty = clampf(ty, POS_MIN, POS_MAX);
  it.tx = tx; it.ty = ty;

  uint32_t dx=0, dy=0;
  if (!computeDurations(useX, useY, tx, ty, hasDur, durSec, hasSpeed, speed, dx, dy)) {
    errCode="bad_timing"; errMsg="Invalid dur or speed";
    return it;
  }
  it.dx = dx; it.dy = dy;

  ok = true;
  return it;
}

// ------------------- Command Lists / Help / Examples -------------------
static void printCommands() {
  Serial.println("Commands (JSON objects, newline-terminated):");
  Serial.println("  {\"cmd\":\"commands\"}  -> short list");
  Serial.println("  {\"cmd\":\"help\"}      -> full reference");
  Serial.println("  {\"cmd\":\"examples\"}  -> copy/paste examples");
  Serial.println("  {\"cmd\":\"status\"}    -> state dump");
  Serial.println();
  Serial.println("Info: commands, help, examples, status");
  Serial.println("Motion: set, adjust, center, stop, stopAll, resetAll, invert, speed");
  Serial.println("Position favs: save, recall");
  Serial.println("Command favs: favSave, favRun, favList, favClear");
  Serial.println("Queue: queue, qAdd, qClear, qAbort, qStatus, qList");
  Serial.println("Macro: sweep");
  Serial.println("Persistence: persist, factoryReset");
}

static void printHelp() {
  Serial.println("Protocol: JSONL (one JSON object per line). Required field: \"cmd\".");
  Serial.println("Serial: 115200 baud. Line ending must be Newline or Both NL/CR.");
  Serial.println();
  Serial.println("Key fields:");
  Serial.println("  axis: \"x\" | \"y\" | \"xy\"");
  Serial.println("  value: degrees for set/adjust, or deg/sec for speed");
  Serial.println("  x,y: degrees when axis=\"xy\"");
  Serial.println("  dur: seconds (float). If present, overrides speed");
  Serial.println("  speed: deg/sec override for a motion command");
  Serial.println("  q: true|false queue hint (depends on queue mode)");
  Serial.println("  id: optional correlation id (bots). Not a servo id.");
  Serial.println();
  Serial.println("Ranges:");
  Serial.println("  position degrees: -90 .. +90 (clamped)");
  Serial.println("  speed deg/sec:    0.1 .. 1000 (recommended typical 60..180)");
  Serial.println("  dur seconds:      0 .. 3600");
  Serial.println();

  Serial.println("COMMAND REFERENCE (description + at least 1 example each)");

  Serial.println("\ncommands");
  Serial.println("  Short list of commands.");
  Serial.println("  Example: {\"cmd\":\"commands\"}");

  Serial.println("\nhelp");
  Serial.println("  Full command reference.");
  Serial.println("  Example: {\"cmd\":\"help\"}");

  Serial.println("\nexamples");
  Serial.println("  Prints copy/paste examples.");
  Serial.println("  Example: {\"cmd\":\"examples\"}");

  Serial.println("\nstatus");
  Serial.println("  Returns current state (x/y, invert, speed, queue state, dirty flag).");
  Serial.println("  Example: {\"cmd\":\"status\"}");

  Serial.println("\nspeed");
  Serial.println("  Set global default speed (deg/sec) used when dur is absent.");
  Serial.println("  Example: {\"cmd\":\"speed\",\"value\":90}");

  Serial.println("\nset");
  Serial.println("  Set absolute target position(s) in degrees.");
  Serial.println("  Example (no id): {\"cmd\":\"set\",\"axis\":\"x\",\"value\":30,\"dur\":1.0}");
  Serial.println("  Example (xy):    {\"id\":10,\"cmd\":\"set\",\"axis\":\"xy\",\"x\":10,\"y\":-20,\"speed\":120}");

  Serial.println("\nadjust");
  Serial.println("  Add delta(s) to current position(s) in degrees.");
  Serial.println("  Example: {\"cmd\":\"adjust\",\"axis\":\"y\",\"value\":-5,\"speed\":120}");

  Serial.println("\ncenter");
  Serial.println("  Move selected axis/axes to 0.");
  Serial.println("  Example: {\"cmd\":\"center\",\"axis\":\"xy\",\"dur\":1.0}");

  Serial.println("\nstop");
  Serial.println("  Stop selected axis/axes immediately (hold current).");
  Serial.println("  Example: {\"cmd\":\"stop\",\"axis\":\"y\"}");

  Serial.println("\nstopAll");
  Serial.println("  Stop all motion. Defaults to flush queue (flush=true).");
  Serial.println("  Example: {\"cmd\":\"stopAll\"}");
  Serial.println("  Example: {\"cmd\":\"stopAll\",\"flush\":false}");

  Serial.println("\nresetAll");
  Serial.println("  Stop + clear queue + center x/y. Does not erase saved config.");
  Serial.println("  Example: {\"cmd\":\"resetAll\"}");

  Serial.println("\ninvert");
  Serial.println("  Toggle inversion for axis/axes without mechanical jump.");
  Serial.println("  Example: {\"cmd\":\"invert\",\"axis\":\"x\"}");

  Serial.println("\nsave");
  Serial.println("  Save current x/y position into slot 1..5 (position favorites).");
  Serial.println("  Example: {\"cmd\":\"save\",\"slot\":1}");

  Serial.println("\nrecall");
  Serial.println("  Move to saved position slot 1..5.");
  Serial.println("  Example: {\"cmd\":\"recall\",\"slot\":1,\"dur\":1.2}");

  Serial.println("\nfavSave");
  Serial.println("  Save a command favorite (macro script) into slot 1..5.");
  Serial.println("  Use \"line\" for a single command or \"script\" for multi-line.");
  Serial.println("  Multi-line is encoded with \\\\n inside the JSON string.");
  Serial.println("  Example (single): {\"cmd\":\"favSave\",\"slot\":1,\"line\":\"{\\\"cmd\\\":\\\"center\\\",\\\"axis\\\":\\\"xy\\\",\\\"dur\\\":1.0}\"}");
  Serial.println("  Example (multi):  {\"cmd\":\"favSave\",\"slot\":2,\"script\":\"{\\\"cmd\\\":\\\"queue\\\",\\\"mode\\\":\\\"on\\\"}\\\\n{\\\"cmd\\\":\\\"set\\\",\\\"axis\\\":\\\"x\\\",\\\"value\\\":-60,\\\"dur\\\":2}\\\\n{\\\"cmd\\\":\\\"set\\\",\\\"axis\\\":\\\"x\\\",\\\"value\\\":60,\\\"dur\\\":2}\"}");

  Serial.println("\nfavRun");
  Serial.println("  Run the saved favorite script in slot 1..5.");
  Serial.println("  Temporarily sets queue mode to 'on' so motion commands enqueue.");
  Serial.println("  After all lines execute, queue mode is restored.");
  Serial.println("  The queued motions then execute in sequence.");
  Serial.println("  Example: {\"cmd\":\"favRun\",\"slot\":2}");
  Serial.println("  Tip: Include {\"cmd\":\"stopAll\"} at end of script to auto-stop when done.");

  Serial.println("\nfavList");
  Serial.println("  List favorite slots and a short preview.");
  Serial.println("  Example: {\"cmd\":\"favList\"}");

  Serial.println("\nfavClear");
  Serial.println("  Clear a favorite slot, or all slots with slot=0.");
  Serial.println("  Example: {\"cmd\":\"favClear\",\"slot\":2}");
  Serial.println("  Example: {\"cmd\":\"favClear\",\"slot\":0}");

  Serial.println("\nqueue");
  Serial.println("  Set queue mode: off|on|step.");
  Serial.println("    off : execute immediately unless q:true");
  Serial.println("    on  : enqueue by default unless q:false");
  Serial.println("    step: enqueue only when q:true (default)");
  Serial.println("  Example: {\"cmd\":\"queue\",\"mode\":\"on\"}");

  Serial.println("\nqAdd");
  Serial.println("  Explicitly enqueue a motion command. Uses cmd2=set|adjust|center.");
  Serial.println("  Example: {\"cmd\":\"qAdd\",\"cmd2\":\"set\",\"axis\":\"x\",\"value\":-90,\"dur\":5}");

  Serial.println("\nqClear");
  Serial.println("  Clear queued steps (does not stop current motion).");
  Serial.println("  Example: {\"cmd\":\"qClear\"}");

  Serial.println("\nqAbort");
  Serial.println("  Stop all motion + clear queue (hard escape).");
  Serial.println("  Example: {\"cmd\":\"qAbort\"}");

  Serial.println("\nqStatus");
  Serial.println("  Return queue mode/count/active.");
  Serial.println("  Example: {\"cmd\":\"qStatus\"}");

  Serial.println("\nqList");
  Serial.println("  List queued items (compact).");
  Serial.println("  Example: {\"cmd\":\"qList\"}");

  Serial.println("\nsweep");
  Serial.println("  Enqueue a sweep macro expanded into queue steps.");
  Serial.println("  Fields: axis, from, to, dur, loops(optional), dwell(optional), q(optional).");
  Serial.println("  Example: {\"cmd\":\"sweep\",\"axis\":\"x\",\"from\":-80,\"to\":80,\"dur\":6,\"loops\":2,\"dwell\":0.2,\"q\":true}");

  Serial.println("\npersist");
  Serial.println("  Write config to flash (NVS wear-leveled). Only writes if changed.");
  Serial.println("  Saves: speed, invert flags, position favorites, command favorites.");
  Serial.println("  Example: {\"cmd\":\"persist\"}");

  Serial.println("\nfactoryReset");
  Serial.println("  Erase saved config in flash and restore defaults.");
  Serial.println("  Example: {\"cmd\":\"factoryReset\"}");
}

static void printExamples() {
  Serial.println("Examples (NO id):");
  Serial.println("{\"cmd\":\"commands\"}");
  Serial.println("{\"cmd\":\"status\"}");
  Serial.println("{\"cmd\":\"speed\",\"value\":120}");
  Serial.println("{\"cmd\":\"center\",\"axis\":\"xy\",\"dur\":1.0}");
  Serial.println("{\"cmd\":\"set\",\"axis\":\"x\",\"value\":45,\"dur\":0.7}");
  Serial.println("{\"cmd\":\"adjust\",\"axis\":\"y\",\"value\":-10,\"speed\":120}");
  Serial.println("{\"cmd\":\"invert\",\"axis\":\"x\"}");
  Serial.println("{\"cmd\":\"save\",\"slot\":1}");
  Serial.println("{\"cmd\":\"recall\",\"slot\":1,\"dur\":1.2}");
  Serial.println();

  Serial.println("Queue sequences:");
  Serial.println("{\"cmd\":\"queue\",\"mode\":\"on\"}");
  Serial.println("{\"cmd\":\"set\",\"axis\":\"x\",\"value\":-60,\"dur\":1.5}");
  Serial.println("{\"cmd\":\"set\",\"axis\":\"x\",\"value\":60,\"dur\":1.5}");
  Serial.println("{\"cmd\":\"set\",\"axis\":\"xy\",\"x\":0,\"y\":-20,\"dur\":1.0}");
  Serial.println("{\"cmd\":\"stopAll\"}");
  Serial.println();

  Serial.println("Sweep:");
  Serial.println("{\"cmd\":\"queue\",\"mode\":\"step\"}");
  Serial.println("{\"cmd\":\"sweep\",\"axis\":\"x\",\"from\":-80,\"to\":80,\"dur\":6,\"loops\":2,\"dwell\":0.2,\"q\":true}");
  Serial.println();

  Serial.println("Command favorites (macros):");
  Serial.println("{\"cmd\":\"favSave\",\"slot\":1,\"line\":\"{\\\"cmd\\\":\\\"center\\\",\\\"axis\\\":\\\"xy\\\",\\\"dur\\\":1.0}\"}");
  Serial.println("{\"cmd\":\"favSave\",\"slot\":2,\"script\":\"{\\\"cmd\\\":\\\"queue\\\",\\\"mode\\\":\\\"on\\\"}\\\\n{\\\"cmd\\\":\\\"set\\\",\\\"axis\\\":\\\"x\\\",\\\"value\\\":-60,\\\"dur\\\":1.5}\\\\n{\\\"cmd\\\":\\\"set\\\",\\\"axis\\\":\\\"x\\\",\\\"value\\\":60,\\\"dur\\\":1.5}\\\\n{\\\"cmd\\\":\\\"stopAll\\\"}\"}");
  Serial.println("{\"cmd\":\"favRun\",\"slot\":2}");
  Serial.println("{\"cmd\":\"favList\"}");
  Serial.println("{\"cmd\":\"favClear\",\"slot\":2}");
  Serial.println();

  Serial.println("Persistence:");
  Serial.println("{\"cmd\":\"persist\"}");
  Serial.println("{\"cmd\":\"factoryReset\"}");
}

// ------------------- Persistence Implementation -------------------
static void applyDefaults() {
  defaultSpeed = 90.0f;
  invX = false;
  invY = false;

  for (int i=0;i<POS_FAV_SLOTS;i++) {
    posFavValid[i] = false;
    posFavX[i] = 0;
    posFavY[i] = 0;
  }
  for (int i=0;i<CMD_FAV_SLOTS;i++) {
    cmdFavValid[i] = false;
    cmdFavScript[i] = "";
  }
  cfgDirty = false;
}

static PersistedConfig makePersistedConfig() {
  PersistedConfig cfg{};
  cfg.magic = CFG_MAGIC;
  cfg.version = CFG_VERSION;
  cfg.defaultSpeed = defaultSpeed;
  cfg.invX = invX ? 1 : 0;
  cfg.invY = invY ? 1 : 0;

  uint8_t mask = 0;
  for (int i=0;i<POS_FAV_SLOTS;i++) {
    if (posFavValid[i]) mask |= (1u << i);
    cfg.posX[i] = posFavX[i];
    cfg.posY[i] = posFavY[i];
  }
  cfg.posValidMask = mask;

  cfg.crc32 = 0;
  uint32_t crc = crc32_update(0, (const uint8_t*)&cfg, sizeof(PersistedConfig));
  cfg.crc32 = crc;
  return cfg;
}

static bool loadConfigFromFlash() {
  prefs.begin(PREF_NS, true);

  PersistedConfig cfg{};
  size_t n = prefs.getBytes("cfg", &cfg, sizeof(cfg));
  if (n != sizeof(cfg) || cfg.magic != CFG_MAGIC || cfg.version != CFG_VERSION) {
    prefs.end();
    return false;
  }

  uint32_t storedCrc = cfg.crc32;
  cfg.crc32 = 0;
  uint32_t calcCrc = crc32_update(0, (const uint8_t*)&cfg, sizeof(PersistedConfig));

  if (calcCrc != storedCrc) {
    prefs.end();
    return false;
  }

  defaultSpeed = cfg.defaultSpeed;
  if (defaultSpeed < 0.1f || defaultSpeed > 1000.0f) defaultSpeed = 90.0f;

  invX = (cfg.invX != 0);
  invY = (cfg.invY != 0);

  for (int i=0;i<POS_FAV_SLOTS;i++) {
    bool valid = (cfg.posValidMask & (1u << i)) != 0;
    posFavValid[i] = valid;
    posFavX[i] = clampf(cfg.posX[i], POS_MIN, POS_MAX);
    posFavY[i] = clampf(cfg.posY[i], POS_MIN, POS_MAX);
  }

  // Load command favorites (stored as strings)
  for (int i=0;i<CMD_FAV_SLOTS;i++) {
    String key = "fav";
    key += (i+1);
    String s = prefs.getString(key.c_str(), "");
    if (s.length() > 0) {
      cmdFavValid[i] = true;
      if (s.length() > FAV_SCRIPT_MAX) s = s.substring(0, FAV_SCRIPT_MAX);
      cmdFavScript[i] = s;
    } else {
      cmdFavValid[i] = false;
      cmdFavScript[i] = "";
    }
  }

  prefs.end();
  cfgDirty = false;
  return true;
}

static bool persistToFlash(String& why) {
  if (!cfgDirty) {
    why = "no_changes";
    return true;
  }

  PersistedConfig cfg = makePersistedConfig();

  prefs.begin(PREF_NS, false);

  bool ok = true;
  ok &= (prefs.putBytes("cfg", &cfg, sizeof(cfg)) == sizeof(cfg));

  // Save command favorites (strings)
  for (int i=0;i<CMD_FAV_SLOTS;i++) {
    String key = "fav";
    key += (i+1);
    if (cmdFavValid[i] && cmdFavScript[i].length() > 0) {
      String s = cmdFavScript[i];
      if (s.length() > FAV_SCRIPT_MAX) s = s.substring(0, FAV_SCRIPT_MAX);
      ok &= prefs.putString(key.c_str(), s) > 0;
    } else {
      prefs.remove(key.c_str());
    }
  }

  prefs.end();

  if (ok) {
    cfgDirty = false;
    why = "persisted";
    return true;
  }

  why = "write_failed";
  return false;
}

static bool factoryResetFlash(String& why) {
  prefs.begin(PREF_NS, false);
  bool ok = prefs.clear();
  prefs.end();

  applyDefaults();
  why = ok ? "factory_reset" : "clear_failed";
  return ok;
}

// ------------------- Macro Runner -------------------
// Guard to prevent runaway recursion
static bool macroRunning = false;

// Saved queue mode before macro started (restore after)
static QueueMode qModeSavedForMacro = Q_STEP;

static bool shouldEnqueue(QueueMode mode, bool hasQ, bool qVal) {
  // During macro execution: motion commands enqueue unless explicitly q:false
  if (macroRunning) {
    // If user explicitly says q:false, execute immediately
    if (hasQ && !qVal) return false;
    // Otherwise enqueue so macro can batch motions
    return true;
  }

  if (mode == Q_ON)  return !(hasQ && qVal == false);
  if (mode == Q_OFF) return (hasQ && qVal == true);
  return (hasQ && qVal == true); // Q_STEP
}

// Reject saving dangerous commands into favorites
static bool looksDangerousFavorite(const String& line) {
  // minimal checks: disallow persist, factoryReset, favRun
  String t = line;
  t.toLowerCase();
  if (t.indexOf("\"cmd\":\"persist\"") >= 0) return true;
  if (t.indexOf("\"cmd\":\"factoryreset\"") >= 0) return true;
  if (t.indexOf("\"cmd\":\"favrun\"") >= 0) return true;
  return false;
}

// Forward declare handler
static void handleCommandLine(String line);

static bool runFavoriteScript(uint32_t id, const String& subsystem, const String& route, const String& scriptRaw) {
  if (macroRunning) {
    sendErr(id, subsystem, route, "macro_busy", "A macro is already running");
    return false;
  }

  // Save current queue mode and switch to "on" so motion commands enqueue
  qModeSavedForMacro = qMode;
  qMode = Q_ON;
  macroRunning = true;

  String script = scriptRaw; // already unescaped when saved
  // Execute each non-empty line
  int start = 0;
  int steps = 0;
  while (start < (int)script.length()) {
    int end = script.indexOf('\n', start);
    if (end < 0) end = script.length();
    String line = script.substring(start, end);
    line.trim();
    start = end + 1;

    if (!line.length()) continue;
    steps++;
    if (steps > 50) {
      sendErr(id, subsystem, route, "macro_too_long", "Macro step limit exceeded (50)");
      macroRunning = false;
      qMode = qModeSavedForMacro;
      return false;
    }

    // Execute the line (motion commands will enqueue)
    handleCommandLine(line);
    Serial.flush();
  }

  macroRunning = false;
  // Restore original queue mode
  qMode = qModeSavedForMacro;
  return true;
}

// ------------------- Scheduler -------------------
static void maybeStartNextQueuedStep() {
  if (qActive) return;
  if (mx.active || my.active) return;
  if (qIsEmpty()) return;

  if (!qDequeue(qCurrent)) return;
  qActive = true;
  qStartedAt = millis();

  uint32_t maxDur = 0;
  if (qCurrent.useX && qCurrent.dx > maxDur) maxDur = qCurrent.dx;
  if (qCurrent.useY && qCurrent.dy > maxDur) maxDur = qCurrent.dy;
  qCurrent.expectedEnd = qStartedAt + maxDur + STEP_TIMEOUT_GRACE_MS;

  sendEventStarted(qCurrent);
  executeStep(qCurrent);

  if (!mx.active && !my.active && qCurXDone && qCurYDone) {
    sendEventStepDone(qCurrent);
    qActive = false;
  }
}

static void updateMotion() {
  const uint32_t now = millis();

  if (mx.active) {
    uint32_t dt = now - mx.t0;
    if (dt >= mx.durMs) {
      v1 = mx.target;
      mx.active = false;
      applyOutputs();
      sendEventDoneAxis('x', mx.cmdRef, qActive ? qCurrent.subsystem : lastSubsystem, qActive ? qCurrent.route : lastRoute);
      if (qActive && qCurrent.useX) qCurXDone = true;
    } else {
      float t = (float)dt / (float)mx.durMs;
      v1 = mx.start + (mx.target - mx.start) * t;
      applyOutputs();
    }
  }

  if (my.active) {
    uint32_t dt = now - my.t0;
    if (dt >= my.durMs) {
      v2 = my.target;
      my.active = false;
      applyOutputs();
      sendEventDoneAxis('y', my.cmdRef, qActive ? qCurrent.subsystem : lastSubsystem, qActive ? qCurrent.route : lastRoute);
      if (qActive && qCurrent.useY) qCurYDone = true;
    } else {
      float t = (float)dt / (float)my.durMs;
      v2 = my.start + (my.target - my.start) * t;
      applyOutputs();
    }
  }

  if (qActive) {
    if (qCurrent.expectedEnd != 0 && now > qCurrent.expectedEnd) {
      sendEventFault(qCurrent.subsystem, qCurrent.route, "step_timeout", qCurrent.id, "Queued step timed out; aborted");
      abortQueueAndMotion();
      applyOutputs();
      return;
    }

    bool moving = mx.active || my.active;
    if (!moving && qCurXDone && qCurYDone) {
      sendEventStepDone(qCurrent);
      qActive = false;
    }
  }

  maybeStartNextQueuedStep();
}

// ------------------- Command Handler -------------------
static void handleCommandLine(String line) {
  line.trim();
  if (!line.length()) return;

  uint32_t id = 0;
  int idInt = 0;
  if (getIntField(line, "id", idInt) && idInt > 0) id = (uint32_t)idInt;
  if (id == 0) { autoId++; id = autoId; }

  String subsystem="", route="";
  (void)getStringField(line, "subsystem", subsystem);
  (void)getStringField(line, "route", route);
  if (subsystem.length()) lastSubsystem = subsystem;
  if (route.length()) lastRoute = route;

  String cmd;
  if (!getStringField(line, "cmd", cmd)) {
    sendErr(id, subsystem, route, "missing_cmd", "Missing required field: cmd (example: {\"cmd\":\"help\"})");
    return;
  }
  cmd.toLowerCase();

  bool qVal=false, hasQ=false;
  if (getBoolField(line, "q", qVal)) hasQ = true;

  // ---- informational ----
  if (cmd == "commands") { sendOk(id, subsystem, route, "commands"); printCommands(); return; }
  if (cmd == "help")     { sendOk(id, subsystem, route, "help");     printHelp();     return; }
  if (cmd == "examples") { sendOk(id, subsystem, route, "examples"); printExamples(); return; }
  if (cmd == "status")   { sendOk(id, subsystem, route, "status");   sendState(nullptr, 0, subsystem, route); return; }

  // ---- persistence ----
  if (cmd == "persist") {
    String why;
    bool ok = persistToFlash(why);
    if (!ok) { sendErr(id, subsystem, route, "persist_failed", why.c_str()); return; }
    sendOk(id, subsystem, route, why.c_str());
    sendState("done", id, subsystem, route);
    return;
  }

  if (cmd == "factoryreset") {
    String why;
    bool ok = factoryResetFlash(why);
    if (!ok) { sendErr(id, subsystem, route, "factory_reset_failed", why.c_str()); return; }
    v1 = 0; v2 = 0;
    abortQueueAndMotion();
    applyOutputs();
    sendOk(id, subsystem, route, why.c_str());
    sendState("done", id, subsystem, route);
    return;
  }

  // ---- queue mode ----
  if (cmd == "queue") {
    String mode;
    if (!getStringField(line, "mode", mode)) { sendErr(id, subsystem, route, "missing_mode", "queue requires mode: off|on|step"); return; }
    mode.toLowerCase();
    if (mode == "off") qMode = Q_OFF;
    else if (mode == "on") qMode = Q_ON;
    else if (mode == "step") qMode = Q_STEP;
    else { sendErr(id, subsystem, route, "bad_mode", "mode must be off|on|step"); return; }
    sendOk(id, subsystem, route, "queue_mode_set");
    sendState("done", id, subsystem, route);
    return;
  }

  if (cmd == "qclear") { qClearAll(); sendOk(id, subsystem, route, "queue_cleared"); sendState("done", id, subsystem, route); return; }

  if (cmd == "qabort") {
    abortQueueAndMotion();
    applyOutputs();
    sendOk(id, subsystem, route, "aborted_all");
    sendState("done", id, subsystem, route);
    return;
  }

  if (cmd == "qstatus") { sendOk(id, subsystem, route, "queue_status"); sendState(nullptr, 0, subsystem, route); return; }

  if (cmd == "qlist") {
    Serial.print("{\"ok\":true,\"id\":"); Serial.print(id);
    printRoutingFields(subsystem, route);
    Serial.print(",\"queue\":{\"mode\":\"");
    Serial.print(qMode==Q_OFF?"off":(qMode==Q_ON?"on":"step"));
    Serial.print("\",\"count\":"); Serial.print(qCount);
    Serial.print(",\"items\":[");
    for (uint8_t i=0;i<qCount;i++) {
      uint8_t idx = (uint8_t)((qHead + i) % QMAX);
      const QueueItem& it = q[idx];
      if (i) Serial.print(",");
      Serial.print("{\"ref\":"); Serial.print(it.id);
      Serial.print(",\"kind\":\""); Serial.print(it.kind); Serial.print("\"");
      Serial.print(",\"axis\":\"");
      if (it.useX && it.useY) Serial.print("xy");
      else if (it.useX) Serial.print("x");
      else Serial.print("y");
      Serial.print("\"");
      if (it.useX) { Serial.print(",\"x\":"); Serial.print(it.tx, 2); }
      if (it.useY) { Serial.print(",\"y\":"); Serial.print(it.ty, 2); }
      Serial.print(",\"dx\":"); Serial.print(it.dx);
      Serial.print(",\"dy\":"); Serial.print(it.dy);
      Serial.print("}");
    }
    Serial.print("]}}");
    Serial.println("}");
    return;
  }

  if (cmd == "qadd") {
    String cmd2;
    if (!getStringField(line, "cmd2", cmd2)) { sendErr(id, subsystem, route, "missing_cmd2", "qAdd requires cmd2"); return; }
    cmd2.toLowerCase();

    String axis="xy";
    (void)getStringField(line, "axis", axis);

    bool ok=false; String ec, em;
    QueueItem it = buildStepFromCommand(id, subsystem, route, cmd2, axis, line, ok, ec, em);
    if (!ok) { sendErr(id, subsystem, route, ec.c_str(), em.c_str()); return; }
    if (!qEnqueue(it)) { sendErr(id, subsystem, route, "queue_full", "Queue full"); return; }
    sendOk(id, subsystem, route, "queued");
    sendState(nullptr, 0, subsystem, route);
    return;
  }

  // ---- motion control ----
  if (cmd == "stop") {
    String axis="xy"; (void)getStringField(line, "axis", axis);
    bool useX=false,useY=false;
    if (!parseAxisMask(axis, useX, useY)) { sendErr(id, subsystem, route, "bad_axis", "axis must be x, y, or xy"); return; }
    if (useX) stopX();
    if (useY) stopY();
    applyOutputs();
    sendOk(id, subsystem, route, "stopped");
    sendState("done", id, subsystem, route);
    return;
  }

  if (cmd == "stopall") {
    bool flush = true;
    (void)getBoolField(line, "flush", flush);
    stopAllMotion();
    if (flush) { qClearAll(); qActive = false; }
    applyOutputs();
    sendOk(id, subsystem, route, flush ? "stopped_all_flushed" : "stopped_all");
    sendState("done", id, subsystem, route);
    return;
  }

  if (cmd == "resetall") {
    abortQueueAndMotion();
    v1 = 0; v2 = 0;
    applyOutputs();
    sendOk(id, subsystem, route, "reset_all_runtime");
    sendState("done", id, subsystem, route);
    return;
  }

  if (cmd == "invert") {
    String axis="xy"; (void)getStringField(line, "axis", axis);
    bool useX=false,useY=false;
    if (!parseAxisMask(axis, useX, useY)) { sendErr(id, subsystem, route, "bad_axis", "axis must be x, y, or xy"); return; }
    if (useX) toggleInvertX();
    if (useY) toggleInvertY();
    applyOutputs();
    sendOk(id, subsystem, route, "invert_toggled");
    sendState("done", id, subsystem, route);
    return;
  }

  if (cmd == "speed") {
    float sp;
    if (!getNumberField(line, "value", sp)) { sendErr(id, subsystem, route, "missing_value", "speed requires value (deg/sec)"); return; }
    if (sp < 0.1f || sp > 1000.0f) { sendErr(id, subsystem, route, "bad_value", "speed out of range"); return; }
    defaultSpeed = sp;
    cfgDirty = true;
    sendOk(id, subsystem, route, "speed_set");
    sendState("done", id, subsystem, route);
    return;
  }

  // ---- position favorites ----
  if (cmd == "save") {
    int slot=0;
    if (!getIntField(line, "slot", slot) || slot < 1 || slot > POS_FAV_SLOTS) { sendErr(id, subsystem, route, "bad_slot", "save requires slot 1..5"); return; }
    int idx = slot - 1;
    posFavValid[idx] = true;
    posFavX[idx] = v1;
    posFavY[idx] = v2;
    cfgDirty = true;
    sendOk(id, subsystem, route, "saved_position");
    sendState("done", id, subsystem, route);
    return;
  }

  if (cmd == "recall") {
    int slot=0;
    if (!getIntField(line, "slot", slot) || slot < 1 || slot > POS_FAV_SLOTS) { sendErr(id, subsystem, route, "bad_slot", "recall requires slot 1..5"); return; }
    int idx = slot - 1;
    if (!posFavValid[idx]) { sendErr(id, subsystem, route, "empty_slot", "slot not saved yet"); return; }

    String axis="xy"; (void)getStringField(line, "axis", axis);
    bool useX=false,useY=false;
    if (!parseAxisMask(axis, useX, useY)) { sendErr(id, subsystem, route, "bad_axis", "axis must be x, y, or xy"); return; }

    float tx = posFavX[idx];
    float ty = posFavY[idx];

    float durSec=-1, sp=-1;
    bool hasDur = getNumberField(line, "dur", durSec);
    bool hasSpeed = getNumberField(line, "speed", sp);

    uint32_t dx=0, dy=0;
    if (!computeDurations(useX,useY,tx,ty,hasDur,durSec,hasSpeed,sp,dx,dy)) { sendErr(id, subsystem, route, "bad_timing", "Invalid dur or speed"); return; }

    QueueItem it;
    it.id = id; it.subsystem=subsystem; it.route=route; it.kind="recall";
    it.useX=useX; it.useY=useY; it.tx=tx; it.ty=ty; it.dx=dx; it.dy=dy;

    bool enqueue = shouldEnqueue(qMode, hasQ, qVal);
    if (enqueue) {
      if (!qEnqueue(it)) { sendErr(id, subsystem, route, "queue_full", "Queue full"); return; }
      sendOk(id, subsystem, route, "queued");
      sendState(nullptr, 0, subsystem, route);
      return;
    }

    executeStep(it);
    sendOk(id, subsystem, route, "executing");
    return;
  }

  // ---- command favorites ----
  if (cmd == "favlist") {
    Serial.print("{\"ok\":true,\"id\":"); Serial.print(id);
    printRoutingFields(subsystem, route);
    Serial.print(",\"favorites\":[");
    for (int i=0;i<CMD_FAV_SLOTS;i++) {
      if (i) Serial.print(",");
      Serial.print("{\"slot\":"); Serial.print(i+1);
      Serial.print(",\"valid\":"); Serial.print(cmdFavValid[i] ? "true":"false");
      if (cmdFavValid[i]) {
        String preview = cmdFavScript[i];
        preview.replace("\n", "\\n");
        if (preview.length() > 120) preview = preview.substring(0, 120) + "...";
        Serial.print(",\"preview\":\""); Serial.print(preview); Serial.print("\"");
      }
      Serial.print("}");
    }
    Serial.println("]}");
    return;
  }

  if (cmd == "favclear") {
    int slot=0;
    if (!getIntField(line, "slot", slot) || slot < 0 || slot > CMD_FAV_SLOTS) {
      sendErr(id, subsystem, route, "bad_slot", "favClear requires slot 0..5 (0 clears all)");
      return;
    }
    if (slot == 0) {
      for (int i=0;i<CMD_FAV_SLOTS;i++) { cmdFavValid[i]=false; cmdFavScript[i]=""; }
      cfgDirty = true;
      sendOk(id, subsystem, route, "fav_cleared_all");
      sendState("done", id, subsystem, route);
      return;
    }
    int idx = slot - 1;
    cmdFavValid[idx] = false;
    cmdFavScript[idx] = "";
    cfgDirty = true;
    sendOk(id, subsystem, route, "fav_cleared");
    sendState("done", id, subsystem, route);
    return;
  }

  if (cmd == "favsave") {
    int slot=0;
    if (!getIntField(line, "slot", slot) || slot < 1 || slot > CMD_FAV_SLOTS) { sendErr(id, subsystem, route, "bad_slot", "favSave requires slot 1..5"); return; }
    int idx = slot - 1;

    String raw;
    bool hasLine = getStringField(line, "line", raw);
    bool hasScript = getStringField(line, "script", raw);

    if (!hasLine && !hasScript) {
      sendErr(id, subsystem, route, "missing_value", "favSave requires \"line\" or \"script\"");
      return;
    }

    String script = unescapeScript(raw);
    script.trim();
    if (!script.length()) { sendErr(id, subsystem, route, "empty_script", "Provided line/script is empty"); return; }
    if (script.length() > FAV_SCRIPT_MAX) { sendErr(id, subsystem, route, "too_long", "Script too long"); return; }

    // safety check on stored lines
    // check each line for disallowed commands
    String check = script;
    int start = 0;
    while (start < (int)check.length()) {
      int end = check.indexOf('\n', start);
      if (end < 0) end = check.length();
      String one = check.substring(start, end);
      one.trim();
      start = end + 1;
      if (!one.length()) continue;
      if (looksDangerousFavorite(one)) {
        sendErr(id, subsystem, route, "disallowed", "Favorite cannot include persist/factoryReset/favRun");
        return;
      }
    }

    cmdFavValid[idx] = true;
    cmdFavScript[idx] = script;
    cfgDirty = true;
    sendOk(id, subsystem, route, "fav_saved");
    sendState("done", id, subsystem, route);
    return;
  }

  if (cmd == "favrun") {
    int slot=0;
    if (!getIntField(line, "slot", slot) || slot < 1 || slot > CMD_FAV_SLOTS) { sendErr(id, subsystem, route, "bad_slot", "favRun requires slot 1..5"); return; }
    int idx = slot - 1;
    if (!cmdFavValid[idx] || !cmdFavScript[idx].length()) { sendErr(id, subsystem, route, "empty_slot", "favorite slot is empty"); return; }

    sendOk(id, subsystem, route, "macro_running");
    bool ok = runFavoriteScript(id, subsystem, route, cmdFavScript[idx]);
    if (ok) {
      sendOk(id, subsystem, route, "macro_complete");
      sendState("done", id, subsystem, route);
    }
    return;
  }

  // ---- sweep macro ----
  if (cmd == "sweep") {
    String axis="x"; (void)getStringField(line, "axis", axis);
    bool useX=false,useY=false;
    if (!parseAxisMask(axis, useX, useY)) { sendErr(id, subsystem, route, "bad_axis", "axis must be x, y, or xy"); return; }

    float from=0, to=0;
    if (!getNumberField(line, "from", from) || !getNumberField(line, "to", to)) {
      sendErr(id, subsystem, route, "missing_value", "sweep requires from and to");
      return;
    }
    from = clampf(from, POS_MIN, POS_MAX);
    to   = clampf(to,   POS_MIN, POS_MAX);

    float durSec=0;
    if (!getNumberField(line, "dur", durSec) || durSec <= 0.0f || durSec > 3600.0f) {
      sendErr(id, subsystem, route, "bad_dur", "sweep dur must be 0<dur<=3600 seconds");
      return;
    }

    int loops=1; (void)getIntField(line, "loops", loops);
    if (loops < 0) loops = 0;
    if (loops > 1000000) loops = 1000000;

    float dwellSec=0.0f; (void)getNumberField(line, "dwell", dwellSec);
    if (dwellSec < 0.0f) dwellSec = 0.0f;
    if (dwellSec > 60.0f) dwellSec = 60.0f;

    // Always enqueues; if you want it immediate, set queue mode off and q:false then use set/adjust loops yourself.
    uint32_t legMs = (uint32_t)(durSec * 1000.0f + 0.5f);
    uint32_t dwellMs = (uint32_t)(dwellSec * 1000.0f + 0.5f);

    int cycles = loops;
    if (cycles == 0) cycles = 1000; // chunk; stopAll/qAbort stops it

    // Estimate queue need: move-to-from + (to+from)*cycles + optional dwells
    int baseSteps = 1 + cycles * 2;
    int dwellSteps = (dwellMs > 0) ? (1 + cycles * 2) : 0;
    int totalSteps = baseSteps + dwellSteps;
    if (totalSteps > (int)QMAX - (int)qCount) { sendErr(id, subsystem, route, "queue_full", "Not enough queue space for sweep steps"); return; }

    float fromX = useX ? from : v1;
    float fromY = useY ? from : v2;
    float toX   = useX ? to   : v1;
    float toY   = useY ? to   : v2;

    // Move to FROM first (speed-derived)
    QueueItem moveToFrom;
    moveToFrom.id = id; moveToFrom.subsystem=subsystem; moveToFrom.route=route;
    moveToFrom.kind="sweepToFrom"; moveToFrom.useX=useX; moveToFrom.useY=useY;
    moveToFrom.tx=fromX; moveToFrom.ty=fromY;
    moveToFrom.dx = useX ? durationFromSpeed(v1, fromX, defaultSpeed) : 0;
    moveToFrom.dy = useY ? durationFromSpeed(v2, fromY, defaultSpeed) : 0;
    if (!qEnqueue(moveToFrom)) { sendErr(id, subsystem, route, "queue_full", "Queue full"); return; }

    auto enqueueHold = [&](float hx, float hy) -> bool {
      if (dwellMs == 0) return true;
      QueueItem hold;
      hold.id = ++autoId; hold.subsystem=subsystem; hold.route=route;
      hold.kind="dwell"; hold.useX=useX; hold.useY=useY;
      hold.tx=hx; hold.ty=hy;
      hold.dx = useX ? dwellMs : 0;
      hold.dy = useY ? dwellMs : 0;
      return qEnqueue(hold);
    };

    if (!enqueueHold(fromX, fromY)) { sendErr(id, subsystem, route, "queue_full", "Queue full"); return; }

    for (int c=0; c<cycles; c++) {
      QueueItem legTo;
      legTo.id=++autoId; legTo.subsystem=subsystem; legTo.route=route;
      legTo.kind="sweepTo"; legTo.useX=useX; legTo.useY=useY;
      legTo.tx=toX; legTo.ty=toY;
      legTo.dx = useX ? legMs : 0;
      legTo.dy = useY ? legMs : 0;
      if (!qEnqueue(legTo)) { sendErr(id, subsystem, route, "queue_full", "Queue full"); return; }
      if (!enqueueHold(toX, toY)) { sendErr(id, subsystem, route, "queue_full", "Queue full"); return; }

      QueueItem legFrom;
      legFrom.id=++autoId; legFrom.subsystem=subsystem; legFrom.route=route;
      legFrom.kind="sweepFrom"; legFrom.useX=useX; legFrom.useY=useY;
      legFrom.tx=fromX; legFrom.ty=fromY;
      legFrom.dx = useX ? legMs : 0;
      legFrom.dy = useY ? legMs : 0;
      if (!qEnqueue(legFrom)) { sendErr(id, subsystem, route, "queue_full", "Queue full"); return; }
      if (!enqueueHold(fromX, fromY)) { sendErr(id, subsystem, route, "queue_full", "Queue full"); return; }
    }

    sendOk(id, subsystem, route, (loops==0) ? "sweep_queued_chunk" : "sweep_queued");
    sendState(nullptr, 0, subsystem, route);
    return;
  }

  // ---- motion commands ----
  if (cmd == "set" || cmd == "adjust" || cmd == "center") {
    String axis="xy"; (void)getStringField(line, "axis", axis);
    bool ok=false; String ec, em;
    QueueItem it = buildStepFromCommand(id, subsystem, route, cmd, axis, line, ok, ec, em);
    if (!ok) { sendErr(id, subsystem, route, ec.c_str(), em.c_str()); return; }

    bool enqueue = shouldEnqueue(qMode, hasQ, qVal);
    if (enqueue) {
      if (!qEnqueue(it)) { sendErr(id, subsystem, route, "queue_full", "Queue full"); return; }
      sendOk(id, subsystem, route, "queued");
      sendState(nullptr, 0, subsystem, route);
      return;
    }

    executeStep(it);
    sendOk(id, subsystem, route, "executing");
    return;
  }

  sendErr(id, subsystem, route, "unknown_cmd", "Unknown cmd (try {\"cmd\":\"commands\"})");
}

// ------------------- Serial line buffer -------------------
static String lineBuf;

// ------------------- Setup / Loop -------------------
void setup() {
  Serial.begin(115200);
  delay(600);

  s1.setPeriodHertz(50);
  s2.setPeriodHertz(50);
  s1.attach(SERVO1_PIN, 500, 2400);
  s2.attach(SERVO2_PIN, 500, 2400);

  applyDefaults();
  bool loaded = loadConfigFromFlash();

  applyOutputs();

  Serial.println();
  Serial.println("PanTilt_JSON ready (JSONL over Serial).");
  Serial.println("Commands must be JSON objects containing \"cmd\" and end with newline.");
  Serial.println("Try:");
  Serial.println("  {\"cmd\":\"commands\"}");
  Serial.println("  {\"cmd\":\"help\"}");
  Serial.println("  {\"cmd\":\"examples\"}");
  Serial.println("  {\"cmd\":\"status\"}");
  Serial.println();
  Serial.print("Config loaded from flash: ");
  Serial.println(loaded ? "true" : "false");
  Serial.println("Queue mode default: step (enqueue only when \"q\":true).");
  Serial.println("Reminder: persist writes config to flash only when you call it.");
}

void loop() {
  updateMotion();

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      String line = lineBuf;
      lineBuf = "";
      handleCommandLine(line);
    } else {
      if (lineBuf.length() < CMD_LINE_MAX) {
        lineBuf += c;
      } else {
        lineBuf = "";
        autoId++;
        sendErr(autoId, "", "", "line_too_long", "Command line exceeded buffer limit");
      }
    }
  }
}
