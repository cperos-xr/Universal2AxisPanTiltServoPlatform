// ------------------------------------------------------------
// Servo Controller + Queue + Sweep (ESP32-C3 + ESP32Servo)
// Protocol: newline-delimited JSON ("JSONL") over Serial
//
// REQUIRED: each command must be a JSON object containing "cmd"
// Example: {"cmd":"help"}   (then press Enter / newline)
//
// Axes:
//   X -> Servo 1
//   Y -> Servo 2
//
// Positions are logical degrees [-90..+90] (0 = center).
// Optional "id" is a command correlation id (for bots), NOT servo id.
//
// ------------------------------------------------------------

#include <Arduino.h>
#include <ESP32Servo.h>

// ------------------- Pins (set to your C3 board) -------------------
Servo s1, s2;
#define SERVO1_PIN 3  // X
#define SERVO2_PIN 4  // Y

// ------------------- Calibrated safe ranges (us) -------------------
static const int S1_MIN_US = 500;
static const int S1_MAX_US = 2400;
static const int S2_MIN_US = 800;
static const int S2_MAX_US = 2050;

// ------------------- Limits -------------------
static const float POS_MIN = -90.0f;
static const float POS_MAX =  90.0f;

// Avoid macro collision with system LINE_MAX:
static const uint16_t CMD_LINE_MAX = 768;

// Queue sizing (fixed for predictability)
static const uint8_t QMAX = 20;

// Timeout guard for queued steps:
static const uint32_t STEP_TIMEOUT_GRACE_MS = 2000;

// ------------------- State -------------------
static float v1 = 0.0f; // logical X degrees
static float v2 = 0.0f; // logical Y degrees

static bool invX = false;
static bool invY = false;

// Default speed used when dur is not specified (deg/sec)
static float defaultSpeed = 90.0f;

// Favorites 1..5
static bool favValid[5] = {false,false,false,false,false};
static float favX[5] = {0,0,0,0,0};
static float favY[5] = {0,0,0,0,0};

// Per-axis motion
struct MoveProfile {
  bool active = false;
  float start = 0;
  float target = 0;
  uint32_t t0 = 0;
  uint32_t durMs = 0;
  uint32_t cmdRef = 0;   // ref/id for done event
  char axis = '?';       // 'x' or 'y'
};

static MoveProfile mx, my;

// ------------------- Routing/Extensibility -------------------
// Optional fields echoed back so a master script can demux responses.
static String lastSubsystem = "";
static String lastRoute = "";

// ------------------- Queue -------------------
enum QueueMode : uint8_t { Q_OFF=0, Q_ON=1, Q_STEP=2 };
static QueueMode qMode = Q_STEP;

struct QueueItem {
  bool used = false;

  uint32_t id = 0;          // command id (or auto)
  String subsystem;
  String route;

  // "set"/"adjust"/"center"/"recall"/"sweepTo"/etc.
  String kind;

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

// Currently executing queued step (if any)
static bool qActive = false;
static QueueItem qCurrent;
static bool qCurXDone = true;
static bool qCurYDone = true;
static uint32_t qStartedAt = 0;

// Auto-IDs if id missing
static uint32_t autoId = 0;

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
static void toggleInvertX() { invX = !invX; v1 = -v1; }
static void toggleInvertY() { invY = !invY; v2 = -v2; }

// ------------------- Minimal JSON helpers -------------------
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
  int q1 = s.indexOf('\"', colon + 1);
  if (q1 < 0) return false;
  int q2 = s.indexOf('\"', q1 + 1);
  if (q2 < 0) return false;
  out = s.substring(q1 + 1, q2);
  return true;
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

// ------------------- Reply helpers -------------------
static void printRoutingFields(const String& subsystem, const String& route) {
  if (subsystem.length()) { Serial.print(",\"subsystem\":\""); Serial.print(subsystem); Serial.print("\""); }
  if (route.length())     { Serial.print(",\"route\":\"");     Serial.print(route);     Serial.print("\""); }
}

static void sendOk(uint32_t id, const String& subsystem, const String& route, const char* msg) {
  Serial.print("{\"ok\":true,\"id\":"); Serial.print(id);
  printRoutingFields(subsystem, route);
  Serial.print(",\"msg\":\""); Serial.print(msg); Serial.println("\"}");
}

static void sendErr(uint32_t id, const String& subsystem, const String& route, const char* code, const char* msg) {
  Serial.print("{\"ok\":false,\"id\":"); Serial.print(id);
  printRoutingFields(subsystem, route);
  Serial.print(",\"error\":\""); Serial.print(code);
  Serial.print("\",\"msg\":\""); Serial.print(msg); Serial.println("\"}");
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

  Serial.print("}}");
  Serial.println("}");
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
}

static void sendEventDoneAxis(char axis, uint32_t ref, const String& subsystem, const String& route) {
  Serial.print("{\"ok\":true,\"event\":\"done\",\"axis\":\""); Serial.print(axis);
  Serial.print("\",\"ref\":"); Serial.print(ref);
  printRoutingFields(subsystem, route);
  Serial.println("}");
}

static void sendEventStepDone(const QueueItem& it) {
  Serial.print("{\"ok\":true,\"event\":\"stepDone\",\"ref\":"); Serial.print(it.id);
  printRoutingFields(it.subsystem, it.route);
  Serial.println("}");
}

static void sendEventFault(const String& subsystem, const String& route, const char* code, uint32_t ref, const char* msg) {
  Serial.print("{\"ok\":false,\"event\":\"fault\",\"error\":\""); Serial.print(code);
  Serial.print("\",\"ref\":"); Serial.print(ref);
  printRoutingFields(subsystem, route);
  Serial.print(",\"msg\":\""); Serial.print(msg); Serial.println("\"}");
}

// ------------------- Queue operations -------------------
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

static void qClear() {
  for (uint8_t i=0;i<QMAX;i++) q[i].used = false;
  qHead = qTail = qCount = 0;
}

static void stopX() { mx.active = false; mx.durMs = 0; }
static void stopY() { my.active = false; my.durMs = 0; }
static void stopAllMotion() { stopX(); stopY(); }

static void abortQueueAndMotion() {
  stopAllMotion();
  qClear();
  qActive = false;
  qCurXDone = qCurYDone = true;
}

// Start a per-axis move (timed or instant)
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

// Execute one atomic step immediately
static void executeStep(const QueueItem& it) {
  stopAllMotion();

  qCurXDone = !it.useX || (it.dx == 0);
  qCurYDone = !it.useY || (it.dy == 0);

  if (it.useX) startMoveX(it.tx, it.dx, it.id);
  if (it.useY) startMoveY(it.ty, it.dy, it.id);

  applyOutputs();
}

// Start next queued item if idle
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

// ------------------- Help / Commands / Examples -------------------
static void printCommands() {
  Serial.println("Commands (JSON objects):");
  Serial.println("  {\"cmd\":\"help\"}      -> full reference (descriptions + examples)");
  Serial.println("  {\"cmd\":\"examples\"}  -> copy/paste examples");
  Serial.println("  {\"cmd\":\"status\"}    -> current state");
  Serial.println();
  Serial.println("Actions:");
  Serial.println("  set, adjust, center, stop, stopAll, resetAll, invert, speed, save, recall");
  Serial.println("Queue:");
  Serial.println("  queue, qAdd, qClear, qAbort, qStatus, qList");
  Serial.println("Macro:");
  Serial.println("  sweep");
}

static void printHelp() {
  Serial.println("Protocol: JSONL (one JSON object per line). Required field: \"cmd\".");
  Serial.println("Examples: {\"cmd\":\"help\"}  {\"cmd\":\"status\"}  {\"cmd\":\"speed\",\"value\":90}");
  Serial.println("id is optional and is a command correlation id (for bots), not a servo id.");
  Serial.println();
  Serial.println("Fields:");
  Serial.println("  axis: \"x\" | \"y\" | \"xy\"");
  Serial.println("  value: degrees (for x/y) or deg/sec (for speed)");
  Serial.println("  x,y: degrees when axis=\"xy\"");
  Serial.println("  dur: seconds (float). If omitted, duration computed from speed.");
  Serial.println("  speed: deg/sec override for that command");
  Serial.println("  q: true|false queue hint (depends on queue mode)");
  Serial.println("  subsystem/route: optional strings for master routing (echoed back)");
  Serial.println();

  Serial.println("COMMAND REFERENCE (each includes at least one example)");

  Serial.println("\ncommands");
  Serial.println("  Short list + quick-start hints.");
  Serial.println("  Example: {\"cmd\":\"commands\"}");

  Serial.println("\nhelp");
  Serial.println("  Full reference.");
  Serial.println("  Example: {\"cmd\":\"help\"}");

  Serial.println("\nexamples");
  Serial.println("  Prints ready-to-paste examples (no-id and with-id).");
  Serial.println("  Example: {\"cmd\":\"examples\"}");

  Serial.println("\nstatus");
  Serial.println("  Returns x/y, invert flags, speed, motion, queue state.");
  Serial.println("  Example: {\"cmd\":\"status\"}");

  Serial.println("\nset");
  Serial.println("  Set absolute target position(s) in degrees [-90..90].");
  Serial.println("  Example (no id): {\"cmd\":\"set\",\"axis\":\"x\",\"value\":30,\"dur\":1.0}");
  Serial.println("  Example (xy):    {\"id\":10,\"cmd\":\"set\",\"axis\":\"xy\",\"x\":10,\"y\":-20,\"speed\":90}");

  Serial.println("\nadjust");
  Serial.println("  Add delta(s) to current position(s).");
  Serial.println("  Example (no id): {\"cmd\":\"adjust\",\"axis\":\"y\",\"value\":-5,\"speed\":120}");
  Serial.println("  Example (xy):    {\"id\":11,\"cmd\":\"adjust\",\"axis\":\"xy\",\"x\":5,\"y\":-5,\"dur\":0.5}");

  Serial.println("\ncenter");
  Serial.println("  Move selected axis/axes to 0.");
  Serial.println("  Example (no id): {\"cmd\":\"center\",\"axis\":\"xy\",\"dur\":1.0}");

  Serial.println("\nstop");
  Serial.println("  Stop motion on selected axis/axes immediately (hold current).");
  Serial.println("  Example (no id): {\"cmd\":\"stop\",\"axis\":\"y\"}");

  Serial.println("\nstopAll");
  Serial.println("  Stop all motion. Defaults to flush queue (flush=true).");
  Serial.println("  Example (no id): {\"cmd\":\"stopAll\"}");
  Serial.println("  Example:         {\"id\":12,\"cmd\":\"stopAll\",\"flush\":false}");

  Serial.println("\nresetAll");
  Serial.println("  Stop + clear queue + center + clear invert + reset speed.");
  Serial.println("  Example (no id): {\"cmd\":\"resetAll\"}");

  Serial.println("\ninvert");
  Serial.println("  Toggle inversion for axis/axes (no jump).");
  Serial.println("  Example (no id): {\"cmd\":\"invert\",\"axis\":\"x\"}");

  Serial.println("\nspeed");
  Serial.println("  Set global default speed (deg/sec) used when dur is absent.");
  Serial.println("  Example (no id): {\"cmd\":\"speed\",\"value\":90}");

  Serial.println("\nsave");
  Serial.println("  Save current x/y to favorite slot 1..5.");
  Serial.println("  Example (no id): {\"cmd\":\"save\",\"slot\":1}");

  Serial.println("\nrecall");
  Serial.println("  Move to a saved slot 1..5. Optional axis/dur/speed.");
  Serial.println("  Example (no id): {\"cmd\":\"recall\",\"slot\":1,\"dur\":1.5}");

  Serial.println("\nqueue");
  Serial.println("  Set queue mode: off|on|step.");
  Serial.println("    off : execute immediately unless q:true");
  Serial.println("    on  : enqueue by default unless q:false");
  Serial.println("    step: enqueue only when q:true (default)");
  Serial.println("  Example (no id): {\"cmd\":\"queue\",\"mode\":\"on\"}");

  Serial.println("\nqAdd");
  Serial.println("  Explicitly enqueue a motion command. Uses cmd2=set|adjust|center.");
  Serial.println("  Example: {\"id\":20,\"cmd\":\"qAdd\",\"cmd2\":\"set\",\"axis\":\"x\",\"value\":-90,\"dur\":5}");

  Serial.println("\nqClear");
  Serial.println("  Clear pending queued steps (does not stop current motion).");
  Serial.println("  Example (no id): {\"cmd\":\"qClear\"}");

  Serial.println("\nqAbort");
  Serial.println("  Stop all motion + clear queue (hard escape).");
  Serial.println("  Example (no id): {\"cmd\":\"qAbort\"}");

  Serial.println("\nqStatus");
  Serial.println("  Returns queue mode/count/active.");
  Serial.println("  Example (no id): {\"cmd\":\"qStatus\"}");

  Serial.println("\nqList");
  Serial.println("  Lists queued items (compact).");
  Serial.println("  Example (no id): {\"cmd\":\"qList\"}");

  Serial.println("\nsweep");
  Serial.println("  Enqueue a sweep expanded into queued steps.");
  Serial.println("  Fields: axis, from, to, dur, total(optional), loops(optional), dwell(optional), q(optional).");
  Serial.println("  Example: {\"id\":30,\"cmd\":\"sweep\",\"axis\":\"x\",\"from\":-90,\"to\":90,\"dur\":55,\"loops\":1,\"dwell\":0.5,\"q\":true}");

  Serial.println("\nRouting example (master-script friendly)");
  Serial.println("  {\"id\":300,\"subsystem\":\"head\",\"route\":\"servo\",\"cmd\":\"set\",\"axis\":\"x\",\"value\":15,\"dur\":0.5}");
}

static void printExamples() {
  Serial.println("Examples (NO id):");
  Serial.println("{\"cmd\":\"commands\"}");
  Serial.println("{\"cmd\":\"status\"}");
  Serial.println("{\"cmd\":\"speed\",\"value\":90}");
  Serial.println("{\"cmd\":\"set\",\"axis\":\"x\",\"value\":30,\"dur\":1.0}");
  Serial.println("{\"cmd\":\"adjust\",\"axis\":\"y\",\"value\":-10,\"speed\":120}");
  Serial.println("{\"cmd\":\"center\",\"axis\":\"xy\",\"dur\":1.0}");
  Serial.println("{\"cmd\":\"invert\",\"axis\":\"y\"}");
  Serial.println("{\"cmd\":\"save\",\"slot\":1}");
  Serial.println("{\"cmd\":\"recall\",\"slot\":1,\"dur\":1.0}");
  Serial.println("{\"cmd\":\"queue\",\"mode\":\"on\"}");
  Serial.println("{\"cmd\":\"set\",\"axis\":\"x\",\"value\":-90,\"dur\":5}");
  Serial.println("{\"cmd\":\"set\",\"axis\":\"x\",\"value\":90,\"dur\":50}");
  Serial.println("{\"cmd\":\"qList\"}");
  Serial.println("{\"cmd\":\"stopAll\"}");
  Serial.println("{\"cmd\":\"resetAll\"}");
  Serial.println();

  Serial.println("Examples (WITH id):");
  Serial.println("{\"id\":101,\"cmd\":\"commands\"}");
  Serial.println("{\"id\":102,\"cmd\":\"status\"}");
  Serial.println("{\"id\":103,\"cmd\":\"speed\",\"value\":90}");
  Serial.println("{\"id\":104,\"cmd\":\"set\",\"axis\":\"xy\",\"x\":20,\"y\":-20,\"dur\":2.0}");
  Serial.println("{\"id\":105,\"cmd\":\"adjust\",\"axis\":\"x\",\"value\":-10,\"speed\":90}");
  Serial.println("{\"id\":106,\"cmd\":\"center\",\"axis\":\"xy\",\"dur\":1.0}");
  Serial.println("{\"id\":107,\"cmd\":\"invert\",\"axis\":\"xy\"}");
  Serial.println("{\"id\":108,\"cmd\":\"save\",\"slot\":2}");
  Serial.println("{\"id\":109,\"cmd\":\"recall\",\"slot\":2,\"dur\":1.2}");
  Serial.println("{\"id\":110,\"cmd\":\"queue\",\"mode\":\"step\"}");
  Serial.println("{\"id\":111,\"cmd\":\"set\",\"axis\":\"x\",\"value\":-60,\"dur\":2.0,\"q\":true}");
  Serial.println("{\"id\":112,\"cmd\":\"set\",\"axis\":\"x\",\"value\":60,\"dur\":2.0,\"q\":true}");
  Serial.println("{\"id\":113,\"cmd\":\"qList\"}");
  Serial.println("{\"id\":114,\"cmd\":\"qAbort\"}");
  Serial.println();

  Serial.println("Sweep:");
  Serial.println("{\"id\":200,\"cmd\":\"sweep\",\"axis\":\"x\",\"from\":-60,\"to\":60,\"dur\":10,\"loops\":2,\"dwell\":0.2,\"q\":true}");
  Serial.println();

  Serial.println("Routing (master-script):");
  Serial.println("{\"id\":300,\"subsystem\":\"head\",\"route\":\"servo\",\"cmd\":\"set\",\"axis\":\"x\",\"value\":15,\"dur\":0.5}");
}

// ------------------- Validation helpers -------------------
static bool parseAxisMask(const String& axis, bool& useX, bool& useY) {
  String a = axis; a.toLowerCase();
  if (a == "x") { useX = true; useY = false; return true; }
  if (a == "y") { useX = false; useY = true; return true; }
  if (a == "xy") { useX = true; useY = true; return true; }
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

static bool shouldEnqueue(QueueMode mode, bool hasQ, bool qVal) {
  if (mode == Q_ON)  return !(hasQ && qVal == false);
  if (mode == Q_OFF) return (hasQ && qVal == true);
  // Q_STEP
  return (hasQ && qVal == true);
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

// ------------------- Motion update + queue scheduler -------------------
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

// ------------------- Command handling -------------------
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

  // ---- info ----
  if (cmd == "commands") { sendOk(id, subsystem, route, "commands"); printCommands(); return; }
  if (cmd == "help")     { sendOk(id, subsystem, route, "help");     printHelp();     return; }
  if (cmd == "examples") { sendOk(id, subsystem, route, "examples"); printExamples(); return; }
  if (cmd == "status")   { sendOk(id, subsystem, route, "status");   sendState(nullptr, 0, subsystem, route); return; }

  // ---- queue ----
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

  if (cmd == "qclear") { qClear(); sendOk(id, subsystem, route, "queue_cleared"); sendState("done", id, subsystem, route); return; }

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

  // ---- stop/reset/invert/speed/favorites ----
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
    if (flush) { qClear(); qActive = false; }
    applyOutputs();
    sendOk(id, subsystem, route, flush ? "stopped_all_flushed" : "stopped_all");
    sendState("done", id, subsystem, route);
    return;
  }

  if (cmd == "resetall") {
    abortQueueAndMotion();
    v1 = 0; v2 = 0;
    invX = false; invY = false;
    defaultSpeed = 90.0f;
    applyOutputs();
    sendOk(id, subsystem, route, "reset_all");
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
    if (sp < 0.1f || sp > 1000.0f) { sendErr(id, subsystem, route, "bad_value", "speed value out of range"); return; }
    defaultSpeed = sp;
    sendOk(id, subsystem, route, "speed_set");
    sendState("done", id, subsystem, route);
    return;
  }

  if (cmd == "save") {
    int slot=0;
    if (!getIntField(line, "slot", slot) || slot < 1 || slot > 5) { sendErr(id, subsystem, route, "bad_slot", "save requires slot 1..5"); return; }
    int idx = slot - 1;
    favValid[idx] = true;
    favX[idx] = v1;
    favY[idx] = v2;
    sendOk(id, subsystem, route, "saved");
    sendState("done", id, subsystem, route);
    return;
  }

  if (cmd == "recall") {
    int slot=0;
    if (!getIntField(line, "slot", slot) || slot < 1 || slot > 5) { sendErr(id, subsystem, route, "bad_slot", "recall requires slot 1..5"); return; }
    int idx = slot - 1;
    if (!favValid[idx]) { sendErr(id, subsystem, route, "empty_slot", "slot not saved yet"); return; }

    String axis="xy"; (void)getStringField(line, "axis", axis);
    bool useX=false,useY=false;
    if (!parseAxisMask(axis, useX, useY)) { sendErr(id, subsystem, route, "bad_axis", "axis must be x, y, or xy"); return; }

    float tx = favX[idx];
    float ty = favY[idx];

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
    bool total=false; (void)getBoolField(line, "total", total);

    int loops=1; (void)getIntField(line, "loops", loops);
    if (loops < 0) loops = 0;
    if (loops > 1000000) loops = 1000000;

    float dwellSec=0.0f; (void)getNumberField(line, "dwell", dwellSec);
    if (dwellSec < 0.0f) dwellSec = 0.0f;
    if (dwellSec > 60.0f) dwellSec = 60.0f;

    // Enqueue decision: sweep is inherently multi-step; we always enqueue.
    (void)shouldEnqueue(qMode, hasQ, qVal);

    float legSec = total ? (durSec * 0.5f) : durSec;
    uint32_t legMs = (uint32_t)(legSec * 1000.0f + 0.5f);
    uint32_t dwellMs = (uint32_t)(dwellSec * 1000.0f + 0.5f);

    int cycles = loops;
    if (cycles == 0) cycles = 1000; // big chunk; stopAll breaks it

    int baseSteps = 1 + cycles * 2;
    int dwellSteps = (dwellMs > 0) ? baseSteps : 0;
    int totalSteps = baseSteps + dwellSteps;
    if (totalSteps > (int)QMAX - (int)qCount) { sendErr(id, subsystem, route, "queue_full", "Not enough queue space for sweep steps"); return; }

    float fromX = useX ? from : v1;
    float fromY = useY ? from : v2;
    float toX   = useX ? to   : v1;
    float toY   = useY ? to   : v2;

    float sp=-1; bool hasSpeed2 = getNumberField(line, "speed", sp);
    float useSp = (hasSpeed2 && sp > 0.1f) ? sp : defaultSpeed;

    uint32_t ref = id;

    // Move to FROM (speed-derived)
    QueueItem moveToFrom;
    moveToFrom.id = ref++; moveToFrom.subsystem=subsystem; moveToFrom.route=route;
    moveToFrom.kind="sweepToFrom"; moveToFrom.useX=useX; moveToFrom.useY=useY;
    moveToFrom.tx=fromX; moveToFrom.ty=fromY;
    moveToFrom.dx = useX ? durationFromSpeed(v1, fromX, useSp) : 0;
    moveToFrom.dy = useY ? durationFromSpeed(v2, fromY, useSp) : 0;
    if (!qEnqueue(moveToFrom)) { sendErr(id, subsystem, route, "queue_full", "Queue full"); return; }

    auto enqueueHold = [&](float hx, float hy) -> bool {
      if (dwellMs == 0) return true;
      QueueItem hold;
      hold.id = ref++; hold.subsystem=subsystem; hold.route=route;
      hold.kind="dwell"; hold.useX=useX; hold.useY=useY;
      hold.tx=hx; hold.ty=hy;
      hold.dx = useX ? dwellMs : 0;
      hold.dy = useY ? dwellMs : 0;
      return qEnqueue(hold);
    };

    if (!enqueueHold(fromX, fromY)) { sendErr(id, subsystem, route, "queue_full", "Queue full"); return; }

    for (int c=0; c<cycles; c++) {
      QueueItem legTo;
      legTo.id=ref++; legTo.subsystem=subsystem; legTo.route=route;
      legTo.kind="sweepTo"; legTo.useX=useX; legTo.useY=useY;
      legTo.tx=toX; legTo.ty=toY;
      legTo.dx = useX ? legMs : 0;
      legTo.dy = useY ? legMs : 0;
      if (!qEnqueue(legTo)) { sendErr(id, subsystem, route, "queue_full", "Queue full"); return; }
      if (!enqueueHold(toX, toY)) { sendErr(id, subsystem, route, "queue_full", "Queue full"); return; }

      QueueItem legFrom;
      legFrom.id=ref++; legFrom.subsystem=subsystem; legFrom.route=route;
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

  // ---- motion commands (set/adjust/center) ----
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

  applyOutputs();

  Serial.println();
  Serial.println("Servo controller ready (JSONL).");
  Serial.println("IMPORTANT: commands must be JSON objects containing \"cmd\".");
  Serial.println("Try one of these:");
  Serial.println("  {\"cmd\":\"commands\"}");
  Serial.println("  {\"cmd\":\"help\"}");
  Serial.println("  {\"cmd\":\"examples\"}");
  Serial.println("  {\"cmd\":\"speed\",\"value\":90}");
  Serial.println();
  Serial.println("Queue mode default: step (enqueue only when \"q\":true).");
  Serial.println("Example no-id move: {\"cmd\":\"center\",\"axis\":\"xy\",\"dur\":1.0}");
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

