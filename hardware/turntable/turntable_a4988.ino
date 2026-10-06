// Insect turntable: NEMA 17 (17HS4401, 1.8 deg, 1.7 A) + A4988 step driver, driven from the
// capture coordinator over USB serial (coordinator/capture_coordinator/turntable.py).
// Board: any Arduino-class board with a USB serial port (Uno / Nano / Leonardo / Pro Micro,
// AVR or ARM); only digitalWrite/delayMicroseconds are used. Wiring in README.md.
//
// The board holds the platter position in MICROSTEPS. Every GOTO rounds the requested
// absolute angle to the nearest microstep, so a 10 deg step that is not an integer number
// of microsteps never accumulates an error. Motion = trapezoidal ramp (v_max, accel from
// CFG), blocking; the reply "OK <pos_deg>" is sent when the motion has finished.
//
// Protocol (115200 8N1, one line per command, one reply per command):
//   CFG <microsteps_per_platter_rev> <max_deg_s> <accel_deg_s2> <dir 1|-1>  -> OK CFG
//   SETPOS <deg>   -> OK <pos_deg>      (current position := deg; used at session start)
//   GOTO <deg>     -> OK <pos_deg>      (absolute, replies after the move)
//   MOVE <deg>     -> OK <pos_deg>      (relative)
//   POS            -> OK <pos_deg>
//   EN 0|1         -> OK EN <0|1>       (1 = driver enabled = hold torque, the default)
//   PING           -> OK PONG <version>
//   bad input      -> ERR <message>
// "READY <version>" is printed once after reset.

#define FW_VERSION "1"

// ---- pins (change to your wiring) ------------------------------------------------------
const int PIN_STEP = 3;
const int PIN_DIR = 4;
const int PIN_EN = 5;          // A4988 ENABLE is active LOW
const int PIN_LED = LED_BUILTIN;

// ---- defaults, overwritten by CFG ------------------------------------------------------
double stepsPerRev = 200.0 * 16.0;   // microsteps per PLATTER revolution (motor steps * microstepping * gear ratio)
double vMaxDeg = 20.0;               // deg/s
double accDeg = 40.0;                // deg/s^2
int dirSign = 1;                     // 1 | -1 flips the DIR pin sense
const double V_MIN_STEPS = 40.0;     // steps/s floor of the ramp (no stall at the very first step)
const unsigned int STEP_PULSE_US = 4; // A4988 needs >= 1 us high and low

long posSteps = 0;                   // current position in microsteps
bool enabled = true;

char line[64];
int lineLen = 0;

void setEnabled(bool on) {
  enabled = on;
  digitalWrite(PIN_EN, on ? LOW : HIGH);
  digitalWrite(PIN_LED, on ? HIGH : LOW);
}

double posDeg() { return posSteps * 360.0 / stepsPerRev; }

long degToSteps(double deg) {
  double s = deg * stepsPerRev / 360.0;
  return (long)(s >= 0 ? s + 0.5 : s - 0.5);   // round half away from zero
}

// Move n microsteps (signed) with a trapezoidal / triangular ramp. Blocking.
void moveSteps(long n) {
  if (n == 0) return;
  bool forward = (n > 0);
  digitalWrite(PIN_DIR, (forward ? dirSign : -dirSign) > 0 ? HIGH : LOW);
  delayMicroseconds(5);              // DIR setup time (A4988: 200 ns)
  long total = forward ? n : -n;
  double vMax = vMaxDeg * stepsPerRev / 360.0;   // steps/s
  double acc = accDeg * stepsPerRev / 360.0;     // steps/s^2
  if (acc < 1.0) acc = 1.0;
  if (vMax < V_MIN_STEPS) vMax = V_MIN_STEPS;
  for (long i = 0; i < total; i++) {
    // speed reachable after i steps of acceleration, and needed to stop within the remaining steps
    double vUp = sqrt(2.0 * acc * (double)(i + 1));
    double vDown = sqrt(2.0 * acc * (double)(total - i));
    double v = vUp < vDown ? vUp : vDown;
    if (v > vMax) v = vMax;
    if (v < V_MIN_STEPS) v = V_MIN_STEPS;
    unsigned long period = (unsigned long)(1000000.0 / v);
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(PIN_STEP, LOW);
    posSteps += forward ? 1 : -1;
    unsigned long rest = period > STEP_PULSE_US ? period - STEP_PULSE_US : 1;
    if (rest > 16000) { delay(rest / 1000); delayMicroseconds(rest % 1000); }   // delayMicroseconds() maxes at 16383
    else delayMicroseconds(rest);
  }
}

void replyPos() {
  Serial.print("OK ");
  Serial.println(posDeg(), 4);
}

void handle(char *cmd) {
  // upper-case the verb
  char *p = cmd;
  while (*p && *p != ' ') { *p = toupper(*p); p++; }
  if (strncmp(cmd, "PING", 4) == 0) {
    Serial.print("OK PONG "); Serial.println(FW_VERSION);
  } else if (strncmp(cmd, "CFG ", 4) == 0) {
    double spr, v, a; int d;
    char *rest = cmd + 4;
    spr = strtod(rest, &rest); v = strtod(rest, &rest); a = strtod(rest, &rest); d = (int)strtol(rest, &rest, 10);
    if (spr <= 0 || v <= 0 || a <= 0 || (d != 1 && d != -1)) { Serial.println("ERR CFG <steps_per_rev> <max_deg_s> <accel_deg_s2> <1|-1>"); return; }
    // keep the physical position when the resolution changes
    double deg = posDeg();
    stepsPerRev = spr; vMaxDeg = v; accDeg = a; dirSign = d;
    posSteps = degToSteps(deg);
    Serial.println("OK CFG");
  } else if (strncmp(cmd, "SETPOS ", 7) == 0) {
    char *end; double deg = strtod(cmd + 7, &end);
    if (end == cmd + 7) { Serial.println("ERR SETPOS <deg>"); return; }
    posSteps = degToSteps(deg);
    replyPos();
  } else if (strncmp(cmd, "GOTO ", 5) == 0 || strncmp(cmd, "MOVE ", 5) == 0) {
    char *end; double deg = strtod(cmd + 5, &end);
    if (end == cmd + 5) { Serial.println("ERR GOTO|MOVE <deg>"); return; }
    if (!enabled) { Serial.println("ERR driver disabled (EN 1 first)"); return; }
    long target = (cmd[0] == 'G') ? degToSteps(deg) : posSteps + degToSteps(deg);
    moveSteps(target - posSteps);
    replyPos();
  } else if (strncmp(cmd, "POS", 3) == 0) {
    replyPos();
  } else if (strncmp(cmd, "EN ", 3) == 0) {
    setEnabled(cmd[3] == '1');
    Serial.print("OK EN "); Serial.println(enabled ? 1 : 0);
  } else if (cmd[0] == 0) {
    // empty line: ignore
  } else {
    Serial.print("ERR unknown command "); Serial.println(cmd);
  }
}

void setup() {
  pinMode(PIN_STEP, OUTPUT); pinMode(PIN_DIR, OUTPUT); pinMode(PIN_EN, OUTPUT); pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_STEP, LOW); digitalWrite(PIN_DIR, LOW);
  setEnabled(true);
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}   // native-USB boards (Leonardo) wait for the host
  Serial.print("READY "); Serial.println(FW_VERSION);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      line[lineLen] = 0;
      if (lineLen > 0) handle(line);
      lineLen = 0;
    } else if (lineLen < (int)sizeof(line) - 1) {
      line[lineLen++] = c;
    } else {
      lineLen = 0; Serial.println("ERR line too long");
    }
  }
}
