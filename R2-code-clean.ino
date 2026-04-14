// R2 robot firmware (BT24 BLE->UART) — HOLD-TO-MOVE + FAILSAFE
// Front RGB: WS2812/NeoPixel on D12
// Rear LED: D9
//
// Commands
//   Movement: U D L R Q E S   (movement does NOT change lights)
//   Sound:    B = horn (non-blocking)
//   Rear LED: t = ON, u = OFF
//   Front RGB: f = ON, g = OFF, Cx = set color (x: W R G B A P C)
//   Sensors:  S: sound level, D: distance, L: lux

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Motor pins
static const uint8_t DIR_L  = 4;
static const uint8_t DIR_R  = 7;
static const uint8_t PWM_L  = 5;
static const uint8_t PWM_R  = 6;

// Buzzer
static const uint8_t BUZZER_PIN = 13;

// Lights
static const uint8_t FRONT_WS2812_PIN = 12; // DJX13 data
static const uint8_t REAR_LIGHT_PIN   = 9;  // simple LED

// Ultrasonic (HC-SR04 via RJ11 port labeled D2/A0)
// Some R2 kits/cables swap TRIG/ECHO between A0 and D2.
// We'll auto-detect at boot between the two common mappings:
//   A) TRIG=A0, ECHO=D2
//   B) TRIG=D2, ECHO=A0
static const uint8_t ULTRASONIC_PIN_A0 = A0;
static const uint8_t ULTRASONIC_PIN_D2 = 2;

// Selected mapping (auto-detected in setup)
static uint8_t usTrigPin = ULTRASONIC_PIN_A0;
static uint8_t usEchoPin = ULTRASONIC_PIN_D2;

// Line tracker (3x digital sensors)
static const uint8_t LINE_L_PIN = A3;
static const uint8_t LINE_C_PIN = A2;
static const uint8_t LINE_R_PIN = A1;
// What logic level means "on the line"? Set to LOW if your module is active-LOW.
static const uint8_t LINE_ON = HIGH;

// Returns true if ANY line sensor currently detects the line.
static inline bool readLineDetected() {
  const bool l = (digitalRead(LINE_L_PIN) == LINE_ON);
  const bool c = (digitalRead(LINE_C_PIN) == LINE_ON);
  const bool r = (digitalRead(LINE_R_PIN) == LINE_ON);
  return (l || c || r);
}


// Extra sensors
static const uint8_t TOUCH_PIN = 11; // D11 touch sensor (digital)
static const uint8_t LDR_PIN   = A4; // A4 LDR (analog)
// Sound sensor is wired to the board pin that may be labeled A7 on Nano/Pro Mini.
// Prefer A7 if available, otherwise fall back to A6, then A5.
//#if defined(A7)
//static const uint8_t SOUND_PIN = A7; // A7 sound sensor (analog-only)
//#elif defined(A6)
static const uint8_t SOUND_PIN = A6; // A6 sound sensor (analog-only)
//#else
//static const uint8_t SOUND_PIN = A5; // fallback for boards without A6/A7 (change wiring if needed)
//#endif


static bool lineFollowEnabled = false;
static bool lineSensorActivated = false;

enum LineAction : uint8_t { LA_NONE, LA_LEFT_PULSE, LA_RIGHT_PULSE };
static LineAction lineAction = LA_NONE;
static unsigned long lineActionUntilMs = 0;
static const uint16_t LINE_PULSE_MS = 50;

// Sensor states (kept compatible with other app naming)
static bool robotActive = true;
static bool sensorActivated = false;

static bool touchPressed = false;
static float lastLux = 0.0f;
static float lastSoundLevel = 0.0f;

// Cached ultrasonic distance (to avoid -1 when polled fast from Scratch)
static float lastDistanceCm = -1.0f;
static unsigned long lastDistanceMs = 0;
static const uint16_t DIST_MIN_INTERVAL_MS = 120; // safe rate (~8 Hz) over BLE + HC-SR04
// If we get no-echo timeouts repeatedly (e.g. nothing in front within range),
// we don't want to "freeze" at the last close reading. After a few consecutive
// timeouts we will report a saturated "far" value.
static uint8_t usNoEchoCount = 0;
static const uint8_t US_NOECHO_TO_FAR_AFTER = 3;   // consecutive failures
static const float   US_FAR_CM = 400.0f;           // HC-SR04 nominal max range


// Thresholds (adjust to taste)
static const float LUX_THRESHOLD   = 250.0f;
static const float SOUND_THRESHOLD = 15.0f;

// Polling (avoid expensive pow() too often)
static unsigned long lastSensorPollMs = 0;
static const uint16_t SENSOR_POLL_MS = 10;


// Non-blocking buzzer pulse (uses tone/noTone)
static unsigned long buzzerUntilMs = 0;

Adafruit_NeoPixel front(1, FRONT_WS2812_PIN, NEO_GRB + NEO_KHZ800);

// Motion tuning
static uint8_t SPEED_FWD  = 80;
static uint8_t SPEED_BACK = 80;
static uint8_t SPEED_TURN = 80;

// Failsafe
static const uint16_t FAILSAFE_MS = 900;
static unsigned long lastCmdMs = 0;
static bool failsafeActive = false;

// Front light mode
enum FrontMode : uint8_t { PARK, FWD, BACK_BLINK, LEFT_BLINK, RIGHT_BLINK };
static FrontMode frontMode = PARK;

static bool frontEnabled = true;
static bool rearOn = false;

// Blink timing
static const uint16_t BLINK_MS = 350;
static unsigned long lastBlinkMs = 0;
static bool blinkOn = false;

// Parking color (user-settable)
static uint32_t parkingColor = 0;

static inline uint8_t half(uint8_t v) { return (uint8_t)(v / 2); }

// ---- Motors ----
void motorsStop() {
  analogWrite(PWM_L, 0);
  analogWrite(PWM_R, 0);
}
void driveForward(uint8_t spL, uint8_t spR) {
  digitalWrite(DIR_L, HIGH);
  digitalWrite(DIR_R, HIGH);
  analogWrite(PWM_L, spL);
  analogWrite(PWM_R, spR);
}
void driveBack(uint8_t spL, uint8_t spR) {
  digitalWrite(DIR_L, LOW);
  digitalWrite(DIR_R, LOW);
  analogWrite(PWM_L, spL);
  analogWrite(PWM_R, spR);
}
void turnLeft(uint8_t sp) {
  digitalWrite(DIR_L, LOW);
  digitalWrite(DIR_R, HIGH);
  analogWrite(PWM_L, sp);
  analogWrite(PWM_R, sp);
}
void turnRight(uint8_t sp) {
  digitalWrite(DIR_L, HIGH);
  digitalWrite(DIR_R, LOW);
  analogWrite(PWM_L, sp);
  analogWrite(PWM_R, sp);
}

// ---- Buzzer ----
void horn() {
  // 1ο μπιπ
  tone(BUZZER_PIN, 880);   // A5
  delay(160);
  noTone(BUZZER_PIN);
  delay(80);

  // 2ο μπιπ (λίγο πιο ψηλά, δίνει “κόρνα” αίσθηση)
  tone(BUZZER_PIN, 988);   // B5
  delay(180);
  noTone(BUZZER_PIN);
}



// Non-blocking horn (allows immediate stop when touch is released / Scratch stops).
static bool hornActive = false;
static uint8_t hornPhase = 0;
static unsigned long hornT0 = 0;

static void hornStart() {
  if (hornActive) return;
  hornActive = true;
  hornPhase = 0;
  hornT0 = millis();
}

static void hornStop() {
  hornActive = false;
  hornPhase = 0;
  noTone(BUZZER_PIN);
}

// Call frequently from loop() (non-blocking).
static void hornService() {
  if (!hornActive) return;
  unsigned long now = millis();

  switch (hornPhase) {
    case 0: // start 1st beep
      tone(BUZZER_PIN, 880);
      hornT0 = now;
      hornPhase = 1;
      break;

    case 1: // hold 160ms
      if (now - hornT0 >= 160) {
        noTone(BUZZER_PIN);
        hornT0 = now;
        hornPhase = 2;
      }
      break;

    case 2: // gap 80ms
      if (now - hornT0 >= 80) {
        tone(BUZZER_PIN, 988);
        hornT0 = now;
        hornPhase = 3;
      }
      break;

    case 3: // hold 180ms then stop
      if (now - hornT0 >= 180) {
        noTone(BUZZER_PIN);
        hornActive = false;
        hornPhase = 0;
      }
      break;
  }
}

// Non-blocking beep (for autonomous actions).
void buzzerBeep(uint16_t freq, uint16_t ms) {
  tone(BUZZER_PIN, freq);
  buzzerUntilMs = millis() + ms;
}

void buzzerService() {
  if (buzzerUntilMs && (long)(millis() - buzzerUntilMs) >= 0) {
    noTone(BUZZER_PIN);
    buzzerUntilMs = 0;
  }
}
// ---- WS2812 helpers ----
uint32_t rgb(uint8_t r, uint8_t g, uint8_t b) { return front.Color(r, g, b); }

void frontShow(uint32_t c) {
  if (!frontEnabled) return;
  front.setPixelColor(0, c);
  front.show();
}
void frontOff() {
  front.setPixelColor(0, 0);
  front.show();
}
void showParking() {
  // default parking color if not set yet
  if (parkingColor == 0) parkingColor = rgb(120, 120, 120); // soft white
  frontShow(parkingColor);
}

// ---- Line follow (3-sensor) ----
static inline void lfApplyForward() {
  driveForward(SPEED_FWD, SPEED_FWD);
  setFrontMode(FWD);
  frontShow(rgb(0, 180, 0)); // green
}

static inline void lfApplyLeft() {
  turnLeft(SPEED_TURN);
  setFrontMode(LEFT_BLINK);
  frontShow(rgb(200, 120, 0)); // amber
  blinkOn = true;
  lastBlinkMs = millis();
}

static inline void lfApplyRight() {
  turnRight(SPEED_TURN);
  setFrontMode(RIGHT_BLINK);
  frontShow(rgb(200, 120, 0)); // amber
  blinkOn = true;
  lastBlinkMs = millis();
}

static inline void lfApplyStop() {
  motorsStop();
  setFrontMode(PARK);
  showParking();
}

void serviceLineFollow() {
  // keep failsafe from triggering while autonomous mode runs
  lastCmdMs = millis();
  failsafeActive = false;

  buzzerService();

  unsigned long now = millis();
  if (lineAction != LA_NONE && (long)(now - lineActionUntilMs) < 0) {
    // keep current pulse active
    if (lineAction == LA_LEFT_PULSE)  lfApplyLeft();
    if (lineAction == LA_RIGHT_PULSE) lfApplyRight();
    return;
  }
  lineAction = LA_NONE;

  // Read sensors ONCE (L/C/R)
  bool L = (digitalRead(LINE_L_PIN) == LINE_ON);
  bool C = (digitalRead(LINE_C_PIN) == LINE_ON);
  bool R = (digitalRead(LINE_R_PIN) == LINE_ON);

  if (C) {
    // On line: go forward (matches your original logic).
    lineSensorActivated = true;
    lfApplyForward();
    return;
  }

  // Off center: try to re-acquire using left/right sensors with short pulses.
  if (L && !R) {
    lineSensorActivated = true;
    lfApplyLeft();
    lineAction = LA_LEFT_PULSE;
    lineActionUntilMs = now + LINE_PULSE_MS;
    buzzerBeep(988, LINE_PULSE_MS);
    return;
  }

  if (R && !L) {
    lineSensorActivated = true;
    lfApplyRight();
    lineAction = LA_RIGHT_PULSE;
    lineActionUntilMs = now + LINE_PULSE_MS;
    buzzerBeep(988, LINE_PULSE_MS);
    return;
  }

  // No clear reading: stop.
  lfApplyStop();
  lineSensorActivated = false;
}

void setFrontMode(uint8_t m) {
  frontMode = (FrontMode)m;
  if (frontMode == LEFT_BLINK || frontMode == RIGHT_BLINK || frontMode == BACK_BLINK) {
    lastBlinkMs = millis();
    blinkOn = true; // start visible
  }
}

// ---- Blink service ----
void serviceBlink() {
  if (!frontEnabled) return;

  if (frontMode == LEFT_BLINK || frontMode == RIGHT_BLINK || frontMode == BACK_BLINK) {
    unsigned long now = millis();
    if (now - lastBlinkMs >= BLINK_MS) {
      lastBlinkMs = now;
      blinkOn = !blinkOn;

      if (blinkOn) {
        if (frontMode == BACK_BLINK) frontShow(rgb(180, 0, 0));     // red
        else                         frontShow(rgb(200, 120, 0));   // amber
      } else {
        frontOff();
      }
    }
  }
}

// ---- Rear helpers ----
void rearSet(bool on) {
  rearOn = on;
  digitalWrite(REAR_LIGHT_PIN, rearOn ? HIGH : LOW);
}
void rearToggle() { rearSet(!rearOn); }

// ---- Front enable helpers ----
void frontSetEnabled(bool on) {
  frontEnabled = on;
  if (!frontEnabled) frontOff();
  else {
    // restore current visual
    if (frontMode == FWD) frontShow(rgb(0, 180, 0));
    else if (frontMode == PARK) showParking();
    else {
      // force blink visible immediately
      blinkOn = true;
      lastBlinkMs = millis();
      if (frontMode == BACK_BLINK) frontShow(rgb(180,0,0));
      else frontShow(rgb(200,120,0));
    }
  }
}
void frontToggle() { frontSetEnabled(!frontEnabled); }

// ---- Color select (dropdown-friendly) ----
uint32_t colorFromCode(char x) {
  switch (x) {
    case 'W': return rgb(180,180,180); // white
    case 'R': return rgb(180,0,0);     // red
    case 'G': return rgb(0,180,0);     // green
    case 'B': return rgb(0,0,180);     // blue
    case 'A': return rgb(200,120,0);   // amber
    case 'P': return rgb(140,0,160);   // purple
    case 'C': return rgb(0,160,160);   // cyan
    default:  return rgb(120,120,120); // fallback soft white
  }
}


// ---- Ultrasonic (cm) ----
// Low-level ping using the currently selected pins.
// Returns distance in cm, or -1 if no echo (timeout).
static float ultrasonicPingCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(12);
  digitalWrite(trigPin, LOW);

  unsigned long us = pulseIn(echoPin, HIGH, 25000UL); // ~4m max
  if (us == 0) {
    // quick retry (occasionally the first trigger misses)
    us = pulseIn(echoPin, HIGH, 25000UL);
  }
  if (us == 0) return -1.0f;
  return (float)us / 58.0f;
}

static void ultrasonicSelectPins(uint8_t trigPin, uint8_t echoPin) {
  usTrigPin = trigPin;
  usEchoPin = echoPin;
  pinMode(usTrigPin, OUTPUT);
  digitalWrite(usTrigPin, LOW);
  pinMode(usEchoPin, INPUT);
}

static bool ultrasonicTryMapping(uint8_t trigPin, uint8_t echoPin) {
  ultrasonicSelectPins(trigPin, echoPin);
  // Try a few pings with safe spacing. Accept a plausible reading.
  for (uint8_t i = 0; i < 3; i++) {
    float cm = ultrasonicPingCm(trigPin, echoPin);
    if (cm >= 2.0f && cm <= 400.0f) {
      lastDistanceCm = cm;
      lastDistanceMs = millis();
      return true;
    }
    delay(80);
  }
  return false;
}

static void ultrasonicAutoDetect() {
  // Preferred mapping (as used previously): TRIG=A0, ECHO=D2
  if (ultrasonicTryMapping(ULTRASONIC_PIN_A0, ULTRASONIC_PIN_D2)) return;
  // Alternate mapping (some cables/modules swap): TRIG=D2, ECHO=A0
  if (ultrasonicTryMapping(ULTRASONIC_PIN_D2, ULTRASONIC_PIN_A0)) return;
  // If both fail, keep the preferred mapping; distance will remain -1 until fixed.
  ultrasonicSelectPins(ULTRASONIC_PIN_A0, ULTRASONIC_PIN_D2);
}

// Background update of the cached distance (safe for Scratch monitors).
// - Enforces a minimum interval between pings.
// - Keeps lastDistanceCm at the last valid reading (doesn't spam -1).
static void ultrasonicService() {
  unsigned long now = millis();
  if ((now - lastDistanceMs) < DIST_MIN_INTERVAL_MS) return;
  lastDistanceMs = now;

  float cm = ultrasonicPingCm(usTrigPin, usEchoPin);
  if (cm >= 0.0f) {
    lastDistanceCm = cm;
    usNoEchoCount = 0;
  } else {
    // No echo. If this happens a few times in a row, treat it as "no obstacle"
    // instead of keeping the last close distance forever.
    if (usNoEchoCount < 255) usNoEchoCount++;
    if (usNoEchoCount >= US_NOECHO_TO_FAR_AFTER) {
      lastDistanceCm = US_FAR_CM;
    }
  }
}

static float getDistanceCmCached() {
  ultrasonicService();
  return lastDistanceCm;
}


// ---- Extra sensors ----
bool readTouchPressed() {
  // Touch modules usually output HIGH when touched.
  return (digitalRead(TOUCH_PIN) == HIGH);
}

float readSoundLevel() {
  static float envelope = 0;
  static float baseline = 512;

  int x = analogRead(SOUND_PIN);

  // baseline tracking (αργό, για να ακολουθεί τη μέση τιμή)
  baseline = baseline * 0.99f + x * 0.01f;

  // στιγμιαίο πλάτος γύρω από baseline
  float amplitude = abs(x - baseline);

  // envelope follower
  if (amplitude > envelope)
    envelope = amplitude;          // γρήγορη άνοδος
  else
    envelope = envelope * 0.97f;   // αργή πτώση

  return envelope;
}

// LDR Lux estimate copied from the other app (depends on your resistor values).
float readLux() {
  int value = analogRead(LDR_PIN); // 0..1023
  float v = ((float)value / 1023.0f) * 5.0f; // volts (assuming 5V ADC ref)

  // guard against division by zero / overflow
  if (v <= 0.001f) return 0.0f;
  if (v >= 4.999f) v = 4.999f;

  float RLDR = (5.0f - v) / v * 5.0f * 1000.0f;
  float lux = 12518931.0f * pow(RLDR, -1.405f);
  return lux;
}

// Poll sensors at a low rate (pow() is expensive on AVR)
void pollSensors() {
  unsigned long now = millis();
  if (now - lastSensorPollMs < SENSOR_POLL_MS) return;
  lastSensorPollMs = now;

  touchPressed = readTouchPressed();
  lastSoundLevel = readSoundLevel();
  lastLux = readLux();

  bool soundTrig = (lastSoundLevel > SOUND_THRESHOLD);
  bool ldrTrig   = (lastLux > LUX_THRESHOLD);

  sensorActivated = touchPressed || soundTrig || ldrTrig;
  if (sensorActivated) robotActive = true;
}

// ---- Setup ----
void setup() {
  pinMode(DIR_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(REAR_LIGHT_PIN, OUTPUT);
  rearSet(false);

  // Ultrasonic auto-detect (handles swapped TRIG/ECHO cables)
  ultrasonicAutoDetect();

  // Extra sensors
  pinMode(TOUCH_PIN, INPUT);
  // LDR_PIN and SOUND_PIN are analog inputs; no pinMode required.


  // Line tracker inputs (adjust to INPUT_PULLUP + LINE_ON=LOW if your module is active-LOW)
  pinMode(LINE_L_PIN, INPUT);
  pinMode(LINE_C_PIN, INPUT);
  pinMode(LINE_R_PIN, INPUT);

  front.begin();
  front.setBrightness(255);
  frontOff();

  motorsStop();

  Serial.begin(9600);
  lastCmdMs = millis();

  parkingColor = rgb(120,120,120); // default
  setFrontMode(PARK);
  showParking();
}

// ---- Command handling ----
void handleCommand(char c) {
  lastCmdMs = millis();
  failsafeActive = false;

  // Manual override: any movement/stop cancels autonomous line-follow immediately.
  if (lineFollowEnabled && (c=='U' || c=='D' || c=='L' || c=='R' || c=='Q' || c=='E' || c=='S')) {
    lineFollowEnabled = false;
    lineAction = LA_NONE;
    lineActionUntilMs = 0;
    buzzerUntilMs = 0;
    noTone(BUZZER_PIN);
  }

  // keep case for 'C' color commands; allow lower-case for others
  if (c >= 'a' && c <= 'z') {
    // keep 't','u','f','g' as-is
  }

  // Movement commands MUST be uppercase to avoid clashing with rear off 'u'
  // (Scratch extension will send uppercase for movement)
  if (c >= 'a' && c <= 'z') {
    // do not auto-uppercase (we use lowercase for lights)
  }

  switch (c) {
    // ---- Movement (UPPERCASE) ----
    case 'U':
      driveForward(SPEED_FWD, SPEED_FWD);
      break;

    case 'D':

      driveBack(SPEED_BACK, SPEED_BACK);
      break;

    case 'L':

      turnLeft(SPEED_TURN);
      break;

    case 'R':

      turnRight(SPEED_TURN);
      break;

    case 'Q':

      driveForward(SPEED_FWD, half(SPEED_FWD));
      break;

    case 'E':

      driveForward(half(SPEED_FWD), SPEED_FWD);
      break;

    case 'S':

      motorsStop();
      break;

    // ---- Line-follow mode ----
    case 'Z': // enable line-follow (autonomous)
      lineFollowEnabled = true;
      lineAction = LA_NONE;
      lineActionUntilMs = 0;
      lfApplyStop();
      break;

    case 'z': // disable line-follow
      lineFollowEnabled = false;
      lineAction = LA_NONE;
      lineActionUntilMs = 0;
      lfApplyStop();
      break;


    // ---- Sound ----
    case 'B':
      hornStart();
      break;

    case 'b':
      hornStop();
      break;

    // ---- Rear explicit ----
    case 't': // rear ON
      rearSet(true);
      break;

    case 'u': // rear OFF
      rearSet(false);
      break;

    // Backward-compatible rear toggle
    case 'T':
      rearToggle();
      break;

    // ---- Front explicit ----
    case 'f': // front ON
      frontSetEnabled(true);
      // if we're stopped, show parking color immediately
      if (frontMode == PARK) showParking();
      break;

    case 'g': // front OFF
      frontSetEnabled(false);
      break;

    // Backward-compatible front toggle
    case 'F':
      frontToggle();
      break;

    // ---- Color set: expect 'C' then one more char ----
    case 'C': {
      // wait for next byte (non-blocking with a short spin)
      unsigned long start = millis();
      while (!Serial.available() && (millis() - start) < 30) { /* wait up to 30ms */ }
      if (Serial.available()) {
        char x = (char)Serial.read();
        if (x >= 'a' && x <= 'z') x = (char)(x - 32); // normalize
        parkingColor = colorFromCode(x);

        // apply immediately if front is enabled and currently stopped
        if (frontEnabled && frontMode == PARK) {
          showParking();
        }
      }
      break;
    }


// ---- Extra sensors (query) ----
// Also accept letter aliases (more reliable in some BLE-UART stacks):
//   Y = touch, H = lux, J = sound
case '!':
case 'Y': { // touch pressed?
  bool pressed = readTouchPressed();
  touchPressed = pressed;
  Serial.print("T:");
  Serial.println(pressed ? 1 : 0);
  break;
}

case '#':
case 'H': { // LDR lux
  float lux = readLux();
  lastLux = lux;
  Serial.print("L:");
  Serial.println(lux, 1);
  break;
}

case '$':
case 'J': { // sound level
  float lvl = readSoundLevel();
  lastSoundLevel = lvl;
  Serial.print("S:");
  Serial.println(lvl, 0);
  break;
}


case 'M': { // line detected? (any of A3/A2/A1)
  bool det = readLineDetected();
  Serial.print("N:");
  Serial.println(det ? 1 : 0);
  break;
}

    // ---- Ultrasonic ----
    case '?': {
      float cm = getDistanceCmCached();
      Serial.print("D:");
      Serial.println(cm, 0);
      break;
    }

    default:
      break;
  }
}

// ---- Loop ----
void loop() {
  while (Serial.available() > 0) {
    handleCommand((char)Serial.read());
  }

  pollSensors();

  // Keep distance updated even when the robot is stopped (helps Scratch monitors).
  // Avoid running it during autonomous line-follow to keep steering responsive.
  if (!lineFollowEnabled) {
    ultrasonicService();
  }

  if (!lineFollowEnabled && !failsafeActive && (millis() - lastCmdMs > FAILSAFE_MS)) {
    failsafeActive = true;
    motorsStop();
    setFrontMode(PARK);
    showParking();
  }


  if (lineFollowEnabled) {
    serviceLineFollow();
  }

  hornService();
  serviceBlink();
}
