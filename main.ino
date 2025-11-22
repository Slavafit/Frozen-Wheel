/* Wheel Strobe v3.2 - no brightness version
   ESP32-C3 SuperMini
   - auto detect IMP/REV (missing tooth)
   - Spokes (3..12), Phase (-5..+5)
   - saves to EEPROM
   - stable encoder processing (debounced, Gray-state -> 1 step per click)
*/

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <EEPROM.h>

// -------------------- PINS --------------------
const uint8_t PIN_ENC_A   = 7;
const uint8_t PIN_ENC_B   = 8;
const uint8_t PIN_ENC_BTN = 9;
const uint8_t PIN_ABS     = 4;
const uint8_t PIN_LED     = 5;
const uint8_t SDA_PIN     = 20;
const uint8_t SCL_PIN     = 21;

// -------------------- DISPLAY --------------------
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, SCL_PIN, SDA_PIN);

// -------------------- SETTINGS --------------------
struct Settings {
  uint16_t impPerRev;
  uint8_t spokes;
  int8_t phase;
};
Settings settings;

// -------------------- STATE --------------------
enum Screen { SCR_WORK, SCR_MENU, SCR_RESET_CONFIRM };
Screen screen = SCR_WORK;

// ISR data
volatile unsigned long isr_last_micros = 0;
volatile bool isr_new_pulse = false;

// pulse processing
unsigned long lastPulseMicros = 0;
double avgInterval = 0;
const double alpha = 0.15;
const double GAP_FACTOR = 1.6;

uint16_t impPerRev_current = 48;
uint16_t pulseIndex = 0;
unsigned int currPulses = 0;
unsigned long lastRevMicros = 0;

// strobe
const unsigned long STROBE_MS = 6;
unsigned long strobeOnUntil = 0;

// -------------------- ENCODER (stable) --------------------
volatile int encAccum = 0;      // accumulation of quarter-steps
uint8_t lastEncState = 0;       // last (A<<1 | B)

// get 1-step turn from accumulated quarter-steps (threshold = 2)
int getEncoderStep() {
  int step = 0;
  noInterrupts();
  if (encAccum >= 2) {
    step = +1;
    encAccum -= 2;
  } else if (encAccum <= -2) {
    step = -1;
    encAccum += 2;
  }
  interrupts();
  return step;
}

// -------------------- BUTTON --------------------
bool btnShort = false;
unsigned long btnDown = 0;
bool btnHandledLong = false;

// menu
int menuIndex = 0;
bool needSave = false;
unsigned long lastSaveMillis = 0;

// -------------------- EEPROM --------------------
const int EEPROM_SIZE = 256;
const int EEPROM_ADDR = 0;

void loadSettings() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ADDR, settings);

  if (settings.impPerRev == 0 || settings.impPerRev > 500 ||
      settings.spokes < 3 || settings.spokes > 12 ||
      settings.phase < -5 || settings.phase > 5) {

    settings.impPerRev = 48;
    settings.spokes = 6;
    settings.phase = 0;
  }

  impPerRev_current = settings.impPerRev;
}

void saveSettingsNow() {
  EEPROM.put(EEPROM_ADDR, settings);
  EEPROM.commit();
  needSave = false;
  lastSaveMillis = millis();
}

// -------------------- ISR --------------------
void IRAM_ATTR absISR() {
  unsigned long t = micros();
  isr_last_micros = t;
  isr_new_pulse = true;
}

// -------------------- LED control --------------------
void strobeOn() {
  digitalWrite(PIN_LED, HIGH);
  strobeOnUntil = millis() + STROBE_MS;
}
void strobeOff() {
  digitalWrite(PIN_LED, LOW);
}

// -------------------- Pulse handling --------------------
void processPulse(unsigned long t) {
  unsigned long delta = (lastPulseMicros == 0 ? 0 : t - lastPulseMicros);
  lastPulseMicros = t;

  if (avgInterval < 1.0) avgInterval = delta;
  else avgInterval = alpha * delta + (1.0 - alpha) * avgInterval;

  currPulses++;
  pulseIndex++;
  if (pulseIndex >= impPerRev_current) pulseIndex = 0;

  // missing tooth detection
  if (delta > GAP_FACTOR * avgInterval && currPulses > 1) {
    impPerRev_current = currPulses;
    settings.impPerRev = impPerRev_current;
    needSave = true;

    currPulses = 0;
    pulseIndex = 0;
  }

  // strobing logic
  int step = impPerRev_current / settings.spokes;
  if (step < 1) step = 1;

  int phaseShift = (settings.phase % (int)impPerRev_current + impPerRev_current) % impPerRev_current;

  if (((int)pulseIndex + phaseShift) % step == 0) {
    strobeOn();
  }
}

// -------------------- Read encoder (poll, fast) --------------------
void readEncoder() {
  // Read A/B pins once
  uint8_t a = digitalRead(PIN_ENC_A);
  uint8_t b = digitalRead(PIN_ENC_B);
  uint8_t state = (a << 1) | b;

  // mapping table 16 entries combining lastState<<2 | state
  static const int8_t table[16] = {
    0, +1, -1, 0,
   -1,  0,  0,+1,
   +1,  0,  0,-1,
    0, -1, +1, 0
  };
  uint8_t idx = (lastEncState << 2) | state;
  int8_t delta = table[idx];

  if (delta != 0) {
    noInterrupts();
    encAccum += delta;
    interrupts();
  }
  lastEncState = state;
}

// -------------------- Read button (debounced & long press) --------------------
void readButton() {
  static bool last = HIGH;
  bool cur = digitalRead(PIN_ENC_BTN);
  if (cur != last) delay(3); // small debounce

  if (cur == LOW && last == HIGH) {
    btnDown = millis();
    btnHandledLong = false;
  }
  if (cur == HIGH && last == LOW) {
    if (!btnHandledLong) btnShort = true;
  }
  if (cur == LOW && !btnHandledLong && (millis() - btnDown >= 1500)) {
    btnHandledLong = true;
    if (screen == SCR_WORK) {
      settings.phase = 0;
      needSave = true;
    }
  }
  last = cur;
}

// -------------------- Display --------------------
void drawWorkScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);

  u8g2.setCursor(0, 12); u8g2.print("IMP/REV: "); u8g2.print(impPerRev_current);
  u8g2.setCursor(0, 28); u8g2.print("SPOKES : "); u8g2.print(settings.spokes);
  u8g2.setCursor(0, 44); u8g2.print("PHASE  : ");
  if (settings.phase >= 0) u8g2.print("+");
  u8g2.print(settings.phase);

  u8g2.setCursor(0, 60); u8g2.print("Click = MENU");
  u8g2.sendBuffer();
}

void drawMenuScreen() {
  const char* items[] = { "Spokes", "Phase", "Reset", "Exit" };
  const int count = 4;

  int step;
  // read encoder steps and move menu selection
  step = getEncoderStep();
  if (step != 0) {
    menuIndex += step;
    if (menuIndex < 0) menuIndex = count - 1;
    if (menuIndex >= count) menuIndex = 0;
  }

  if (btnShort) {
    btnShort = false;

    if (menuIndex == 0) {          // spokes
      settings.spokes++;
      if (settings.spokes > 12) settings.spokes = 3;
      needSave = true;
    } else if (menuIndex == 1) {   // phase
      // enter quick-phase-increment: each click increments phase
      settings.phase++;
      if (settings.phase > 5) settings.phase = -5;
      needSave = true;
    } else if (menuIndex == 2) {   // reset
      settings.spokes = 6;
      settings.phase = 0;
      settings.impPerRev = 48;
      impPerRev_current = 48;
      needSave = true;
    } else if (menuIndex == 3) {   // exit
      screen = SCR_WORK;
    }
  }

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 10, "SETTINGS");

  for (int i = 0; i < count; i++) {
    u8g2.setCursor(0, 26 + i * 14);
    u8g2.print(i == menuIndex ? "> " : "  ");
    u8g2.print(items[i]);

    if (i == 0) { u8g2.print(" "); u8g2.print(settings.spokes); }
    if (i == 1) {
      u8g2.print(" ");
      if (settings.phase >= 0) u8g2.print("+");
      u8g2.print(settings.phase);
    }
  }

  u8g2.sendBuffer();
}

// -------------------- Setup --------------------
void setup() {
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_BTN, INPUT_PULLUP);
  pinMode(PIN_ABS, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);
  u8g2.begin();

  loadSettings();

  // initialize lastEncState to current pins (avoid random jump on start)
  lastEncState = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);

  attachInterrupt(digitalPinToInterrupt(PIN_ABS), absISR, RISING);

  drawWorkScreen();
  delay(500);
}

// -------------------- Loop --------------------
void loop() {
  // ISR pulse
  if (isr_new_pulse) {
    noInterrupts();
    unsigned long t = isr_last_micros;
    isr_new_pulse = false;
    interrupts();
    processPulse(t);
  }

  // strobe off
  if (strobeOnUntil && millis() > strobeOnUntil) {
    strobeOff();
    strobeOnUntil = 0;
  }

  // inputs
  readEncoder();
  readButton();

  // short click → menu
  if (btnShort && screen == SCR_WORK) {
    btnShort = false;
    screen = SCR_MENU;
    menuIndex = 0;
  }

  if (screen == SCR_WORK) {
    int step = getEncoderStep();
    if (step != 0) {
      settings.phase = constrain((int)settings.phase + step, -5, 5);
      needSave = true;
    }
    drawWorkScreen();
  }
  else if (screen == SCR_MENU) {
    drawMenuScreen();
  }

  // auto save
  if (needSave && millis() - lastSaveMillis > 3000) {
    saveSettingsNow();
  }

  delay(4);
}
