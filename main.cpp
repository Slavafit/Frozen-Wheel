/* Wheel Strobe - final (no TEST mode)
   ESP32-C3 SuperMini
   - auto detect IMP/REV (missing tooth)
   - Spokes (3..12), Phase (-5..+5), Bright (0..10)
   - saves to EEPROM
   - software PWM for brightness (safe for bootloader)
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
  uint16_t impPerRev;   // IMP/REV auto-detected or user-saved
  uint8_t spokes;       // 3..12
  int8_t phase;         // -5..+5
  uint8_t bright;       // 0..10
};
Settings settings;

// -------------------- STATE --------------------
enum Screen { SCR_WORK, SCR_MENU, SCR_RESET_CONFIRM };
Screen screen = SCR_WORK;

volatile unsigned long isr_last_micros = 0;
volatile unsigned long isr_pulse_count = 0;
volatile bool isr_new_pulse = false;

unsigned long pulseCountTotal = 0;
unsigned int currPulses = 0;
unsigned long lastPulseMicros = 0;
double avgInterval = 0.0;
const double alpha = 0.15;
const double GAP_FACTOR = 1.6;
unsigned long lastRevMicros = 0;
float revPeriodSec = 0.0;

uint16_t impPerRev_current = 0;
uint16_t pulseIndex = 0;

const unsigned long STROBE_MS = 6;
unsigned long strobeOnUntil = 0;

// encoder/button
long lastEnc = 0;
long encAccum = 0;
bool btnState = false;
unsigned long btnDownTime = 0;
bool btnHandledLong = false;

// menu
int menuIndex = 0;
bool needSave = false;
unsigned long lastSaveMillis = 0;

// software PWM
const unsigned long PWM_PERIOD_MS = 10; // 10 ms period
unsigned long pwmLastToggle = 0;
bool pwmLedState = false;

// -------------------- EEPROM helpers --------------------
const int EEPROM_SIZE = 256;
const int EEPROM_ADDR = 0;

void loadSettings() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ADDR, settings);
  // defaults if invalid
  if (settings.impPerRev == 0xFFFF || settings.impPerRev == 0 || settings.spokes < 3 || settings.spokes > 12) {
    settings.impPerRev = 48;
    settings.spokes = 6;
    settings.phase = 0;
    settings.bright = 7;
  }
  impPerRev_current = settings.impPerRev;
}

void saveSettingsNow() {
  EEPROM.put(EEPROM_ADDR, settings);
  EEPROM.commit();
  needSave = false;
  lastSaveMillis = millis();
}

// -------------------- ISR for ABS (minimal) --------------------
void IRAM_ATTR absISR() {
  unsigned long t = micros();
  isr_last_micros = t;
  isr_pulse_count++;
  isr_new_pulse = true;
}

// Helper to copy ISR data safely
void copyIsrDataSafe(unsigned long &out_micros, unsigned long &out_count) {
  noInterrupts();
  out_micros = isr_last_micros;
  out_count = isr_pulse_count;
  isr_new_pulse = false;
  interrupts();
}

// -------------------- Pulse processing and strobe --------------------
void strobeOn() {
  digitalWrite(PIN_LED, HIGH);
  strobeOnUntil = millis() + STROBE_MS;
}

void strobeOff() {
  digitalWrite(PIN_LED, LOW);
}

// compute step and phase offset
int computeStep() {
  if (settings.spokes == 0) return 1;
  if (impPerRev_current == 0) return 1;
  int step = impPerRev_current / settings.spokes;
  if (step < 1) step = 1;
  return step;
}
int computePhaseOffset() {
  if (impPerRev_current == 0) return 0;
  int offset = (int)settings.phase; // directly use -5..+5 as pulses shift
  // clamp into [0..impPerRev_current)
  int res = (offset % (int)impPerRev_current);
  if (res < 0) res += impPerRev_current;
  return res;
}

void processPulse(unsigned long t_micros) {
  pulseCountTotal++;
  unsigned long delta = 0;
  if (lastPulseMicros != 0) delta = t_micros - lastPulseMicros;
  lastPulseMicros = t_micros;

  // EMA
  if (avgInterval < 1.0) avgInterval = delta;
  else avgInterval = alpha * (double)delta + (1.0 - alpha) * avgInterval;

  currPulses++;
  pulseIndex++;
  if (impPerRev_current > 0 && pulseIndex >= impPerRev_current) pulseIndex = 0;

  // detect gap (missing tooth)
  if (delta > (unsigned long)(GAP_FACTOR * avgInterval) && currPulses > 1) {
    // end of revolution
    impPerRev_current = currPulses;
    // update rev period
    unsigned long nowMicros = t_micros;
    if (lastRevMicros != 0) {
      unsigned long revPeriodMicros = nowMicros - lastRevMicros;
      revPeriodSec = revPeriodMicros / 1e6;
    }
    lastRevMicros = nowMicros;
    currPulses = 0;
    pulseIndex = 0;
    // auto-save impPerRev to settings after a short stabilization (we mark needSave)
    settings.impPerRev = impPerRev_current;
    needSave = true;
  }

  // strobe decision
  if (impPerRev_current > 0 && settings.spokes > 0) {
    int step = computeStep();
    int phaseOffset = computePhaseOffset();
    if (((int)pulseIndex + phaseOffset) % step == 0) {
      strobeOn(); // will be turned off later
    }
  }
}

// -------------------- Controls (encoder + button) --------------------
void readControls() {
  // Button debounce and long press detection
  static bool lastBtn = HIGH;
  bool cur = digitalRead(PIN_ENC_BTN);
  if (cur != lastBtn) {
    delay(5); // tiny debounce
    cur = digitalRead(PIN_ENC_BTN);
  }
  // falling edge => pressed
  if (cur == LOW && lastBtn == HIGH) {
    btnDownTime = millis();
    btnHandledLong = false;
  }
  // release
  if (cur == HIGH && lastBtn == LOW) {
    unsigned long dur = millis() - btnDownTime;
    if (!btnHandledLong) {
      // short press -> action
      btnState = true; // signal short press
    }
  }
  // long press detection
  if (cur == LOW && lastBtn == LOW) {
    if (!btnHandledLong && (millis() - btnDownTime >= 2000)) {
      // long press action
      btnHandledLong = true;
      // in work mode -> reset phase
      if (screen == SCR_WORK) {
        settings.phase = 0;
        needSave = true;
      } else if (screen == SCR_MENU) {
        // in menu long press -> Reset All
        // handle when confirm selected
      }
    }
  }
  lastBtn = cur;

  // Encoder simple polling (works reliably when read frequently)
  static int lastA = HIGH;
  int a = digitalRead(PIN_ENC_A);
  int b = digitalRead(PIN_ENC_B);
  if (a != lastA) {
    if (b != a) encAccum++;
    else encAccum--;
  }
  lastA = a;
}

// -------------------- Display screens --------------------
void drawWorkScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);

  u8g2.setCursor(0, 12);
  u8g2.print(F("IMP/REV: "));
  u8g2.print(impPerRev_current);

  u8g2.setCursor(0, 28);
  u8g2.print(F("SPOKES : "));
  u8g2.print(settings.spokes);

  u8g2.setCursor(0, 44);
  u8g2.print(F("PHASE  : "));
  if (settings.phase >= 0) u8g2.print(F("+"));
  u8g2.print(settings.phase);

  u8g2.setCursor(0, 60);
  u8g2.print(F("BRIGHT : "));
  u8g2.print(settings.bright);

  u8g2.setCursor(0, 78);
  u8g2.print(F("CLICK = MENU"));

  u8g2.sendBuffer();
}

void drawMenuScreen() {
  const char* items[] = { "Spokes", "Phase", "Brightness", "Reset", "Exit" };
  const int count = 5;

  // navigation: if encoder changed, move selection or edit
  static bool editing = false;
  long encVal = encAccum;
  if (encVal != lastEnc) {
    if (!editing) {
      menuIndex += (encVal - lastEnc);
      if (menuIndex < 0) menuIndex = count - 1;
      if (menuIndex >= count) menuIndex = 0;
    } else {
      // editing a value
      if (menuIndex == 0) { // Spokes
        settings.spokes = constrain((int)settings.spokes + (encVal - lastEnc), 3, 12);
        needSave = true;
      } else if (menuIndex == 1) { // Phase
        settings.phase = constrain((int)settings.phase + (encVal - lastEnc), -5, 5);
        needSave = true;
      } else if (menuIndex == 2) { // Brightness
        settings.bright = constrain((int)settings.bright + (encVal - lastEnc), 0, 10);
        needSave = true;
      }
    }
    lastEnc = encVal;
  }

  // Enter/Exit editing on button short press
  if (btnState) {
    btnState = false;
    if (!editing) {
      // start editing item
      if (menuIndex == 3) {
        // Reset -> show confirm submenu by switching screen
        screen = SCR_RESET_CONFIRM;
      } else if (menuIndex == 4) {
        screen = SCR_WORK;
      } else {
        editing = true;
      }
    } else {
      // finish editing
      editing = false;
    }
  }

  // draw menu
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 10, "SETTINGS");
  for (int i = 0; i < count; ++i) {
    u8g2.setCursor(0, 26 + i*14);
    if (i == menuIndex) u8g2.print("> ");
    else u8g2.print("  ");
    u8g2.print(items[i]);
    u8g2.print(" ");
    // show values
    if (i == 0) u8g2.print(settings.spokes);
    else if (i == 1) {
      if (settings.phase >= 0) u8g2.print("+");
      u8g2.print(settings.phase);
    }
    else if (i == 2) u8g2.print(settings.bright);
  }
  if (editing) {
    u8g2.setCursor(0, 26 + menuIndex*14);
    u8g2.print(" *");
  }
  u8g2.sendBuffer();
}

void drawResetConfirm() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 10, "RESET:");
  u8g2.drawStr(0, 28, "Short=Reset this item");
  u8g2.drawStr(0, 44, "Long=Reset ALL");
  u8g2.drawStr(0, 60, "Click=Back");
  u8g2.sendBuffer();

  if (btnState) {
    // short press: reset selected item (menuIndex still points to Reset)
    btnState = false;
    // For simplicity reset phase, bright, spokes & impPerRev individually
    settings.phase = 0;
    settings.bright = 7;
    settings.spokes = 6;
    settings.impPerRev = 48;
    impPerRev_current = settings.impPerRev;
    needSave = true;
    screen = SCR_MENU;
  }
  // long press in this screen already handled in readControls -> if long press triggered while in menu we'll treat here:
  if (btnHandledLong) {
    // perform full reset-all
    settings.phase = 0;
    settings.bright = 7;
    settings.spokes = 6;
    settings.impPerRev = 48;
    impPerRev_current = settings.impPerRev;
    needSave = true;
    btnHandledLong = false;
    screen = SCR_MENU;
  }
}

// -------------------- setup --------------------
void setup() {
  // pins
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_BTN, INPUT_PULLUP);
  pinMode(PIN_ABS, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // I2C display
  Wire.begin(SDA_PIN, SCL_PIN);
  u8g2.begin();
  u8g2.enableUTF8Print();

  // EEPROM load
  loadSettings();

  // attach ABS interrupt correctly
  attachInterrupt(digitalPinToInterrupt(PIN_ABS), absISR, RISING);

  // initial displays
  drawWorkScreen();
  delay(600);
}

// -------------------- main loop --------------------
void loop() {
  // handle ISR pulses
  if (isr_new_pulse) {
    unsigned long t_micros;
    unsigned long tot;
    copyIsrDataSafe(t_micros, tot);
    processPulse(t_micros);
  }

  // strobe off when duration passed
  if (strobeOnUntil && millis() > strobeOnUntil) {
    strobeOff();
    strobeOnUntil = 0;
  }

  // read controls
  readControls();

  // process short button in work mode -> menu
  if (btnState) {
    btnState = false;
    if (screen == SCR_WORK) {
      screen = SCR_MENU;
      menuIndex = 0;
      lastEnc = encAccum;
    }
  }

  // in work mode: encoder adjusts phase live
  if (screen == SCR_WORK) {
    if (encAccum != lastEnc) {
      int delta = (int)(encAccum - lastEnc);
      lastEnc = encAccum;
      settings.phase = constrain((int)settings.phase + delta, -5, 5);
      needSave = true;
    }
    drawWorkScreen();
  }
  else if (screen == SCR_MENU) {
    drawMenuScreen();
  }
  else if (screen == SCR_RESET_CONFIRM) {
    drawResetConfirm();
  }

  // software PWM for brightness (only when not strobing)
  if (strobeOnUntil == 0) {
    unsigned long now = millis();
    unsigned long duty = map(settings.bright, 0, 10, 0, PWM_PERIOD_MS); // ms ON
    if (duty == 0) {
      digitalWrite(PIN_LED, LOW);
    } else if (duty >= PWM_PERIOD_MS) {
      digitalWrite(PIN_LED, HIGH);
    } else {
      if (now - pwmLastToggle >= PWM_PERIOD_MS) {
        pwmLastToggle = now;
      }
      unsigned long phase = (now - pwmLastToggle) % PWM_PERIOD_MS;
      digitalWrite(PIN_LED, (phase < duty) ? HIGH : LOW);
    }
  }

  // autosave settings if changed
  if (needSave && (millis() - lastSaveMillis > 5000)) {
    saveSettingsNow();
  }

  delay(10); // small yield
}
