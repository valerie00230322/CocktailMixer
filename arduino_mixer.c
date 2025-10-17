// =====================
// I2C-SLAVE + TMC2209 + AccelStepper
// Adresse: 0x13
// CMDs: 0=fahren(int8), 1=home, 2=status
// Status-Reply: [busy(1), pos(4 LE bytes)]
// STEP=3, DIR=4, EN=2, HOME=13 (INPUT_PULLUP)
// UART: Serial1 -> TMC2209 (ADDR=0b00)
// 1 Einheit = 400 Steps
// =====================

#include <TMCStepper.h>
#include <AccelStepper.h>
#include <Wire.h>

// --- Pins/Hardware ---
#define HOME_PIN      13
#define EN_PIN        2
#define STEP_PIN_PLF  3
#define DIR_PIN_PLF   4

// --- TMC / I2C ---
#define SERIAL_PORT   Serial1     // Hardware-UART für TMC2209
#define R_SENSE       0.11f
#define I2C_ADDR      0x13
#define ADDR_PLF      0b00        // TMC2209 UART-Adresse (CFG-Pins)

// --- Protokoll-CMDs ---
enum : uint8_t {
  CMD_FAHR   = 0,
  CMD_HOME   = 1,
  CMD_STATUS = 2
};

// --- Globale Objekte ---
TMC2209Stepper driver_plf(&SERIAL_PORT, R_SENSE, ADDR_PLF);
AccelStepper    PLF(AccelStepper::DRIVER, STEP_PIN_PLF, DIR_PIN_PLF);

// --- I2C geteilte Variablen (ISR <-> loop) ---
volatile uint8_t  aufgabe_i2c = 0;     // letzter empfangener CMD
volatile int16_t  abstand_i2c = 0;     // Param (int8 genutzt)
volatile bool     new_message = false; // Flag: neue Nachricht da
volatile uint8_t  last_cmd = 0;        // für onRequest(), um Status live zu senden

// optional: vorbereiteter TX-Puffer (nicht zwingend genutzt)
volatile uint8_t  tx_buf[8];
volatile uint8_t  tx_len = 0;

// --- Bewegungs-Parameter ---
static const long  STEPS_PER_UNIT = 400;  // 1 Einheit = 400 Steps
float mixerMax = 600.0f;                  // maxSpeed in steps/s
float mixerAcc = 300.0f;                  // acceleration in steps/s^2

// =====================
// Hilfsfunktionen
// =====================
long abstand_steps(int abstand_units) {
  return (long)abstand_units * (long)STEPS_PER_UNIT;
}

// Non-blocking Fahrt: nur Ziel setzen
void fahren(long abstand_units) {
  PLF.moveTo(abstand_steps((int)abstand_units));
}

// Homing: fährt mit negativer Speed bis Endstop (LOW), backoff, Null setzen
bool home() {
  const float HOME_SPEED        = -4000.0f; // Richtung Schalter
  const float HOME_ACCEL        = 800.0f;
  const long  BACKOFF_STEPS     = 800;
  const unsigned long TIMEOUT_MS = 20000;

  float oldMax = PLF.maxSpeed();
  float oldAcc = mixerAcc;

  PLF.setAcceleration(HOME_ACCEL);
  PLF.setMaxSpeed(HOME_SPEED < 0 ? -HOME_SPEED : HOME_SPEED);
  PLF.setSpeed(HOME_SPEED); // für runSpeed()

  unsigned long t0 = millis();
  while (digitalRead(HOME_PIN) == HIGH) {
    PLF.runSpeed();
    if (millis() - t0 > TIMEOUT_MS) {
      // Werte zurück
      PLF.setMaxSpeed(oldMax);
      PLF.setAcceleration(oldAcc);
      return false;
    }
  }

  // Stoppen & absetzen
  PLF.stop();
  while (PLF.isRunning()) PLF.run();
  PLF.move(BACKOFF_STEPS);
  while (PLF.distanceToGo() != 0) PLF.run();

  // Nullpunkt
  PLF.setCurrentPosition(0);

  // Ursprungswerte wiederherstellen
  PLF.setMaxSpeed(oldMax);
  PLF.setAcceleration(oldAcc);
  mixerMax = oldMax;
  mixerAcc = oldAcc;
  return true;
}

// (Optional) vorbereiteter Status in tx_buf (nicht zwingend nötig)
void status_prepare() {
  bool busy = (PLF.distanceToGo() != 0);
  long pos  = PLF.currentPosition();
  tx_buf[0] = busy ? 1 : 0;
  tx_buf[1] = (uint8_t)(pos & 0xFF);
  tx_buf[2] = (uint8_t)((pos >> 8) & 0xFF);
  tx_buf[3] = (uint8_t)((pos >> 16) & 0xFF);
  tx_buf[4] = (uint8_t)((pos >> 24) & 0xFF);
  tx_len = 5;
}

// =====================
// I2C Handler
// =====================
void onI2CReceive(int count) {
  if (count < 2) { // Mindestens CMD + 1 Param erwartet
    while (Wire.available()) (void)Wire.read();
    return;
  }
  uint8_t b0 = Wire.read();          // CMD
  int16_t b1 = (int8_t)Wire.read();  // Param (signed 8 Bit)

  aufgabe_i2c = b0;
  abstand_i2c = b1;
  last_cmd    = b0;                  // für onRequest()

  new_message = true;

  // Rest verwerfen
  while (Wire.available()) (void)Wire.read();
}

// Sendeantwort: bei STATUS live generieren, sonst ACK oder Puffer
void onI2CRequest() {
  if (last_cmd == CMD_STATUS) {
    uint8_t out[5];
    out[0] = (PLF.distanceToGo() != 0) ? 1 : 0;
    long pos = PLF.currentPosition();
    out[1] = (uint8_t)(pos & 0xFF);
    out[2] = (uint8_t)((pos >> 8) & 0xFF);
    out[3] = (uint8_t)((pos >> 16) & 0xFF);
    out[4] = (uint8_t)((pos >> 24) & 0xFF);
    Wire.write(out, 5);
    return;
  }

  if (tx_len) { // falls etwas vorbereitet wurde
    Wire.write((const uint8_t*)tx_buf, tx_len);
    tx_len = 0;
    return;
  }

  // Default: 1-Byte-ACK
  const uint8_t ack = 0x06; // ASCII ACK
  Wire.write(&ack, 1);
}

// =====================
// Arduino-Setup/Loop
// =====================
void setup() {
  // I2C als Slave
  Wire.begin(I2C_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  // TMC UART
  SERIAL_PORT.begin(57600);

  // Pins
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);         // LOW = Treiber aktiv (ENn)
  pinMode(HOME_PIN, INPUT_PULLUP);

  // TMC2209 Grund-Init
  driver_plf.begin();
  driver_plf.toff(5);                // Chopper an (>=1)
  driver_plf.rms_current(600);       // mA (an Motor anpassen)
  driver_plf.microsteps(16);
  driver_plf.pwm_autoscale(true);    // automatische Stromregelung
  // optional:
  // driver_plf.en_spreadCycle(false);  // StealthChop bevorzugen

  // AccelStepper Basiswerte
  PLF.setMaxSpeed(mixerMax);
  PLF.setAcceleration(mixerAcc);
  PLF.setMinPulseWidth(2);           // 2 µs Step-Puls (TMC2209-tauglich)
}

void loop() {
  // Stepper kontinuierlich betreiben
  PLF.run();

  // Eingegangene Kommandos abarbeiten
  if (new_message) {
    noInterrupts();
    uint8_t cmd = aufgabe_i2c;
    int16_t par = abstand_i2c;
    new_message = false;
    interrupts();

    switch (cmd) {
      case CMD_FAHR:
        fahren(par);        // non-blocking: setzt Ziel
        break;

      case CMD_HOME:
        (void)home();       // blockiert kurz, bis Homing fertig
        break;

      case CMD_STATUS:
        // nichts nötig; Antwort kommt in onI2CRequest() live
        // optional zusätzlich: status_prepare();
        break;

      default:
        // unbekannt -> ignoriere
        break;
    }
  }
}
