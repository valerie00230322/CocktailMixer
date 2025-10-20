#include <TMCStepper.h>
#include <AccelStepper.h>
#include <Wire.h>

// ==============================
//       Konfiguration
// ==============================
static const long STEPS_PER_UNIT = 400; // 1 Einheit = 400 Steps

// Index: 0=PLF, 1=BAND, 2..7=PUMPE1..6
float max_speed[8]  = { 6000.0f, 1000.0f, 1000.0f, 1000.0f, 1000.0f, 1000.0f, 1000.0f, 1000.0f };
float accel_sps2[8] = { 30000.0f, 1000.0f, 1000.0f, 1000.0f, 1000.0f, 1000.0f, 1000.0f, 1000.0f };

const int steps_u[8]    = { 4, 16, 16, 16, 16, 16, 16, 16 }; // Microsteps
const int current_mA[8] = { 800, 800, 800, 800, 800, 800, 800, 800 }; // RMS-Strom

// ==============================
//       Befehls-Codes
// ==============================
enum : uint8_t { CMD_FAHR = 0, CMD_HOME = 1, CMD_STATUS = 2, CMD_PUMPE = 3 };

// ==============================
//        Hardware-Setup
// ==============================
#define SERIAL_PORT    Serial1
#define SERIAL_PORT_2  Serial2
#define R_SENSE        0.11f
#define I2C_ADDR       0x13

// UART-Adressen (pro Port 0b00..0b11)
#define ADDR_PLF       0b00
#define ADDR_BAND      0b01
#define ADDR_PUMPE1    0b10
#define ADDR_PUMPE2    0b11

#define ADDR_PUMPE3    0b00
#define ADDR_PUMPE4    0b01
#define ADDR_PUMPE5    0b10
#define ADDR_PUMPE6    0b11

// Gemeinsamer Enable (LOW = aktiv)
#define EN_PIN         2

// STEP/DIR-Pins (Beispiel: Arduino MEGA – bitte anpassen!)
#define STEP_PIN_PLF   3
#define DIR_PIN_PLF    4

#define STEP_PIN_BAND  5
#define DIR_PIN_BAND   6

#define STEP_PIN_PUMPE1 22
#define DIR_PIN_PUMPE1  23
#define STEP_PIN_PUMPE2 24
#define DIR_PIN_PUMPE2  25
#define STEP_PIN_PUMPE3 26
#define DIR_PIN_PUMPE3  27
#define STEP_PIN_PUMPE4 28
#define DIR_PIN_PUMPE4  29
#define STEP_PIN_PUMPE5 30
#define DIR_PIN_PUMPE5  31
#define STEP_PIN_PUMPE6 32
#define DIR_PIN_PUMPE6  33

// Sensoren
#define HOME_PIN       13  // nur PLF

// ==============================
//        Objekte
// ==============================
TMC2209Stepper driver_plf   (&SERIAL_PORT,   R_SENSE, ADDR_PLF);
TMC2209Stepper driver_band  (&SERIAL_PORT,   R_SENSE, ADDR_BAND);
TMC2209Stepper driver_pumpe1(&SERIAL_PORT,   R_SENSE, ADDR_PUMPE1);
TMC2209Stepper driver_pumpe2(&SERIAL_PORT,   R_SENSE, ADDR_PUMPE2);
TMC2209Stepper driver_pumpe3(&SERIAL_PORT_2, R_SENSE, ADDR_PUMPE3);
TMC2209Stepper driver_pumpe4(&SERIAL_PORT_2, R_SENSE, ADDR_PUMPE4);
TMC2209Stepper driver_pumpe5(&SERIAL_PORT_2, R_SENSE, ADDR_PUMPE5);
TMC2209Stepper driver_pumpe6(&SERIAL_PORT_2, R_SENSE, ADDR_PUMPE6);

AccelStepper PLF    (AccelStepper::DRIVER, STEP_PIN_PLF,    DIR_PIN_PLF);
AccelStepper BAND   (AccelStepper::DRIVER, STEP_PIN_BAND,   DIR_PIN_BAND);
AccelStepper PUMPE1 (AccelStepper::DRIVER, STEP_PIN_PUMPE1, DIR_PIN_PUMPE1);
AccelStepper PUMPE2 (AccelStepper::DRIVER, STEP_PIN_PUMPE2, DIR_PIN_PUMPE2);
AccelStepper PUMPE3 (AccelStepper::DRIVER, STEP_PIN_PUMPE3, DIR_PIN_PUMPE3);
AccelStepper PUMPE4 (AccelStepper::DRIVER, STEP_PIN_PUMPE4, DIR_PIN_PUMPE4);
AccelStepper PUMPE5 (AccelStepper::DRIVER, STEP_PIN_PUMPE5, DIR_PIN_PUMPE5);
AccelStepper PUMPE6 (AccelStepper::DRIVER, STEP_PIN_PUMPE6, DIR_PIN_PUMPE6);


// ==============================
//   I2C-Variablen
// ==============================
volatile uint8_t  aufgabe_i2c = 0;
volatile int16_t  abstand_i2c = 0;  // signed int8 genutzt
volatile bool     new_message = false;
volatile uint8_t  last_cmd    = 0;

// ==============================
//   Helper-Funktionen
// ==============================
inline long unitsToSteps(int units) { return (long)units * (long)STEPS_PER_UNIT; }

void driverCommonInit(TMC2209Stepper& d, int microsteps, int current_mA) {
  d.begin();
  d.pdn_disable(true);
  d.mstep_reg_select(true);
  d.toff(8);
  d.microsteps(microsteps);
  d.en_spreadCycle(true);
  d.pwm_autoscale(true);
  d.I_scale_analog(false);
  d.vsense(true);
  d.ihold(29);
  d.irun(31);
  d.iholddelay(0);
  d.rms_current(current_mA);
}

void stepperCommonInit(AccelStepper& s, float vmax, float a) {
  s.setMaxSpeed(vmax);
  s.setAcceleration(a);
  s.setMinPulseWidth(5);
}

// ---- Bewegungs-API (nur PLF) ----
void fahren(long units) { PLF.moveTo(unitsToSteps((int)units)); }

bool home() {
  const float HOME_SPEED = -400.0f;
  const float HOME_ACCEL =  8000.0f;
  const long  BACKOFF_STEPS = 800;
  const unsigned long TIMEOUT_MS = 20000;

  float oldMax = PLF.maxSpeed();
  float oldAcc = PLF.acceleration();
  float homeAbs = (HOME_SPEED < 0) ? -HOME_SPEED : HOME_SPEED;

  PLF.setAcceleration(HOME_ACCEL);
  PLF.setMaxSpeed(homeAbs);
  PLF.setSpeed(HOME_SPEED);

  unsigned long t0 = millis();
  while (digitalRead(HOME_PIN) == HIGH) {
    PLF.runSpeed();
    if (millis() - t0 > TIMEOUT_MS) { PLF.setMaxSpeed(oldMax); PLF.setAcceleration(oldAcc); return false; }
  }
  PLF.stop();
  while (PLF.isRunning()) PLF.run();
  PLF.move(BACKOFF_STEPS);
  while (PLF.distanceToGo() != 0) PLF.run();

  PLF.setCurrentPosition(0);
  PLF.setMaxSpeed(oldMax);
  PLF.setAcceleration(oldAcc);
  return true;
}

void pumpe() {

//Platzhalter

}

// ==============================
//           I2C-Handler
// ==============================
void onI2CReceive(int count) {
  if (count <= 0) { while (Wire.available()) (void)Wire.read(); return; }
  uint8_t cmd = Wire.read();
  int16_t par = 0;
  if (count >= 2 && Wire.available()) par = (int8_t)Wire.read();
  aufgabe_i2c = cmd;
  abstand_i2c = par;
  last_cmd    = cmd;
  new_message = true;
  while (Wire.available()) (void)Wire.read();
}

void onI2CRequest() {
  if (last_cmd == CMD_STATUS) {
    uint8_t out[5];
    bool busy = (PLF.distanceToGo() != 0) || (PLF.speed() != 0);
    long pos  = PLF.currentPosition();
    out[0] = busy ? 1 : 0;
    out[1] = (uint8_t)(pos & 0xFF);
    out[2] = (uint8_t)((pos >> 8) & 0xFF);
    out[3] = (uint8_t)((pos >> 16) & 0xFF);
    out[4] = (uint8_t)((pos >> 24) & 0xFF);
    Wire.write(out, 5);
    return;
  }
  const uint8_t ack = 0x06;
  Wire.write(&ack, 1);
}

// ==============================
//         Setup / Loop
// ==============================
void setup() {
  // I2C
  Wire.begin(I2C_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  // UART
  SERIAL_PORT.begin(57600);
  SERIAL_PORT_2.begin(57600);

  // Pins
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  pinMode(HOME_PIN, INPUT_PULLUP);

  // Treiber/Stepper initialisieren
  for (int i = 0; i < 8; ++i) {
    driverCommonInit(*DRV[i], steps_u[i], current_mA[i]);
    stepperCommonInit(*MOT[i],  max_speed[i], accel_sps2[i]);
    MOT[i]->setSpeed(0);
    MOT[i]->moveTo(MOT[i]->currentPosition()); // distanceToGo = 0
  }
}

void loop() {
  // Nur PLF betreiben
  PLF.run();

  if (new_message) {
    noInterrupts();
    uint8_t cmd = aufgabe_i2c; int16_t par = abstand_i2c;
    new_message = false; interrupts();

    switch (cmd) {
      case CMD_FAHR:   fahren(par); break;
      case CMD_HOME:   (void)home(); break;
      case CMD_STATUS: break; // Antwort in onI2CRequest()
      case CMD_PUMPE: pumpe();
      default: break;
    }
  }
}
