#include <TMCStepper.h>
#include <AccelStepper.h>
#include <Wire.h>

// ==============================
//         Konfiguration
// ==============================
static const long STEPS_PER_UNIT = 400; // 1 Einheit = 400 Steps

// Globale Busy-Variable (volatile, da sie in ISRs und Loop verwendet wird)
volatile bool busy = false;

// Zeitkonstante für Entladen (in Millisekunden)
const unsigned long BAND_TIMEOUT_MS = 10000UL; // 10 Sekunden Laufzeit Entladen

// Schwellenwert für den Ultraschallsensor (in cm)
const int DISTANCE_THRESHOLD_CM = 20; // Objekt erkannt, wenn Abstand <= 20 cm

// Index in den Arrays unten:
// 0 = PLF
// 1 = BAND
// 2 = PUMPE1
// 3 = PUMPE2
// 4 = PUMPE3
// 5 = PUMPE4
// 6 = PUMPE5
// 7 = PUMPE6
float max_speed[8]  = { 6000.0f, 1000.0f, 100000.0f, 10000.0f, 2500.0f, 10000.0f, 10000.0f, 10000.0f };
float accel_sps2[8] = {30000.0f, 1000.0f, 10000.0f, 10000.0f, 10000.0f, 10000.0f, 10000.0f, 10000.0f};

const int steps_u[8]    = { 4, 16, 32, 4, 4, 4, 4, 4 }; // Microsteps
const int current_mA[8] = { 800, 800, 800, 800, 800, 800, 800, 800 }; // RMS-Strom

// ==============================
//         Befehls-Codes
// ==============================
enum : uint8_t {
  CMD_FAHR     = 0,
  CMD_HOME     = 1,
  CMD_STATUS   = 2,
  CMD_PUMPE    = 3,
  CMD_BELADEN  = 4,
  CMD_ENTLADEN = 5
};

// ==============================
//         Hardware-Setup
// ==============================
#define SERIAL_PORT    Serial1
#define SERIAL_PORT_2  Serial2
#define R_SENSE        0.11f
#define I2C_ADDR       0x13

// UART-Adressen (TMC2209 adressen)
#define ADDR_PLF       0b00
#define ADDR_BAND      0b01
#define ADDR_PUMPE1    0b10
#define ADDR_PUMPE2    0b11
#define ADDR_PUMPE3    0b00
#define ADDR_PUMPE4    0b01
#define ADDR_PUMPE5    0b10
#define ADDR_PUMPE6    0b11

#define EN_PIN         2   // Gemeinsamer Enable für alle Treiber

// STEP/DIR-Pins (deine letzte Version)
#define STEP_PIN_PLF    3
#define DIR_PIN_PLF     4

#define STEP_PIN_BAND   5
#define DIR_PIN_BAND    6

#define STEP_PIN_PUMPE1 7
#define DIR_PIN_PUMPE1  8
#define STEP_PIN_PUMPE2 9
#define DIR_PIN_PUMPE2  10
#define STEP_PIN_PUMPE3 22
#define DIR_PIN_PUMPE3  23
#define STEP_PIN_PUMPE4 24
#define DIR_PIN_PUMPE4  25
#define STEP_PIN_PUMPE5 26
#define DIR_PIN_PUMPE5  27
#define STEP_PIN_PUMPE6 28
#define DIR_PIN_PUMPE6  29

// Sensoren
#define HOME_PIN       13  // nur PLF (Referenzschalter)
#define SR04_TRIG      34
#define SR04_ECHO      35

// ==============================
//         Objekte
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
//       Globale Helfer-Arrays
// ==============================
AccelStepper*   MOT[] = { &PLF, &BAND, &PUMPE1, &PUMPE2, &PUMPE3, &PUMPE4, &PUMPE5, &PUMPE6 };
TMC2209Stepper* DRV[] = { &driver_plf, &driver_band, &driver_pumpe1, &driver_pumpe2,
                          &driver_pumpe3, &driver_pumpe4, &driver_pumpe5, &driver_pumpe6 };

// ==============================
//   Task-Status-Strukturen
// ==============================
struct PumpTask {
  AccelStepper* motor;
  unsigned long stopTime;
};
PumpTask activePump = { nullptr, 0 };

struct BandTask {
  AccelStepper* motor;
  unsigned long stopTime; // >0: Timer (Entladen), 0: Sensor-Modus (Beladen)
};
BandTask activeBandTask = { nullptr, 0 };

// ==============================
//         I2C-Variablen
// ==============================
volatile uint8_t  aufgabe_i2c  = 0;
volatile int16_t  par_generic  = 0;
volatile uint8_t  pumpe_id_i2c = 0;
volatile uint8_t  zeit_sek_i2c = 0;
volatile bool     new_message  = false;
volatile uint8_t  last_cmd     = 0;

// ==============================
//         Helper-Funktionen
// ==============================
inline long unitsToSteps(int units) {
  return (long)units * (long)STEPS_PER_UNIT;
}

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

// Ultraschallsensor messen (HC-SR04)
float getDistance_cm() {
  digitalWrite(SR04_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(SR04_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SR04_TRIG, LOW);

  long duration = pulseIn(SR04_ECHO, HIGH, 10000); // 10 ms Timeout
  if (duration == 0) return -1.0;

  return duration / 58.0; // µs -> cm
}

// ---- Bewegungs-API ----
void fahren(long units) {
  busy = true;
  PLF.moveTo(unitsToSteps((int)units));
}

void entladen() {
  float band_max_speed = max_speed[1];

  // Pumpe stoppen
  if (activePump.motor != nullptr) {
    activePump.motor->setSpeed(0);
    activePump.motor->runSpeed();
    activePump.motor = nullptr;
  }

  // Band stoppen falls vorher aktiv
  if (activeBandTask.motor != nullptr) {
    activeBandTask.motor->setSpeed(0);
    activeBandTask.motor->runSpeed();
  }

  // Band rückwärts laufen lassen
  BAND.setSpeed(-band_max_speed);

  // Timer-Modus aktivieren
  activeBandTask.motor = &BAND;
  activeBandTask.stopTime = millis() + BAND_TIMEOUT_MS;

  // PLF stoppen
  PLF.setSpeed(0);
  PLF.runSpeed();

  busy = true;
}

void beladen() {
  float band_max_speed = max_speed[1];

  // Pumpe stoppen
  if (activePump.motor != nullptr) {
    activePump.motor->setSpeed(0);
    activePump.motor->runSpeed();
    activePump.motor = nullptr;
  }

  // Band stoppen falls vorher aktiv
  if (activeBandTask.motor != nullptr) {
    activeBandTask.motor->setSpeed(0);
    activeBandTask.motor->runSpeed();
  }

  // Band vorwärts laufen lassen
  BAND.setSpeed(band_max_speed);

  // Sensor-Modus aktivieren (stopTime = 0)
  activeBandTask.motor = &BAND;
  activeBandTask.stopTime = 0;

  // PLF stoppen
  PLF.setSpeed(0);
  PLF.runSpeed();

  busy = true;
}

bool home() {
  busy = true;
  const float HOME_SPEED     = -400.0f;
  const float HOME_ACCEL     =  8000.0f;
  const long  BACKOFF_STEPS  = 800;
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
    if (millis() - t0 > TIMEOUT_MS) {
      PLF.setMaxSpeed(oldMax);
      PLF.setAcceleration(oldAcc);
      busy = false;
      return false;
    }
  }

  PLF.stop();
  while (PLF.isRunning()) PLF.run();

  PLF.move(BACKOFF_STEPS);
  while (PLF.distanceToGo() != 0) PLF.run();

  PLF.setCurrentPosition(0);
  PLF.setMaxSpeed(oldMax);
  PLF.setAcceleration(oldAcc);
  busy = false;
  return true;
}

void pumpe() {
  busy = true;

  uint8_t pump_id    = pumpe_id_i2c;
  long    duration_s = zeit_sek_i2c;
  int     motor_array_index = pump_id + 1; // pump_id 1 -> MOT[2], usw.

  // Nur gültige Pumpen (1..6)
  if (pump_id >= 1 && pump_id <= 6) {
    AccelStepper* motor = MOT[motor_array_index];
    float speed = max_speed[motor_array_index];

    // Falls vorher eine andere Pumpe lief: stoppen
    if (activePump.motor != nullptr && activePump.motor != motor) {
      activePump.motor->setSpeed(0);
      activePump.motor->runSpeed();
    }

    if (duration_s > 0) {
      // Start der Pumpe mit fixer Geschwindigkeit
      motor->setSpeed(speed);

      activePump.motor    = motor;
      activePump.stopTime = millis() + (duration_s * 1000UL);

    } else {
      // dauer == 0 -> explizit stoppen
      motor->setSpeed(0);
      motor->runSpeed();

      activePump.motor    = nullptr;
      activePump.stopTime = 0;
      busy = false;
    }

  } else {
    // ungültige Pumpen-ID
    busy = false;
  }
}

// ==============================
//         I2C-Handler
// ==============================
void onI2CReceive(int count) {
  if (count <= 0) {
    while (Wire.available()) (void)Wire.read();
    return;
  }

  uint8_t cmd = Wire.read();
  aufgabe_i2c = cmd;
  last_cmd    = cmd;

  // Parameter je nach Kommando lesen
  if (cmd == CMD_FAHR && count >= 2 && Wire.available()) {
    par_generic = (int8_t)Wire.read();
    while (Wire.available()) (void)Wire.read();
  }

  if ((cmd == CMD_ENTLADEN || cmd == CMD_BELADEN) && count > 1) {
    while (Wire.available()) (void)Wire.read();
  }

  if (cmd == CMD_PUMPE && count >= 3) {
    if (Wire.available()) pumpe_id_i2c = Wire.read();
    if (Wire.available()) zeit_sek_i2c = Wire.read();
    while (Wire.available()) (void)Wire.read();
  } else {
    while (Wire.available()) (void)Wire.read();
  }

  new_message = true;
}

void onI2CRequest() {
  if (last_cmd == CMD_STATUS) {
    uint8_t out[5];

    bool current_busy = busy;
    if (!current_busy) {
      current_busy = (PLF.distanceToGo() != 0) || (PLF.speed() != 0);
    }

    long pos = PLF.currentPosition();

    out[0] = current_busy ? 1 : 0;
    out[1] = (uint8_t)(pos & 0xFF);
    out[2] = (uint8_t)((pos >> 8) & 0xFF);
    out[3] = (uint8_t)((pos >> 16) & 0xFF);
    out[4] = (uint8_t)((pos >> 24) & 0xFF);

    Wire.write(out, 5);
    return;
  }

  // ACK für alle anderen Kommandos
  const uint8_t ack = 0x06;
  Wire.write(&ack, 1);
}

// ==============================
//         Setup / Loop
// ==============================
void setup() {
  // I2C Slave
  Wire.begin(I2C_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  // UART zu den TMC2209 Treibern
  SERIAL_PORT.begin(57600);
  SERIAL_PORT_2.begin(57600);

  // EN der Treiber
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // LOW = Treiber aktiv bei üblichen TMC2209 StepStick-Modulen

  // Sensor-Pins
  pinMode(HOME_PIN, INPUT_PULLUP);
  pinMode(SR04_TRIG, OUTPUT);
  pinMode(SR04_ECHO, INPUT);

  // alle Stepper + Treiber konfigurieren
  for (int i = 0; i < 8; ++i) {
    driverCommonInit(*DRV[i], steps_u[i], current_mA[i]);
    stepperCommonInit(*MOT[i],  max_speed[i], accel_sps2[i]);
    MOT[i]->setSpeed(0);
    MOT[i]->moveTo(MOT[i]->currentPosition()); // distanceToGo = 0
  }
}

void loop() {
  // 1. PLF bewegen (Zielposition fahren)
  PLF.run();

  // 2. Bandtask abarbeiten (Timer oder Sensor)
  if (activeBandTask.motor != nullptr) {
    activeBandTask.motor->runSpeed();

    if (activeBandTask.stopTime > 0) {
      // Timer-Modus (ENTLADEN)
      if (millis() >= activeBandTask.stopTime) {
        activeBandTask.motor->setSpeed(0);
        activeBandTask.motor->runSpeed();
        activeBandTask.motor = nullptr;
        activeBandTask.stopTime = 0;
        busy = false;
      }
    } else {
      // Sensor-Modus (BELADEN)
      static unsigned long lastMeasureMs = 0;
      const unsigned long MEASURE_INTERVAL_MS = 100;
      if (millis() - lastMeasureMs >= MEASURE_INTERVAL_MS) {
        lastMeasureMs = millis();
        float dist = getDistance_cm();
        if (dist > 0 && dist <= DISTANCE_THRESHOLD_CM) {
          // Objekt erkannt -> Band stoppen
          activeBandTask.motor->setSpeed(0);
          activeBandTask.motor->runSpeed();
          activeBandTask.motor = nullptr;
          busy = false;
        }
      }
    }
  }

  // 3. Pumpentask abarbeiten
  if (activePump.motor != nullptr) {
    activePump.motor->runSpeed();

    if (millis() >= activePump.stopTime) {
      activePump.motor->setSpeed(0);
      activePump.motor->runSpeed();
      activePump.motor = nullptr;
      activePump.stopTime = 0;
      busy = false;
    }
  }

  // 4. Neue I2C-Kommandos anwenden
  if (new_message) {
    noInterrupts();
    uint8_t cmd = aufgabe_i2c;
    int16_t par = par_generic;
    new_message = false;
    interrupts();

    switch (cmd) {
      case CMD_FAHR:     fahren(par);    break;
      case CMD_HOME:     (void)home();   break;
      case CMD_STATUS:   /* wird in onI2CRequest beantwortet */ break;
      case CMD_PUMPE:    pumpe();        break;
      case CMD_ENTLADEN: entladen();     break;
      case CMD_BELADEN:  beladen();      break;
      default: break;
    }
  }
}
