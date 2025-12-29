#include "pins.h"
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <stdint.h>
#include <math.h>

// ==============================
//         Konfiguration
// ==============================
static const long STEPS_PER_UNIT = 400;         // konstante für umrechnung Abstand zu Steps für Schlitten
volatile bool busy = false;                     // Variable für Status Beschäftigt
const unsigned long BAND_TIMEOUT_MS = 10000UL;  // Zeit in ms bis das Förderband in das Timeout beim entladen fährt
const int DISTANCE_THRESHOLD_CM = 5;            // Abstand beim Sensor, dass ein Glas erkannt wird
const long CONTINUOUS_STEPS = 100000000L;

// ==============================
//         Motor Parameter
// ==============================
// Maximale Geschwindigkeit für Stepper (steps/s)
float max_speed[12] = {
  60000.0f,  // PLF
  10000.0f,  // BAND
  30000.0f,  // P1
  30000.0f,  // P2
  30000.0f,  // P3
  30000.0f,  // P4
  30000.0f,  // P5
  30000.0f,  // P6
  30000.0f,  // P7
  30000.0f,  // P8
  30000.0f,  // P9
  30000.0f   // P10
};

// Beschleunigung (steps/s^2)
float accel_sps2[12] = {
  30000.0f,  // PLF
  10000.0f,  // BAND
  10000.0f,  // P1
  10000.0f,  // P2
  10000.0f,  // P3
  10000.0f,  // P4
  10000.0f,  // P5
  10000.0f,  // P6
  10000.0f,  // P7
  10000.0f,  // P8
  10000.0f,  // P9
  10000.0f   // P10
};

// Haltestrom (IHOLD 0..31)
uint8_t ihold_vals[12] = {
  29,  // PLF
  15,  // BAND
  1,   // P1
  1,   // P2
  1,   // P3
  1,   // P4
  1,   // P5
  1,   // P6
  1,   // P7
  1,   // P8
  1,   // P9
  1    // P10
};

// Microsteps
const int steps_u[12] = {
  4,   // PLF
  16,  // BAND
  4,   // P1
  4,   // P2
  4,   // P3
  4,   // P4
  4,   // P5
  4,   // P6
  4,   // P7
  4,   // P8
  4,   // P9
  4    // P10
};

// Strom (mA RMS)
const int current_mA[12] = {
  1200,  // PLF
  1200,  // BAND
  1200,  // P1
  1200,  // P2
  1200,  // P3
  1200,  // P4
  1200,  // P5
  1200,  // P6
  1200,  // P7
  1200,  // P8
  1200,  // P9
  1200   // P10
};

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
#define SERIAL_PORT_3  Serial3
#define R_SENSE        0.11f   // interner Shunt
#define I2C_ADDR       0x13    // Slave Adresse

// UART-Adressen (TMC2209 adressen)
#define ADDR_PLF       0b00
#define ADDR_BAND      0b01
#define ADDR_PUMPE1    0b10
#define ADDR_PUMPE2    0b11
#define ADDR_PUMPE3    0b00
#define ADDR_PUMPE4    0b01
#define ADDR_PUMPE5    0b10
#define ADDR_PUMPE6    0b11
#define ADDR_PUMPE7    0b00
#define ADDR_PUMPE8    0b01
#define ADDR_PUMPE9    0b10
#define ADDR_PUMPE10   0b11

// ============================================================
// Motor STEP/DIR Arduino-Pinnummern
// ============================================================
// PLF: PB5 / PA0
static const uint8_t STEP_PIN_PLF = 11;  // PB5
static const uint8_t DIR_PIN_PLF  = 22;  // PA0

// BAND: PB6 / PA1
static const uint8_t STEP_PIN_BAND = 12; // PB6
static const uint8_t DIR_PIN_BAND  = 23; // PA1

// P1: PB7 / PA2
static const uint8_t STEP_PIN_PUMPE1 = 13; // PB7
static const uint8_t DIR_PIN_PUMPE1  = 24; // PA2

// P2 STEP = X4_1 (PE0 / Arduino Pin 0)
static const uint8_t STEP_PIN_PUMPE2 = 0;  // X4_1 = PE0
static const uint8_t DIR_PIN_PUMPE2  = 25; // PA3

// P3 STEP = X4_2 (PE1 / Arduino Pin 1)
static const uint8_t STEP_PIN_PUMPE3 = 1;  // X4_2 = PE1
static const uint8_t DIR_PIN_PUMPE3  = 26; // PA4

// P4 STEP = X4_4 (PE3 / Arduino Pin 5)
static const uint8_t STEP_PIN_PUMPE4 = 5;  // X4_4 = PE3
static const uint8_t DIR_PIN_PUMPE4  = 27; // PA5

// P5: PH3 / PA6
static const uint8_t STEP_PIN_PUMPE5 = 6;  // PH3
static const uint8_t DIR_PIN_PUMPE5  = 28; // PA6

// P6: PH4 / PA7
static const uint8_t STEP_PIN_PUMPE6 = 7;  // PH4
static const uint8_t DIR_PIN_PUMPE6  = 29; // PA7

// P7: PH5 / PC0
static const uint8_t STEP_PIN_PUMPE7 = 8;  // PH5
static const uint8_t DIR_PIN_PUMPE7  = 37; // PC0

// P8: PL3 / PC1
static const uint8_t STEP_PIN_PUMPE8 = 46; // PL3
static const uint8_t DIR_PIN_PUMPE8  = 36; // PC1

// P9: PL4 / PC2
static const uint8_t STEP_PIN_PUMPE9 = 45; // PL4
static const uint8_t DIR_PIN_PUMPE9  = 35; // PC2

// P10: PL5 / PC3
static const uint8_t STEP_PIN_PUMPE10 = 44; // PL5
static const uint8_t DIR_PIN_PUMPE10  = 34; // PC3

// EN = PL6
static const uint8_t EN_PIN_NUM = 43; // PL6

// ==============================
//         Objekte
// ==============================
TMC2209Stepper driver_plf     (&SERIAL_PORT,   R_SENSE, ADDR_PLF);
TMC2209Stepper driver_band    (&SERIAL_PORT,   R_SENSE, ADDR_BAND);
TMC2209Stepper driver_pumpe1  (&SERIAL_PORT,   R_SENSE, ADDR_PUMPE1);
TMC2209Stepper driver_pumpe2  (&SERIAL_PORT,   R_SENSE, ADDR_PUMPE2);
TMC2209Stepper driver_pumpe3  (&SERIAL_PORT_2, R_SENSE, ADDR_PUMPE3);
TMC2209Stepper driver_pumpe4  (&SERIAL_PORT_2, R_SENSE, ADDR_PUMPE4);
TMC2209Stepper driver_pumpe5  (&SERIAL_PORT_2, R_SENSE, ADDR_PUMPE5);
TMC2209Stepper driver_pumpe6  (&SERIAL_PORT_2, R_SENSE, ADDR_PUMPE6);
TMC2209Stepper driver_pumpe7  (&SERIAL_PORT_3, R_SENSE, ADDR_PUMPE7);
TMC2209Stepper driver_pumpe8  (&SERIAL_PORT_3, R_SENSE, ADDR_PUMPE8);
TMC2209Stepper driver_pumpe9  (&SERIAL_PORT_3, R_SENSE, ADDR_PUMPE9);
TMC2209Stepper driver_pumpe10 (&SERIAL_PORT_3, R_SENSE, ADDR_PUMPE10);

AccelStepper PLF     (AccelStepper::DRIVER, STEP_PIN_PLF,     DIR_PIN_PLF);
AccelStepper BAND    (AccelStepper::DRIVER, STEP_PIN_BAND,    DIR_PIN_BAND);
AccelStepper PUMPE1  (AccelStepper::DRIVER, STEP_PIN_PUMPE1,  DIR_PIN_PUMPE1);
AccelStepper PUMPE2  (AccelStepper::DRIVER, STEP_PIN_PUMPE2,  DIR_PIN_PUMPE2);
AccelStepper PUMPE3  (AccelStepper::DRIVER, STEP_PIN_PUMPE3,  DIR_PIN_PUMPE3);
AccelStepper PUMPE4  (AccelStepper::DRIVER, STEP_PIN_PUMPE4,  DIR_PIN_PUMPE4);
AccelStepper PUMPE5  (AccelStepper::DRIVER, STEP_PIN_PUMPE5,  DIR_PIN_PUMPE5);
AccelStepper PUMPE6  (AccelStepper::DRIVER, STEP_PIN_PUMPE6,  DIR_PIN_PUMPE6);
AccelStepper PUMPE7  (AccelStepper::DRIVER, STEP_PIN_PUMPE7,  DIR_PIN_PUMPE7);
AccelStepper PUMPE8  (AccelStepper::DRIVER, STEP_PIN_PUMPE8,  DIR_PIN_PUMPE8);
AccelStepper PUMPE9  (AccelStepper::DRIVER, STEP_PIN_PUMPE9,  DIR_PIN_PUMPE9);
AccelStepper PUMPE10 (AccelStepper::DRIVER, STEP_PIN_PUMPE10, DIR_PIN_PUMPE10);

// ==============================
//       Globale Helfer-Arrays
// ==============================
AccelStepper* MOT[] = {
  &PLF, &BAND,
  &PUMPE1, &PUMPE2, &PUMPE3, &PUMPE4,
  &PUMPE5, &PUMPE6, &PUMPE7, &PUMPE8, &PUMPE9, &PUMPE10
};

TMC2209Stepper* DRV[] = {
  &driver_plf, &driver_band,
  &driver_pumpe1, &driver_pumpe2,
  &driver_pumpe3, &driver_pumpe4,
  &driver_pumpe5, &driver_pumpe6,
  &driver_pumpe7, &driver_pumpe8,
  &driver_pumpe9, &driver_pumpe10
};

// ==============================
//   Task-Status-Strukturen
// ==============================
struct PumpTask {
  AccelStepper* motor;
  unsigned long stopTime;
  bool stopping;                // <--- NEU: stop() wurde schon ausgelöst
};
PumpTask activePump = { nullptr, 0, false };

struct BandTask {
  AccelStepper* motor;
  unsigned long stopTime;       // >0: Timer (Entladen), 0: Sensor (Beladen)
  bool stopping;                // <--- NEU: stop() wurde schon ausgelöst
};
BandTask activeBandTask = { nullptr, 0, false };

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

// Umrechnung Abstand zu Steps
inline long unitsToSteps(int units) {
  return (long)units * (long)STEPS_PER_UNIT;
}

// Treiber initialisierung
void driverCommonInit(TMC2209Stepper& d, int microsteps, int current_mA, uint8_t ihold_val) {
  d.begin();
  d.pdn_disable(true);
  d.mstep_reg_select(true);
  d.toff(8);
  d.microsteps(microsteps);
  d.en_spreadCycle(true);
  d.pwm_autoscale(true);
  d.I_scale_analog(false);
  d.vsense(true);

  // Wichtig: erst Run-Strom in mA setzen, dann Hold-Wert explizit setzen
  d.rms_current(current_mA);
  d.ihold(ihold_val);
  d.iholddelay(0);
}

void stepperCommonInit(AccelStepper& s, float vmax, float a) {
  s.setMaxSpeed(vmax);
  s.setAcceleration(a);
  s.setMinPulseWidth(5);
}

// --- Port-basierte pulseIn für SR04_ECHO (pins.h) ---
static inline unsigned long pulseInEchoHigh_timeout(uint32_t timeout_us) {
  const uint32_t start = micros();

  while (PIN_READ(SR04_ECHO)) {
    if ((uint32_t)(micros() - start) > timeout_us) return 0;
  }
  while (!PIN_READ(SR04_ECHO)) {
    if ((uint32_t)(micros() - start) > timeout_us) return 0;
  }

  const uint32_t t0 = micros();
  while (PIN_READ(SR04_ECHO)) {
    if ((uint32_t)(micros() - start) > timeout_us) return 0;
  }
  return (unsigned long)(micros() - t0);
}

// Ultraschallsensor messen (HC-SR04) über pins.h
float getDistance_cm() {
  PIN_LOW(SR04_TRIG);
  delayMicroseconds(2);
  PIN_HIGH(SR04_TRIG);
  delayMicroseconds(10);
  PIN_LOW(SR04_TRIG);

  unsigned long duration = pulseInEchoHigh_timeout(10000UL);
  if (duration == 0) return -1.0f;
  return (float)duration / 58.0f;
}

void startContinuousMove(AccelStepper& m, bool forward) {
  long target = m.currentPosition() + (forward ? CONTINUOUS_STEPS : -CONTINUOUS_STEPS);
  m.moveTo(target);
}

void requestSmoothStop(AccelStepper* m) {
  if (!m) return;
  m->stop(); // AccelStepper: weiches Abbremsen
}

// ==============================
//         Bewegungsfunktionen
// ==============================

// Fahr funktion (Abstand wird in Funktion unitsToSteps berechnet)
void fahren(long units) {
  busy = true;
  PLF.moveTo(unitsToSteps((int)units));
}

// Entlade Funktion (Timer)
void entladen() {
  float band_max_speed = max_speed[1];

  // Pumpe stoppen falls aktiv
  if (activePump.motor != nullptr) {
    requestSmoothStop(activePump.motor);
    activePump.motor = nullptr;
    activePump.stopTime = 0;
    activePump.stopping = false;
  }

  // Band-Task ggf. stoppen
  if (activeBandTask.motor != nullptr) {
    requestSmoothStop(activeBandTask.motor);
  }

  BAND.setMaxSpeed(band_max_speed);
  BAND.setAcceleration(accel_sps2[1]);
  startContinuousMove(BAND, false);

  activeBandTask.motor    = &BAND;
  activeBandTask.stopTime = millis() + BAND_TIMEOUT_MS;
  activeBandTask.stopping = false;

  PLF.stop();
  busy = true;
}

// belade Funktion (Sensor)
void beladen() {
  float band_max_speed = max_speed[1];

  // Pumpe stoppen falls aktiv
  if (activePump.motor != nullptr) {
    requestSmoothStop(activePump.motor);
    activePump.motor = nullptr;
    activePump.stopTime = 0;
    activePump.stopping = false;
  }

  // Band-Task ggf. stoppen
  if (activeBandTask.motor != nullptr) {
    requestSmoothStop(activeBandTask.motor);
  }

  BAND.setMaxSpeed(band_max_speed);
  BAND.setAcceleration(accel_sps2[1]);
  startContinuousMove(BAND, true);

  activeBandTask.motor    = &BAND;
  activeBandTask.stopTime = 0;     // Sensor-Modus
  activeBandTask.stopping = false;

  PLF.stop();
  busy = true;
}

// home Funktion
bool home() {
  busy = true;

  const float HOME_SPEED = -4000.0f;
  const unsigned long TIMEOUT_MS = 20000;

  float oldMax = PLF.maxSpeed();
  float oldAcc = PLF.acceleration();

  PLF.setMaxSpeed(fabsf(HOME_SPEED));
  PLF.setAcceleration(accel_sps2[0]);
  PLF.setSpeed(HOME_SPEED);

  unsigned long t0 = millis();

  while (PIN_READ(HOME)) {
    PLF.runSpeed();
    if (millis() - t0 > TIMEOUT_MS) {
      PLF.setMaxSpeed(oldMax);
      PLF.setAcceleration(oldAcc);
      busy = false;
      return false;
    }
  }

  PLF.setSpeed(0);
  PLF.moveTo(PLF.currentPosition());
  PLF.setCurrentPosition(0);

  PLF.setMaxSpeed(oldMax);
  PLF.setAcceleration(oldAcc);

  busy = false;
  return true;
}

// Pumpen Funktion
void pumpe() {
  busy = true;

  uint8_t pump_id    = pumpe_id_i2c;
  long    duration_s = zeit_sek_i2c;
  int     motor_array_index = pump_id + 1; // Achtung: Array: 0=PLF,1=BAND,2=P1... => pump_id(1) -> index 2

  if (pump_id >= 1 && pump_id <= 10) {
    AccelStepper* motor = MOT[motor_array_index];
    float speed = max_speed[motor_array_index];

    // Falls andere Pumpe aktiv -> stoppen
    if (activePump.motor != nullptr && activePump.motor != motor) {
      requestSmoothStop(activePump.motor);
      activePump.motor = nullptr;
      activePump.stopTime = 0;
      activePump.stopping = false;
    }

    if (duration_s > 0) {
      motor->setMaxSpeed(speed);
      motor->setAcceleration(accel_sps2[motor_array_index]);
      startContinuousMove(*motor, true);

      activePump.motor    = motor;
      activePump.stopTime = millis() + (duration_s * 1000UL);
      activePump.stopping = false;   // <--- wichtig
    } else {
      // duration_s == 0 => Stop
      requestSmoothStop(motor);
      activePump.motor = nullptr;
      activePump.stopTime = 0;
      activePump.stopping = false;
      busy = false;
    }
  } else {
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

  if (cmd == CMD_FAHR) {
    if (count >= 3 && Wire.available() >= 2) {
      uint8_t low  = Wire.read();
      uint8_t high = Wire.read();
      par_generic = (int16_t)((uint16_t)low | ((uint16_t)high << 8));
      while (Wire.available()) (void)Wire.read();
    } else if (count >= 2 && Wire.available() >= 1) {
      uint8_t b = Wire.read();
      par_generic = (int16_t)((int8_t)b);  // SIGN EXTEND int8 -> int16
      while (Wire.available()) (void)Wire.read();
    } else {
      while (Wire.available()) (void)Wire.read();
    }

    new_message = true;
    return;
  }

  if (cmd == CMD_PUMPE) {
    if (count >= 3 && Wire.available() >= 2) {
      pumpe_id_i2c = Wire.read();
      zeit_sek_i2c = Wire.read();
    }
    while (Wire.available()) (void)Wire.read();
    new_message = true;
    return;
  }

  while (Wire.available()) (void)Wire.read();
  new_message = true;
}

// Status abfrage
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

  const uint8_t ack = 0x06;
  Wire.write(&ack, 1);
}

// ==============================
//         Setup / Loop
// ==============================
void setup() {
  Wire.begin(I2C_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  SERIAL_PORT.begin(57600);
  SERIAL_PORT_2.begin(57600);
  SERIAL_PORT_3.begin(57600);

  pinMode(EN_PIN_NUM, OUTPUT);
  digitalWrite(EN_PIN_NUM, HIGH); // deaktiviert

  // HOME: X1_4 (Pullup)
  PIN_INPUT(HOME);
  PIN_PULLUP_ON(HOME);

  // SR04: TRIG = X1_3, ECHO = X1_2
  PIN_OUTPUT(SR04_TRIG);
  PIN_LOW(SR04_TRIG);
  PIN_INPUT(SR04_ECHO);

  for (int i = 0; i < 12; ++i) {
    driverCommonInit(*DRV[i], steps_u[i], current_mA[i], ihold_vals[i]);
    stepperCommonInit(*MOT[i], max_speed[i], accel_sps2[i]);
    MOT[i]->setSpeed(0);
    MOT[i]->moveTo(MOT[i]->currentPosition());
  }

  digitalWrite(EN_PIN_NUM, LOW); // aktiv
}

void loop() {
  PLF.run();

  // Sicherheitsabfrage ob was gemacht wird; wenn nicht, busy=false
  if (PLF.distanceToGo() == 0 && PLF.speed() == 0 &&
      activeBandTask.motor == nullptr &&
      activePump.motor == nullptr) {
    busy = false;
  }

  // ==========================
  // Band Task
  // ==========================
  if (activeBandTask.motor != nullptr) {
    activeBandTask.motor->run();

    if (activeBandTask.stopTime > 0) {
      // Timer-Modus (Entladen)
      if (!activeBandTask.stopping && millis() >= activeBandTask.stopTime) {
        activeBandTask.motor->stop();        // stop() NUR EINMAL
        activeBandTask.stopping = true;
      }

      if (activeBandTask.stopping && activeBandTask.motor->distanceToGo() == 0) {
        activeBandTask.motor = nullptr;
        activeBandTask.stopTime = 0;
        activeBandTask.stopping = false;
        busy = false;
      }

    } else {
      // Sensor-Modus (Beladen)
      static unsigned long lastMeasureMs = 0;
      const unsigned long MEASURE_INTERVAL_MS = 100;

      if (!activeBandTask.stopping && (millis() - lastMeasureMs >= MEASURE_INTERVAL_MS)) {
        lastMeasureMs = millis();
        float dist = getDistance_cm();
        if (dist > 0 && dist <= DISTANCE_THRESHOLD_CM) {
          activeBandTask.motor->stop();      // stop() NUR EINMAL
          activeBandTask.stopping = true;
        }
      }

      if (activeBandTask.stopping && activeBandTask.motor->distanceToGo() == 0) {
        activeBandTask.motor = nullptr;
        activeBandTask.stopTime = 0;
        activeBandTask.stopping = false;
        busy = false;
      }
    }
  }

  // ==========================
  // Pump Task
  // ==========================
  if (activePump.motor != nullptr) {
    activePump.motor->run();

    // Stop nur EINMAL auslösen (sonst kann er "nie fertig werden")
    if (!activePump.stopping && millis() >= activePump.stopTime) {
      activePump.motor->stop();
      activePump.stopping = true;
    }

    // Wenn er fertig abgebremst hat -> Task beenden
    if (activePump.stopping && activePump.motor->distanceToGo() == 0) {
      activePump.motor = nullptr;
      activePump.stopTime = 0;
      activePump.stopping = false;
      busy = false;
    }
  }

  // ==========================
  // I2C Kommandos ausführen
  // ==========================
  if (new_message) {
    noInterrupts();
    uint8_t cmd = aufgabe_i2c;
    int16_t par = par_generic;
    new_message = false;
    interrupts();

    switch (cmd) {
      case CMD_FAHR:     fahren(par);   break;
      case CMD_HOME:     (void)home();  break;
      case CMD_STATUS:   break;
      case CMD_PUMPE:    pumpe();       break;
      case CMD_ENTLADEN: entladen();    break;
      case CMD_BELADEN:  beladen();     break;
      default: break;
    }
  }
}
