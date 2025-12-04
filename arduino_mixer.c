#include <TMCStepper.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <stdint.h>

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

// Sehr große Schrittzahl für quasi "endloses" Fahren
const long CONTINUOUS_STEPS = 100000000L;

float max_speed[12]  = {
  6000.0f,   // PLF
  1000.0f,   // BAND
  100000.0f, // P1
  10000.0f,  // P2
  2500.0f,   // P3
  10000.0f,  // P4
  10000.0f,  // P5
  10000.0f,  // P6
  2500.0f,   // P7
  10000.0f,  // P8
  10000.0f,  // P9
  10000.0f   // P10
};

float accel_sps2[12] = {
  30000.0f, // PLF
  1000.0f,  // BAND
  10000.0f, // P1
  10000.0f, // P2
  10000.0f, // P3
  10000.0f, // P4
  10000.0f, // P5
  10000.0f, // P6
  10000.0f, // P7
  10000.0f, // P8
  10000.0f, // P9
  10000.0f  // P10
};

// Hold-Current (TMC ihold Register, 0..31) pro Motor
uint8_t ihold_vals[12] = {
  29, // PLF
  29, // BAND
  0,  // P1
  0,  // P2
  0,  // P3
  0,  // P4
  0,  // P5
  0,  // P6
  0,  // P7
  0,  // P8
  0,  // P9
  0   // P10
};

const int steps_u[12] = {
  4,  // PLF
  16, // BAND
  32, // P1
  4,  // P2
  4,  // P3
  4,  // P4
  4,  // P5
  4,  // P6
  4,  // P7
  4,  // P8
  4,  // P9
  4   // P10
};

const int current_mA[12] = {
  800, //PLF
  800, //BAND
  800, //P1
  800, //P2
  800, //P3
  800, //P4
  800, //P5
  800, //P6
  800, //P7
  800, //P8
  800, //P9
  800 //P10
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
#define ADDR_PUMPE7    0b00
#define ADDR_PUMPE8    0b01
#define ADDR_PUMPE9    0b10
#define ADDR_PUMPE10   0b11

// PLF:  Step = Pin 50 (PD7), Dir = Pin 49 (PD6)
#define STEP_PIN_PLF     50
#define DIR_PIN_PLF      49

// Band: Step = 47 (PD4), Dir = 41 (PL6)
#define STEP_PIN_BAND    47
#define DIR_PIN_BAND     41

// Pumpe1: Step = 55 (PC2), Dir = 56 (PC3)
#define STEP_PIN_PUMPE1  55
#define DIR_PIN_PUMPE1   56

// Pumpe2: Step = 35 (PL0), Dir = 36 (PL1)
#define STEP_PIN_PUMPE2  35
#define DIR_PIN_PUMPE2   36

// Pumpe3: Step = 6 (PE4), Dir = 5 (PE3)
#define STEP_PIN_PUMPE3  6
#define DIR_PIN_PUMPE3   5

// Pumpe4: Step = 51 (PG0), Dir = 7 (PE5)
#define STEP_PIN_PUMPE4  51
#define DIR_PIN_PUMPE4   7

// Pumpe5: Step = 15 (PH3), Dir = 14 (PH2)
#define STEP_PIN_PUMPE5  15
#define DIR_PIN_PUMPE5   14

// Pumpe6: Step = 2 (PE0/RXD0), Dir = 1 (PG5)
#define STEP_PIN_PUMPE6  2
#define DIR_PIN_PUMPE6   1

// Pumpe7: Step = 91 (PF6/JTAG-TDO/A6), Dir = 90 (PF7/JTAG-TDI/A7)
#define STEP_PIN_PUMPE7  91
#define DIR_PIN_PUMPE7   90

// Pumpe8: Step = 72 (PA6), Dir = 71 (PA7)
#define STEP_PIN_PUMPE8  72
#define DIR_PIN_PUMPE8   71

// Pumpe9: Step = 68 (PJ5), Dir = 67 (PJ4)
#define STEP_PIN_PUMPE9  68
#define DIR_PIN_PUMPE9   67

// Pumpe10: Step = 66 (PJ3), Dir = 60 (PC7)
#define STEP_PIN_PUMPE10 66
#define DIR_PIN_PUMPE10  60

// Gemeinsamer Enable für alle Treiber: Pin 19 (PB0/SS)
#define EN_PIN           19

// Sensoren
#define HOME_PIN       13  // nur PLF (Referenzschalter)
#define SR04_TRIG      34
#define SR04_ECHO      35

// ==============================
//         Objekte
// ==============================
TMC2209Stepper driver_plf    (&SERIAL_PORT,   R_SENSE, ADDR_PLF);
TMC2209Stepper driver_band   (&SERIAL_PORT,   R_SENSE, ADDR_BAND);
TMC2209Stepper driver_pumpe1 (&SERIAL_PORT,   R_SENSE, ADDR_PUMPE1);
TMC2209Stepper driver_pumpe2 (&SERIAL_PORT,   R_SENSE, ADDR_PUMPE2);
TMC2209Stepper driver_pumpe3 (&SERIAL_PORT_2, R_SENSE, ADDR_PUMPE3);
TMC2209Stepper driver_pumpe4 (&SERIAL_PORT_2, R_SENSE, ADDR_PUMPE4);
TMC2209Stepper driver_pumpe5 (&SERIAL_PORT_2, R_SENSE, ADDR_PUMPE5);
TMC2209Stepper driver_pumpe6 (&SERIAL_PORT_2, R_SENSE, ADDR_PUMPE6);
TMC2209Stepper driver_pumpe7 (&SERIAL_PORT_3, R_SENSE, ADDR_PUMPE7);
TMC2209Stepper driver_pumpe8 (&SERIAL_PORT_3, R_SENSE, ADDR_PUMPE8);
TMC2209Stepper driver_pumpe9 (&SERIAL_PORT_3, R_SENSE, ADDR_PUMPE9);
TMC2209Stepper driver_pumpe10(&SERIAL_PORT_3, R_SENSE, ADDR_PUMPE10);

AccelStepper PLF    (AccelStepper::DRIVER, STEP_PIN_PLF,    DIR_PIN_PLF);
AccelStepper BAND   (AccelStepper::DRIVER, STEP_PIN_BAND,   DIR_PIN_BAND);
AccelStepper PUMPE1 (AccelStepper::DRIVER, STEP_PIN_PUMPE1, DIR_PIN_PUMPE1);
AccelStepper PUMPE2 (AccelStepper::DRIVER, STEP_PIN_PUMPE2, DIR_PIN_PUMPE2);
AccelStepper PUMPE3 (AccelStepper::DRIVER, STEP_PIN_PUMPE3, DIR_PIN_PUMPE3);
AccelStepper PUMPE4 (AccelStepper::DRIVER, STEP_PIN_PUMPE4, DIR_PIN_PUMPE4);
AccelStepper PUMPE5 (AccelStepper::DRIVER, STEP_PIN_PUMPE5, DIR_PIN_PUMPE5);
AccelStepper PUMPE6 (AccelStepper::DRIVER, STEP_PIN_PUMPE6, DIR_PIN_PUMPE6);
AccelStepper PUMPE7 (AccelStepper::DRIVER, STEP_PIN_PUMPE7, DIR_PIN_PUMPE7);
AccelStepper PUMPE8 (AccelStepper::DRIVER, STEP_PIN_PUMPE8, DIR_PIN_PUMPE8);
AccelStepper PUMPE9 (AccelStepper::DRIVER, STEP_PIN_PUMPE9, DIR_PIN_PUMPE9);
AccelStepper PUMPE10(AccelStepper::DRIVER, STEP_PIN_PUMPE10, DIR_PIN_PUMPE10);

// ==============================
//       Globale Helfer-Arrays
// ==============================
AccelStepper*   MOT[] = {
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
  d.ihold(ihold_val);
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

// Hilfsfunktion: kontinuierliche Fahrt mit Rampe (großes Ziel)
void startContinuousMove(AccelStepper& m, bool forward) {
  long target = m.currentPosition() + (forward ? CONTINUOUS_STEPS : -CONTINUOUS_STEPS);
  m.moveTo(target);
}

// Hilfsfunktion: sanft anhalten (per stop(), run() kümmert sich dann um Bremsrampe)
void requestSmoothStop(AccelStepper* m) {
  if (!m) return;
  m->stop();
}

// ---- Bewegungs-API ----
void fahren(long units) {
  busy = true;
  PLF.moveTo(unitsToSteps((int)units));  // PLF hat sowieso Rampe via run()
}

void entladen() {
  float band_max_speed = max_speed[1];

  // Pumpe stoppen (sanft)
  if (activePump.motor != nullptr) {
    requestSmoothStop(activePump.motor);
    activePump.motor = nullptr;
    activePump.stopTime = 0;
  }

  // Band ggf. stoppen (alter Task)
  if (activeBandTask.motor != nullptr) {
    requestSmoothStop(activeBandTask.motor);
    // wir überscheiben den Task gleich mit neuem Entlade-Task
  }

  // Band rückwärts laufen lassen (mit Rampe)
  BAND.setMaxSpeed(band_max_speed);
  BAND.setAcceleration(accel_sps2[1]);
  startContinuousMove(BAND, false); // false = rückwärts

  // Timer-Modus aktivieren
  activeBandTask.motor    = &BAND;
  activeBandTask.stopTime = millis() + BAND_TIMEOUT_MS;

  // PLF sanft stoppen, falls er gerade fährt
  PLF.stop();

  busy = true;
}

void beladen() {
  float band_max_speed = max_speed[1];

  // Pumpe stoppen (sanft)
  if (activePump.motor != nullptr) {
    requestSmoothStop(activePump.motor);
    activePump.motor = nullptr;
    activePump.stopTime = 0;
  }

  // Band ggf. stoppen (alter Task)
  if (activeBandTask.motor != nullptr) {
    requestSmoothStop(activeBandTask.motor);
  }

  // Band vorwärts laufen lassen (mit Rampe)
  BAND.setMaxSpeed(band_max_speed);
  BAND.setAcceleration(accel_sps2[1]);
  startContinuousMove(BAND, true); // true = vorwärts

  // Sensor-Modus aktivieren (stopTime = 0)
  activeBandTask.motor    = &BAND;
  activeBandTask.stopTime = 0;

  // PLF sanft stoppen
  PLF.stop();

  busy = true;
}

bool home() {
  // HOMING bleibt wie bisher: direktes runSpeed() ohne "sanftes" Anfahren,
  // dafür sanftes Abbremsen am Ende.
  busy = true;
  const float HOME_SPEED     = -400.0f;
  const float HOME_ACCEL     =  8000.0f;
  const long  BACKOFF_STEPS  = 800;
  const unsigned long TIMEOUT_MS = 20000;

  float oldMax  = PLF.maxSpeed();
  float oldAcc  = PLF.acceleration();
  float homeAbs = (HOME_SPEED < 0) ? -HOME_SPEED : HOME_SPEED;

  PLF.setAcceleration(HOME_ACCEL);
  PLF.setMaxSpeed(homeAbs);
  PLF.setSpeed(HOME_SPEED);

  unsigned long t0 = millis();
  while (digitalRead(HOME_PIN) == HIGH) {
    PLF.runSpeed();  // ohne Rampe anfahren
    if (millis() - t0 > TIMEOUT_MS) {
      PLF.setMaxSpeed(oldMax);
      PLF.setAcceleration(oldAcc);
      busy = false;
      return false;
    }
  }

  PLF.stop();
  while (PLF.isRunning()) PLF.run(); // sanft abbremsen

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

  // gültige Pumpen IDs 1..10
  if (pump_id >= 1 && pump_id <= 10) {
    AccelStepper* motor = MOT[motor_array_index];
    float speed = max_speed[motor_array_index];

    // Falls vorher eine andere Pumpe lief: sanft stoppen
    if (activePump.motor != nullptr && activePump.motor != motor) {
      requestSmoothStop(activePump.motor);
      activePump.motor = nullptr;
      activePump.stopTime = 0;
    }

    if (duration_s > 0) {
      // Start der Pumpe mit Rampe
      motor->setMaxSpeed(speed);
      motor->setAcceleration(accel_sps2[motor_array_index]);
      startContinuousMove(*motor, true);  // Richtung vorwärts

      activePump.motor    = motor;
      activePump.stopTime = millis() + (duration_s * 1000UL);

    } else {
      // dauer == 0 -> explizit stoppen (sanft)
      requestSmoothStop(motor);
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
  if (cmd == CMD_FAHR && count >= 3) {
    uint8_t low  = 0;
    uint8_t high = 0;

    if (Wire.available()) low  = Wire.read();
    if (Wire.available()) high = Wire.read();
    par_generic = (int16_t)((uint16_t)low | ((uint16_t)high << 8));
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
  SERIAL_PORT_3.begin(57600);

  // EN der Treiber
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH); // HIGH = Treiber bei Start deaktiviert

  // Sensor-Pins
  pinMode(HOME_PIN, INPUT_PULLUP);
  pinMode(SR04_TRIG, OUTPUT);
  pinMode(SR04_ECHO, INPUT);

  // alle Stepper + Treiber konfigurieren
  for (int i = 0; i < 12; ++i) {
    driverCommonInit(*DRV[i], steps_u[i], current_mA[i], ihold_vals[i]);
    stepperCommonInit(*MOT[i],  max_speed[i], accel_sps2[i]);
    MOT[i]->setSpeed(0);
    MOT[i]->moveTo(MOT[i]->currentPosition()); // distanceToGo = 0
  }
  digitalWrite(EN_PIN, LOW); // LOW = Treiber werden nach Konfig aktiviert
}

void loop() {
  // 1. PLF bewegen (Zielposition fahren) mit Rampe
  PLF.run();

  // Wenn PLF fertig und weder Band noch Pumpe aktiv sind -> busy false
  if (PLF.distanceToGo() == 0 && PLF.speed() == 0 &&
      activeBandTask.motor == nullptr &&
      activePump.motor == nullptr) {
    busy = false;
  }

  // 2. Bandtask abarbeiten (Timer oder Sensor) mit Rampe
  if (activeBandTask.motor != nullptr) {
    activeBandTask.motor->run();

    if (activeBandTask.stopTime > 0) {
      // Timer-Modus (ENTLADEN)
      if (millis() >= activeBandTask.stopTime) {
        // Sanft stoppen einleiten
        requestSmoothStop(activeBandTask.motor);
        // Wenn vollständig gestoppt: Task beenden
        if (activeBandTask.motor->distanceToGo() == 0 &&
            activeBandTask.motor->speed() == 0) {
          activeBandTask.motor    = nullptr;
          activeBandTask.stopTime = 0;
          busy = false;
        }
      }
    } else {
      // Sensor-Modus (BELADEN)
      static unsigned long lastMeasureMs = 0;
      const unsigned long MEASURE_INTERVAL_MS = 100;
      if (millis() - lastMeasureMs >= MEASURE_INTERVAL_MS) {
        lastMeasureMs = millis();
        float dist = getDistance_cm();
        if (dist > 0 && dist <= DISTANCE_THRESHOLD_CM) {
          // Objekt erkannt -> sanft stoppen
          requestSmoothStop(activeBandTask.motor);
        }
      }

      // Wenn gestoppt -> Task beenden
      if (activeBandTask.motor->distanceToGo() == 0 &&
          activeBandTask.motor->speed() == 0) {
        activeBandTask.motor    = nullptr;
        activeBandTask.stopTime = 0;
        busy = false;
      }
    }
  }

  // 3. Pumpentask abarbeiten (mit Rampe)
  if (activePump.motor != nullptr) {
    activePump.motor->run();

    if (millis() >= activePump.stopTime) {
      // Sanft stoppen einleiten
      requestSmoothStop(activePump.motor);
      // Wenn vollständig gestoppt -> Task beenden
      if (activePump.motor->distanceToGo() == 0 &&
          activePump.motor->speed() == 0) {
        activePump.motor    = nullptr;
        activePump.stopTime = 0;
        busy = false;
      }
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
