#include <TMCStepper.h>
#include <AccelStepper.h>
#include <Wire.h>

// ==============================
//       Konfiguration
// ==============================
const int freifahrts_zeit = 1;   // Sekunden – nicht-blockierender Delay
const int SPEED_CONVEYOR  = 800; // Steps/s für Band & Ebenen-Förderband

// Motorbelegung
// Aufzug = Motor 1
// Richtung = Motor 2
// Band    = Motor 3

// ==============================
//       Befehls-Codes (CMD_ID)
// ==============================
#define CMD_FAHREN  0
#define CMD_HOME    1
#define CMD_STATUS  2

// ==============================
//        Hardware-Setup
// ==============================
#define SERIAL_PORT     Serial1
#define SERIAL_PORT_2   Serial2
#define R_SENSE         0.11f
#define I2C_ADDR        0x12

// UART-Adressen Lift-Treiber
#define ADDR_AUFZUG         0b00
#define ADDR_RICHTUNG       0b01
#define ADDR_BAND           0b10

// UART-Adressen Förderband (weitere)
#define ADDR_EBENE1         0b00
#define ADDR_EBENE2         0b01
#define ADDR_EBENE3         0b10

// Gemeinsamer Enable (LOW = aktiv)
#define EN_PIN          2

// STEP/DIR pro Motor
#define STEP_PIN_AUFZUG     3
#define DIR_PIN_AUFZUG      4
#define STEP_PIN_RICHTUNG   5
#define DIR_PIN_RICHTUNG    6
#define STEP_PIN_BAND       7
#define DIR_PIN_BAND        8

#define STEP_PIN_EBENE1     10
#define DIR_PIN_EBENE1      11
#define STEP_PIN_EBENE2     12
#define DIR_PIN_EBENE2      13
#define STEP_PIN_EBENE3     14
#define DIR_PIN_EBENE3      15

// Sensoren (LOGISCHE IDs beibehalten!)
#define HOME_PIN            9
#define Endschalter_Ebene1  16
#define Endschalter_Ebene2  17
#define Endschalter_Ebene3  18 // Achtung: kollidiert mit TX1, wird NICHT mehr als echter Pin benutzt
#define Belegung_Lift       19 // Achtung: kollidiert mit RX1, wird NICHT mehr als echter Pin benutzt

// ==============================
//   HC-SR04 Zuordnung & Params
// ==============================
// Index: 0=Ebene1, 1=Ebene2, 2=Ebene3, 3=Lift-Belegung
const uint8_t SR04_TRIG[4] = {22, 24, 26, 28};
const uint8_t SR04_ECHO[4] = {23, 25, 27, 29};

// Distanz-Schwellen (cm) – anpassen!
const uint16_t EB_DIST_THRESHOLD_CM   = 8;   // Ebene „Endschalter aktiv“, wenn <= 8 cm
const uint16_t LIFT_OCC_THRESHOLD_CM  = 8;   // Lift belegt, wenn <= 8 cm

// Max Echo-Wartezeit (us) ~ Reichweite/58 cm; klein halten, um Stepper nicht zu stören
const unsigned long SR04_TIMEOUT_US = 6000UL; // ~100 cm

// Mess-Cache
volatile uint16_t us_dist_cm[4] = {0,0,0,0};

// ==============================
//        Objekte
// ==============================
TMC2209Stepper driver_aufzug(&SERIAL_PORT, R_SENSE, ADDR_AUFZUG);
TMC2209Stepper driver_richtung(&SERIAL_PORT, R_SENSE, ADDR_RICHTUNG);
TMC2209Stepper driver_band(&SERIAL_PORT, R_SENSE, ADDR_BAND);
TMC2209Stepper driver_Ebene1(&SERIAL_PORT_2, R_SENSE, ADDR_EBENE1);
TMC2209Stepper driver_Ebene2(&SERIAL_PORT_2, R_SENSE, ADDR_EBENE2);
TMC2209Stepper driver_Ebene3(&SERIAL_PORT_2, R_SENSE, ADDR_EBENE3);

AccelStepper aufzug(AccelStepper::DRIVER, STEP_PIN_AUFZUG, DIR_PIN_AUFZUG);
AccelStepper richtung(AccelStepper::DRIVER, STEP_PIN_RICHTUNG, DIR_PIN_RICHTUNG);
AccelStepper band(AccelStepper::DRIVER, STEP_PIN_BAND, DIR_PIN_BAND);
AccelStepper stepper_Ebene1(AccelStepper::DRIVER, STEP_PIN_EBENE1, DIR_PIN_EBENE1);
AccelStepper stepper_Ebene2(AccelStepper::DRIVER, STEP_PIN_EBENE2, DIR_PIN_EBENE2);
AccelStepper stepper_Ebene3(AccelStepper::DRIVER, STEP_PIN_EBENE3, DIR_PIN_EBENE3);

// Arrays für einfache Ebenen-Zuordnung
AccelStepper* const EB_STEPPER[3] = { &stepper_Ebene1, &stepper_Ebene2, &stepper_Ebene3 };
const uint8_t       EB_ENDSTOP[3] = { Endschalter_Ebene1, Endschalter_Ebene2, Endschalter_Ebene3 };

// ==============================
//        Motor Parameter
// ==============================
float max_speed_sps[6]  = { 1000, 600, 600, 1000, 600, 600 };
float accel_sps2[6]     = { 300, 300, 300, 300, 300, 300 };
int   steps[6]          = { 16, 16, 16, 16, 16, 16 };              // Microsteps
int   current[6]        = { 1500, 1500, 1500, 1500, 1500, 1500 };  // mA RMS

// ==============================
//   I2C-Empfangsvariablen
// ==============================
volatile uint8_t  aufgabe_isr = 0;
volatile uint16_t hoehe_isr = 0;
volatile uint8_t  richtung_isr = 0;           // Richtung als Parameter (0/1)
volatile uint8_t  richtung_distanz_isr = 0;   // Distanzparameter
volatile bool     cmd_pending = false;

// Lokale Variablen (im Loop verwendet)
uint8_t  aufgabe = 0;
uint16_t hoehe = 0;
uint8_t  richtung_val = 0;
uint8_t  richtung_distanz = 0;

// ==============================
//   Ebenen-Grenzen
// ==============================
const int hoehe_ebene1 = 10;
const int hoehe_ebene2 = 20;
const int hoehe_ebene3 = 30;

// ==============================
//   Helper-Funktionen
// ==============================
static inline long heightToSteps(uint16_t h) {
  return (long)h * 400L;
}

// Schritte pro Einheit für Richtungsachse (anpassen!)
const long STEPS_PER_UNIT_DIR = 400;
static inline long distToSteps(uint16_t d) {
  return (long)d * STEPS_PER_UNIT_DIR;
}

// Bestimme Ebene aus Höhe (0..2) oder -1 wenn > Ebene3
static inline int levelFromHeight(uint16_t h) {
  if (h <= hoehe_ebene1) return 0;
  if (h <= hoehe_ebene2) return 1;
  if (h <= hoehe_ebene3) return 2;
  return -1;
}

// ==============================
//   HC-SR04: Messung & Abstraktion
// ==============================
static inline uint16_t sr04_read_cm(uint8_t trig, uint8_t echo, unsigned long timeout_us) {
  // Trigger-Impuls
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Echo-Laufzeit messen (blockiert max. timeout_us)
  unsigned long dur = pulseIn(echo, HIGH, timeout_us); // us
  if (dur == 0) return 0;                              // 0 = kein Echo
  return (uint16_t)(dur / 58UL);                       // us -> cm
}

// Pro Aufruf genau EINEN Sensor messen (Round-Robin), geringe Blockzeit
void updateUltraschall() {
  static uint8_t  idx = 0;
  static uint32_t last_ms = 0;

  const uint32_t PERIOD_MS = 40;           // Messrate pro Sensor (~25 Hz / Sensor)
  uint32_t now = millis();
  if (now - last_ms < PERIOD_MS) return;
  last_ms = now;

  us_dist_cm[idx] = sr04_read_cm(SR04_TRIG[idx], SR04_ECHO[idx], SR04_TIMEOUT_US);
  idx = (uint8_t)((idx + 1) & 0x03);       // 0..3
}

// Abfrage-Funktion: mappt logische „Pins“ auf SR04-Distanzen
inline bool endschalterAktiv(uint8_t pin) {
  if (pin == Endschalter_Ebene1) return (us_dist_cm[0] > 0 && us_dist_cm[0] <= EB_DIST_THRESHOLD_CM);
  if (pin == Endschalter_Ebene2) return (us_dist_cm[1] > 0 && us_dist_cm[1] <= EB_DIST_THRESHOLD_CM);
  if (pin == Endschalter_Ebene3) return (us_dist_cm[2] > 0 && us_dist_cm[2] <= EB_DIST_THRESHOLD_CM);
  if (pin == Belegung_Lift)      return (us_dist_cm[3] > 0 && us_dist_cm[3] <= LIFT_OCC_THRESHOLD_CM);

  // Fallback: echte digitale Eingänge (werden in diesem Sketch nicht genutzt)
  return digitalRead(pin) == HIGH;
}

// ==============================
//   State-Maschine (generisch)
// ==============================
enum EbeneState {
  EB_IDLE,
  EB_START_MOVE,     // Aufzug fährt zur Ebene
  EB_WAIT_AT_HEIGHT, // Warten bis Höhe/Endschalter erreicht
  EB_RUN_RICHTUNG,   // Richtungsmotor fährt Distanz
  EB_WAIT_RICHTUNG,  // Warten bis Richtung fertig
  EB_CONV_START,     // passender Ebenenmotor + Band starten
  EB_CONV_DELAY,     // Freifahrtszeit nicht-blockierend
  EB_CONV_CHECK      // Abfragen & selektives Stoppen
};
EbeneState eb_state = EB_IDLE;

// Aktiver Auftrag
int       eb_level    = -1;    // 0=Eb1, 1=Eb2, 2=Eb3
uint16_t  eb_h        = 0;
uint8_t   eb_r        = 0;
uint8_t   eb_rdist    = 0;
uint32_t  eb_t0       = 0;     // Zeitmarker Lift
uint32_t  eb_t_conv   = 0;     // Startzeitpunkt Freifahrt
bool      conv_started = false;

// Komfort: Zugriff auf aktiven Ebenen-Stepper & Endschalter
inline AccelStepper* activeEbStepper() { return (eb_level >= 0) ? EB_STEPPER[eb_level] : nullptr; }
inline uint8_t       activeEbEndstop() { return (eb_level >= 0) ? EB_ENDSTOP[eb_level] : 0xFF;   }

// ==============================
//   Ebenen-Logik (generisch)
// ==============================
void ebeneRoutine() {
  AccelStepper* ebStepper = activeEbStepper();
  uint8_t ebEndstopPin    = activeEbEndstop();

  switch (eb_state) {
    case EB_START_MOVE: {
      long target = heightToSteps(eb_h);
      aufzug.moveTo(target);
      eb_t0 = millis();
      eb_state = EB_WAIT_AT_HEIGHT;
    } break;

    case EB_WAIT_AT_HEIGHT:
      if (aufzug.distanceToGo() == 0 || endschalterAktiv(ebEndstopPin)) {
        Serial.print(F("[Ebene ")); Serial.print(eb_level+1); Serial.println(F("] Aufzug: Höhe erreicht"));
        eb_state = EB_RUN_RICHTUNG;
      } else if (millis() - eb_t0 > 20000UL) {
        Serial.print(F("[Ebene ")); Serial.print(eb_level+1); Serial.println(F("] Timeout beim Anfahren der Höhe"));
        aufzug.stop();
        eb_state = EB_IDLE;
      }
      break;

    case EB_RUN_RICHTUNG:
      if (eb_r == 0) richtung.move(distToSteps(eb_rdist));   // vorwärts (relativ)
      else           richtung.move(-distToSteps(eb_rdist));  // rückwärts (relativ)
      eb_state = EB_WAIT_RICHTUNG;
      break;

    case EB_WAIT_RICHTUNG:
      if (richtung.distanceToGo() == 0) {
        Serial.print(F("[Ebene ")); Serial.print(eb_level+1); Serial.println(F("] Richtungsmotor: Ziel erreicht"));
        eb_state = EB_CONV_START;
      }
      break;

    case EB_CONV_START:
      if (!conv_started) {
        // Richtung für aktives Ebenenband + Hauptband setzen
        int v = SPEED_CONVEYOR * ((eb_r == 0) ? +1 : -1);
        if (ebStepper) ebStepper->setSpeed(v);
        band.setSpeed(v);
        conv_started = true;
        eb_t_conv = millis();
      }
      eb_state = EB_CONV_DELAY;
      break;

    case EB_CONV_DELAY: {
      // Dauerlauf weiter bedienen
      if (ebStepper) ebStepper->runSpeed();
      band.runSpeed();

      uint32_t wait_ms = (uint32_t)freifahrts_zeit * 1000UL;
      if (millis() - eb_t_conv >= wait_ms) {
        eb_state = EB_CONV_CHECK;
      }
    } break;

    case EB_CONV_CHECK: {
      // Während der Abfragen weiterlaufen lassen
      if (ebStepper) ebStepper->runSpeed();
      band.runSpeed();

      // 1) Aktives Ebenenband: Endschalter der AKTIVEN Ebene
      if (ebStepper && endschalterAktiv(ebEndstopPin)) {
        ebStepper->setSpeed(0);
      }
      // 2) Haupt-Band: Belegung_Lift (jetzt ebenfalls via SR04)
      if (endschalterAktiv(Belegung_Lift)) {
        band.setSpeed(0);
      }

      // Ende erst, wenn beide stehen
      bool ebStopped = (ebStepper ? (ebStepper->speed() == 0) : true);
      if (ebStopped && band.speed() == 0) {
        conv_started = false;
        eb_state = EB_IDLE;
      }
    } break;

    case EB_IDLE:
    default:
      break;
  }
}

// ==============================
//   Statusabfrage Funktion
// ==============================
void status() {
  // Hier kannst du Statusmeldungen oder Sensorwerte ausgeben
  Serial.print(F("US Ebenen/Lift [cm]: "));
  Serial.print(us_dist_cm[0]); Serial.print(' ');
  Serial.print(us_dist_cm[1]); Serial.print(' ');
  Serial.print(us_dist_cm[2]); Serial.print(' ');
  Serial.print(us_dist_cm[3]); Serial.println();
}

// ------------------------------
//   Setupfunktionen
// ------------------------------
void driverCommonInit(TMC2209Stepper& d, int step, int current_mA) {
  d.begin();
  d.pdn_disable(true);
  d.mstep_reg_select(true);
  d.toff(8);
  d.microsteps(step);
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

// ==============================
//   HOMING-FUNKTION
// ==============================
bool homeLift() {
  const float HOME_SPEED    = -400.0f;  // Schritte/s, negativ = nach unten
  const float HOME_ACCEL    =  800.0f;  // Schritte/s^2
  const long  BACKOFF_STEPS =  800;     // von Schalter wegfahren
  const uint32_t DEBOUNCE_MS = 20;
  const uint32_t TIMEOUT_MS  = 20000;

  pinMode(HOME_PIN, INPUT_PULLUP); // aktiv LOW

  float oldMax = aufzug.maxSpeed();
  float oldAcc = aufzug.acceleration();

  aufzug.setAcceleration(HOME_ACCEL);
  aufzug.setMaxSpeed(fabs(HOME_SPEED));
  aufzug.setSpeed(HOME_SPEED);

  uint32_t t0 = millis();
  while (digitalRead(HOME_PIN) == HIGH) { // HIGH = nicht gedrückt
    aufzug.runSpeed();
    richtung.run();
    band.run();
    if (millis() - t0 > TIMEOUT_MS) {
      aufzug.setMaxSpeed(oldMax);
      aufzug.setAcceleration(oldAcc);
      return false;
    }
  }

  delay(DEBOUNCE_MS);

  aufzug.move(BACKOFF_STEPS);
  while (aufzug.distanceToGo() != 0) {
    aufzug.run();
    richtung.run();
    band.run();
  }

  aufzug.setCurrentPosition(0);
  aufzug.setMaxSpeed(oldMax);
  aufzug.setAcceleration(oldAcc);
  return true;
}

// Vorwärtsdeklaration
void onI2CReceive(int count);

// ==============================
//            Setup
// ==============================
void setup() {
  Wire.begin(I2C_ADDR);
  Wire.onReceive(onI2CReceive);

  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("\nStart Liftsteuerung mit TMC2209 + AccelStepper (alle Ebenen) + HC-SR04"));

  SERIAL_PORT.begin(57600);
  SERIAL_PORT_2.begin(57600);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);    // alle aktiv

  // WICHTIG: keine pinMode() auf den „logischen“ Endschalter-Pins 16..19,
  // damit es keine Konflikte mit Serial1 (18/19) gibt.

  // HC-SR04 vorbereiten
  for (int i = 0; i < 4; ++i) {
    pinMode(SR04_TRIG[i], OUTPUT);
    digitalWrite(SR04_TRIG[i], LOW);
    pinMode(SR04_ECHO[i], INPUT);
  }

  driverCommonInit(driver_aufzug,   steps[0], current[0]);
  driverCommonInit(driver_richtung, steps[1], current[1]);
  driverCommonInit(driver_band,     steps[2], current[2]);
  driverCommonInit(driver_Ebene1,   steps[3], current[3]);
  driverCommonInit(driver_Ebene2,   steps[4], current[4]);
  driverCommonInit(driver_Ebene3,   steps[5], current[5]);

  stepperCommonInit(aufzug,   max_speed_sps[0], accel_sps2[0]);
  stepperCommonInit(richtung, max_speed_sps[1], accel_sps2[1]);
  stepperCommonInit(band,     max_speed_sps[2], accel_sps2[2]);
  stepperCommonInit(stepper_Ebene1, max_speed_sps[3], accel_sps2[3]);
  stepperCommonInit(stepper_Ebene2, max_speed_sps[4], accel_sps2[4]);
  stepperCommonInit(stepper_Ebene3, max_speed_sps[5], accel_sps2[5]);
}

// ==============================
//   I2C-ISR (Receive)
//   Erwartet 5 Bytes: Aufgabe, H_LSB, H_MSB, Richtung, Distanz
// ==============================
void onI2CReceive(int count) {
  if (count < 5) {
    while (Wire.available()) (void)Wire.read();
    return;
  }

  uint8_t b0 = Wire.read();  // Aufgabe (CMD_ID)
  uint8_t b1 = Wire.read();  // Höhe LSB
  uint8_t b2 = Wire.read();  // Höhe MSB
  uint8_t b3 = Wire.read();  // Richtung
  uint8_t b4 = Wire.read();  // Distanz

  while (Wire.available()) (void)Wire.read();

  aufgabe_isr = b0;
  hoehe_isr = (uint16_t)b1 | ((uint16_t)b2 << 8);
  richtung_isr = b3;
  richtung_distanz_isr = b4;

  cmd_pending = true;
}

// ==============================
//             Loop
// ==============================
void loop() {
  // Motoren non-blockierend fahren lassen (Positionsprofile)
  aufzug.run();
  richtung.run();
  band.run();
  stepper_Ebene1.run();
  stepper_Ebene2.run();
  stepper_Ebene3.run();

  // Ultraschall zyklisch updaten (geringe Blockzeit)
  updateUltraschall();

  // Ebenen-Logik zyklisch bedienen
  if (eb_state != EB_IDLE) {
    ebeneRoutine();
  }

  // Neue I2C-Daten vorhanden?
  if (cmd_pending) {
    noInterrupts();
    uint8_t cmd_id = aufgabe_isr;
    uint16_t a = hoehe_isr;
    uint8_t  b = richtung_isr;
    uint8_t  c = richtung_distanz_isr;
    cmd_pending = false;
    interrupts();

    aufgabe = cmd_id;
    hoehe = a; richtung_val = b; richtung_distanz = c;

    Serial.print("CMD="); Serial.print(aufgabe);
    Serial.print(" H=");  Serial.print(hoehe);
    Serial.print(" R=");  Serial.print(richtung_val);
    Serial.print(" D=");  Serial.println(richtung_distanz);

    switch (aufgabe) {
      case CMD_HOME: {
        bool ok = homeLift();
        Serial.println(ok ? F("Homing OK") : F("Homing FAIL"));
      } break;

      case CMD_STATUS:
        status();
        break;

      case CMD_FAHREN: {
        int lvl = levelFromHeight(hoehe);
        if (lvl >= 0) {
          // Auftrag für diese Ebene starten
          eb_level     = lvl;
          eb_h         = hoehe;
          eb_r         = richtung_val;
          eb_rdist     = richtung_distanz;
          conv_started = false;

          // Sicherheit: alle Ebenenbänder stoppen außer dem aktiven (Speed=0)
          for (int i = 0; i < 3; ++i) {
            if (EB_STEPPER[i] && i != eb_level) EB_STEPPER[i]->setSpeed(0);
          }
          band.setSpeed(0); // Band erst in EB_CONV_START wieder setzen

          eb_state = EB_START_MOVE;
        } else {
          // Höhe > Ebene 3 -> nur Aufzug absolut anfahren
          aufzug.moveTo(heightToSteps(hoehe));
        }
      } break;

      default:
        Serial.print(F("Unbekannte Aufgabe: "));
        Serial.println(aufgabe);
        break;
    }
  }
}
