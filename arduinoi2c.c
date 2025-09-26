#include <TMCStepper.h>
#include <AccelStepper.h>
#include <Wire.h>

// Motor 1 Aufzug  | Motor 2 Richtung  | Motor 3 Förderband

// ==============================
//       Befehls-Codes (CMD_ID)
// ==============================
#define CMD_FAHREN  0  // Standardbefehl: Motorbewegung ausführen
#define CMD_HOME    1  // Befehl: Homing starten
#define CMD_STATUS  2  // Befehl: Status-Ausgabe auf Serial

// ==============================
//        Hardware-Setup
// ==============================
#define SERIAL_PORT     Serial1    // Mega: RX1=19, TX1=18
#define R_SENSE         0.11f
#define I2C_ADDR        0x12

// UART-Adressen pro Treiber (über MS1/MS2 gesetzt)
#define ADDR_M1         0b00
#define ADDR_M2         0b01
#define ADDR_M3         0b10

// Gemeinsamer Enable (LOW = aktiv)
#define EN_PIN          2

// STEP/DIR pro Motor
#define STEP_PIN_M1     3
#define DIR_PIN_M1      4
#define STEP_PIN_M2     5
#define DIR_PIN_M2      6
#define STEP_PIN_M3     7
#define DIR_PIN_M3      8

// Endschalter (unten, aktiv LOW -> gegen GND schalten)
#define HOME_PIN        9

// ==============================
//        Objekte
// ==============================
TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, ADDR_M1);
TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, ADDR_M2);
TMC2209Stepper driver3(&SERIAL_PORT, R_SENSE, ADDR_M3);

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_M1, DIR_PIN_M1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_M2, DIR_PIN_M2);
AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN_M3, DIR_PIN_M3);

TMC2209Stepper* drivers[3] = { &driver1, &driver2, &driver3 };
AccelStepper* steppers[3] = { &stepper1, &stepper2, &stepper3 };

// ==============================
//        Motor Parameter
// ==============================
long  step_chunk[3]     = { 5000, 5000, 5000 };  // aktuell ungenutzt
float max_speed_sps[3]  = { 1000, 600, 600 };
float accel_sps2[3]     = { 300, 300, 300 };
int   steps[3]          = { 16, 16, 16 };        // Microsteps, ohne Prüfung
int   current[3]        = { 1500, 1500, 1500 };  // mA RMS

// ==============================
//   I2C-Empfangsvariablen (volatile für ISR)
// ==============================
volatile uint8_t  aufgabe_isr = 0;             // NEU: Aufgabe/CMD_ID
volatile uint16_t hoehe_isr = 0;               // Höhe
volatile uint8_t  richtung_isr = 0;            // Richtung
volatile uint8_t  richtung_distanz_isr = 0;    // Distanz
volatile bool     cmd_pending = false;

// Lokale Variablen (im Loop verwendet)
uint8_t  aufgabe = 0;
uint16_t hoehe = 0;
uint8_t  richtung = 0;
uint8_t  richtung_distanz = 0;

// ==============================
//   Ebenen-Grenzen
// ==============================
const int hoehe_ebene1 = 10;
const int hoehe_ebene2 = 20;
const int hoehe_ebene3 = 30;

// ==============================
//   Helper: Höhe -> Steps
// ==============================
static inline long heightToSteps(uint16_t h) {
  return (long)h * 400L;
}

// ==============================
//   Ebenen-Funktionen
// ==============================
void ebene1(uint16_t h, uint8_t r, uint8_t rdist) { stepper1.moveTo(heightToSteps(h)); }
void ebene2(uint16_t h, uint8_t r, uint8_t rdist) { stepper1.moveTo(heightToSteps(h)); }
void ebene3(uint16_t h, uint8_t r, uint8_t rdist) { stepper1.moveTo(heightToSteps(h)); }

// ==============================
//   Statusabfrage Funktion
// ==============================
void status() {

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

  // Endschalter als INPUT_PULLUP erwarten (aktiv LOW)
  pinMode(HOME_PIN, INPUT_PULLUP);

  // temporäre Motion-Parameter merken
  float oldMax = stepper1.maxSpeed();
  float oldAcc = stepper1.acceleration();

  stepper1.setAcceleration(HOME_ACCEL);
  stepper1.setMaxSpeed(fabs(HOME_SPEED));
  stepper1.setSpeed(HOME_SPEED); // für runSpeed()

  // 1) Nach unten fahren, bis Schalter triggert
  uint32_t t0 = millis();
  while (digitalRead(HOME_PIN) == HIGH) { // HIGH = nicht gedrückt
    stepper1.runSpeed();    // konstante Geschwindigkeit
    stepper2.run();
    stepper3.run();
    if (millis() - t0 > TIMEOUT_MS) {
      // Timeout -> alte Parameter wiederherstellen
      stepper1.setMaxSpeed(oldMax);
      stepper1.setAcceleration(oldAcc);
      return false;
    }
  }

  // kleine Entprell-Pause und sicherstellen, dass wir wirklich anliegen
  delay(DEBOUNCE_MS);

  // 2) Von Schalter wegfahren (positiv)
  stepper1.move(BACKOFF_STEPS);
  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }

  // 3) Jetzt Position nullen
  stepper1.setCurrentPosition(0);

  // 4) Ursprungs-Parameter zurück
  stepper1.setMaxSpeed(oldMax);
  stepper1.setAcceleration(oldAcc);

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
  while(!Serial);
  Serial.println(F("\nStart 3x TMC2209 via UART + AccelStepper..."));

  SERIAL_PORT.begin(57600);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);    // alle aktiv

  driverCommonInit(driver1, steps[0], current[0]);
  driverCommonInit(driver2, steps[1], current[1]);
  driverCommonInit(driver3, steps[2], current[2]);

  stepperCommonInit(stepper1, max_speed_sps[0], accel_sps2[0]);
  stepperCommonInit(stepper2, max_speed_sps[1], accel_sps2[1]);
  stepperCommonInit(stepper3, max_speed_sps[2], accel_sps2[2]);

  Serial.print(F("test_connection M1: ")); Serial.println(driver1.test_connection());
  Serial.print(F("test_connection M2: ")); Serial.println(driver2.test_connection());
  Serial.print(F("test_connection M3: ")); Serial.println(driver3.test_connection());
}

// ==============================
//   I2C-ISR (Receive) - KORRIGIERT
//   Erwartet 5 Bytes: Aufgabe, H_LSB, H_MSB, Richtung, Distanz
// ==============================
void onI2CReceive(int count) {
  if (count < 5) {
    while (Wire.available()) (void)Wire.read();
    return;
  }
  
  uint8_t b0 = Wire.read();  // 1. Byte: Aufgabe (CMD_ID)
  uint8_t b1 = Wire.read();  // 2. Byte: Höhe LSB
  uint8_t b2 = Wire.read();  // 3. Byte: Höhe MSB
  uint8_t b3 = Wire.read();  // 4. Byte: Richtung
  uint8_t b4 = Wire.read();  // 5. Byte: Distanz
  
  while (Wire.available()) (void)Wire.read(); // Restlichen Puffer leeren

  // Zuweisung zu den globalen VOLATILE ISR-Variablen
  aufgabe_isr = b0;
  hoehe_isr = (uint16_t)b1 | ((uint16_t)b2 << 8); // Höhe zusammensetzen
  richtung_isr = b3;
  richtung_distanz_isr = b4;
  
  cmd_pending = true;
}

// ==============================
//             Loop
// ==============================
void loop() {
  // nicht blockierend fahren
  stepper1.run();
  stepper2.run();
  stepper3.run();

  // Neue I2C-Daten vorhanden?
  if (cmd_pending) {
    // 1. Daten sicher aus der ISR-Zone laden
    noInterrupts();
    uint8_t cmd_id = aufgabe_isr;
    uint16_t a = hoehe_isr;
    uint8_t  b = richtung_isr;
    uint8_t  c = richtung_distanz_isr;
    cmd_pending = false;
    interrupts();

    // 2. Daten für den Haupt-Loop speichern
    aufgabe = cmd_id;
    hoehe = a; richtung = b; richtung_distanz = c;

    // Debug (dezimal)
    Serial.print("CMD="); Serial.print(aufgabe);
    Serial.print(" H="); Serial.print(hoehe);
    Serial.print(" R="); Serial.print(richtung);
    Serial.print(" D="); Serial.println(richtung_distanz);

    switch (aufgabe) {
      case CMD_HOME: // 1: Homing
        {
          bool ok = homeLift();
          Serial.println(ok ? F("Homing OK") : F("Homing FAIL"));
        }
        break;

      case CMD_STATUS: // 2: Statusabfrage
        status();
        break;

      case CMD_FAHREN: // 0: Bewegung
        // Ebenen-Logik
        if (hoehe <= hoehe_ebene1) {
          ebene1(hoehe, richtung, richtung_distanz);
        } else if (hoehe <= hoehe_ebene2) {
          ebene2(hoehe, richtung, richtung_distanz);
        } else if (hoehe <= hoehe_ebene3) {
          ebene3(hoehe, richtung, richtung_distanz);
        } else {
          // > Ebene 3: direkt Ziel anfahren (absolute Steps)
          stepper1.moveTo(heightToSteps(hoehe));
        }
        break;

      default:
        Serial.print(F("Unbekannte Aufgabe: "));
        Serial.println(aufgabe);
        break;
    }
  }
}
