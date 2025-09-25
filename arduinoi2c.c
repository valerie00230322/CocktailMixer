#include <TMCStepper.h>
#include <AccelStepper.h>
#include <Wire.h>
// Motor 1 Aufzug  | Motor 2 Richtung  | Motor 3 Förderband

// ==============================
//        Hardware-Setup
// ==============================
#define SERIAL_PORT      Serial1    // Mega: RX1=19, TX1=18
#define R_SENSE          0.11f
#define I2C_ADDR         0x12

// UART-Adressen pro Treiber (über MS1/MS2 gesetzt)
#define ADDR_M1          0b00
#define ADDR_M2          0b01
#define ADDR_M3          0b10

// Gemeinsamer Enable (LOW = aktiv)
#define EN_PIN           2

// STEP/DIR pro Motor
#define STEP_PIN_M1      3
#define DIR_PIN_M1       4
#define STEP_PIN_M2      5
#define DIR_PIN_M2       6
#define STEP_PIN_M3      7
#define DIR_PIN_M3       8

// Endschalter (unten, aktiv LOW -> gegen GND schalten)
#define HOME_PIN         9

// ==============================
//         Objekte
// ==============================
TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, ADDR_M1);
TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, ADDR_M2);
TMC2209Stepper driver3(&SERIAL_PORT, R_SENSE, ADDR_M3);

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_M1, DIR_PIN_M1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_M2, DIR_PIN_M2);
AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN_M3, DIR_PIN_M3);

TMC2209Stepper* drivers[3] = { &driver1, &driver2, &driver3 };
AccelStepper*  steppers[3] = { &stepper1, &stepper2, &stepper3 };

// ==============================
//       Motor Parameter
// ==============================
long  step_chunk[3]    = { 5000, 5000, 5000 };  // aktuell ungenutzt
float max_speed_sps[3] = { 1000, 600, 600 };
float accel_sps2[3]    = { 300, 300, 300 };
int   steps[3]         = { 16, 16, 16 };        // Microsteps, ohne Prüfung
int   current[3]       = { 1500, 1500, 1500 };  // mA RMS

// ==============================
//   I2C-Empfangsvariablen
// ==============================
volatile uint16_t hoehe_isr = 0;            // 0..1500
volatile uint8_t  richtung_isr = 0;         // 0/1
volatile uint8_t  richtung_distanz_isr = 0; // 0..150
volatile bool     cmd_pending = false;

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
//   (Passe 400 an deine Mechanik an: steps pro "Höheneinheit")
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
//   HOMING-FUNKTION (auf Schalter fahren, Steps=0)
//   - Fährt nach unten (neg. Richtung) bis HOME_PIN LOW ist
//   - fährt ein Stück ab (Backoff) und setzt Position auf 0
// ==============================
bool homeLift() {
  const float HOME_SPEED   = -400.0f;  // Schritte/s, negativ = nach unten
  const float HOME_ACCEL   =  800.0f;  // Schritte/s^2
  const long  BACKOFF_STEPS=  800;     // von Schalter wegfahren
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
    stepper1.runSpeed();   // konstante Geschwindigkeit
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
  digitalWrite(EN_PIN, LOW);   // alle aktiv

  driverCommonInit(driver1, steps[0], current[0]);
  driverCommonInit(driver2, steps[1], current[1]);
  driverCommonInit(driver3, steps[2], current[2]);

  stepperCommonInit(stepper1, max_speed_sps[0], accel_sps2[0]);
  stepperCommonInit(stepper2, max_speed_sps[1], accel_sps2[1]);
  stepperCommonInit(stepper3, max_speed_sps[2], accel_sps2[2]);

  Serial.print(F("test_connection M1: ")); Serial.println(driver1.test_connection());
  Serial.print(F("test_connection M2: ")); Serial.println(driver2.test_connection());
  Serial.print(F("test_connection M3: ")); Serial.println(driver3.test_connection());

  // --- OPTIONAL: Beim Start einmal homing machen ---
  // if (!homeLift()) {
  //   Serial.println(F("Homing Timeout!"));
  // }
}

// ==============================
//         I2C-ISR (Receive)
// ==============================
void onI2CReceive(int count) {
  if (count < 4) {
    while (Wire.available()) (void)Wire.read();
    return;
  }

  uint8_t b0 = Wire.read();  // A LSB
  uint8_t b1 = Wire.read();  // A MSB
  uint8_t b2 = Wire.read();  // B
  uint8_t b3 = Wire.read();  // C
  while (Wire.available()) (void)Wire.read();

  uint16_t a = (uint16_t)b0 | ((uint16_t)b1 << 8);
  uint8_t  b = b2 ? 1 : 0;
  uint8_t  c = b3;

  if (a > 1500) a = 1500;
  if (c > 150)  c = 150;

  hoehe_isr = a;
  richtung_isr = b;
  richtung_distanz_isr = c;
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
    noInterrupts();
    uint16_t a = hoehe_isr;
    uint8_t  b = richtung_isr;
    uint8_t  c = richtung_distanz_isr;
    cmd_pending = false;
    interrupts();

    hoehe = a; richtung = b; richtung_distanz = c;

    // Debug (dezimal)
    Serial.print("A="); Serial.print(hoehe);
    Serial.print(" B="); Serial.print(richtung);
    Serial.print(" C="); Serial.println(richtung_distanz);

    // Beispiel: I2C-Befehl "hoehe=0 && richtung=0 && c=0" als Homing-Trigger
    if (hoehe == 0 && richtung == 0 && richtung_distanz == 0) {
      bool ok = homeLift();
      Serial.println(ok ? F("Homing OK") : F("Homing FAIL"));
    } else {
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
    }
  }
}
