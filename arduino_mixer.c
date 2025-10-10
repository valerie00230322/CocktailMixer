#include <TMCStepper.h>
#include <AccelStepper.h>
#include <Wire.h>

// ... (deine Defines & Objekte bleiben unverändert)

float mixerMax = 600;   // Spiegel der Startwerte – damit wir sie nach Homing wieder setzen können
float mixerAcc = 300;

void onI2CReceive(int count);

void onI2CReceive(int count) {
  if (count < 2) {
    while (Wire.available()) (void)Wire.read();
    return;
  }
  uint8_t b0 = Wire.read();  // 1. Byte: Aufgabe (CMD_ID)
  uint8_t b1 = Wire.read();  // 2. Byte: Höhe/Param LSB

  aufgabe_i2c = b0;
  abstand_i2c = b1;
  new_message = true;

  while (Wire.available()) (void)Wire.read(); // Rest verwerfen
}

// --- FIX: korrekte Signatur & Rückgabewerttyp ---
long abstand_steps(int abstand) {
  long s = (long)abstand * 400L;
  return s;
}

// --- FIXED: homing nur mit stepperMIXER, bool-Return ---
bool home() {
  const float HOME_SPEED     = -400.0f; // negativ = Richtung Schalter
  const float HOME_ACCEL     = 800.0f;
  const long  BACKOFF_STEPS  = 800;
  const unsigned long TIMEOUT_MS = 20000;

  // Vorherige Werte merken (max aus Stepper, accel aus unserer Variable)
  float oldMax = stepperMIXER.maxSpeed();
  float oldAcc = mixerAcc;

  // Homing-Setup
  stepperMIXER.setAcceleration(HOME_ACCEL);
  stepperMIXER.setMaxSpeed(HOME_SPEED < 0 ? -HOME_SPEED : HOME_SPEED);
  stepperMIXER.setSpeed(HOME_SPEED); // für runSpeed()

  unsigned long t0 = millis();
  // Fahr bis Schalter schließt (HOME_PIN ist PULLUP → LOW = ausgelöst)
  while (digitalRead(HOME_PIN) == HIGH) {
    stepperMIXER.runSpeed();
    if (millis() - t0 > TIMEOUT_MS) {
      // Werte zurücksetzen
      stepperMIXER.setMaxSpeed(oldMax);
      stepperMIXER.setAcceleration(oldAcc);
      return false; // Timeout
    }
  }

  // Stoppen & Absetzen vom Schalter
  stepperMIXER.stop();
  while (stepperMIXER.isRunning()) stepperMIXER.run();
  stepperMIXER.move(BACKOFF_STEPS);
  while (stepperMIXER.distanceToGo() != 0) stepperMIXER.run();

  // Nullpunkt setzen
  stepperMIXER.setCurrentPosition(0);

  // Ursprungswerte wiederherstellen + Spiegel aktualisieren
  stepperMIXER.setMaxSpeed(oldMax);
  stepperMIXER.setAcceleration(oldAcc);
  mixerMax = oldMax;
  mixerAcc = oldAcc;

  return true; // <-- fehlte
}

// --- FIX: korrekte Signatur & distanceToGo() ---
void fahren(long abstand) {
  stepperMIXER.moveTo(abstand_steps((int)abstand));
  while (stepperMIXER.distanceToGo() != 0) {
    stepperMIXER.run();
  }
}

void setup() {
  Wire.begin(I2C_ADDR);
  Wire.onReceive(onI2CReceive);

  SERIAL_PORT.begin(57600);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);  // LOW = Treiber aktiv
  pinMode(HOME_PIN, INPUT_PULLUP);

  driverCommonInit(driverMIXER, steps[0], current[0]);
  stepperCommonInit(stepperMIXER, max_speed_sps[0], accel_sps2[0]);
  driverCommonInit(driverBAND,  steps[1], current[1]);
  stepperCommonInit(stepperBAND, max_speed_sps[1], accel_sps2[1]);

  // Spiegel initialisieren
  mixerMax = max_speed_sps[0];
  mixerAcc = accel_sps2[0];
}

void loop() {
  stepperMIXER.run();
  stepperBAND.run();

  if (new_message) {
    // sicher kopieren (verhindert Race)
    noInterrupts();
    int cmd = aufgabe_i2c;
    int par = abstand_i2c;
    new_message = false;
    interrupts();

    switch (cmd) {
      case 0: { // fahren
        fahren(par);    // <-- nutze die lokale Kopie
      } break;

      case 1: { // homing
        (void)home();   // Rückgabewert optional auswerten
      } break;

      case 2: { // status
        status();
      } break;

      default: break;
    }
  }
}
