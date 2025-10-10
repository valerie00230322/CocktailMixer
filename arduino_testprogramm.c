#include <TMCStepper.h>
#include <AccelStepper.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

#define SERIAL_PORT     Serial1            // UART zum TMC2209 (TX1=D18, RX1=D19 am Mega)
#define R_SENSE         0.11f

// Hardware
#define ADDR_M1         0b00               // TMC2209 Adressbrücken MS1/MS2
#define STEP_PIN_M1     22
#define DIR_PIN_M1      23
#define EN_PIN          24                 // LOW = Treiber aktiv

// Start-Parameter
long  step_chunk[6]     = { 5000, 5000, 5000, 5000, 5000, 5000 }; // ungenutzt
float max_speed_sps[6]  = { 1000, 600, 600, 1000, 600, 600 };
float accel_sps2[6]     = { 300, 300, 300, 300, 300, 300 };
int   steps[6]          = { 16, 16, 16, 16, 16, 16 };             // Microsteps
int   current[6]        = { 1500,1500,1500,1500,1500,1500 };      // mA RMS

// Objekte
TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, ADDR_M1);
AccelStepper   stepper1(AccelStepper::DRIVER, STEP_PIN_M1, DIR_PIN_M1);

// Laufstatus
bool  runSpeedMode = false;       // true = Dauerlauf (runSpeed), false = nur Bewegungen (run)
float runSpeedSet  = 0.0f;        // steps/s (Vorzeichen = Richtung)

// -------- Helpers --------
void driverCommonInit(TMC2209Stepper& d, int micro, int current_mA) {
  d.begin();
  d.pdn_disable(true);
  d.mstep_reg_select(true);
  d.toff(8);
  d.microsteps(micro);
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

inline void enableDriver(bool en) {
  digitalWrite(EN_PIN, en ? LOW : HIGH); // LOW = enable
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  START [speed]    -> Dauerlauf starten (steps/s, signed)"));
  Serial.println(F("  STOP             -> Dauerlauf stoppen"));
  Serial.println(F("  SPEED <v>        -> Geschwindigkeit setzen (steps/s, signed)"));
  Serial.println(F("  MOVE <steps>     -> relative Bewegung (mit Beschleunigung)"));
  Serial.println(F("  DIR FWD|REV      -> Richtung fuer Dauerlauf"));
  Serial.println(F("  EN ON|OFF        -> Treiber ein/aus (EN low/high)"));
  Serial.println(F("  STATUS           -> Status anzeigen"));
  Serial.println(F("  HELP             -> diese Hilfe"));
}

void printStatus() {
  Serial.print(F("EN="));
  Serial.print(digitalRead(EN_PIN) == LOW ? F("ON") : F("OFF"));
  Serial.print(F(" | runSpeed="));
  Serial.print(runSpeedMode ? F("yes") : F("no"));
  Serial.print(F(" | speed="));
  Serial.print(runSpeedSet, 1);
  Serial.print(F(" steps/s | pos="));
  Serial.println(stepper1.currentPosition());
}

// -------- Kommando-Parser (zeilenbasiert) --------
char lineBuf[64];
uint8_t lineLen = 0;

void handleLine(char *line) {
  // Uppercase in-place
  for (char *p = line; *p; ++p) *p = (char)toupper(*p);

  // Tokenize
  char *cmd = strtok(line, " \t");
  if (!cmd) return;
  char *arg1 = strtok(NULL, " \t");
  char *arg2 = strtok(NULL, " \t");

  if (!strcmp(cmd, "HELP")) {
    printHelp();
  }
  else if (!strcmp(cmd, "STATUS")) {
    printStatus();
  }
  else if (!strcmp(cmd, "EN")) {
    if (!arg1) { Serial.println(F("ERR: EN ON|OFF")); return; }
    if (!strcmp(arg1, "ON"))  { enableDriver(true);  Serial.println(F("EN=ON"));  }
    else if (!strcmp(arg1, "OFF")) { enableDriver(false); Serial.println(F("EN=OFF")); }
    else Serial.println(F("ERR: EN ON|OFF"));
  }
  else if (!strcmp(cmd, "DIR")) {
    if (!arg1) { Serial.println(F("ERR: DIR FWD|REV")); return; }
    if (!strcmp(arg1, "FWD")) {
      runSpeedSet = fabs(runSpeedSet);
      if (runSpeedMode) stepper1.setSpeed(runSpeedSet);
      Serial.println(F("DIR=FWD"));
    } else if (!strcmp(arg1, "REV")) {
      runSpeedSet = -fabs(runSpeedSet);
      if (runSpeedMode) stepper1.setSpeed(runSpeedSet);
      Serial.println(F("DIR=REV"));
    } else {
      Serial.println(F("ERR: DIR FWD|REV"));
    }
  }
  else if (!strcmp(cmd, "SPEED")) {
    if (!arg1) { Serial.println(F("ERR: SPEED <steps_per_sec>")); return; }
    long v = strtol(arg1, NULL, 10);
    runSpeedSet = (float)v;
    if (runSpeedMode) stepper1.setSpeed(runSpeedSet);
    Serial.print(F("SPEED=")); Serial.println(runSpeedSet, 1);
  }
  else if (!strcmp(cmd, "START")) {
    if (arg1) {
      long v = strtol(arg1, NULL, 10);
      runSpeedSet = (float)v;
    }
    enableDriver(true);
    stepper1.setSpeed(runSpeedSet);
    runSpeedMode = true;
    Serial.print(F("RUNNING at ")); Serial.print(runSpeedSet, 1); Serial.println(F(" steps/s"));
  }
  else if (!strcmp(cmd, "STOP")) {
    runSpeedMode = false;
    stepper1.setSpeed(0);
    Serial.println(F("STOPPED"));
  }
  else if (!strcmp(cmd, "MOVE")) {
    if (!arg1) { Serial.println(F("ERR: MOVE <steps>")); return; }
    long s = strtol(arg1, NULL, 10);
    enableDriver(true);
    stepper1.move(s);          // relative Fahrt, non-blocking
    runSpeedMode = false;      // Dauerlauf aussetzen bis Bewegung fertig
    Serial.print(F("MOVE ")); Serial.println(s);
  }
  else {
    Serial.println(F("ERR: unknown command (HELP for list)"));
  }
}

void setup() {
  Serial.begin(115200);             // USB-Konsole
  SERIAL_PORT.begin(57600);         // UART zum TMC2209

  pinMode(EN_PIN, OUTPUT);
  enableDriver(false);              // sicher deaktiviert starten

  driverCommonInit(driver1, steps[0], current[0]);
  stepperCommonInit(stepper1, max_speed_sps[0], accel_sps2[0]);

  // Default-Speed für Dauerlauf
  runSpeedSet = 300.0f;

  Serial.println(F("TMC2209 Test ready. Type HELP"));
  printStatus();
}

void loop() {
  // Serielle Zeilen lesen
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      handleLine(lineBuf);
      lineLen = 0;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }

  // Motor ansteuern
  if (runSpeedMode) {
    stepper1.runSpeed();     // konstanter Dauerlauf
  } else {
    stepper1.run();          // bedient MOVE-Fahrten (Beschleunigung)
  }
}
