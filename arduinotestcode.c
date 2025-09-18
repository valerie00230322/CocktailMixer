#include <TMCStepper.h>
#include <AccelStepper.h>

// ==============================
//        Hardware-Setup
// ==============================
#define SERIAL_PORT      Serial1    // Mega: RX1=19, TX1=18
#define R_SENSE          0.11f

// UART-Adressen pro Treiber (über MS1/MS2 gesetzt)
#define ADDR_M1          0b00
#define ADDR_M2          0b01
#define ADDR_M3          0b10

// Gemeinsamer Enable (LOW = aktiv)
#define EN_PIN           2

// STEP/DIR pro Motor (BITTE an deine Pins anpassen!)
#define STEP_PIN_M1      3
#define DIR_PIN_M1       4
#define STEP_PIN_M2      5
#define DIR_PIN_M2       6
#define STEP_PIN_M3      7
#define DIR_PIN_M3       8

// ==============================
//         Objekte
// ==============================
TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, ADDR_M1);
TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, ADDR_M2);
TMC2209Stepper driver3(&SERIAL_PORT, R_SENSE, ADDR_M3);

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_M1, DIR_PIN_M1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_M2, DIR_PIN_M2);
AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN_M3, DIR_PIN_M3);

// Bequeme Arrays für Schleifen
TMC2209Stepper* drivers[3] = { &driver1, &driver2, &driver3 };
AccelStepper*  steppers[3] = { &stepper1, &stepper2, &stepper3 };

// ==============================
//       Tuning / Defaults
// ==============================
// pro Motor eigene Parameter
long  step_chunk[3]    = { 5000, 5000, 5000 };  // Schritte pro Kommando
float max_speed_sps[3] = { 1000,  600,  600  };  // Schritte/s
float accel_sps2[3]    = { 300,  300,  300  };  // Schritte/s^2

bool enabled_all = true;  // gemeinsames Enable
int  selected    = 0;     // 0..2 => aktuell ausgewählter Motor (Default: M1)

// ==============================
//       Hilfsfunktionen
// ==============================
void printStatus(int i) {
  // i: 0..2
  auto& d = *drivers[i];
  auto& s = *steppers[i];

  Serial.print(F("=== STATUS M")); Serial.print(i+1); Serial.println(F(" ==="));
  Serial.print(F("IFCNT       : ")); Serial.println(d.IFCNT());
  Serial.print(F("rms_current : ")); Serial.println(d.rms_current());
  Serial.print(F("microsteps  : ")); Serial.println(d.microsteps());
  Serial.print(F("toff        : ")); Serial.println(d.toff());
  Serial.print(F("GCONF       : 0x")); Serial.println(d.GCONF(), HEX);
  Serial.print(F("DRV_STATUS  : 0x")); Serial.println(d.DRV_STATUS(), HEX);
  Serial.print(F("pos         : ")); Serial.println(s.currentPosition());
  Serial.print(F("target      : ")); Serial.println(s.targetPosition());
  Serial.print(F("maxSpeed    : ")); Serial.println(s.maxSpeed());
  Serial.print(F("OTPW="));  Serial.print(d.otpw());
  Serial.print(" OT=");    Serial.print(d.ot());
  Serial.print(" S2GA=");  Serial.print(d.s2ga());
  Serial.print(" S2GB=");  Serial.print(d.s2gb());
  Serial.print(" OLA=");   Serial.print(d.ola());
  Serial.print(" OLB=");   Serial.print(d.olb());
  Serial.print(" CS_ACT=");Serial.println(d.cs_actual());
  Serial.println();
}

void driverCommonInit(TMC2209Stepper& d) {
  d.begin();
  d.pdn_disable(true);         // PDN als UART
  d.mstep_reg_select(true);    // Microsteps via Register
  d.toff(8);
  d.microsteps(4);
  d.en_spreadCycle(true);      // spreadCycle an (stealthChop aus) – ändere auf false für leiser
  d.pwm_autoscale(true);
  d.I_scale_analog(false);
  d.vsense(true);
  d.ihold(29);                 // ~65% Hold
  d.irun(31);                  // 100% Run
  d.iholddelay(0);
  d.rms_current(2500);         // ggf. thermisch anpassen
}

void stepperCommonInit(AccelStepper& s, float vmax, float a) {
  s.setMaxSpeed(vmax);
  s.setAcceleration(a);
  s.setMinPulseWidth(5);
}

// Parse: optionales "m1|m2|m3" vorn, sonst selected
// gibt Motorindex 0..2 zurück
int parseMotorIndex(String& cmd) {
  if (cmd.length() >= 2 && (cmd[0] == 'm' || cmd[0] == 'M')) {
    int n = cmd.substring(1,2).toInt();
    if (n >= 1 && n <= 3) {
      // prefix + evtl. Leerzeichen entfernen
      int space = cmd.indexOf(' ');
      if (space >= 0) {
        cmd = cmd.substring(space+1);
      } else {
        cmd = ""; // nur mX ohne Rest
      }
      return n-1;
    }
  }
  return selected;
}

// ==============================
//            Setup
// ==============================
void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println(F("\nStart 3x TMC2209 via UART + AccelStepper..."));

  // UART für TMC2209 – 57600 ist oft stabiler als 115200
  SERIAL_PORT.begin(57600);

  // Pins
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);      // alle aktiv
  enabled_all = true;

  // Treiber initialisieren
  driverCommonInit(driver1);
  driverCommonInit(driver2);
  driverCommonInit(driver3);

  // AccelStepper-Parameter
  stepperCommonInit(stepper1, max_speed_sps[0], accel_sps2[0]);
  stepperCommonInit(stepper2, max_speed_sps[1], accel_sps2[1]);
  stepperCommonInit(stepper3, max_speed_sps[2], accel_sps2[2]);

  // UART-Test
  Serial.print(F("test_connection M1: ")); Serial.println(driver1.test_connection());
  Serial.print(F("test_connection M2: ")); Serial.println(driver2.test_connection());
  Serial.print(F("test_connection M3: ")); Serial.println(driver3.test_connection());

  // Startstatus
  printStatus(0);
  printStatus(1);
  printStatus(2);

  Serial.println(F("Befehle: [m1|m2|m3] r/u/s, +/-, p(status), steps N, acc A, ms M, sel K, e on|off"));
}

// ==============================
//             Loop
// ==============================
void loop() {
  // nicht blockierend fahren
  stepper1.run();
  stepper2.run();
  stepper3.run();

  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  // spezielle globale Befehle zuerst prüfen
  if (cmd.startsWith("sel ")) {
    int k = cmd.substring(4).toInt();
    if (k >= 1 && k <= 3) {
      selected = k-1;
      Serial.print(F(">> selected = M")); Serial.println(k);
    } else {
      Serial.println(F(">> sel erwartet 1..3"));
    }
    return;
  } else if (cmd.startsWith("e ")) {
    String v = cmd.substring(2);
    v.trim();
    if (v == "on") {
      enabled_all = true;
      digitalWrite(EN_PIN, LOW);
      Serial.println(F(">> enable = ON (alle)"));
    } else if (v == "off") {
      enabled_all = false;
      digitalWrite(EN_PIN, HIGH);
      Serial.println(F(">> enable = OFF (alle)"));
    } else {
      Serial.println(F(">> e on|off"));
    }
    return;
  }

  // Motorpräfix parsen (m1|m2|m3), sonst aktueller
  int i = parseMotorIndex(cmd);
  auto& d = *drivers[i];
  auto& s = *steppers[i];

  // Einzelkommandos (auf Motor i)
  if (cmd == "r") {                    // vorwärts
    Serial.print(F(">> M")); Serial.print(i+1); Serial.println(F(" vorwärts"));
    s.move(step_chunk[i]);
  } else if (cmd == "u") {             // rückwärts
    Serial.print(F(">> M")); Serial.print(i+1); Serial.println(F(" rückwärts"));
    s.move(-step_chunk[i]);
  } else if (cmd == "s") {             // sanft stoppen
    Serial.print(F(">> M")); Serial.print(i+1); Serial.println(F(" stop"));
    s.stop();
  } else if (cmd == "+") {             // schneller
    max_speed_sps[i] += 200;
    s.setMaxSpeed(max_speed_sps[i]);
    Serial.print(F(">> M")); Serial.print(i+1); Serial.print(F(" maxSpeed = "));
    Serial.println(max_speed_sps[i]);
  } else if (cmd == "-") {             // langsamer
    max_speed_sps[i] = max(100.0f, max_speed_sps[i] - 200);
    s.setMaxSpeed(max_speed_sps[i]);
    Serial.print(F(">> M")); Serial.print(i+1); Serial.print(F(" maxSpeed = "));
    Serial.println(max_speed_sps[i]);
  } else if (cmd == "p" || cmd == "status") {
    printStatus(i);
  } else if (cmd.startsWith("steps ")) {
    long v = cmd.substring(6).toInt();
    if (v != 0) step_chunk[i] = v;
    Serial.print(F(">> M")); Serial.print(i+1); Serial.print(F(" step_chunk = "));
    Serial.println(step_chunk[i]);
  } else if (cmd.startsWith("acc ")) {
    float a = cmd.substring(4).toFloat();
    if (a > 0) { accel_sps2[i] = a; s.setAcceleration(accel_sps2[i]); }
    Serial.print(F(">> M")); Serial.print(i+1); Serial.print(F(" accel = "));
    Serial.println(accel_sps2[i]);
  } else if (cmd.startsWith("ms ")) {
    int ms = cmd.substring(3).toInt();
    d.microsteps(ms);
    Serial.print(F(">> M")); Serial.print(i+1); Serial.print(F(" microsteps = "));
    Serial.println(d.microsteps());
  } else {
    Serial.println(F("Befehle: [m1|m2|m3] r/u/s, +/-, p(status), steps N, acc A, ms M, sel K, e on|off"));
  }
}
