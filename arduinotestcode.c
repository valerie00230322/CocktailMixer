#include <TMCStepper.h>
#include <AccelStepper.h>

#define EN_PIN           2
#define DIR_PIN          4
#define STEP_PIN         3
#define SERIAL_PORT      Serial1    // Mega: RX1=19, TX1=18
#define DRIVER_ADDRESS   0b00
#define R_SENSE          0.11f

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Tuning
long   step_chunk     = 5000;  // Schritte pro Kommando
float  max_speed_sps  = 600;  // Schritte/s
float  accel_sps2     = 300;   // Schritte/s^2
bool   enabled        = true;

void printStatus() {
  Serial.println(F("=== STATUS ==="));
  Serial.print(F("IFCNT       : ")); Serial.println(driver.IFCNT());
  Serial.print(F("rms_current : ")); Serial.println(driver.rms_current());
  Serial.print(F("microsteps  : ")); Serial.println(driver.microsteps());
  Serial.print(F("toff        : ")); Serial.println(driver.toff());
  Serial.print(F("GCONF       : 0x")); Serial.println(driver.GCONF(), HEX);
  Serial.print(F("DRV_STATUS  : 0x")); Serial.println(driver.DRV_STATUS(), HEX);
  Serial.print(F("pos         : ")); Serial.println(stepper.currentPosition());
  Serial.print(F("target      : ")); Serial.println(stepper.targetPosition());
  Serial.print(F("maxSpeed    : ")); Serial.println(stepper.maxSpeed());
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println(F("\nStart UART + AccelStepper..."));

  // UART für TMC2209
  SERIAL_PORT.begin(57600);          // oft stabiler als 115200
  driver.begin();
  driver.pdn_disable(true);          // PDN als UART freigeben
  driver.mstep_reg_select(true);     // Microsteps via Register

  // Pins
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);         // Treiber aktivieren
  enabled = true;

  // Treiber-Parameter
  driver.toff(8);
  driver.microsteps(4);
  driver.en_spreadCycle(true);      // stealthChop (leise)
  driver.pwm_autoscale(true);
  driver.I_scale_analog(false);     // nur per UART skalieren
driver.vsense(true);              // höhere Sense-Spannung -> feinere Regelung
driver.ihold(29);        // Haltestrom (0..31)  ~65%
driver.irun(31);         // Fahrstrom  (0..31)  100%
driver.iholddelay(0);    // Hold-Delay (0..15)    // IHOLD ~65%, IRUN 100%, Delay 0
driver.rms_current(2500);          // falls thermisch ok, etwas erhöhen


  // AccelStepper-Parameter
  stepper.setMaxSpeed(max_speed_sps);
  stepper.setAcceleration(accel_sps2);
stepper.setMinPulseWidth(5); 
  // UART-Test
  uint8_t res = driver.test_connection();
  Serial.print(F("test_connection: ")); Serial.println(res); // 0 = OK

  printStatus();
}

void loop() {
  // Kommandos als ganze Zeile lesen (ein Zeichen oder Worte)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "r") {                          // vorwärts (positiv)
      Serial.println(F(">> vorwärts"));
      stepper.move(step_chunk);
    } else if (cmd == "u") {                   // rückwärts (negativ)
      Serial.println(F(">> rückwärts"));
      stepper.move(-step_chunk);
    } else if (cmd == "s") {                   // sanft stoppen
      Serial.println(F(">> stop"));
      stepper.stop();                          // bremst auf 0
    } else if (cmd == "+") {                   // schneller
      max_speed_sps += 200;
      stepper.setMaxSpeed(max_speed_sps);
      Serial.print(F(">> maxSpeed = ")); Serial.println(max_speed_sps);
    } else if (cmd == "-") {                   // langsamer
      max_speed_sps = max(100.0f, max_speed_sps - 200);
      stepper.setMaxSpeed(max_speed_sps);
      Serial.print(F(">> maxSpeed = ")); Serial.println(max_speed_sps);
    } else if (cmd == "p" || cmd == "status") {// Status
      printStatus();
      Serial.print("OTPW=");  Serial.print(driver.otpw());
Serial.print(" OT=");    Serial.print(driver.ot());
Serial.print(" S2GA=");  Serial.print(driver.s2ga());
Serial.print(" S2GB=");  Serial.print(driver.s2gb());
Serial.print(" OLA=");   Serial.print(driver.ola());
Serial.print(" OLB=");   Serial.print(driver.olb());
Serial.print(" CS_ACT=");Serial.println(driver.cs_actual());

    } else if (cmd == "e") {                   // Enable toggle
      enabled = !enabled;
      digitalWrite(EN_PIN, enabled ? LOW : HIGH);
      Serial.print(F(">> enable = ")); Serial.println(enabled ? F("ON") : F("OFF"));
    } else if (cmd.startsWith("steps ")) {     // z.B. "steps 5000"
      long v = cmd.substring(6).toInt();
      step_chunk = (v != 0) ? v : step_chunk;
      Serial.print(F(">> step_chunk = ")); Serial.println(step_chunk);
    } else if (cmd.startsWith("acc ")) {       // z.B. "acc 500"
      float a = cmd.substring(4).toFloat();
      if (a > 0) { accel_sps2 = a; stepper.setAcceleration(accel_sps2); }
      Serial.print(F(">> accel = ")); Serial.println(accel_sps2);
    } else if (cmd.startsWith("ms ")) {        // z.B. "ms 16" (1,2,4,8,16,32,64,128,256)
      int ms = cmd.substring(3).toInt();
      driver.microsteps(ms);
      Serial.print(F(">> microsteps = ")); Serial.println(driver.microsteps());
    } else {
      Serial.println(F("Befehle: r/u/s, +/-, p(status), e, steps N, acc A, ms M"));
    }
  }

  // Motor laufen lassen (nicht blockierend)
  stepper.run();
}
