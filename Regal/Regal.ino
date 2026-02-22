#include "pins.h"
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <stdint.h>
#include <math.h>

volatile bool busy = false;
volatile bool homing_active = false;
volatile bool is_homed = false;

static const uint8_t EN_PIN_NUM = 43;
static const long STEPS_PER_MM = 17;

// ==============================
//         I2C-Variablen
// ==============================
volatile uint8_t  aufgabe_i2c  = 0;
volatile int16_t  par_generic  = 0;     // bei FAHR: mm
volatile uint8_t  ebene_id_i2c = 0;
volatile bool     new_message  = false;
volatile uint8_t  last_cmd     = 0;

// ==============================
//         Befehls-Codes
// ==============================
enum : uint8_t {
  CMD_Lift     = 0,
  CMD_HOME     = 1,
  CMD_STATUS   = 2,
  CMD_Ebene    = 3,
  CMD_BELADEN  = 4,
  CMD_ENTLADEN = 5
};


// ==============================
//         Motor Parameter
// ==============================
float max_speed[12] = {
  3000.0f,  // Lift
  2000.0f,  // Ausschub
  2000.0f,  // Lift Band
  2000.0f,  // Ebene1
  2000.0f,  // Ebene2
  2000.0f,  //
  2000.0f,  //
  2000.0f,  //
  2000.0f,  //
  2000.0f,  //
  2000.0f,  //
  2000.0f   //
};

float accel_sps2[12] = {
  1000.0f,  // Lift
  1000.0f,  // Ausschub
  1000.0f,  // Lift Band
  1000.0f,  // Ebene1
  1000.0f,  // Ebene2
  1000.0f,  //
  1000.0f,  //
  1000.0f,  //
  1000.0f,  //
  1000.0f,  //
  1000.0f,  //
  1000.0f   //
};

uint8_t ihold_vals[12] = {
  15,  // Lift
  10,  // Ausschub
  1,   // Lift Band
  1,   // Ebene1
  1,   // Ebene2
  1,   //
  1,   //
  1,   //
  1,   //
  1,   //
  1,   //
  1    //
};

const int steps_u[12] = {
  4,   // Lift
  16,  // Ausschub
  2,   // Lift Band
  2,   // Ebene1
  2,   // Ebene2
  2,   //
  2,   //
  2,   //
  2,   //
  2,   //
  2,   //
  2    //
};

const int current_mA[12] = {
  1200,  // Lift
  1200,  // Ausschub
  2000,  // Lift Band
  2000,  // Ebene1
  2000,  // Ebene2
  2000,  //
  2000,  //
  2000,  //
  2000,  //
  2000,  //
  2000,  //
  2000   //
};

// ==============================
//         Hardware-Setup
// ==============================
#define SERIAL_PORT Serial1
#define SERIAL_PORT_2 Serial2
#define SERIAL_PORT_3 Serial3
#define R_SENSE 0.11f
#define I2C_ADDR 0x12

// UART-Adressen
#define ADDR_lift 0b00
#define ADDR_ausschub 0b01
#define ADDR_lift_band 0b10
#define ADDR_Ebene1 0b11
#define ADDR_Ebene2 0b00


TMC2209Stepper driver_lift(&SERIAL_PORT, R_SENSE, ADDR_lift);
TMC2209Stepper driver_ausschub(&SERIAL_PORT, R_SENSE, ADDR_ausschub);
TMC2209Stepper driver_lift_band(&SERIAL_PORT, R_SENSE, ADDR_lift_band);
TMC2209Stepper driver_Ebene1(&SERIAL_PORT, R_SENSE, ADDR_Ebene1);
TMC2209Stepper driver_Ebene2(&SERIAL_PORT_2, R_SENSE, ADDR_Ebene2);

AccelStepper lift(AccelStepper::DRIVER, STEP_PIN_lift, DIR_PIN_lift);
AccelStepper ausschub(AccelStepper::DRIVER, STEP_PIN_ausschub, DIR_PIN_ausschub);
AccelStepper lift_band(AccelStepper::DRIVER, STEP_PIN_lift_band, DIR_PIN_lift_band);
AccelStepper Ebene1(AccelStepper::DRIVER, STEP_PIN_Ebene1, DIR_PIN_Ebene1);
AccelStepper Ebene2(AccelStepper::DRIVER, STEP_PIN_Ebene2, DIR_PIN_Ebene2);

//== == == == == == == == == == == == == == ==
//       Arrays
// ==============================
AccelStepper* MOT[] = {
  &lift, &ausschub,
  &lift_band, &Ebene1, &Ebene2
};

TMC2209Stepper* DRV[] = {
  &driver_lift, &driver_ausschub,
  &driver_lift_band, &driver_Ebene1,
  &driver_Ebene2
};

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
  d.rms_current(current_mA);
  d.ihold(ihold_val);
  d.iholddelay(0);
}

void stepperCommonInit(AccelStepper& s, float vmax, float a) {
  s.setMaxSpeed(vmax);
  s.setAcceleration(a);
  s.setMinPulseWidth(5);
}

bool home() {
  busy = true;
  homing_active = true;
  is_homed = false;

  const float HOME_SPEED = -800.0f;
  const unsigned long TIMEOUT_MS = 200000;

  float oldMax = lift.maxSpeed();
  float oldAcc = lift.acceleration();

  lift.setMaxSpeed(fabsf(HOME_SPEED));
  lift.setAcceleration(accel_sps2[0]);
  lift.setSpeed(HOME_SPEED);

  unsigned long t0 = millis();

  while (PIN_READ(HOME)) {
    lift.runSpeed();
    if (millis() - t0 > TIMEOUT_MS) {
      lift.setMaxSpeed(oldMax);
      lift.setAcceleration(oldAcc);
      busy = false;
      homing_active = false;
      is_homed = false;
      return false;
    }
  }

  lift.setSpeed(0);
  lift.moveTo(lift.currentPosition());
  lift.setCurrentPosition(0);

  lift.setMaxSpeed(oldMax);
  lift.setAcceleration(oldAcc);

  busy = false;
  homing_active = false;
  is_homed = true;
  return true;
}

// ==============================
//         I2C
// ==============================
void onI2CReceive(int count) {
  if (count <= 0) {
    while (Wire.available()) (void)Wire.read();
    return;
  }

  uint8_t cmd = Wire.read();
  aufgabe_i2c = cmd;
  last_cmd = cmd;

  if (cmd == CMD_Lift) {
    if (count >= 3 && Wire.available() >= 2) {
      uint8_t low = Wire.read();
      uint8_t high = Wire.read();
      par_generic = (int16_t)((uint16_t)low | ((uint16_t)high << 8));  // mm
    }
    while (Wire.available()) (void)Wire.read();
    new_message = true;
    return;
  }

  if (cmd == CMD_Ebene) {
    if (count >= 3 && Wire.available() >= 2) {
      ebene_id_i2c = Wire.read();
    }
    while (Wire.available()) (void)Wire.read();
    new_message = true;
    return;
  }

  while (Wire.available()) (void)Wire.read();
  new_message = true;

    if (cmd == CMD_BELADEN) {
    while (Wire.available()) (void)Wire.read();
    new_message = true;
    return;
  }
      if (cmd == CMD_ENTLADEN) {
    while (Wire.available()) (void)Wire.read();
    new_message = true;
    return;
  }
}

// Status: [busy, band, pos_low, pos_high, homing]
void onI2CRequest() {
  if (last_cmd == CMD_STATUS) {
    
    return;
  }

  const uint8_t ack = 0x06;
  Wire.write(&ack, 1);
}

// =========================================
// Umrechnung
//==========================================

inline long mmToSteps(int mm) {
  return (long)mm * (long)STEPS_PER_MM;
}

// =========================================
// Arbeitsaufgaben
//==========================================

void fahren_mm(int16_t mm) {
  busy = true;
  lift.moveTo(mmToSteps((int)mm));
}

void Ebene(ebenen_id_i2c) {


}

void entladen(){


}

void beladen(){


}

void setup() {
  Wire.begin(I2C_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  SERIAL_PORT.begin(57600);
  SERIAL_PORT_2.begin(57600);
  SERIAL_PORT_3.begin(57600);

  pinMode(EN_PIN_NUM, OUTPUT);
  digitalWrite(EN_PIN_NUM, HIGH);  // deaktiviert

  // HOME: X1_4 (Pullup)
  PIN_INPUT(HOME);
  PIN_PULLUP_ON(HOME);

  is_homed = false;
  homing_active = false;


  for (int i = 0; i < 5; ++i) {
    driverCommonInit(*DRV[i], steps_u[i], current_mA[i], ihold_vals[i]);
    stepperCommonInit(*MOT[i], max_speed[i], accel_sps2[i]);
    MOT[i]->setSpeed(0);
    MOT[i]->moveTo(MOT[i]->currentPosition());
  }
  digitalWrite(EN_PIN_NUM, LOW);  // aktiv
}

void loop() {

 // I2C Kommandos ausführen
  if (new_message) {
    noInterrupts();
    uint8_t cmd = aufgabe_i2c;
    int16_t mm  = par_generic;
    new_message = false;
    interrupts();

    switch (cmd) {
      case CMD_Lift:     fahren_mm(mm); break;
      case CMD_HOME:     (void)home();  break;
      case CMD_STATUS:   break;
      case CMD_Ebene:    Ebene(ebenen_id_i2c);       break;
      case CMD_ENTLADEN: entladen();    break;
      case CMD_BELADEN:  beladen();     break;
      default: break;
    }
  }
}
