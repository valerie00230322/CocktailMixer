// pins.h
#pragma once
#include <avr/io.h>

// ------------------------------------------------------------
// Generic helpers: use like PIN_OUTPUT(PUMPE7_STEP), PIN_HIGH(...)
// ------------------------------------------------------------
#define PIN_OUTPUT(sig)     do { sig##_DDR  |=  _BV(sig##_BIT); } while (0)
#define PIN_INPUT(sig)      do { sig##_DDR  &= ~_BV(sig##_BIT); } while (0)

#define PIN_HIGH(sig)       do { sig##_PORT |=  _BV(sig##_BIT); } while (0)
#define PIN_LOW(sig)        do { sig##_PORT &= ~_BV(sig##_BIT); } while (0)
#define PIN_TOGGLE(sig)     do { sig##_PORT ^=  _BV(sig##_BIT); } while (0)

#define PIN_PULLUP_ON(sig)  do { sig##_PORT |=  _BV(sig##_BIT); } while (0)
#define PIN_PULLUP_OFF(sig) do { sig##_PORT &= ~_BV(sig##_BIT); } while (0)

#define PIN_READ(sig)       ((sig##_PIN & _BV(sig##_BIT)) != 0)


// ============================================================
// Motor / Pumpen Signale
// ============================================================

// ---------------- PLF ----------------
#define PLF_STEP_DDR   DDRB
#define PLF_STEP_PORT  PORTB
#define PLF_STEP_PIN   PINB
#define PLF_STEP_BIT   PB5

#define PLF_DIR_DDR    DDRA
#define PLF_DIR_PORT   PORTA
#define PLF_DIR_PIN    PINA
#define PLF_DIR_BIT    PA0

#define PLF_DIAG_DDR   DDRF
#define PLF_DIAG_PORT  PORTF
#define PLF_DIAG_PIN   PINF
#define PLF_DIAG_BIT   PF0


// ---------------- BAND ----------------
#define BAND_STEP_DDR  DDRB
#define BAND_STEP_PORT PORTB
#define BAND_STEP_PIN  PINB
#define BAND_STEP_BIT  PB6

#define BAND_DIR_DDR   DDRA
#define BAND_DIR_PORT  PORTA
#define BAND_DIR_PIN   PINA
#define BAND_DIR_BIT   PA1

#define BAND_DIAG_DDR  DDRF
#define BAND_DIAG_PORT PORTF
#define BAND_DIAG_PIN  PINF
#define BAND_DIAG_BIT  PF1


// ---------------- PUMPE 1 ----------------
#define PUMPE1_STEP_DDR  DDRB
#define PUMPE1_STEP_PORT PORTB
#define PUMPE1_STEP_PIN  PINB
#define PUMPE1_STEP_BIT  PB7

#define PUMPE1_DIR_DDR   DDRA
#define PUMPE1_DIR_PORT  PORTA
#define PUMPE1_DIR_PIN   PINA
#define PUMPE1_DIR_BIT   PA2

#define PUMPE1_DIAG_DDR  DDRH
#define PUMPE1_DIAG_PORT PORTH
#define PUMPE1_DIAG_PIN  PINH
#define PUMPE1_DIAG_BIT  PH2


// ---------------- PUMPE 2 (STEP über X4_1) ----------------
#define PUMPE2_STEP_DDR   DDRE
#define PUMPE2_STEP_PORT  PORTE
#define PUMPE2_STEP_PIN   PINE
#define PUMPE2_STEP_BIT   PE0

#define PUMPE2_DIR_DDR    DDRA
#define PUMPE2_DIR_PORT   PORTA
#define PUMPE2_DIR_PIN    PINA
#define PUMPE2_DIR_BIT    PA3

#define PUMPE2_DIAG_DDR   DDRH
#define PUMPE2_DIAG_PORT  PORTH
#define PUMPE2_DIAG_PIN   PINH
#define PUMPE2_DIAG_BIT   PH3


// ---------------- PUMPE 3 (STEP über X4_2) ----------------
#define PUMPE3_STEP_DDR   DDRE
#define PUMPE3_STEP_PORT  PORTE
#define PUMPE3_STEP_PIN   PINE
#define PUMPE3_STEP_BIT   PE1

#define PUMPE3_DIR_DDR    DDRA
#define PUMPE3_DIR_PORT   PORTA
#define PUMPE3_DIR_PIN    PINA
#define PUMPE3_DIR_BIT    PA4

#define PUMPE3_DIAG_DDR   DDRG
#define PUMPE3_DIAG_PORT  PORTG
#define PUMPE3_DIAG_PIN   PING
#define PUMPE3_DIAG_BIT   PG0


// ---------------- PUMPE 4 (STEP über X4_4) ----------------
#define PUMPE4_STEP_DDR   DDRE
#define PUMPE4_STEP_PORT  PORTE
#define PUMPE4_STEP_PIN   PINE
#define PUMPE4_STEP_BIT   PE3

#define PUMPE4_DIR_DDR    DDRA
#define PUMPE4_DIR_PORT   PORTA
#define PUMPE4_DIR_PIN    PINA
#define PUMPE4_DIR_BIT    PA5

#define PUMPE4_DIAG_DDR   DDRG
#define PUMPE4_DIAG_PORT  PORTG
#define PUMPE4_DIAG_PIN   PING
#define PUMPE4_DIAG_BIT   PG1


// ---------------- PUMPE 5 ----------------
#define PUMPE5_STEP_DDR   DDRH
#define PUMPE5_STEP_PORT  PORTH
#define PUMPE5_STEP_PIN   PINH
#define PUMPE5_STEP_BIT   PH3

#define PUMPE5_DIR_DDR    DDRA
#define PUMPE5_DIR_PORT   PORTA
#define PUMPE5_DIR_PIN    PINA
#define PUMPE5_DIR_BIT    PA6

#define PUMPE5_DIAG_DDR   DDRG
#define PUMPE5_DIAG_PORT  PORTG
#define PUMPE5_DIAG_PIN   PING
#define PUMPE5_DIAG_BIT   PG2


// ---------------- PUMPE 6 ----------------
#define PUMPE6_STEP_DDR   DDRH
#define PUMPE6_STEP_PORT  PORTH
#define PUMPE6_STEP_PIN   PINH
#define PUMPE6_STEP_BIT   PH4

#define PUMPE6_DIR_DDR    DDRA
#define PUMPE6_DIR_PORT   PORTA
#define PUMPE6_DIR_PIN    PINA
#define PUMPE6_DIR_BIT    PA7

#define PUMPE6_DIAG_DDR   DDRG
#define PUMPE6_DIAG_PORT  PORTG
#define PUMPE6_DIAG_PIN   PING
#define PUMPE6_DIAG_BIT   PG3


// ---------------- PUMPE 7 ----------------
#define PUMPE7_STEP_DDR   DDRH
#define PUMPE7_STEP_PORT  PORTH
#define PUMPE7_STEP_PIN   PINH
#define PUMPE7_STEP_BIT   PH5

#define PUMPE7_DIR_DDR    DDRC
#define PUMPE7_DIR_PORT   PORTC
#define PUMPE7_DIR_PIN    PINC
#define PUMPE7_DIR_BIT    PC0

#define PUMPE7_DIAG_DDR   DDRG
#define PUMPE7_DIAG_PORT  PORTG
#define PUMPE7_DIAG_PIN   PING
#define PUMPE7_DIAG_BIT   PG4


// ---------------- PUMPE 8 ----------------
#define PUMPE8_STEP_DDR   DDRL
#define PUMPE8_STEP_PORT  PORTL
#define PUMPE8_STEP_PIN   PINL
#define PUMPE8_STEP_BIT   PL3

#define PUMPE8_DIR_DDR    DDRC
#define PUMPE8_DIR_PORT   PORTC
#define PUMPE8_DIR_PIN    PINC
#define PUMPE8_DIR_BIT    PC1

#define PUMPE8_DIAG_DDR   DDRG
#define PUMPE8_DIAG_PORT  PORTG
#define PUMPE8_DIAG_PIN   PING
#define PUMPE8_DIAG_BIT   PG5


// ---------------- PUMPE 9 ----------------
#define PUMPE9_STEP_DDR   DDRL
#define PUMPE9_STEP_PORT  PORTL
#define PUMPE9_STEP_PIN   PINL
#define PUMPE9_STEP_BIT   PL4

#define PUMPE9_DIR_DDR    DDRC
#define PUMPE9_DIR_PORT   PORTC
#define PUMPE9_DIR_PIN    PINC
#define PUMPE9_DIR_BIT    PC2

#define PUMPE9_DIAG_DDR   DDRH
#define PUMPE9_DIAG_PORT  PORTH
#define PUMPE9_DIAG_PIN   PINH
#define PUMPE9_DIAG_BIT   PH6


// ---------------- PUMPE 10 ----------------
#define PUMPE10_STEP_DDR  DDRL
#define PUMPE10_STEP_PORT PORTL
#define PUMPE10_STEP_PIN  PINL
#define PUMPE10_STEP_BIT  PL5

#define PUMPE10_DIR_DDR   DDRC
#define PUMPE10_DIR_PORT  PORTC
#define PUMPE10_DIR_PIN   PINC
#define PUMPE10_DIR_BIT   PC3

#define PUMPE10_DIAG_DDR  DDRH
#define PUMPE10_DIAG_PORT PORTH
#define PUMPE10_DIAG_PIN  PINH
#define PUMPE10_DIAG_BIT  PH7


// ---------------- EN ----------------
#define EN_DDR   DDRL
#define EN_PORT  PORTL
#define EN_PIN   PINL
#define EN_BIT   PL6


// ============================================================
// Klemmen X1..X4
// ============================================================

// ---------------- X1 (PJ2..PJ5) ----------------
#define X1_1_DDR   DDRJ
#define X1_1_PORT  PORTJ
#define X1_1_PIN   PINJ
#define X1_1_BIT   PJ2

#define X1_2_DDR   DDRJ
#define X1_2_PORT  PORTJ
#define X1_2_PIN   PINJ
#define X1_2_BIT   PJ3

#define X1_3_DDR   DDRJ
#define X1_3_PORT  PORTJ
#define X1_3_PIN   PINJ
#define X1_3_BIT   PJ4

#define X1_4_DDR   DDRJ
#define X1_4_PORT  PORTJ
#define X1_4_PIN   PINJ
#define X1_4_BIT   PJ5


// ---------------- X2 (PK0..PK3) ----------------
#define X2_1_DDR   DDRK
#define X2_1_PORT  PORTK
#define X2_1_PIN   PINK
#define X2_1_BIT   PK0

#define X2_2_DDR   DDRK
#define X2_2_PORT  PORTK
#define X2_2_PIN   PINK
#define X2_2_BIT   PK1

#define X2_3_DDR   DDRK
#define X2_3_PORT  PORTK
#define X2_3_PIN   PINK
#define X2_3_BIT   PK2

#define X2_4_DDR   DDRK
#define X2_4_PORT  PORTK
#define X2_4_PIN   PINK
#define X2_4_BIT   PK3


// ---------------- X3 (PK4..PK7) ----------------
#define X3_1_DDR   DDRK
#define X3_1_PORT  PORTK
#define X3_1_PIN   PINK
#define X3_1_BIT   PK4

#define X3_2_DDR   DDRK
#define X3_2_PORT  PORTK
#define X3_2_PIN   PINK
#define X3_2_BIT   PK5

#define X3_3_DDR   DDRK
#define X3_3_PORT  PORTK
#define X3_3_PIN   PINK
#define X3_3_BIT   PK6

#define X3_4_DDR   DDRK
#define X3_4_PORT  PORTK
#define X3_4_PIN   PINK
#define X3_4_BIT   PK7


// ---------------- X4 (PE0..PE3) ----------------
#define X4_1_DDR   DDRE
#define X4_1_PORT  PORTE
#define X4_1_PIN   PINE
#define X4_1_BIT   PE0

#define X4_2_DDR   DDRE
#define X4_2_PORT  PORTE
#define X4_2_PIN   PINE
#define X4_2_BIT   PE1

#define X4_3_DDR   DDRE
#define X4_3_PORT  PORTE
#define X4_3_PIN   PINE
#define X4_3_BIT   PE2

#define X4_4_DDR   DDRE
#define X4_4_PORT  PORTE
#define X4_4_PIN   PINE
#define X4_4_BIT   PE3


// ============================================================
// Aliase für Sensoren (damit PIN_* Makros sauber funktionieren)
// HOME = X1_4, SR04_TRIG = X1_3, SR04_ECHO = X1_2
// ============================================================
#define HOME_DDR   X1_4_DDR
#define HOME_PORT  X1_4_PORT
#define HOME_PIN   X1_4_PIN
#define HOME_BIT   X1_4_BIT

#define SR04_TRIG_DDR   X1_3_DDR
#define SR04_TRIG_PORT  X1_3_PORT
#define SR04_TRIG_PIN   X1_3_PIN
#define SR04_TRIG_BIT   X1_3_BIT

#define SR04_ECHO_DDR   X1_2_DDR
#define SR04_ECHO_PORT  X1_2_PORT
#define SR04_ECHO_PIN   X1_2_PIN
#define SR04_ECHO_BIT   X1_2_BIT
