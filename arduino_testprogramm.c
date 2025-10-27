/*
    I2C Master für die Anlage
    Board: Arduino UNO (I2C auf A4 = SDA, A5 = SCL)

    Bedienung über den Seriellen Monitor (115200 Baud), Befehle:
      home
      fahr <einheiten>
      status
      pumpe <id 1..6> <sekunden 0..255>
      beladen
      entladen
      help

    Dieses Programm spricht exakt das Protokoll,
    das dein Slave-Sketch (mit TMC2209, Band, Pumpen usw.) implementiert.

    Struktur und Stil angelehnt an deine Liftsteuerung-Referenz:
    - Klare Abschnitte
    - zentrale Konstanten
    - eigene I2C-Sende-Funktionen
    - klarer "Command Parser" aus der seriellen Schnittstelle
*/

#include <Wire.h>

// ==============================
//         Konfiguration
// ==============================

// Muss mit dem Slave übereinstimmen
static const uint8_t I2C_ADDR        = 0x13;
static const long    STEPS_PER_UNIT  = 400L;   // vom Slave: 1 Einheit = 400 Steps

// Serielle Eingabe
#define SERIAL_BAUD        115200
#define SERIAL_BUF_LEN     32

// ==============================
//         Befehls-Codes
// ==============================
// Muss 1:1 zu deinem enum im Slave passen
enum : uint8_t {
    CMD_FAHR     = 0,
    CMD_HOME     = 1,
    CMD_STATUS   = 2,
    CMD_PUMPE    = 3,
    CMD_BELADEN  = 4,
    CMD_ENTLADEN = 5
};

// ==============================
//   Globale Variablen (Master)
// ==============================
char     serialBuf[SERIAL_BUF_LEN];
uint8_t  serialPos = 0;
bool     lineReady = false;


// ==============================
//   Low-Level I2C Hilfsfunktionen
// ==============================

// sendet genau 1 Byte (reiner Befehl ohne Parameter)
bool i2cSendSimple(uint8_t cmd) {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(cmd);
    uint8_t err = Wire.endTransmission();   // 0 = OK
    return (err == 0);
}

// sendet FAHR + signed Ziel-Einheiten (1 Byte int8_t)
bool i2cSendFahr(int8_t einheiten) {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(CMD_FAHR);
    Wire.write((uint8_t)einheiten); // wird im Slave als int8_t gelesen
    uint8_t err = Wire.endTransmission();
    return (err == 0);
}

// sendet PUMPE + pumpenID + dauerSek
bool i2cSendPumpe(uint8_t pumpenId, uint8_t dauerSek) {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(CMD_PUMPE);
    Wire.write(pumpenId);
    Wire.write(dauerSek);
    uint8_t err = Wire.endTransmission();
    return (err == 0);
}

// nach *allen* Kommandos außer STATUS fragt der Master im Slave ein ACK (1 Byte) ab
// Der Slave schickt 0x06 als Bestätigung
void i2cReadAck() {
    delayMicroseconds(500); // ganz kleines Delay, damit Slave in onRequest() ist
    Wire.requestFrom(I2C_ADDR, (uint8_t)1);
    if (Wire.available()) {
        uint8_t ack = Wire.read();
        Serial.print(F("ACK: 0x"));
        if (ack < 16) Serial.print('0');
        Serial.println(ack, HEX);
    } else {
        Serial.println(F("ACK FEHLT / keine Antwort"));
    }
}

// STATUS ist speziell: Slave liefert bei Request 5 Bytes:
//  [0] busy (1 oder 0)
//  [1..4] aktuelle PLF-Position (long, little endian)
void i2cRequestStatus() {
    // 1) STATUS-Kommando schicken
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(CMD_STATUS);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        Serial.print(F("I2C Fehler (STATUS TX): "));
        Serial.println(err);
        return;
    }

    // 2) Antwort anfordern (5 Bytes)
    delayMicroseconds(500); // Slave hat onRequest()
    Wire.requestFrom(I2C_ADDR, (uint8_t)5);

    if (Wire.available() < 5) {
        Serial.println(F("STATUS FEHLT / unvollständig"));
        while (Wire.available()) Wire.read(); // buffer leeren
        return;
    }

    uint8_t busy_flag = Wire.read();
    uint8_t b0 = Wire.read();
    uint8_t b1 = Wire.read();
    uint8_t b2 = Wire.read();
    uint8_t b3 = Wire.read();

    long posSteps =
        ((long)b0) |
        ((long)b1 << 8) |
        ((long)b2 << 16) |
        ((long)b3 << 24);

    float posUnits = (float)posSteps / (float)STEPS_PER_UNIT;

    // Ausgabe
    Serial.println(F("------ STATUS ------"));
    Serial.print(F("busy:        "));
    Serial.println(busy_flag ? F("1 (BUSY)") : F("0 (IDLE)"));

    Serial.print(F("posSteps:    "));
    Serial.println(posSteps);

    Serial.print(F("posUnits:    "));
    Serial.println(posUnits, 3);
    Serial.println(F("--------------------"));
}


// ==============================
//   Serielle Hilfsfunktionen
// ==============================

// Kleines Hilfs-Trim vorne
char* skipSpaces(char* p) {
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    return p;
}

// Liest eine komplette Zeile aus Serial in serialBuf
// lineReady wird true sobald \n oder \r kam
void pollSerialInput() {
    while (Serial.available() && !lineReady) {
        char c = Serial.read();

        // Zeilenende?
        if (c == '\n' || c == '\r') {
            if (serialPos > 0) {
                serialBuf[serialPos] = '\0';
                lineReady = true;
            }
            serialPos = 0; // reset für nächstes Kommando
        } else {
            if (serialPos < (SERIAL_BUF_LEN - 1)) {
                serialBuf[serialPos++] = c;
            } else {
                // Überlauf -> abschneiden, aber noch beenden
                serialBuf[SERIAL_BUF_LEN - 1] = '\0';
                lineReady = true;
                serialPos = 0;
            }
        }
    }
}

// druckt kurze Hilfe
void printHelp() {
    Serial.println(F("Befehle:"));
    Serial.println(F("  home"));
    Serial.println(F("  fahr <einheiten signed -128..127>"));
    Serial.println(F("  status"));
    Serial.println(F("  pumpe <id 1..6> <sekunden 0..255>"));
    Serial.println(F("  beladen"));
    Serial.println(F("  entladen"));
    Serial.println(F("  help"));
}

// Parsed eine fertige Zeile aus serialBuf und führt sie aus
void handleCommand(char* line) {
    line = skipSpaces(line);
    if (*line == '\0') return;

    // Wir zerstückeln mit strtok() (einfach, UNO kann das)
    char* cmd = strtok(line, " \t");
    if (!cmd) return;

    // --- "home" ---
    if (!strcmp(cmd, "home")) {
        Serial.println(F("[CMD] HOME"));
        if (!i2cSendSimple(CMD_HOME)) {
            Serial.println(F("I2C Fehler bei HOME (TX)"));
            return;
        }
        i2cReadAck();
        return;
    }

    // --- "fahr <einheiten>" ---
    if (!strcmp(cmd, "fahr")) {
        char* arg = strtok(NULL, " \t");
        if (!arg) {
            Serial.println(F("fahr braucht eine Zahl"));
            return;
        }

        long val = atol(arg); // long -> wir kürzen später auf int8_t
        if (val > 127)  val = 127;
        if (val < -128) val = -128;

        Serial.print(F("[CMD] FAHR "));
        Serial.println((int)val);

        if (!i2cSendFahr((int8_t)val)) {
            Serial.println(F("I2C Fehler bei FAHR (TX)"));
            return;
        }
        i2cReadAck();
        return;
    }

    // --- "status" ---
    if (!strcmp(cmd, "status")) {
        Serial.println(F("[CMD] STATUS"));
        i2cRequestStatus();
        return;
    }

    // --- "pumpe <id> <sek>" ---
    if (!strcmp(cmd, "pumpe")) {
        char* idStr  = strtok(NULL, " \t");
        char* tStr   = strtok(NULL, " \t");

        if (!idStr || !tStr) {
            Serial.println(F("pumpe braucht: pumpe <id 1..6> <sek>"));
            return;
        }

        long id  = atol(idStr);
        long sek = atol(tStr);

        if (id < 1)   id = 1;
        if (id > 6)   id = 6;
        if (sek < 0)  sek = 0;
        if (sek > 255) sek = 255;

        Serial.print(F("[CMD] PUMPE ID="));
        Serial.print(id);
        Serial.print(F(" SEK="));
        Serial.println(sek);

        if (!i2cSendPumpe((uint8_t)id, (uint8_t)sek)) {
            Serial.println(F("I2C Fehler bei PUMPE (TX)"));
            return;
        }
        i2cReadAck();
        return;
    }

    // --- "beladen" ---
    if (!strcmp(cmd, "beladen")) {
        Serial.println(F("[CMD] BELADEN (Band vorwärts, stoppt mit Sensor)"));
        if (!i2cSendSimple(CMD_BELADEN)) {
            Serial.println(F("I2C Fehler bei BELADEN (TX)"));
            return;
        }
        i2cReadAck();
        return;
    }

    // --- "entladen" ---
    if (!strcmp(cmd, "entladen")) {
        Serial.println(F("[CMD] ENTLADEN (Band rückwärts, Timer 10s)"));
        if (!i2cSendSimple(CMD_ENTLADEN)) {
            Serial.println(F("I2C Fehler bei ENTLADEN (TX)"));
            return;
        }
        i2cReadAck();
        return;
    }

    // --- "help" ---
    if (!strcmp(cmd, "help")) {
        printHelp();
        return;
    }

    // Unbekannt
    Serial.println(F("Unbekannter Befehl. 'help' eingeben."));
}



// ==============================
//         Setup / Loop
// ==============================
void setup() {
    // Serielle Schnittstelle für Benutzer
    Serial.begin(SERIAL_BAUD);
    while (!Serial) { /* warten bis USB bereit ist (bei echten UNO meist sofort) */ }

    Serial.println(F("=== I2C MASTER START ==="));
    Serial.println(F("Adresse Slave: 0x13"));
    Serial.println(F("Tippe 'help' für Befehle."));
    Serial.println();

    // I2C als Master starten
    Wire.begin(); // UNO als Master -> kein Parameter
}

void loop() {
    // Always read serial input
    pollSerialInput();

    // Wenn eine komplette Zeile da ist -> parsen
    if (lineReady) {
        handleCommand(serialBuf);
        lineReady = false;
        Serial.println(); // Leerzeile fürs Auge
    }

    // hier keine blockierenden Delays nötig
}
