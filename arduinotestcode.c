#include <Wire.h>

// --------------------------------------
// Konfiguration
// --------------------------------------
static uint8_t i2c_addr = 0x13;   // Slave-Adresse (anpassbar mit 'addr')
static const unsigned long BAUD = 115200;

// Protokoll (muss zum Slave passen)
enum : uint8_t {
  CMD_FAHR   = 0,  // + Param int8_t
  CMD_HOME   = 1,  // Param ignoriert
  CMD_STATUS = 2   // Param ignoriert, Slave liefert 5 Bytes: [busy, pos0..3]
};

// Konsolenzeilenpuffer
static char lineBuf[96];
static size_t lineLen = 0;

// --------------------------------------
// Hilfsfunktionen
// --------------------------------------
void printPrompt() {
  Serial.print("> ");
}

void printHelp() {
  Serial.println(F("Befehle:"));
  Serial.println(F("  fahr <n>     - bewegen um n Einheiten (-128..127)"));
  Serial.println(F("  home         - Homing ausloesen"));
  Serial.println(F("  status       - Status vom Slave lesen"));
  Serial.println(F("  addr <wert>  - I2C-Adresse setzen (z.B. 0x13 oder 19)"));
  Serial.println(F("  help         - diese Hilfe"));
}

bool sendCommand(uint8_t cmd, int8_t par) {
  Wire.beginTransmission(i2c_addr);
  Wire.write(cmd);
  Wire.write((uint8_t)par);
  uint8_t rc = Wire.endTransmission(); // 0 = OK
  if (rc != 0) {
    Serial.print(F("I2C Fehler endTransmission(): "));
    Serial.println(rc);
    return false;
  }
  return true;
}

// Liest genau 'need' Bytes (wenn moeglich) vom Slave
int requestBytes(uint8_t need, uint8_t *buf) {
  int n = Wire.requestFrom((int)i2c_addr, (int)need);
  int got = 0;
  while (Wire.available() && got < n) {
    buf[got++] = Wire.read();
  }
  return got;
}

static inline void trim(char *s) {
  // führende Spaces überspringen
  char *p = s;
  while (*p == ' ' || *p == '\t' || *p == '\r') p++;
  if (p != s) memmove(s, p, strlen(p) + 1);
  // trailing entfernen
  size_t L = strlen(s);
  while (L && (s[L-1] == ' ' || s[L-1] == '\t' || s[L-1] == '\r' || s[L-1] == '\n')) {
    s[--L] = '\0';
  }
}

long parseLongAutoBase(const char *tok, bool *ok) {
  if (!tok || !*tok) { if (ok) *ok = false; return 0; }
  char *endptr = nullptr;
  long v = strtol(tok, &endptr, 0); // auto base (0x.., 0.., dec)
  if (ok) *ok = (endptr && *endptr == '\0');
  return v;
}

void handleStatus() {
  if (!sendCommand(CMD_STATUS, 0)) return;

  uint8_t buf[5] = {0};
  int got = requestBytes(5, buf);
  if (got < 1) {
    Serial.println(F("Keine Statusdaten empfangen."));
    return;
  }

  uint8_t busy = buf[0];
  Serial.print(F("busy: "));
  Serial.println(busy ? F("ja") : F("nein"));

  if (got >= 5) {
    // Little-Endian -> 32-bit signed
    int32_t pos = (int32_t)(
        (uint32_t)buf[1] |
        ((uint32_t)buf[2] << 8) |
        ((uint32_t)buf[3] << 16) |
        ((uint32_t)buf[4] << 24)
    );
    Serial.print(F("position (steps): "));
    Serial.println(pos);
    // Umrechnung in Einheiten (1 Einheit = 400 Steps)
    float units = pos / 400.0f;
    Serial.print(F("position (einheiten): "));
    Serial.println(units, 3);
  } else {
    Serial.println(F("(Hinweis: Slave hat weniger als 5 Bytes geliefert.)"));
  }
}

void handleFahr(long val) {
  // In int8_t Bereich beschneiden (Slave erwartet 1 Byte)
  if (val < -128) { Serial.println(F("Warnung: < -128, auf -128 begrenzt.")); val = -128; }
  if (val >  127) { Serial.println(F("Warnung: > 127, auf 127 begrenzt."));  val = 127;  }
  int8_t par = (int8_t)val;

  if (!sendCommand(CMD_FAHR, par)) return;

  // optional: ACK (1 Byte) abholen
  uint8_t ack = 0;
  int got = requestBytes(1, &ack);
  if (got == 1) {
    Serial.print(F("ACK: 0x"));
    if (ack < 16) Serial.print('0');
    Serial.println(ack, HEX);
  } else {
    Serial.println(F("Kein ACK empfangen (optional)."));
  }
}

void handleHome() {
  if (!sendCommand(CMD_HOME, 0)) return;

  // optional: ACK (1 Byte)
  uint8_t ack = 0;
  int got = requestBytes(1, &ack);
  if (got == 1) {
    Serial.print(F("ACK: 0x"));
    if (ack < 16) Serial.print('0');
    Serial.println(ack, HEX);
  } else {
    Serial.println(F("Kein ACK empfangen (optional)."));
  }
}

void handleAddr(long val, bool ok) {
  if (!ok || val < 0 || val > 127) {
    Serial.println(F("Ungueltige Adresse. Erlaubt: 0..127 bzw. 0x00..0x7F"));
    return;
  }
  i2c_addr = (uint8_t)val;
  Serial.print(F("Neue I2C-Adresse: 0x"));
  if (i2c_addr < 16) Serial.print('0');
  Serial.println(i2c_addr, HEX);
}

// --------------------------------------
// Parser für Konsolenzeilen
// --------------------------------------
void handleLine(char *line) {
  trim(line);
  if (!*line) return;

  // Token 1
  char *cmd = strtok(line, " \t");
  if (!cmd) return;

  // bewusst nur Kleinbuchstaben erwarten (einfacher)
  if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) {
    printHelp();
    return;
  }

  if (!strcmp(cmd, "fahr") || !strcmp(cmd, "move")) {
    char *p = strtok(nullptr, " \t");
    bool ok = false;
    long v = parseLongAutoBase(p, &ok);
    if (!ok) {
      Serial.println(F("Syntax: fahr <n>"));
      return;
    }
    handleFahr(v);
    return;
  }

  if (!strcmp(cmd, "home")) {
    handleHome();
    return;
  }

  if (!strcmp(cmd, "status") || !strcmp(cmd, "stat")) {
    handleStatus();
    return;
  }

  if (!strcmp(cmd, "addr")) {
    char *p = strtok(nullptr, " \t");
    bool ok = false;
    long v = parseLongAutoBase(p, &ok);
    handleAddr(v, ok);
    return;
  }

  Serial.println(F("Unbekannter Befehl. 'help' fuer Hilfe."));
}

// --------------------------------------
// Arduino-Standard
// --------------------------------------
void setup() {
  Serial.begin(BAUD);
  while (!Serial) { /* warten (z.B. Leonardo) */ }

  Wire.begin(); // Master
  delay(50);

  Serial.println(F("I2C-Master-Konsole bereit."));
  Serial.print(F("Slave-Adresse: 0x"));
  if (i2c_addr < 16) Serial.print('0');
  Serial.println(i2c_addr, HEX);
  printHelp();
  printPrompt();
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue; // ignorieren
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      handleLine(lineBuf);
      lineLen = 0;
      printPrompt();
    } else {
      if (lineLen < sizeof(lineBuf) - 1) {
        lineBuf[lineLen++] = c;
      }
      // sonst: Überlaufschutz – weitere Zeichen verwerfen
    }
  }
}
