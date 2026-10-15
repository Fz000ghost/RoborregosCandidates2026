// COLOR SENSOR

// One color channel: sets the S2/S3 filter, waits 10 ms, returns the pulse length (0 = no signal).
int readFilter(byte s2, byte s3) {
  digitalWrite(S2, s2);
  digitalWrite(S3, s3);
  delay(10);
  return pulseIn(COLOR_OUT, LOW, 20000);
}

// Reads red, green and blue into rgb[].
void readColor() {
  rgb[0] = readFilter(LOW, LOW);
  rgb[1] = readFilter(HIGH, HIGH);
  rgb[2] = readFilter(LOW, HIGH);
}

struct Ref { const char *name; int r, g, b; };
Ref COLORS[] = {
  {"WHITE",   8,  8,  7},
  {"RED",     10, 27, 21},
  {"GREEN",   27,  22,  23},
  {"YELLOW",  7,  9,  13},
  {"CYAN",    25,  13,  9},
  {"ORANGE",  9,  19,  19},
  {"PINK",    5,  15,  9},
  {"Light Green",    7,  7,  11}
};
const byte N_COLORS = sizeof(COLORS) / sizeof(COLORS[0]);

// Name of the reference color closest to the last rgb[] read, or "NO SIGNAL" if a channel read 0.
const char *colorName() {
  if (rgb[0] == 0 || rgb[1] == 0 || rgb[2] == 0) return "NO SIGNAL";
  const char *best = "?";
  long bestD = 2147483647L;
  for (byte i = 0; i < N_COLORS; i++) {
    long d = sq((long)rgb[0] - COLORS[i].r) + sq((long)rgb[1] - COLORS[i].g) + sq((long)rgb[2] - COLORS[i].b);
    if (d < bestD) { bestD = d; best = COLORS[i].name; }
  }
  return best;
}

// Tile color by vote. 15 reads, returns the most common color if it got at least minPct % of the votes, else "?".
const char *tileColor(byte minPct) {
  const byte N = 15;
  byte votes[N_COLORS] = {0};
  for (byte i = 0; i < N; i++) {
    readColor();
    const char *c = colorName();
    for (byte k = 0; k < N_COLORS; k++) if (c == COLORS[k].name) votes[k]++;
  }
  byte best = 0;
  for (byte k = 1; k < N_COLORS; k++) if (votes[k] > votes[best]) best = k;
  return votes[best] * 100 >= minPct * N ? COLORS[best].name : "?";
}

// Sets the RGB LED, 0-255 per channel.
void setLed(byte r, byte g, byte b) {
  analogWrite(LED_R, r);
  analogWrite(LED_G, g);
  analogWrite(LED_B, b);
}

// Shows color name c on the RGB LED; anything unknown turns it off.
void displayColor(const char *c) {
  if      (!strcmp(c, "WHITE"))  setLed(255, 255, 255);
  else if (!strcmp(c, "RED"))    setLed(255, 0, 0);
  else if (!strcmp(c, "GREEN") || !strcmp(c, "Light Green")) setLed(0, 255, 0);
  else if (!strcmp(c, "YELLOW")) setLed(255, 100, 0);
  else if (!strcmp(c, "CYAN"))   setLed(0, 255, 255);
  else if (!strcmp(c, "ORANGE")) setLed(255, 40, 0);
  else if (!strcmp(c, "PINK"))   setLed(255, 20, 100);
  else                           setLed(0, 0, 0);
}
