// COLOR SENSOR

// THE SENSOR AND THE LED ARE ON THE COLOR BOARD (SECOND UNO). IT SENDS THE INDEX OF THE COLOR IT SAW,
// SO THESE NAMES MUST BE IN THE SAME ORDER AS COLORS[] IN ColorBoard.ino.
const char *COLOR_NAMES[] = {"WHITE", "RED", "GREEN", "YELLOW", "CYAN", "ORANGE", "PINK", "Light Green"};
const byte N_COLORS = sizeof(COLOR_NAMES) / sizeof(COLOR_NAMES[0]);
byte colorCount = 0;   // THE BOARD'S READ COUNTER AT THE LAST getColor(): IT CHANGES WITH EVERY NEW READ

// LATEST COLOR THE BOARD READ (UNDER 1 MS); "NO SIGNAL" IF A CHANNEL READ 0, "?" IF THE BOARD DIDN'T ANSWER.
const char *getColor() {
  if (Wire.requestFrom(COLOR_ADDR, (byte)2) != 2) return "?";
  byte i = Wire.read();
  colorCount = Wire.read();
  return i < N_COLORS ? COLOR_NAMES[i] : "NO SIGNAL";
}

// Tile color by vote. 15 reads, returns the most common color if it got at least minPct % of the votes, else "?".
// EACH VOTE IS A NEW READ OF THE BOARD (ITS COUNTER CHANGED). NO NEW READ FOR 1 S = "?".
const char *tileColor(byte minPct) {
  const byte N = 15;
  byte votes[N_COLORS] = {0};
  getColor();
  byte last = colorCount;
  unsigned long t0 = millis();
  for (byte i = 0; i < N; ) {
    if (millis() - t0 > 1000) return "?";
    delay(5);   // THE BOARD READS ~30 TIMES/S: DON'T FLOOD IT WITH REQUESTS
    const char *c = getColor();
    if (colorCount == last) continue;
    last = colorCount;
    t0 = millis();
    i++;
    for (byte k = 0; k < N_COLORS; k++) if (c == COLOR_NAMES[k]) votes[k]++;
  }
  byte best = 0;
  for (byte k = 1; k < N_COLORS; k++) if (votes[k] > votes[best]) best = k;
  return votes[best] * 100 >= minPct * N ? COLOR_NAMES[best] : "?";
}

// Shows color name c on the RGB LED; anything unknown turns it off.
// SENDS THE COLOR'S INDEX TO THE BOARD, WHICH DRIVES THE LED (255 = OFF).
void displayColor(const char *c) {
  byte i = 0;
  while (i < N_COLORS && strcmp(c, COLOR_NAMES[i])) i++;
  Wire.beginTransmission(COLOR_ADDR);
  Wire.write(i < N_COLORS ? i : 255);
  Wire.endTransmission();
}
