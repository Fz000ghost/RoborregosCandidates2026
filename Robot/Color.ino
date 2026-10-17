// Color sensor

// The sensor and the LED are on the color board (second Uno). It sends the index of the color it saw,
// so these names must be in the same order as COLORS[] in ColorBoard.ino.
const char *COLOR_NAMES[] = {"WHITE", "RED", "GREEN", "YELLOW", "CYAN", "ORANGE", "PINK", "Light Green"};
const byte N_COLORS = sizeof(COLOR_NAMES) / sizeof(COLOR_NAMES[0]);
byte colorCount = 0;   // The board's read counter at the last getColor(): it changes with every new read

// Latest color the board read (under 1 ms); "NO SIGNAL" if a channel read 0, "?" if the board didn't answer.
const char *getColor() {
  if (Wire.requestFrom(COLOR_ADDR, (byte)2) != 2) return "?";
  byte i = Wire.read();
  colorCount = Wire.read();
  return i < N_COLORS ? COLOR_NAMES[i] : "NO SIGNAL";
}

// Tile color by vote. 15 reads, returns the most common color if it got at least minPct % of the votes, else "?".
// Each vote is a new read of the board (its counter changed). No new read for 1 s = "?".
const char *tileColor(byte minPct) {
  const byte N = 15;
  byte votes[N_COLORS] = {0};
  getColor();
  byte last = colorCount;
  unsigned long t0 = millis();
  for (byte i = 0; i < N; ) {
    if (millis() - t0 > 1000) return "?";
    imuWait(5);   // The board reads ~30 times/s: don't flood it with requests
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
// Sends the color's index to the board, which drives the LED (255 = off).
void displayColor(const char *c) {
  byte i = 0;
  while (i < N_COLORS && strcmp(c, COLOR_NAMES[i])) i++;
  Wire.beginTransmission(COLOR_ADDR);
  Wire.write(i < N_COLORS ? i : 255);
  Wire.endTransmission();
}
