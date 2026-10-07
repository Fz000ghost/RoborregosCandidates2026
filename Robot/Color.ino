// COLOR SENSOR

int readFilter(byte s2, byte s3) {
  digitalWrite(S2, s2);
  digitalWrite(S3, s3);
  delay(10);
  return pulseIn(COLOR_OUT, LOW, 20000);
}

void readColor() {
  rgb[0] = readFilter(LOW, LOW); 
  rgb[1] = readFilter(HIGH, HIGH);
  rgb[2] = readFilter(LOW, HIGH);
}


struct Ref { const char *name; int r, g, b; };
Ref COLORS[] = {
  {"WHITE",   10,  10,  10},
  {"BLACK",  30, 30, 30},
  {"RED",     12, 26,  21},
  {"GREEN",   23,  20,  20},
  {"BLUE",   30,  20,  10},
  {"YELLOW",  10,  15,  17},
  {"CYAN",    18,  18,  15},
  {"ORANGE",  11,  20,  19},
  {"PINK",    11,  22,  16},
};
const byte N_COLORS = sizeof(COLORS) / sizeof(COLORS[0]);


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