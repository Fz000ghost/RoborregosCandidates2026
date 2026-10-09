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


/*
struct Ref { const char *name; int r, g, b; };
Ref COLORS[] = {
  {"WHITE",   10,  10,  10},
  {"RED",     12, 26,  21},
  {"GREEN",   18,  12,  15},
  {"YELLOW",  10,  15,  17},
  {"CYAN",    18,  18,  15},
  {"ORANGE",  6,  15,  15},
  {"PINK",    11,  22,  16},
};
*/

struct Ref { const char *name; int r, g, b; };
Ref COLORS[] = {
  {"WHITE",   30,  30,  30},
  {"RED",     35, 160,  100},
  {"GREEN",   85,  60,  70},
  {"YELLOW",  20,  30,  45},
  {"CYAN",    45,  30,  20},
  {"ORANGE",  30,  80,  80},
  {"PINK",    30,  80,  50},
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

void displayColor() {
  if (colorName() == "WHITE"){
  analogWrite(LED_R, 255);
  analogWrite(LED_G, 255);
  analogWrite(LED_B, 255);
  } else if (colorName() == "RED"){
    analogWrite(LED_R, 255);
    analogWrite(LED_G, 0);
    analogWrite(LED_B, 0);
  } else if (colorName() == "GREEN"){
    analogWrite(LED_R, 0);
    analogWrite(LED_G, 255);
    analogWrite(LED_B, 0);
  } else if (colorName() == "YELLOW"){
    analogWrite(LED_R, 255);
    analogWrite(LED_G, 100);
    analogWrite(LED_B, 0);
  } else if (colorName() == "CYAN"){
    analogWrite(LED_R, 0);
    analogWrite(LED_G, 255);
    analogWrite(LED_B, 255);
  } else if (colorName() == "ORANGE"){
    analogWrite(LED_R, 255);
    analogWrite(LED_G, 40);
    analogWrite(LED_B, 0);
  } else if (colorName() == "PINK"){
    analogWrite(LED_R, 255);
    analogWrite(LED_G, 20);
    analogWrite(LED_B, 100);
  }
}