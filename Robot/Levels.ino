// LEVELS (Pista B)

// True if sensor i (0 L, 1 F, 2 R) sees no wall on 3 reads in a row.
bool isFree(byte i) {
  for (byte k = 0; k < 3; k++) {
    readSensors();
    if (dist[i]) return false;
    delay(20);
  }
  return true;
}

// Tail already faces the ball tile: back in, close the claw, drive back out.
void grab() {
  backMs(MS_GRAB_BACK);
  toggleClaw();
  fwdMs(MS_GRAB_BACK);
}

// Section 1 cells: 3x3 = 1..3, ball (2,2), CP1 (4,2). RING = the 8 tiles around the ball, in order.
// SIDE = ring index of the tile N, E, S, W of the ball.
const byte RING[8][2] = {{1, 1}, {2, 1}, {3, 1}, {3, 2}, {3, 3}, {2, 3}, {1, 3}, {1, 2}};
const byte SIDE[4] = {5, 3, 1, 7};

// Index in RING of the robot's current tile (0 if it is not on the ring).
byte ringIndex() {
  for (byte i = 0; i < 8; i++) if (RING[i][0] == posX && RING[i][1] == posY) return i;
  return 0;
}

// Steps between ring tiles a and b, the short way around.
byte ringDist(byte a, byte b) { return min((a + 8 - b) % 8, (b + 8 - a) % 8); }

// Slide around the ball tile to ring tile `to`, without turning. Takes the short way unless it passes
// SIDE[heading], the tile where the tail (claw) points into the ball tile's wall.

void ringTo(byte to) {
  byte i = ringIndex();
  byte s = (to + 8 - i) % 8 <= 4 ? 1 : 7;
  for (byte k = (i + s) % 8; k != to; k = (k + s) % 8) if (k == SIDE[heading]) { s = 8 - s; break; }
  while (i != to) {
    i = (i + s) % 8;
    for (byte d = 0; d < 4; d++) if (posX + DX[d] == RING[i][0] && posY + DY[d] == RING[i][1]) { slide(d); break; }
  }
}

// Sensor (0 L, 1 F, 2 R) that looks at the ball from the tile on side d of it; 3 = behind, no sensor.
byte sensorTo(byte d) { return ((d + 2) % 4 + 5 - heading) % 4; }

// Starts on INICIO (START_X, START_Y, set each round in calibration) facing into the 3x3,
// and slides onto its ring tile. Checks the ball tile's visible sides, nearest unchecked first;
// if all three are closed, the opening is the one behind. Goes to the opening, turns its tail toward
// the ball and grabs it, then faces east (track forward for sections 2 and 3), slides to the E tile
// and drives onto checkpoint 1.
void section1() {
  posX = START_X; posY = START_Y; heading = START_HEADING;
  slide(heading);
  byte open = NO_SIDE;
  bool checked[4] = {false, false, false, false};
  while (open == NO_SIDE) {
    byte best = NO_SIDE;
    for (byte d = 0; d < 4; d++)
      if (sensorTo(d) < 3 && !checked[d] && (best == NO_SIDE || ringDist(SIDE[d], ringIndex()) < ringDist(SIDE[best], ringIndex()))) best = d;
    if (best == NO_SIDE) {
      for (byte d = 0; d < 4; d++) if (sensorTo(d) == 3) open = d;
      break;
    }
    ringTo(SIDE[best]);
    checked[best] = true;
    if (isFree(sensorTo(best))) open = best;
  }
  ringTo(SIDE[open]);
  turnTo(open);
  grab();
  turnTo(1);
  ringTo(SIDE[1]);
  forward30();
}

// Drive forward until one raw read says WHITE (or RED if redStops), or until travel reaches `until`.
// travel = forward ms since checkpoint 1. Returns the color that stopped it, or "" if it reached `until`.
const char *creep(bool redStops, long until, long &travel) {
  const char *c = "";
  unsigned long t0 = millis();
  setAll(FWD);
  while ((long)(millis() - t0) < until - travel) {
    readColor();
    const char *r = colorName();
    if (!strcmp(r, "WHITE") || (redStops && !strcmp(r, "RED"))) { c = r; break; }
  }
  setAll(0, 0, 0, 0);
  travel += millis() - t0;
  delay(MS_SETTLE);
  return c;
}

// Sensor on a line: strafe along it from the left limit to the right limit, then center the robot on the gap.
// y = sideways position in strafe ms from the middle (+ = right). A 2 s sweep takes ~60 of the N reads.
// The gap is the longest run of green; up to 2 non-green samples inside it are flicker, not white, so they
// don't split it, and the same 2-sample slack applies at the sweep's ends. If the gap runs past a limit,
// only one edge was seen and the center is set GAP_HALF from it. Returns false if the sweep didn't see both
// a gap and real white: a real line shows 10+ cm of white, while a few flickers on all-green mean the sensor
// is off the line.
bool centerOnGap(long &y) {
  leftMs(y + Y_LIM);
  const byte N = 80;
  bool g[N];
  unsigned t[N];
  byte n = 0, whites = 0;
  unsigned long t0 = millis();
  setAll(RIGHT);
  while (millis() - t0 < 2 * Y_LIM && n < N) {
    readColor();
    const char *c = colorName();
    g[n] = !strcmp(c, "GREEN");
    whites += !strcmp(c, "WHITE");
    t[n++] = millis() - t0;
  }
  setAll(0, 0, 0, 0);
  y = (long)(millis() - t0) - Y_LIM;
  delay(MS_SETTLE);

  byte bs = 0, bl = 0, start = 0, miss = 0;
  bool in = false;
  for (byte i = 0; i < n; i++) {
    if (g[i]) {
      if (!in) { in = true; start = i; }
      miss = 0;
      if (i + 1 - start > bl) { bl = i + 1 - start; bs = start; }
    } else if (in && ++miss > 2) in = false;
  }
  byte be = bs + bl - 1;
  bool atLeft = bs <= 2, atRight = be + 3 >= n;

  if (bl < 3 || (atLeft && atRight) || whites < 6) return false;

  long a = (long)t[bs] - Y_LIM, b = (long)t[be] - Y_LIM;
  long c = atLeft  ? b - GAP_HALF
         : atRight ? a + GAP_HALF
         : (a + b) / 2;
  c = constrain(c, -Y_LIM, Y_LIM);
  if (c > y) rightMs(c - y); else leftMs(y - c);
  y = c;
  return true;
}

// Cross the line the sensor reaches near travel `at`. The 30 cm gap is narrower than the sweep, so the line
// is under at least one sweep end: creep forward at each end until the sensor stops on white, then sweep there.
// Stopping on the sensor (not on timing) is what keeps short, slow-starting moves from missing the line.
bool crossLine(long at, long &travel, long &y) {
  for (int8_t end = -1; end <= 1; end += 2) {
    long d = at - 2 * MS_PAST - travel;
    if (d > 0) fwdMs(d); else backMs(-d);
    travel = at - 2 * MS_PAST;
    long to = end * Y_LIM;
    if (to > y) rightMs(to - y); else leftMs(y - to);
    y = to;
    if (!strcmp(creep(false, at + MS_PAST, travel), "WHITE") && centerOnGap(y)) return true;
  }
  return false;
}

// 2x4 green tiles; on each of the 3 tile boundaries inside it, a white line with a 30 cm gap.
// travel = forward ms since checkpoint 1, y = sideways ms from the middle (+ = right), origin = travel when
// the sensor reaches the green. The white border before the green sets where the boundaries are, and every
// line found resets it, so the timing error never adds up. For each boundary k: creep to just short of it,
// ignore one bad white read on green, then sweep for the gap. A line found far from the boundary is not
// boundary k, so k stays ahead. Afterwards the sensor is moved clear of the line.
// Shortcut: if no sweep finds the line, the robot drives on from wherever it is.
// Then it crosses the white border to checkpoint 2, stops if it overshoots, returns to the middle, and
// moves its body onto checkpoint 2.
void section2() {
  long travel = 0, y = 0;
  long origin = MS_TO_GREEN;
  if (!strcmp(creep(false, MS_STRAIGHT / 2, travel), "WHITE")) origin = travel;
  fwdMs(MS_CROSS);
  travel += MS_CROSS;
  byte k = 1;
  while (k <= 3) {
    long at = origin + (long)k * MS_STRAIGHT;
    bool raw = !strcmp(creep(false, at - 2 * MS_PAST, travel), "WHITE");
    bool white = raw && !strcmp(tileColor(70), "WHITE");
    bool near = travel > at - 3 * MS_PAST;
    if (raw && !white && !near) continue;

    bool swept = (white && centerOnGap(y)) || crossLine(white ? travel : at, travel, y);
    if (near && swept) origin = travel - (long)k * MS_STRAIGHT;
    if (near) k++;
    fwdMs(2 * MS_PAST);
    travel += 2 * MS_PAST;
  }
  while (true) {
    const char *c = creep(true, origin + 5L * MS_STRAIGHT, travel);
    if (!strcmp(c, "RED")) { if (isRed()) break; continue; }
    if (!strcmp(c, "WHITE")) { fwdMs(MS_CROSS); travel += MS_CROSS; continue; }
    break;
  }
  if (y > 0) leftMs(y); else rightMs(-y);
  fwdMs(MS_ONTO_CP2);
}

// Tile colors are directions in the track's frame, and the robot never turns, so each color
// is one move (red = checkpoint 2, the path starts ahead). Stops on GREEN (FIN) or 3 unclear reads in a row.
void section3() {
  byte unclear = 0;
  while (unclear < 3) {
    const char *c = tileColor(70);
    displayColor(c);
    if (!strcmp(c, "ORANGE") || !strcmp(c, "RED")) forward30();
    else if (!strcmp(c, "PINK"))   back30();
    else if (!strcmp(c, "CYAN"))   right30();
    else if (!strcmp(c, "YELLOW")) left30();
    else if (!strcmp(c, "GREEN"))  return;
    else { unclear++; continue; }
    unclear = 0;
  }
}

// Starting on red means a restart on a checkpoint after a lack of progress: it peeks at the
// next tile (green = checkpoint 1, so run section 2; else checkpoint 2) and backs up. Otherwise it runs
// sections 1 and 2. Section 3 always runs last.
void levels() {
  if (isRed()) {
    fwdMs(MS_STRAIGHT / 2);
    bool cp1 = !strcmp(tileColor(70), "GREEN");
    backMs(MS_STRAIGHT / 2);
    if (cp1) section2();
  } else {
    section1();
    section2();
  }
  section3();
}
