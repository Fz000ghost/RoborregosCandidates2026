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

// Index in RING of the robot's current tile.
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

// Starts on INICIO facing into the 3x3, and slides onto its ring tile. Checks the ball tile's visible sides.
// If all three are closed, the opening is the one behind. Goes to the opening, turns its tail toward
// the ball and grabs it, then faces east, slides to the E tile and drives onto checkpoint 1.
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

// Drive forward until one raw read says WHITE, or until travel reaches "until".
// travel = forward ms since checkpoint 1. Returns the color that stopped it, or "" if it reached "until".
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

// Sensor at sideways position `to` (strafe ms from the middle, + = right) creeps toward the line at travel `at`.
// Starts 2 * MS_PAST short and stops by MS_PAST past it, so the front wheels stay behind the line.
// Returns true if it stopped on white there (one bad white read on green is ignored).
bool lineAt(long to, long at, long &travel, long &y) {
  long d = at - 2 * MS_PAST - travel;
  if (d > 0) fwdMs(d); else backMs(-d);
  travel = at - 2 * MS_PAST;
  if (to > y) rightMs(to - y); else leftMs(y - to);
  y = to;
  while (!strcmp(creep(false, at + MS_PAST, travel), "WHITE"))
    if (!strcmp(tileColor(70), "WHITE")) return true;
  return false;
}

// The start row comes from the left sensor in column 1  (free = the top row is beside it = robot in the bottom row). 
// At each boundary it probes GAP_HALF / 2 from the middle, its own side first: white there = line covers that row, go to the other. 
// White only on the other side = go to its own row. No white = gap in the middle. A centered line has no white at either probe, 
// so one more probe at Y_LIM finds its end piece to resync. In column 4 it steps into a row and switches rows if the front sensor 
// sees a wall, then crosses the white border and moves onto checkpoint 2.
void section2() {
  long travel = 0, y = 0;
  long origin = MS_TO_GREEN;
  if (!strcmp(creep(false, MS_STRAIGHT / 2, travel), "WHITE")) origin = travel;
  fwdMs(MS_CROSS);
  travel += MS_CROSS;
  long probe = origin + MS_STRAIGHT - 2 * MS_PAST;
  fwdMs(probe - travel);
  travel = probe;
  y = isFree(0) ? GAP_HALF : -GAP_HALF;
  for (byte k = 1; k <= 3; k++) {
    long at = origin + (long)k * MS_STRAIGHT;
    long s = y < 0 ? -1 : 1;
    bool own = lineAt(s * GAP_HALF / 2, at, travel, y);
    bool other = !own && lineAt(-s * GAP_HALF / 2, at, travel, y);
    if (own || other || lineAt(-s * Y_LIM, at, travel, y)) origin = travel - (long)k * MS_STRAIGHT;
    long gap = own ? -s * GAP_HALF : other ? s * GAP_HALF : 0;
    if (gap > y) rightMs(gap - y); else leftMs(y - gap);
    y = gap;
  }
  long end = origin + 4L * MS_STRAIGHT - 3 * MS_PAST;
  fwdMs(end - travel);
  travel = end;
  if (!y) { rightMs(GAP_HALF); y = GAP_HALF; }
  if (!isFree(1)) { if (y > 0) leftMs(2 * y); else rightMs(-2 * y); y = -y; }
  while (true) {
    const char *c = creep(true, origin + 5L * MS_STRAIGHT, travel);
    if (!strcmp(c, "RED")) { if (isRed()) break; continue; }
    if (!strcmp(c, "WHITE")) { fwdMs(MS_CROSS); travel += MS_CROSS; continue; }
    break;
  }
  fwdMs(MS_ONTO_CP2);
}

// Tile colors are directions in the track's frame, and the robot never turns, so each color
// is one move (red = checkpoint 2, the path starts ahead). Stops on Light Green (FIN) or 3 unclear reads in a row.
void section3() {
  byte unclear = 0;
  while (unclear < 3) {
    const char *c = tileColor(70);
    displayColor(c);
    if (!strcmp(c, "ORANGE") || !strcmp(c, "RED")) forward30();
    else if (!strcmp(c, "PINK"))   back30();
    else if (!strcmp(c, "CYAN"))   right30();
    else if (!strcmp(c, "YELLOW")) left30();
    else if (!strcmp(c, "Light Green")) return;
    else { unclear++; continue; }
    unclear = 0;
  }
}

// Starting on red means a restart on a checkpoint after a lack of progress. It peeks at the
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
