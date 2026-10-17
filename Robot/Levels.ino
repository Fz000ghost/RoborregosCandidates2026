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

// Tail already faces the ball tile: open the claw, back in, close it, drive back out.
// THE CLAW STARTS CLOSED, SO IT OPENS HERE FIRST.
void grab() {
  setClaw(true);
  moveCm(2, GRAB_CM);   // MEASURED WITH THE FRONT SENSOR, WHICH FACES AWAY FROM THE BALL
  setClaw(false);
  moveCm(0, GRAB_CM);
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
// 3 IS NOW THE BACK SENSOR, SO EVERY SIDE CAN BE CHECKED.
byte sensorTo(byte d) { return ((d + 2) % 4 + 5 - heading) % 4; }

// Starts on INICIO facing into the 3x3, and slides onto its ring tile. Checks the ball tile's visible sides.
// If all three are closed, the opening is the one behind. Goes to the opening, turns its tail toward
// the ball and grabs it, then faces east, slides to the E tile and drives onto checkpoint 1.
// WITH THE BACK SENSOR ALL FOUR SIDES ARE CHECKED DIRECTLY. ALL FOUR CLOSED MEANS A MISREAD: IT CHECKS THEM ALL
// ONCE MORE, THEN TRIES THE EAST SIDE.
void section1() {
  posX = START_X; posY = START_Y; heading = START_HEADING;
  slide(heading);
  byte open = NO_SIDE;
  bool checked[4] = {false, false, false, false};
  bool retried = false;
  while (open == NO_SIDE) {
    byte best = NO_SIDE;
    for (byte d = 0; d < 4; d++)
      if (!checked[d] && (best == NO_SIDE || ringDist(SIDE[d], ringIndex()) < ringDist(SIDE[best], ringIndex()))) best = d;
    if (best == NO_SIDE) {
      if (retried) { open = 1; break; }
      retried = true;
      memset(checked, 0, sizeof(checked));
      continue;
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
  while ((long)(millis() - t0) < until - travel) {
    steer(FWD);   // HOLDS HEADING BETWEEN COLOR READS
    const char *r = getColor();   // UNDER 1 MS: THE COLOR BOARD READS ON ITS OWN
    if (!strcmp(r, "WHITE") || (redStops && !strcmp(r, "RED"))) { c = r; break; }
  }
  setAll(0, 0, 0, 0);
  travel += millis() - t0;
  imuWait(MS_SETTLE);
  return c;
}

// Sensor at sideways position `to` (strafe ms from the middle, + = right) creeps toward the line at travel `at`.
// Starts 2 * MS_PAST short and stops by MS_PAST past it, so the front wheels stay behind the line.
// Returns true if it stopped on white there (one bad white read on green is ignored).
// SIDEWAYS POSITIONS (to, y) ARE NOW CM, MEASURED FROM THE CORRIDOR WALLS BY strafeTo().
bool lineAt(long to, long at, long &travel, long &y) {
  long d = at - 2 * MS_PAST - travel;
  if (d > 0) fwdMs(d); else backMs(-d);
  travel = at - 2 * MS_PAST;
  strafeTo(to, y);
  y = to;
  while (!strcmp(creep(false, at + MS_PAST, travel), "WHITE"))
    if (!strcmp(tileColor(70), "WHITE")) return true;
  return false;
}

// ROBOT CENTER'S POSITION ALONG THE CORRIDOR IN CM (0 = CP1/GREEN EDGE, 120 = FAR END), FROM THE END WALL AHEAD
// OR THE START WALL BEHIND. EACH CHECKPOINT IS IN LINE WITH ONE ROW, SO THE OTHER ROW HAS A WALL AT THAT END.
// A READING MORE THAN 10 CM FROM THE TIMED ESTIMATE xTimed SAW THROUGH A CHECKPOINT (30+ CM OFF) AND IS IGNORED.
long wallX(long xTimed) {
  int f = readCm(1), b = readCm(3);
  long xf = 105 + FRONT_CENTER_CM - f, xb = 15 + b - BACK_CENTER_CM;
  if (f && abs(xf - xTimed) <= 10) return xf;
  if (b && abs(xb - xTimed) <= 10) return xb;
  return xTimed;
}

// travel CORRECTED FROM THE CORRIDOR WALLS (wallX). origin = travel WHEN THE COLOR SENSOR WAS ON THE CP1/GREEN EDGE;
// travel COUNTS THE COLOR SENSOR, THE WALLS GIVE THE ROBOT'S CENTER, COLOR_AHEAD_CM CONVERTS BETWEEN THEM.
long resync(long travel, long origin) {
  long xTimed = (travel - origin) * 30 / MS_STRAIGHT - COLOR_AHEAD_CM;
  return origin + (wallX(xTimed) + COLOR_AHEAD_CM) * MS_STRAIGHT / 30;
}

// The start row comes from the left sensor in column 1  (free = the top row is beside it = robot in the bottom row). 
// At each boundary it probes GAP_HALF / 2 from the middle, its own side first: white there = line covers that row, go to the other. 
// White only on the other side = go to its own row. No white = gap in the middle. A centered line has no white at either probe, 
// so one more probe at Y_LIM finds its end piece to resync. In column 4 it steps into a row and switches rows if the front sensor 
// sees a wall, then crosses the white border and moves onto checkpoint 2.
// THE START ROW IS NOW MEASURED FROM BOTH CORRIDOR WALLS (THE LEFT SENSOR IS THE FALLBACK), AND THE COLUMN 4
// CHECK SEES THE BLOCKED ROW'S END WALL FROM END_WALL_CM AWAY. BEFORE EACH LINE AND BEFORE COLUMN 4,
// travel IS CORRECTED FROM THE CORRIDOR'S END WALLS (resync), SO THE ROBOT STOPS RIGHT BEHIND EACH LINE.
void section2() {
  long travel = 0, y = 0;
  long origin = MS_TO_GREEN;
  if (!strcmp(creep(false, MS_STRAIGHT / 2, travel), "WHITE")) origin = travel;
  fwdMs(MS_CROSS);
  travel += MS_CROSS;
  long probe = origin + MS_STRAIGHT - 2 * MS_PAST;
  fwdMs(probe - travel);
  travel = probe;
  y = sideY(isFree(0) ? GAP_HALF : -GAP_HALF);
  for (byte k = 1; k <= 3; k++) {
    travel = resync(travel, origin);
    long at = origin + (long)k * MS_STRAIGHT;
    long s = y < 0 ? -1 : 1;
    bool own = lineAt(s * GAP_HALF / 2, at, travel, y);
    bool other = !own && lineAt(-s * GAP_HALF / 2, at, travel, y);
    if (own || other || lineAt(-s * Y_LIM, at, travel, y)) origin = travel - (long)k * MS_STRAIGHT;
    long gap = own ? -s * GAP_HALF : other ? s * GAP_HALF : 0;
    strafeTo(gap, y);
    y = gap;
  }
  travel = resync(travel, origin);
  long end = origin + 4L * MS_STRAIGHT - 3 * MS_PAST;
  if (end > travel) fwdMs(end - travel); else backMs(travel - end);
  travel = end;
  if (!y) { strafeTo(GAP_HALF, y); y = GAP_HALF; }
  int f = readCm(1);
  if (f && f < END_WALL_CM) { strafeTo(-y, y); y = -y; }
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
    center();   // THE NEXT COLOR IS READ FROM THE TILE'S MIDDLE, NOT THE WHITE EDGE
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
