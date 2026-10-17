// Levels (Pista B)

// True if sensor i (0 L, 1 F, 2 R) sees no wall on 3 reads in a row.
bool isFree(byte i) {
  for (byte k = 0; k < 3; k++) {
    readSensors();
    if (dist[i]) return false;
    imuWait(20);
  }
  return true;
}

// Tail already faces the ball tile: open the claw, back in, close it, drive back out.
// The claw starts closed, so it opens here first.
void grab() {
  setClaw(true);
  moveCm(2, GRAB1_CM);   // Measured with the front sensor, which faces away from the ball
  moveCm(3, GRAB2_CM);
  setClaw(false);
  moveCm(0, GRAB1_CM);
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
// 3 is now the back sensor, so every side can be checked.
byte sensorTo(byte d) { return ((d + 2) % 4 + 5 - heading) % 4; }

// Starts on INICIO facing into the 3x3, and slides onto its ring tile. Checks the ball tile's visible sides.
// If all three are closed, the opening is the one behind. Goes to the opening, turns its tail toward
// the ball and grabs it, then faces east, slides to the E tile and drives onto checkpoint 1.
// It centers on INICIO first.
// With the back sensor all four sides are checked directly. All four closed means a misread: it checks them all
// once more, then tries the east side.
void section1() {
  posX = START_X; posY = START_Y; heading = START_HEADING;
  center();   // Placed by hand: start from the tile's center, or every move after carries the error
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
    else if (!strcmp(c, "Light Green")) left30();
    else { unclear++; continue; }
    center();   // The next color is read from the tile's middle, not the white edge
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
