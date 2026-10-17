// Maze (Pista A)

// Marks which sides (L, F, R) are open in freeSide[], by majority of 3 reads: one bad read must not hide an opening or invent one.
// Now also the back (i = 3): (heading + 3 + 3) % 4 is the side behind, so the same formula works.
void scan() {
  byte frees[4] = {0, 0, 0, 0};
  for (byte k = 0; k < 3; k++) {
    readSensors();
    for (byte i = 0; i < 4; i++) frees[i] += !dist[i];
    imuWait(20);
  }
  for (byte i = 0; i < 4; i++) if (frees[i] >= 2) freeSide[(heading + 3 + i) % 4] = true;
}

// Turns in place to face direction d (0 N, 1 E, 2 S, 3 W), the short way.
void turnTo(byte d) {
  byte diff = (d + 4 - heading) % 4;
  if (diff == 0) return;
  if (diff == 3) rotateLeft90();
  else for (byte i = 0; i < diff; i++) rotateRight90();
  heading = d;
}

// Tiles since the last heading reset on a wall. Starts full so the first left wall squares.
byte sinceSquare = SQUARE_EVERY;

// Turns to face d, drives one tile and updates the position. Then resets the heading on a wall of the new tile: the
// one ahead every time, else the left one if the last reset was SQUARE_EVERY or more tiles ago (each costs ~1.5 s).
// Without a wall, or if the reset was refused, it squares up on the gyro.
void step(byte d) {
  turnTo(d);
  forward30();
  center();   // Measured moves land close; this fixes what's left against the walls
  sinceSquare++;
  bool pushed = false, reset = false;
  if (!tilted()) {
    if (isWall(1, readCm3(1))) { pushed = true; reset = squareOn(1, 0, SQUARE_TOUCH_CM); }
    else if (sinceSquare >= SQUARE_EVERY && isWall(0, readCm3(0))) { pushed = true; reset = squareOn(0, 3, LEFT_TOUCH_CM); }
  }
  if (reset) sinceSquare = 0;
  if (pushed) center();   // Back off the wall it pushed against
  if (!reset) settleHeading();
  posX += DX[d];
  posY += DY[d];
}

// One tile toward d without turning, and updates the position.
void slide(byte d) {
  byte r = (d + 4 - heading) % 4;
  if (r == 0) forward30(); else if (r == 1) right30(); else if (r == 2) back30(); else left30();
  center();
  settleHeading();
  posX += DX[d];
  posY += DY[d];
}

// True if the tile is red, with a stricter vote than other colors.
bool isRed() {
  return !strcmp(tileColor(80), "RED");
}

// Tile color for the maze. Unclear ("?") or a red that fails the stricter red vote is read again, re-centering first,
// up to 3 tries: leaving the red tile on one bad vote would lose it for good, since visited tiles are never re-entered.
// Returns "RED" only when confirmed.
const char *readTile() {
  const char *c = "?";
  for (byte k = 0; k < 3; k++) {
    if (k) center();
    c = tileColor(70);
    if (!strcmp(c, "RED")) { if (isRed()) return c; c = "?"; }
    if (strcmp(c, "?")) return c;
  }
  return c;
}

// Turns to face d and checks the front sensor (median of 3) agrees there's no wall. scan() reads sides with the side
// sensors, from wherever the robot ended up; a wrong "open" would send it into a wall and add a tile that isn't there.
bool openAhead(byte d) {
  turnTo(d);
  if (!isWall(1, readCm3(1))) return true;
  Serial.println("look: wall ahead after all");
  return false;
}

// First open, unvisited side; NO_SIDE if there is none. Red is at the far end of the maze from the start, and the robot
// starts facing into the maze, so the order is: grid north (where it faced on the start tile) first, east/west next,
// south last. Among equals, straight beats a turn (each costs ~2 s), then left beats right. A maze built to waste time
// on one fixed preference (always right first) can't trap it.
byte nextSide() {
  const byte GOAL_RANK[4] = {0, 1, 2, 1};   // N, E, S, W
  byte best = NO_SIDE, bestKey = 255;
  for (byte d = 0; d < 4; d++) {
    byte nx = posX + DX[d], ny = posY + DY[d];
    if (!freeSide[d] || nx >= GRID || ny >= GRID || visited[nx][ny]) continue;
    byte r = (d + 4 - heading) % 4;   // 0 straight, 1 right, 2 back, 3 left
    byte key = GOAL_RANK[d] * 9 + (r == 2 ? 2 : r ? 1 : 0) * 3 + (r == 1);
    if (key < bestKey) { bestKey = key; best = d; }
  }
  return best;
}

// Depth-first search: shows each tile's color, goes to the next open unvisited tile after confirming it with
// openAhead(), and backtracks along path[] at dead ends. On the start tile it also turns around to check behind.
// Stops on red, then drives the whole route in hist[] backwards to return to the start.
// The return bonus requires the same tiles in reverse order, dead ends included: don't shorten it to path[].
// The back sensor now checks behind, so the start tile no longer turns around.
void explore() {
  visited[posX][posY] = true;
  center();   // Every scan() happens centered, so the wall rule holds (also on the start tile)

  while (true) {
    const char *c = readTile();
    displayColor(c);
    if (!strcmp(c, "RED")) break;

    memset(freeSide, 0, sizeof(freeSide));
    scan();
    byte d;
    while ((d = nextSide()) != NO_SIDE && !openAhead(d)) freeSide[d] = false;   // New tiles only: backtracking is known
    if (d != NO_SIDE) path[pathLen++] = d;
    else if (pathLen) d = (path[--pathLen] + 2) % 4;
    else { return; }

    hist[histLen++] = d;
    step(d);
    visited[posX][posY] = true;
  }

  while (histLen) step((hist[--histLen] + 2) % 4);
}
