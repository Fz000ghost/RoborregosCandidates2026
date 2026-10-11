// MAZE (Pista A)

// Marks which sides (L, F, R) are open in freeSide[], by majority of 3 reads: one bad read
// must not hide an opening or invent one. (isFree() is wrong here: it says "wall" on any single wall read.)
void scan() {
  byte frees[3] = {0, 0, 0};
  for (byte k = 0; k < 3; k++) {
    readSensors();
    for (byte i = 0; i < 3; i++) frees[i] += !dist[i];
    delay(20);
  }
  for (byte i = 0; i < 3; i++) if (frees[i] >= 2) freeSide[(heading + 3 + i) % 4] = true;
}

// Turns in place to face direction d (0 N, 1 E, 2 S, 3 W), the short way.
void turnTo(byte d) {
  byte diff = (d + 4 - heading) % 4;
  if (diff == 0) return;
  if (diff == 3) rotateLeft90();
  else for (byte i = 0; i < diff; i++) rotateRight90();
  heading = d;
}

// Turns to face d, drives one tile and updates the position.
void step(byte d) {
  turnTo(d);
  forward30();
  posX += DX[d];
  posY += DY[d];
}

// One tile toward d without turning (mecanum strafe), and updates the position.
void slide(byte d) {
  byte r = (d + 4 - heading) % 4;
  if (r == 0) forward30(); else if (r == 1) right30(); else if (r == 2) back30(); else left30();
  posX += DX[d];
  posY += DY[d];
}

// True if the tile is red, with a stricter vote than other colors: a flicker on orange or pink must not end the search.
bool isRed() {
  return !strcmp(tileColor(80), "RED");
}

// First open, unvisited side in the order front, right, left, back; NO_SIDE if there is none.
byte nextSide() {
  const byte ORDER[4] = {0, 1, 3, 2};
  for (byte k = 0; k < 4; k++) {
    byte d = (heading + ORDER[k]) % 4;
    byte nx = posX + DX[d], ny = posY + DY[d];
    if (freeSide[d] && nx < GRID && ny < GRID && !visited[nx][ny]) return d;
  }
  return NO_SIDE;
}

// Depth-first search: shows each tile's color, goes to the next open unvisited tile,
// and backtracks along path[] at dead ends. On the start tile it also turns around to check behind.
// Stops on red, then drives the whole route in hist[] backwards to return to the start.
void explore() {
  visited[posX][posY] = true;
  bool startTile = true;

  while (true) {
    const char *c = tileColor(70);
    displayColor(c);
    if (!strcmp(c, "RED") && isRed()) break;

    memset(freeSide, 0, sizeof(freeSide));
    scan();
    byte d = nextSide();
    if (d == NO_SIDE && startTile) {
      turnTo((heading + 2) % 4);
      scan();
      d = nextSide();
    }
    startTile = false;
    if (d != NO_SIDE) path[pathLen++] = d;
    else if (pathLen) d = (path[--pathLen] + 2) % 4;
    else { return; }

    hist[histLen++] = d;
    step(d);
    visited[posX][posY] = true;
  }

  while (histLen) step((hist[--histLen] + 2) % 4);
}
