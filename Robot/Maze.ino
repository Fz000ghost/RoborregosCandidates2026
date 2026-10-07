// MAZE

void scan() {
  readSensors();
  for (byte i = 0; i < 3; i++) if (!dist[i]) freeSide[(heading + 3 + i) % 4] = true;   // L, F, R
}

void turnTo(byte d) {
  byte diff = (d + 4 - heading) % 4;
  if (diff == 0) return;
  if (diff == 3) rotateLeft90();
  else for (byte i = 0; i < diff; i++) rotateRight90();
  heading = d;
  delay(MS_SETTLE);
}

void step(byte d) {
  turnTo(d);
  forward30();
  delay(MS_SETTLE);
  posX += DX[d];
  posY += DY[d];
}

bool isRed() {
  for (byte i = 0; i < 3; i++) {
    readColor();
    if (strcmp(colorName(), "RED") != 0) return false;
  }
  return true;
}


byte nextSide() {
  const byte ORDER[4] = {0, 1, 3, 2};
  for (byte k = 0; k < 4; k++) {
    byte d = (heading + ORDER[k]) % 4;
    byte nx = posX + DX[d], ny = posY + DY[d];
    if (freeSide[d] && nx < GRID && ny < GRID && !visited[nx][ny]) return d;
  }
  return NO_SIDE;
}

void explore() {
  visited[posX][posY] = true;
  bool startTile = true;

  while (!isRed()) {
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
    else { Serial.println("Explored every tile, no red found"); return; }

    hist[histLen++] = d;
    step(d);
    visited[posX][posY] = true;
  }

  Serial.println("Red tile reached, returning");
  while (histLen) step((hist[--histLen] + 2) % 4);
  Serial.println("Back at start");
}
