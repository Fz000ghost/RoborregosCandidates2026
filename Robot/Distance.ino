// Distance sensors

// cm seen by ultrasonic i (0 L, 1 F, 2 R, 3 B); 0 = no echo or farther than US_MAX_CM.
// Under ~3 cm the number is rough, but an echo still means "very close", so it is kept (at least 1).
// Waits 15 ms since the last ping (tracking the gyro) so an old echo isn't heard as a new one.
int readCm(byte i) {
  static unsigned long lastPing = 0;
  while (millis() - lastPing < 15) updateYaw();
  digitalWrite(US_TRIG[i], LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG[i], LOW);
  unsigned long us = pulseIn(US_ECHO[i], HIGH, US_TIMEOUT_US);
  lastPing = millis();
  int cm = us / 58;
  return !us || cm > US_MAX_CM ? 0 : max(cm, 1);
}

// True if reading d of sensor i is a wall of this tile.
bool isWall(byte i, int d) {
  return d && d < CENTER_CM[i] + WALL_MARGIN_CM;
}

// Reads the three sensors (0 L, 1 F, 2 R) into dist[]: 1 = wall seen.
// Now four: 3 = back.
void readSensors() {
  for (byte i = 0; i < 4; i++) dist[i] = isWall(i, readCm(i));
}
