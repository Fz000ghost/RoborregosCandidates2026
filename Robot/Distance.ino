// DISTANCE SENSORS

// CM SEEN BY ULTRASONIC i (0 L, 1 F, 2 R, 3 B); 0 = NO ECHO OR FARTHER THAN US_MAX_CM.
// UNDER ~3 CM THE NUMBER IS ROUGH, BUT AN ECHO STILL MEANS "VERY CLOSE", SO IT IS KEPT (AT LEAST 1).
// WAITS 15 MS SINCE THE LAST PING (TRACKING THE GYRO) SO AN OLD ECHO ISN'T HEARD AS A NEW ONE.
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

// TRUE IF READING d OF SENSOR i IS A WALL OF THIS TILE.
bool isWall(byte i, int d) {
  return d && d < CENTER_CM[i] + WALL_MARGIN_CM;
}

// Reads the three sensors (0 L, 1 F, 2 R) into dist[]: 1 = wall seen.
// NOW FOUR: 3 = BACK.
void readSensors() {
  for (byte i = 0; i < 4; i++) dist[i] = isWall(i, readCm(i));
}
