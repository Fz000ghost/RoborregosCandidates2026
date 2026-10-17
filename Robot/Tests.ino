// Tests

// Spins FL, FR, RL, RR one at a time: 1 s forward, 1 s stopped. Each wheel should spin forward.
void testWheels() {
  for (byte i = 0; i < 4; i++) {
    setMotor(i, 1); delay(1000);
    setMotor(i, 0); delay(1000);
  }
}

// One tile forward, back, right and left, then a 90° turn right and left, 1 s apart.
void testMoves() {
  forward30();     delay(1000);
  back30();        delay(1000);
  right30();       delay(1000);
  left30();        delay(1000);
  rotateRight90(); delay(1000);
  rotateLeft90();  delay(1000);
}

// Same moves in reverse order: back, forward, left, right, turn left, turn right, 1 s apart.
void testMovesReverse() {
  back30();        delay(1000);
  forward30();     delay(1000);
  left30();        delay(1000);
  right30();       delay(1000);
  rotateLeft90();  delay(1000);
  rotateRight90(); delay(1000);
}

// Opens the claw if closed, closes it if open, then waits 1 s.
void testClaw() {
  toggleClaw();    delay(1000);
}

// Prints the three distance sensors to Serial as "L F R: 0 1 0" (1 = wall).
// Now four sensors, cm first, then the wall flags: "L F R B: 5 34 6 5 cm  walls: 1 0 1 1".
// Use it to set SIDE_CENTER_CM / FRONT_CENTER_CM / BACK_CENTER_CM.
void testDistance() {
  int cm[4];
  for (byte i = 0; i < 4; i++) cm[i] = readCm(i);
  Serial.print("L F R B: ");
  for (byte i = 0; i < 4; i++) { Serial.print(cm[i]); Serial.print(" "); }
  Serial.print("cm  walls: ");
  for (byte i = 0; i < 4; i++) { Serial.print(isWall(i, cm[i])); Serial.print(" "); }
  Serial.println();

  delay(200);
}

// Prints the raw R G B readings and the matched color to Serial, and shows it on the LED.
// Raw R G B now print on the color board's own Serial (115200); this prints the color it sends and shows it,
// so it also tests the I2C link ("?" = the board isn't answering).
void testColor() {
  const char *c = getColor();
  Serial.print("Color board: ");
  Serial.println(c);

  displayColor(c);

  delay(200);
}

// Call from loop(). Turn the robot by hand: LED green = left of where it started, red = right, off = within 2°.
// If a left turn shows red, set GYRO_SIGN = -1.
void testImu() {
  updateYaw();
  displayColor(yaw > 2 ? "GREEN" : yaw < -2 ? "RED" : "");
}

// Call from loop(). Centers on the tile, then waits 5 s: move the robot off-center by hand meanwhile.
void testCenter() {
  center();
  delay(5000);
}

// Call from loop(). LED white while the robot is tilted (ramp slope), off when flat. Try it on the ramp, stopped.
void testTilt() {
  displayColor(tilted() ? "WHITE" : "");
  delay(100);
}
