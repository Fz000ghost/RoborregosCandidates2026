// TESTS

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
void testDistance() {
  readSensors();
  Serial.print("L F R: ");
  for (byte i = 0; i < 3; i++) { Serial.print(dist[i]); Serial.print(" "); }

  delay(200);
}

// Prints the raw R G B readings and the matched color to Serial, and shows it on the LED.
void testColor() {
  readColor();
  Serial.print("R G B: ");
  for (byte i = 0; i < 3; i++) { Serial.print(rgb[i]); Serial.print(" "); }
  Serial.print("-> ");
  Serial.println(colorName());

  displayColor(colorName());

  delay(200);
}
