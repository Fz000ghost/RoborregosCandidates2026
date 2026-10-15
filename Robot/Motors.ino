// WHEEL MOTORS

// One wheel. s: +1 forward, -1 backward, 0 stop (dir flips wheels mounted the other way).
void setMotor(byte i, int8_t s) {
  s *= m[i].dir;
  digitalWrite(m[i].a, s > 0);
  digitalWrite(m[i].b, s < 0);
}

// All four wheels at once, in FL, FR, RL, RR order.
void setAll(int8_t fl, int8_t fr, int8_t rl, int8_t rr) {
  int8_t s[4] = {fl, fr, rl, rr};
  for (byte i = 0; i < 4; i++) setMotor(i, s[i]);
}

// All four wheels for ms milliseconds, then stop and wait MS_SETTLE.
void drive(int8_t fl, int8_t fr, int8_t rl, int8_t rr, unsigned ms) {
  setAll(fl, fr, rl, rr);
  delay(ms);
  setAll(0, 0, 0, 0);
  delay(MS_SETTLE);
}

// Forward for ms milliseconds.
void fwdMs(unsigned ms)   { drive(FWD, ms); }

// Backward for ms milliseconds.
void backMs(unsigned ms)  { drive(BACK, ms); }

// Strafe right for ms milliseconds.
void rightMs(unsigned ms) { drive(RIGHT, ms); }

// Strafe left for ms milliseconds.
void leftMs(unsigned ms)  { drive(LEFT, ms); }

// One 30 cm tile forward.
void forward30()     { fwdMs(MS_STRAIGHT); }

// One 30 cm tile backward.
void back30()        { backMs(MS_STRAIGHT); }

// One 30 cm tile to the right, without turning.
void right30()       { rightMs(MS_STRAFE); }

// One 30 cm tile to the left, without turning.
void left30()        { leftMs(MS_STRAFE); }

// 90° turn right in place.
void rotateRight90() { drive(1, 1, -1, 1, MS_TURN90); }

// 90° turn left in place.
void rotateLeft90()  { drive(-1, -1, 1, -1, MS_TURN90); }
