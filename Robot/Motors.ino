// WHEEL MOTORS

void setMotor(byte i, int8_t s) {
  s *= m[i].dir;
  digitalWrite(m[i].a, s > 0);
  digitalWrite(m[i].b, s < 0);   // both LOW = brake (EN is held HIGH by the jumper)
}

void drive(int8_t fl, int8_t fr, int8_t rl, int8_t rr, unsigned ms) {
  int8_t s[4] = {fl, fr, rl, rr};
  for (byte i = 0; i < 4; i++) setMotor(i, s[i]);
  delay(ms);
  for (byte i = 0; i < 4; i++) setMotor(i, 0);
}

void forward30()     { drive( 1,  -1,  1,  1, MS_STRAIGHT); }
void back30()        { drive(-1, 1, -1, -1, MS_STRAIGHT); }
void right30()       { drive( 1, 1, 1,  -1, MS_STRAFE); }
void left30()        { drive(-1,  -1,  -1, 1, MS_STRAFE); }
void rotateRight90()  { drive(-1,  -1, 1,  -1, MS_TURN90); }
void rotateLeft90() { drive( 1, 1,  -1, 1, MS_TURN90); }