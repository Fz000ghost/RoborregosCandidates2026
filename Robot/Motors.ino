// Wheel motors

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

// Wheel signs for a right turn in place.
const int8_t TURN_R[4] = {ROT_R};

// All four wheels spin in place: 1 right, -1 left, 0 stop.
void spin(int8_t dir) {
  for (byte i = 0; i < 4; i++) setMotor(i, dir * TURN_R[i]);
}

// Sets the wheels to a move while holding yawTarget. Past HOLD_DEG off, each wheel gets the sign of move + turn:
// wheels fighting the turn stop, so the robot does half the move plus half the turn. True while correcting.
bool steer(int8_t fl, int8_t fr, int8_t rl, int8_t rr) {
  updateYaw();
  float e = yaw - yawTarget;                                // + = left of target: turn right
  int8_t c = e > HOLD_DEG ? 1 : e < -HOLD_DEG ? -1 : 0;
  int8_t s[4] = {fl, fr, rl, rr};
  for (byte i = 0; i < 4; i++) setMotor(i, constrain(s[i] + c * TURN_R[i], -1, 1));
  return c;
}

// All four wheels for ms milliseconds, then stop and wait MS_SETTLE.
// Holds heading on the way; time spent correcting counts half, since the robot only goes half as far.
void drive(int8_t fl, int8_t fr, int8_t rl, int8_t rr, unsigned ms) {
  unsigned long done = 0, last = micros();
  while (done < ms * 1000UL) {
    bool c = steer(fl, fr, rl, rr);
    unsigned long now = micros();
    done += c ? (now - last) / 2 : now - last;
    last = now;
  }
  setAll(0, 0, 0, 0);
  imuWait(MS_SETTLE);
}

// Turns 90° in place (dir 1 = right, -1 = left) to an exact multiple of 90°, so errors never add up:
// spins until TURN_LEAD_DEG short, coasts, then nudges until within TURN_TOL_DEG. 2 * MS_TURN90 is a safety timeout.
void turn90(int8_t dir) {
  yawTarget -= dir * 90;
  unsigned long t0 = millis();
  spin(dir);
  while (millis() - t0 < 2UL * MS_TURN90) {
    updateYaw();
    if (dir * (yaw - yawTarget) < TURN_LEAD_DEG) break;   // Degrees still to go
  }
  spin(0);
  imuWait(MS_SETTLE);
  for (byte k = 0; k < 5 && fabs(yaw - yawTarget) > TURN_TOL_DEG; k++) {
    spin(yaw > yawTarget ? 1 : -1);
    imuWait(NUDGE_MS);
    spin(0);
    imuWait(MS_SETTLE);
  }
}

// Forward for ms milliseconds.
void fwdMs(unsigned ms)   { drive(FWD, ms); }

// Backward for ms milliseconds.
void backMs(unsigned ms)  { drive(BACK, ms); }

// Strafe right for ms milliseconds.
void rightMs(unsigned ms) { drive(RIGHT, ms); }

// Strafe left for ms milliseconds.
void leftMs(unsigned ms)  { drive(LEFT, ms); }

// Wheel signs per move, in slide() order: 0 FWD, 1 RIGHT, 2 BACK, 3 LEFT.
const int8_t MOVE[4][4] = {{FWD}, {RIGHT}, {BACK}, {LEFT}};
// Sensor facing each move (its reading drops) and the one facing away (it grows). 4 sensors: every move has both.
const byte TOWARD[4] = {1, 2, 3, 0};
const byte AWAY[4]   = {3, 0, 1, 2};

// Moves cm toward dir (0 FWD, 1 RIGHT, 2 BACK, 3 LEFT) without turning, measured by a wall: the sensor facing
// the move must drop by cm, or if it can't see one, the sensor facing away must grow by cm. The rest is timed if:
// no wall in range, robot tilted at the start (ramp), or 3 bad reads in a row (echo lost, or a jump = the beam
// hit the floor/went over a wall on a ramp). Never runs more than 1.5x its timed length.
void moveCm(byte dir, int cm) {
  const int8_t *w = MOVE[dir];
  unsigned long msPerTile = dir % 2 ? MS_STRAFE : MS_STRAIGHT;
  const byte NONE = 4;
  byte s = NONE;
  int8_t sign = 0;
  int d0 = 0;
  if (!tilted()) {
    d0 = readCm(TOWARD[dir]);
    if (d0 >= cm + 3) { s = TOWARD[dir]; sign = -1; }
    else { d0 = readCm(AWAY[dir]); if (d0 && d0 + cm <= US_MAX_CM) { s = AWAY[dir]; sign = 1; } }
  }
  int done = 0;
  bool timedRest = s == NONE;
  if (s != NONE) {
    int last = d0;
    byte bad = 0;
    unsigned long t0 = millis(), cap = cm * msPerTile * 3 / 60;
    while (millis() - t0 < cap) {
      steer(w[0], w[1], w[2], w[3]);
      int d = readCm(s);
      if (!d || abs(d - last) > JUMP_CM) { if (++bad >= 3) { timedRest = true; break; } continue; }
      bad = 0;
      last = d;
      done = sign * (d - d0);
      if (done >= cm) break;
    }
  }
  if (timedRest && done < cm) drive(w[0], w[1], w[2], w[3], (cm - max(done, 0)) * msPerTile / 30);
  else { setAll(0, 0, 0, 0); imuWait(MS_SETTLE); }
}

// One 30 cm tile forward.
void forward30()     { moveCm(0, 30); }

// One 30 cm tile backward.
void back30()        { moveCm(2, 30); }

// One 30 cm tile to the right, without turning.
void right30()       { moveCm(1, 30); }

// One 30 cm tile to the left, without turning.
void left30()        { moveCm(3, 30); }

// Moves to the middle between the wall seen by sensor a and the one seen by the opposite sensor b:
// both walls = average of both, one wall = its center reading, no wall = stays. toA / toB = move index toward a / b.
// Each try moves by the measured error on time (at least NUDGE_MS), then measures again. At most 10 tries.
void centerAxis(byte a, byte b, byte toA, byte toB) {
  unsigned long msPerTile = toA % 2 ? MS_STRAFE : MS_STRAIGHT;
  for (byte k = 0; k < 10; k++) {
    int da = readCm(a), db = readCm(b);
    bool wa = isWall(a, da), wb = isWall(b, db);
    int ea = da - CENTER_CM[a], eb = CENTER_CM[b] - db;   // Each sensor's own "how far from centered"
    int e = wa && wb ? (ea + eb) / 2 : wa ? ea : wb ? eb : 0;   // + = go toward a
    if (abs(e) <= CENTER_TOL_CM) break;
    const int8_t *w = MOVE[e > 0 ? toA : toB];
    drive(w[0], w[1], w[2], w[3], max((unsigned long)NUDGE_MS, abs(e) * msPerTile / 30));
  }
}

// Centers on the tile: sideways (left / right walls), then front-back (front / back walls, not while tilted on a ramp).
void center() {
  centerAxis(0, 2, 3, 1);
  if (!tilted()) centerAxis(1, 3, 0, 2);
}

// Sideways position in cm from the middle between the side walls (+ = right), or fallback if a side doesn't read.
long sideY(long fallback) {
  int l = readCm(0), r = readCm(2);
  return l && r ? (l - r) / 2 : fallback;
}

// Strafes to y cm from the middle between the side walls. yWas = where it should be now, used if a side doesn't read.
void strafeTo(long y, long yWas) {
  long d = y - sideY(yWas);
  if (d > CENTER_TOL_CM) moveCm(1, d);
  else if (d < -CENTER_TOL_CM) moveCm(3, -d);
}

// 90° turn right in place.
void rotateRight90() { turn90(1); }

// 90° turn left in place.
void rotateLeft90()  { turn90(-1); }
