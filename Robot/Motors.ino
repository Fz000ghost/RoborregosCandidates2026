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

// WHEEL SIGNS FOR A RIGHT TURN IN PLACE.
const int8_t TURN_R[4] = {ROT_R};

// ALL FOUR WHEELS SPIN IN PLACE: 1 RIGHT, -1 LEFT, 0 STOP.
void spin(int8_t dir) {
  for (byte i = 0; i < 4; i++) setMotor(i, dir * TURN_R[i]);
}

// SETS THE WHEELS TO A MOVE WHILE HOLDING yawTarget. PAST HOLD_DEG OFF, EACH WHEEL GETS THE SIGN OF MOVE + TURN:
// WHEELS FIGHTING THE TURN STOP, SO THE ROBOT DOES HALF THE MOVE PLUS HALF THE TURN. TRUE WHILE CORRECTING.
bool steer(int8_t fl, int8_t fr, int8_t rl, int8_t rr) {
  updateYaw();
  float e = yaw - yawTarget;                                // + = LEFT OF TARGET: TURN RIGHT
  int8_t c = e > HOLD_DEG ? 1 : e < -HOLD_DEG ? -1 : 0;
  int8_t s[4] = {fl, fr, rl, rr};
  for (byte i = 0; i < 4; i++) setMotor(i, constrain(s[i] + c * TURN_R[i], -1, 1));
  return c;
}

// All four wheels for ms milliseconds, then stop and wait MS_SETTLE.
// HOLDS HEADING ON THE WAY; TIME SPENT CORRECTING COUNTS HALF, SINCE THE ROBOT ONLY GOES HALF AS FAR.
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

// TURNS 90° IN PLACE (dir 1 = RIGHT, -1 = LEFT) TO AN EXACT MULTIPLE OF 90°, SO ERRORS NEVER ADD UP:
// SPINS UNTIL TURN_LEAD_DEG SHORT, COASTS, THEN NUDGES UNTIL WITHIN TURN_TOL_DEG. 2 * MS_TURN90 IS A SAFETY TIMEOUT.
void turn90(int8_t dir) {
  yawTarget -= dir * 90;
  unsigned long t0 = millis();
  spin(dir);
  while (millis() - t0 < 2UL * MS_TURN90) {
    updateYaw();
    if (dir * (yaw - yawTarget) < TURN_LEAD_DEG) break;   // DEGREES STILL TO GO
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

// WHEEL SIGNS PER MOVE, IN slide() ORDER: 0 FWD, 1 RIGHT, 2 BACK, 3 LEFT.
const int8_t MOVE[4][4] = {{FWD}, {RIGHT}, {BACK}, {LEFT}};
// SENSOR FACING EACH MOVE (ITS READING DROPS) AND THE ONE FACING AWAY (IT GROWS). 4 SENSORS: EVERY MOVE HAS BOTH.
const byte TOWARD[4] = {1, 2, 3, 0};
const byte AWAY[4]   = {3, 0, 1, 2};

// MOVES cm TOWARD dir (0 FWD, 1 RIGHT, 2 BACK, 3 LEFT) WITHOUT TURNING, MEASURED BY A WALL: THE SENSOR FACING
// THE MOVE MUST DROP BY cm, OR IF IT CAN'T SEE ONE, THE SENSOR FACING AWAY MUST GROW BY cm. THE REST IS TIMED IF:
// NO WALL IN RANGE, ROBOT TILTED AT THE START (RAMP), OR 3 BAD READS IN A ROW (ECHO LOST, OR A JUMP = THE BEAM
// HIT THE FLOOR/WENT OVER A WALL ON A RAMP). NEVER RUNS MORE THAN 1.5x ITS TIMED LENGTH.
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

// MOVES TO THE MIDDLE BETWEEN THE WALL SEEN BY SENSOR a AND THE ONE SEEN BY THE OPPOSITE SENSOR b:
// BOTH WALLS = AVERAGE OF BOTH, ONE WALL = ITS CENTER READING, NO WALL = STAYS. toA / toB = MOVE INDEX TOWARD a / b.
// EACH TRY MOVES BY THE MEASURED ERROR ON TIME (AT LEAST NUDGE_MS), THEN MEASURES AGAIN. AT MOST 10 TRIES.
void centerAxis(byte a, byte b, byte toA, byte toB) {
  unsigned long msPerTile = toA % 2 ? MS_STRAFE : MS_STRAIGHT;
  for (byte k = 0; k < 10; k++) {
    int da = readCm(a), db = readCm(b);
    bool wa = isWall(a, da), wb = isWall(b, db);
    int ea = da - CENTER_CM[a], eb = CENTER_CM[b] - db;   // EACH SENSOR'S OWN "HOW FAR FROM CENTERED"
    int e = wa && wb ? (ea + eb) / 2 : wa ? ea : wb ? eb : 0;   // + = GO TOWARD a
    if (abs(e) <= CENTER_TOL_CM) break;
    const int8_t *w = MOVE[e > 0 ? toA : toB];
    drive(w[0], w[1], w[2], w[3], max((unsigned long)NUDGE_MS, abs(e) * msPerTile / 30));
  }
}

// CENTERS ON THE TILE: SIDEWAYS (LEFT / RIGHT WALLS), THEN FRONT-BACK (FRONT / BACK WALLS, NOT WHILE TILTED ON A RAMP).
void center() {
  centerAxis(0, 2, 3, 1);
  if (!tilted()) centerAxis(1, 3, 0, 2);
}

// SIDEWAYS POSITION IN CM FROM THE MIDDLE BETWEEN THE SIDE WALLS (+ = RIGHT), OR fallback IF A SIDE DOESN'T READ.
long sideY(long fallback) {
  int l = readCm(0), r = readCm(2);
  return l && r ? (l - r) / 2 : fallback;
}

// STRAFES TO y CM FROM THE MIDDLE BETWEEN THE SIDE WALLS. yWas = WHERE IT SHOULD BE NOW, USED IF A SIDE DOESN'T READ.
void strafeTo(long y, long yWas) {
  long d = y - sideY(yWas);
  if (d > CENTER_TOL_CM) moveCm(1, d);
  else if (d < -CENTER_TOL_CM) moveCm(3, -d);
}

// 90° turn right in place.
void rotateRight90() { turn90(1); }

// 90° turn left in place.
void rotateLeft90()  { turn90(-1); }
