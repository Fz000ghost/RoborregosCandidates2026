// Wheel motors

// Wheel signs per move, in slide() order: 0 FWD, 1 RIGHT, 2 BACK, 3 LEFT.
const int8_t MOVE[4][4] = {{FWD}, {RIGHT}, {BACK}, {LEFT}};
// Sensor facing each move (its reading drops) and the one facing away (it grows). 4 sensors: every move has both.
const byte TOWARD[4] = {1, 2, 3, 0};
const byte AWAY[4]   = {3, 0, 1, 2};

// Software PWM, so wheels can run slower than full speed. Timer2 interrupts 16000 times a second, and every 32 ticks is
// one 500 Hz cycle: each wheel's active pin (a forward, b backward) is high for pwmDuty of the 32 ticks, the other stays
// low. The servo uses Timer1, millis() Timer0, so Timer2 is free. Pins are written straight to their port registers:
// digitalWrite() would be too slow 16000 times a second.
volatile uint8_t *outA[4], *outB[4];
uint8_t bitA[4], bitB[4];
volatile uint8_t *volatile pwmOut[4];
volatile uint8_t pwmBit[4], pwmDuty[4];

ISR(TIMER2_COMPA_vect) {
  static uint8_t t = 0;
  t = (t + 1) & 31;
  for (uint8_t i = 0; i < 4; i++) {
    if (t < pwmDuty[i]) *pwmOut[i] |= pwmBit[i];
    else *pwmOut[i] &= ~pwmBit[i];
  }
}

// Call once in setup(), after the motor pins are outputs.
void pwmBegin() {
  for (byte i = 0; i < 4; i++) {
    outA[i] = portOutputRegister(digitalPinToPort(m[i].a)); bitA[i] = digitalPinToBitMask(m[i].a);
    outB[i] = portOutputRegister(digitalPinToPort(m[i].b)); bitB[i] = digitalPinToBitMask(m[i].b);
    pwmOut[i] = outA[i]; pwmBit[i] = bitA[i]; pwmDuty[i] = 0;
  }
  cli();
  TCCR2A = _BV(WGM21);    // CTC: count to OCR2A, then interrupt
  TCCR2B = _BV(CS21);     // 16 MHz / 8 = 2 MHz
  OCR2A = 124;            // 2 MHz / 125 = 16 kHz
  TIMSK2 = _BV(OCIE2A);
  sei();
}

// One wheel at pct % of full speed: + forward, - backward, 0 stop (dir flips wheels mounted the other way).
// Also tracks which wheels are on, so the gyro knows when the robot is still.
void setSpeed(byte i, int pct) {
  if (pct || bitRead(wheelsOn, i)) lastWheelMs = millis();
  bitWrite(wheelsOn, i, pct != 0);
  pct = constrain(pct * m[i].dir, -100, 100);
  bool fwd = pct >= 0;
  uint8_t sreg = SREG;
  cli();
  *pwmOut[i] &= ~pwmBit[i];   // Pin that was pulsing goes low
  pwmOut[i] = fwd ? outA[i] : outB[i];
  pwmBit[i] = fwd ? bitA[i] : bitB[i];
  if (fwd) *outB[i] &= ~bitB[i]; else *outA[i] &= ~bitA[i];
  pwmDuty[i] = (abs(pct) * 32 + 50) / 100;
  SREG = sreg;
}

// One wheel at full speed. s: +1 forward, -1 backward, 0 stop.
void setMotor(byte i, int8_t s) { setSpeed(i, s * 100); }

// All four wheels at once, in FL, FR, RL, RR order, at full speed.
void setAll(int8_t fl, int8_t fr, int8_t rl, int8_t rr) {
  int8_t s[4] = {fl, fr, rl, rr};
  for (byte i = 0; i < 4; i++) setMotor(i, s[i]);
}

// Wheel signs for a right turn in place.
const int8_t TURN_R[4] = {ROT_R};

// All four wheels spin in place at pct %: dir 1 right, -1 left, 0 stop.
void spinAt(int8_t dir, int pct) {
  for (byte i = 0; i < 4; i++) setSpeed(i, dir * TURN_R[i] * pct);
}

// Spin in place at full speed.
void spin(int8_t dir) { spinAt(dir, 100); }

// Sets the wheels to move w (wheel signs) at pct %, holding yawTarget, plus an optional sideways push lat (wheel signs)
// at latPct %. The turn correction is TURN_KP % per degree off, up to TURN_MAX_PCT. If a wheel would pass 100 %, all
// are scaled down together, so the mix stays the same. Returns how fast the robot goes along w, as a share of full
// speed: time x share = full-speed time covered. (Same as the old rule: correcting at full strength counts half.)
float steerAt(const int8_t w[4], int pct, const int8_t *lat, int latPct) {
  updateYaw();
  float c = constrain((yaw - yawTarget) * TURN_KP, -TURN_MAX_PCT, TURN_MAX_PCT);   // + = left of target: turn right
  float s[4], top = 0;
  for (byte i = 0; i < 4; i++) {
    s[i] = w[i] * pct + c * TURN_R[i] + (lat ? lat[i] * latPct : 0);
    top = max(top, fabs(s[i]));
  }
  float k = top > 100 ? 100 / top : 1, along = 0;
  for (byte i = 0; i < 4; i++) {
    setSpeed(i, lround(s[i] * k));
    along += s[i] * k * w[i];
  }
  return along / 400;
}

// Full speed along a move, holding heading.
float steer(int8_t fl, int8_t fr, int8_t rl, int8_t rr) {
  int8_t w[4] = {fl, fr, rl, rr};
  return steerAt(w, 100, NULL, 0);
}

// Speed for a move that started at t0: climbs from MIN_PCT to pct over RAMP_MS (a full-power start makes mecanum slip).
int rampPct(unsigned long t0, int pct) {
  unsigned long t = millis() - t0;
  return t >= RAMP_MS ? pct : min(pct, (int)(MIN_PCT + (100L - MIN_PCT) * t / RAMP_MS));
}

// Side guard, for moves along `dir`. Each third call it pings one of the two sensors across the move (L/R for straight
// moves, F/B for strafes), taking turns, and stores how hard to push away from it: GUARD_KP % per cm closer than
// GUARD_CM short of centered, up to GUARD_MAX_PCT. Returns the net push in % and sets lat to its wheel signs.
// guardReset() before each move.
int guardPush[4];
byte guardTick;
void guardReset() { memset(guardPush, 0, sizeof(guardPush)); guardTick = 0; }
int guard(byte dir, const int8_t *&lat) {
  byte a = dir % 2, b = a + 2;   // Sensors across the move: 0 L / 2 R, or 1 F / 3 B
  if (++guardTick % 3 == 0) {
    byte s = guardTick % 6 ? a : b;
    int d = readCm(s);
    int close = CENTER_CM[s] - GUARD_CM - d;   // + = too close to that wall
    guardPush[s] = d && close > 0 ? min(close * GUARD_KP, GUARD_MAX_PCT) : 0;
  }
  lat = MOVE[(a + 1) % 4];   // The move away from sensor a (L -> RIGHT, F -> BACK)
  return guardPush[a] - guardPush[b];
}

// Drives w for ms of full-speed time (it runs longer when slower), at up to pct %, then stops and waits MS_SETTLE.
// Holds heading on the way. guardDir < 4 = side guard on, for a move along guardDir.
void driveAt(const int8_t w[4], unsigned long ms, int pct, byte guardDir) {
  unsigned long t0 = millis(), last = micros();
  float done = 0;
  guardReset();
  while (done < ms * 1000.0) {
    const int8_t *lat = NULL;
    int lp = guardDir < 4 ? guard(guardDir, lat) : 0;
    float f = steerAt(w, rampPct(t0, pct), lat, lp);
    unsigned long now = micros();
    done += (now - last) * f;
    last = now;
  }
  setAll(0, 0, 0, 0);
  imuWait(MS_SETTLE);
}

// All four wheels at full speed for ms milliseconds, holding heading, then stop and wait MS_SETTLE.
void drive(int8_t fl, int8_t fr, int8_t rl, int8_t rr, unsigned ms) {
  int8_t w[4] = {fl, fr, rl, rr};
  driveAt(w, ms, 100, 4);
}

// Turns 90° in place (dir 1 = right, -1 = left) to an exact multiple of 90°, so errors never add up:
// full speed, TURN_SLOW_PCT for the last TURN_SLOW_DEG, stops TURN_LEAD_DEG short and coasts, then settleHeading().
// 2 * MS_TURN90 is a safety timeout.
void turn90(int8_t dir) {
  yawTarget -= dir * 90;
  unsigned long t0 = millis();
  while (millis() - t0 < 2UL * MS_TURN90) {
    updateYaw();
    float left = dir * (yaw - yawTarget);   // Degrees still to go
    if (left < TURN_LEAD_DEG) break;
    spinAt(dir, left > TURN_SLOW_DEG ? 100 : TURN_SLOW_PCT);
  }
  spin(0);
  imuWait(MS_SETTLE);
  settleHeading();
}

// Stopped: nudges in place until yaw is within TURN_TOL_DEG of yawTarget, checking after each nudge has settled.
// Nudges start at TURN_SLOW_PCT. One that overshoots (error changes sign) halves the next; one that didn't move the
// robot makes the next 10 % faster. At most 8 nudges.
void settleHeading() {
  unsigned pulse = NUDGE_MS * 2;
  int pct = TURN_SLOW_PCT;
  float e = yaw - yawTarget;
  for (byte k = 0; k < 8 && fabs(e) > TURN_TOL_DEG; k++) {
    spinAt(e > 0 ? 1 : -1, pct);
    imuWait(pulse);
    spin(0);
    imuWait(MS_SETTLE);
    float e2 = yaw - yawTarget;
    if ((e2 > 0) != (e > 0)) pulse = max(pulse / 2, 20u);
    else if (fabs(e2 - e) < 0.3) pct = min(pct + 10, 100);
    e = e2;
  }
}

// Squares up against a wall (Pista A): the front (sensor 1, move 0) or the left panels (sensor 0, move 3). Never the
// right side (LED) or the back (claw). Moves toward the wall at SQUARE_PCT with no heading hold until that sensor reads
// touchCm (1.5 s at most), pushes SQUARE_PUSH_MS more so that side sits flush, and then trusts that: yaw = yawTarget.
// Only if the gyro was within SQUARE_MAX_DEG; more means something wasn't flush (a wall's end, a bump), so the heading
// is kept. Returns true if it reset the heading. Call center() afterwards to move back off the wall.
bool squareOn(byte sensor, byte move, int touchCm) {
  unsigned long t0 = millis();
  for (byte i = 0; i < 4; i++) setSpeed(i, MOVE[move][i] * SQUARE_PCT);
  bool close = false;
  while (millis() - t0 < 1500) {
    int d = readCm(sensor);
    if ((d && d <= touchCm) || (!d && close)) break;   // No echo right at the wall = touching
    close = d && d <= CENTER_CM[sensor];
  }
  imuWait(SQUARE_PUSH_MS);
  setAll(0, 0, 0, 0);
  imuWait(MS_SETTLE);
  float e = yaw - yawTarget;
  if (fabs(e) > SQUARE_MAX_DEG) return false;
  yaw = yawTarget;
  Serial.print(sensor ? "square F: heading fixed by " : "square L: heading fixed by ");
  Serial.println(-e);
  return true;
}

// Forward for ms milliseconds.
void fwdMs(unsigned ms)   { drive(FWD, ms); }

// Backward for ms milliseconds.
void backMs(unsigned ms)  { drive(BACK, ms); }

// Strafe right for ms milliseconds.
void rightMs(unsigned ms) { drive(RIGHT, ms); }

// Strafe left for ms milliseconds.
void leftMs(unsigned ms)  { drive(LEFT, ms); }

// Moves cm toward dir (0 FWD, 1 RIGHT, 2 BACK, 3 LEFT) without turning, measured by a wall when it can: the sensor
// facing the move must drop to the target, or if it can't see a wall, the sensor facing away must grow to it.
// The wall must read steadily at the start (median of 3) and be within MEASURE_MAX_CM: a far echo can come off
// something slanted, which shrinks slower than the robot moves and made it overshoot.
// tile = a whole-tile move: the target becomes the reading of a robot centered on a tile (CENTER_CM + 30 per tile)
// when that is within SNAP_MAX_CM of the plain move, so error left over from earlier moves is fixed instead of kept.
// While moving, the wall's progress is checked against the timed progress. The wall is dropped and the rest is timed if
// 3 reads in a row are bad (echo lost, a jump) or disagree with time by more than TRUST_SLACK_CM + 1/8.
// Also timed: no wall, or tilted at the start (ramp); a whole tile then goes through timedTile(). Never drives more than
// 1.2x the timed length.
// Speed: ramps up at the start, and a measured move slows to APPROACH_PCT for its last APPROACH_CM, so it barely coasts.
// The side guard is on the whole way (guard()). After stopping it measures the wall again and nudges at NUDGE_PCT until
// within MOVE_TOL_CM (at most 3 nudges).
void moveBy(byte dir, int cm, bool tile) {
  const int8_t *w = MOVE[dir], *wBack = MOVE[(dir + 2) % 4];
  unsigned long msPerTile = dir % 2 ? MS_STRAFE : MS_STRAIGHT;
  const byte NONE = 4;
  byte s = NONE;
  int8_t sign = 0;
  int d0 = 0, target = 0;
  if (!tilted()) {
    for (byte k = 0; k < 2 && s == NONE; k++) {
      byte i = k ? AWAY[dir] : TOWARD[dir];
      int8_t sg = k ? 1 : -1;
      int d = readCm3(i);
      if (!d || d > MEASURE_MAX_CM) continue;
      int t = d + sg * cm;
      if (tile) {
        int snap = CENTER_CM[i] + 30 * (int)lround((t - CENTER_CM[i]) / 30.0);
        if (snap >= 3 && abs(snap - t) <= SNAP_MAX_CM) t = snap;
      }
      if (t >= 3 && t <= MEASURE_MAX_CM) { s = i; sign = sg; d0 = d; target = t; }
    }
  }
  if (s == NONE) { if (tile && cm == 30) timedTile(dir); else driveAt(w, cm * msPerTile / 30, 100, dir); return; }

  int plan = sign * (target - d0);   // cm this move should cover
  unsigned long last = micros(), t0 = millis(), capUs = (unsigned long)plan * msPerTile * 40;   // 1.2x timed, in us
  float tUs = 0;   // Full-speed time covered so far
  int prev = d0;
  byte bad = 0, off = 0;
  bool measured = true;
  long est = 0;
  guardReset();
  while (true) {
    const int8_t *lat = NULL;
    int lp = guard(dir, lat);
    float f = steerAt(w, rampPct(t0, sign * (target - prev) > APPROACH_CM ? 100 : APPROACH_PCT), lat, lp);
    int d = readCm(s);
    unsigned long now = micros();
    tUs += (now - last) * f;
    last = now;
    est = tUs * 30 / (msPerTile * 1000);   // cm moved by time
    if (tUs >= capUs) break;
    if (!d || abs(d - prev) > JUMP_CM) { if (++bad >= 3) { measured = false; break; } continue; }
    bad = 0;
    prev = d;
    if (abs(sign * (d - d0) - est) > TRUST_SLACK_CM + est / 8) { if (++off >= 3) { measured = false; break; } continue; }
    off = 0;
    if (sign * (target - d) <= STOP_LEAD_CM) break;
  }
  if (!measured) {
    Serial.println("move: wall not trusted, rest timed");
    if (est < cm) driveAt(w, (cm - est) * msPerTile / 30, 100, dir);
    else { setAll(0, 0, 0, 0); imuWait(MS_SETTLE); }
    return;
  }
  setAll(0, 0, 0, 0);
  imuWait(MS_SETTLE);
  int lastRem = 1000;
  for (byte k = 0; k < 3; k++) {
    int d = readCm3(s);
    int rem = sign * (target - d);   // + = still short, - = went past
    if (!d || abs(rem) <= MOVE_TOL_CM || abs(rem) > 10 || abs(rem) >= lastRem) break;   // > 10 = the wall reads wrong now
    lastRem = abs(rem);
    const int8_t *v = rem > 0 ? w : wBack;
    driveAt(v, max((unsigned long)NUDGE_MS, abs(rem) * msPerTile / 30), NUDGE_PCT, 4);
  }
}

// A whole-tile move with no wall in reach ahead or behind: timed, but anchored on the sides. The side sensors sit at the
// robot's middle, so when one changes between wall and opening (2 reads in a row agree), the robot's middle is at a tile
// border: POST_CM past it when a wall or pillar ends, POST_CM short of it when one starts. The first change while the
// timed progress is 8-22 cm in (near the border) resets the progress to that spot, so the rest of the tile is timed from
// there instead of from the start. Walls end and pillars stand exactly at external corners and in open areas, where
// nothing else can be measured. It pings both side sensors every loop and runs the side guard from those readings.
void timedTile(byte dir) {
  const int8_t *w = MOVE[dir];
  float total = (dir % 2 ? MS_STRAFE : MS_STRAIGHT) * 1000.0, done = 0;   // Full-speed us for one tile, and so far
  byte side[2] = {(byte)(dir % 2), (byte)(dir % 2 + 2)};   // L/R for straight moves, F/B for strafes
  int8_t state[2] = {-1, -1}, cand[2] = {-1, -1};          // -1 = not known yet, 0 = open, 1 = wall
  int push[2] = {0, 0};
  bool anchored = false;
  unsigned long t0 = millis(), last = micros();
  for (byte k = 0; done < total; k ^= 1) {
    float f = steerAt(w, rampPct(t0, 100), MOVE[(side[0] + 1) % 4], push[0] - push[1]);
    int d = readCm(side[k]);
    unsigned long now = micros();
    done += (now - last) * f;
    last = now;
    int close = CENTER_CM[side[k]] - GUARD_CM - d;   // Side guard, as in guard()
    push[k] = d && close > 0 ? min(close * GUARD_KP, GUARD_MAX_PCT) : 0;
    int8_t s = isWall(side[k], d);
    if (s != cand[k]) { cand[k] = s; continue; }    // First read of a change: wait for a second one
    if (state[k] >= 0 && s != state[k] && !anchored) {
      float cm = done * 30 / total, at = 15 + (s ? -POST_CM : POST_CM);
      if (cm >= 8 && cm <= 22) {
        done = at * total / 30;
        anchored = true;
        Serial.print("anchor: timing was ");
        Serial.print(cm - at);
        Serial.println(" cm off");
      }
    }
    state[k] = s;
  }
  setAll(0, 0, 0, 0);
  imuWait(MS_SETTLE);
}

// cm toward dir, without snapping to the tile grid (part-tile moves: grabbing, section 2, strafeTo).
void moveCm(byte dir, int cm) { moveBy(dir, cm, false); }

// One 30 cm tile forward.
void forward30()     { moveBy(0, 30, true); }

// One 30 cm tile backward.
void back30()        { moveBy(2, 30, true); }

// One 30 cm tile to the right, without turning.
void right30()       { moveBy(1, 30, true); }

// One 30 cm tile to the left, without turning.
void left30()        { moveBy(3, 30, true); }

// How far sensor i reads from a tile-centered robot (e, + = the robot is too far from that wall) and whether to use it.
// near = a wall of this tile (isWall). Otherwise a wall one tile beyond counts if it's within GRID_TOL_CM of where it
// would be: the ball (12.9 cm off the grid) and odd echoes are left out. near walls win when there are any.
bool wallError(byte i, int d, int &e, bool &near) {
  near = isWall(i, d);
  e = near ? d - CENTER_CM[i] : d - CENTER_CM[i] - 30;
  return near || (d && abs(e) <= GRID_TOL_CM);
}

// Moves to the middle between the walls seen by sensor a and by the opposite sensor b: both = average, one = its own
// center reading, none = stays. Walls of this tile are used first; with none on this axis, walls one tile beyond are
// used, so an axis with no wall beside it (most of section 1's ring around the ball, open maze areas) is still fixed.
// toA / toB = move index toward a / b. Measures with the median of 3 pings per sensor, then nudges, at most 6 tries:
// - Each nudge covers at most CENTER_STEP_CM, at NUDGE_PCT so it barely coasts. Full-speed nudges overshot, and the
//   robot chased its own overshoot back and forth into the walls.
// - It stops as soon as the error isn't smaller than the try before: the nudges aren't helping (a bad echo, an odd
//   wall), so stopping is safer than chasing it.
// Serial: "center L/R e=3" per nudge (axis by the sensor pair, e in cm, + = toward the first one).
void centerAxis(byte a, byte b, byte toA, byte toB) {
  unsigned long msPerTile = toA % 2 ? MS_STRAFE : MS_STRAIGHT;
  int prev = 1000;
  for (byte k = 0; k < 6; k++) {
    int ea, eb;
    bool na, nb;
    bool wa = wallError(a, readCm3(a), ea, na), wb = wallError(b, readCm3(b), eb, nb);
    eb = -eb;                             // + = go toward a, for both
    if (na || nb) { wa = na; wb = nb; }   // A wall of this tile beats any farther one
    int e = wa && wb ? (ea + eb) / 2 : wa ? ea : wb ? eb : 0;
    if (abs(e) <= CENTER_TOL_CM || abs(e) >= prev) break;
    prev = abs(e);
    Serial.print(a ? "center F/B e=" : "center L/R e=");
    Serial.println(e);
    int step = constrain(e, -CENTER_STEP_CM, CENTER_STEP_CM);
    const int8_t *w = MOVE[step > 0 ? toA : toB];
    driveAt(w, max((unsigned long)NUDGE_MS, abs(step) * msPerTile / 30), NUDGE_PCT, 4);
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
