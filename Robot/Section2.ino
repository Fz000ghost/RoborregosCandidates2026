// Section 2 (Pista B): 2x4 tiles, a line on each of the 3 inner boundaries, each with one 30 cm gap.
// Only driving forward across a boundary can cross a line; strafing with the wheels behind it never can.
// So at each line the robot first maps it (probes across the corridor), then drives forward only through the middle
// of a measured gap. Position along the corridor is re-anchored every time the sensor touches a line.

// Floor the robot may drive on. Anything else is a line. "WHITE" for the test board (black lines), "GREEN" on the track.
const char *SAFE_COLOR = "GREEN";

const int PROBE_START_CM = 5;   // Probes start with the sensor this far before a line's center
const int PROBE_REACH_CM = 6;   // A probe goes at most this far: sensor to the line's far edge, tires (1 cm behind) to its center
const int SWEEP_STEP_CM = 4;    // Sideways spacing of the probes
const int GAP_CM = 30;          // The gap in every line, from the rules
const int CP2_WALL_CM = 15;     // In column 4 the row without checkpoint 2 has its end wall closer than this
const unsigned DUTY_MS = 20;    // Slow creep: motors on DUTY_MS, off DUTY_MS
const unsigned SLOW_SETTLE_MS = 60;
const int SLOW_MM_PER_S = 70;   // Speed of the slow creep. Calibrate with testSlow() before anything else
const long NO_LINE = 998, NO_GAP = 999;

// Drive forward at full speed until a raw read says WHITE (or RED if stop is 1), or until travel reaches "until".
// travel = forward ms so far. Returns the color that stopped it, or "" if it reached "until".
// Time counts by steer()'s share of full speed, like drive(), so travel matches the real distance.
const char *creep(byte stop, long until, long &travel) {
  const char *c = "";
  unsigned long last = micros();
  float done = 0;   // Full-speed microseconds
  while (travel + (long)(done / 1000) < until) {
    float f = steer(FWD);
    unsigned long now = micros();
    done += (now - last) * f;
    last = now;
    const char *r = getColor();
    if (!strcmp(r, "WHITE") || (stop && !strcmp(r, "RED"))) { c = r; break; }
  }
  setAll(0, 0, 0, 0);
  travel += done / 1000;
  imuWait(MS_SETTLE);
  return c;
}

// Creeps forward (dir 1) or back (-1) at about half speed (motors on/off every DUTY_MS) for up to mm, holding heading,
// and adds the mm moved (time x SLOW_MM_PER_S) to moved. With stopOff it stops on the first read off the safe floor and
// returns true. The color board is asked every 5 ms: asking nonstop would disturb its readings.
bool slow(int8_t dir, int mm, bool stopOff, int &moved) {
  unsigned long t0 = millis(), limit = (unsigned long)max(mm, 0) * 1000 / SLOW_MM_PER_S, lastAsk = 0;
  bool off = false;
  while (millis() - t0 < limit) {
    if ((millis() - t0) / DUTY_MS % 2) { setAll(0, 0, 0, 0); updateYaw(); }
    else if (dir > 0) steer(FWD);
    else steer(BACK);
    if (stopOff && millis() - lastAsk >= 5) {
      lastAsk = millis();
      if (strcmp(getColor(), SAFE_COLOR)) { off = true; break; }
    }
  }
  moved += (millis() - t0) * SLOW_MM_PER_S / 1000;
  setAll(0, 0, 0, 0);
  imuWait(SLOW_SETTLE_MS);
  return off;
}

// Confirms a stop with 5 fresh reads, robot still (~0.2 s). Off the safe floor if 2 or more aren't SAFE_COLOR
// (a mixed edge counts as a line) or if the color board stops answering.
bool offSafe() {
  getColor();
  byte n = colorCount, got = 0, off = 0;
  unsigned long t0 = millis();
  while (got < 5 && millis() - t0 < 500) {
    imuWait(5);
    const char *c = getColor();
    if (colorCount == n) continue;
    n = colorCount;
    got++;
    if (strcmp(c, SAFE_COLOR)) off++;
  }
  return got < 5 || off >= 2;
}

// From the probe spot, creeps forward until the sensor leaves the safe floor or has gone PROBE_REACH_CM, then backs up.
// True = line here. After a line it backs up to PROBE_START_CM before the line's center: the line re-anchors the position.
bool probe() {
  const int REACH = PROBE_REACH_CM * 10;
  int moved = 0, back = 0;
  bool line = false;
  while (!line && moved < REACH && slow(1, REACH - moved, true, moved)) line = offSafe();
  slow(-1, line ? (PROBE_START_CM - 1) * 10 : moved, false, back);
  return line;
}

// Sideways position (cm from the middle, + = right) of probe i: every SWEEP_STEP_CM from -Y_LIM, the last one at +Y_LIM.
const byte N_PROBES = (2 * Y_LIM + SWEEP_STEP_CM - 1) / SWEEP_STEP_CM + 1;
long probeY(byte i) { return min(-Y_LIM + (long)i * SWEEP_STEP_CM, Y_LIM); }

// y of the gap's middle from the probes (line[i] at probeY(i)). NO_LINE if no probe found the line; NO_GAP unless they
// show exactly one gap of about GAP_CM. An edge the sensor can't reach (gap against a wall) is the other edge +- GAP_CM.
long gapMiddle(const bool line[]) {
  byte a = N_PROBES, b = 0, runs = 0;
  for (byte i = 0; i < N_PROBES; i++)
    if (!line[i]) { if (i == 0 || line[i - 1]) runs++; if (a == N_PROBES) a = i; b = i; }
  if (runs != 1) return NO_GAP;
  bool seeLo = a > 0, seeHi = b < N_PROBES - 1;
  if (!seeLo && !seeHi) return NO_LINE;
  long lo = seeLo ? (probeY(a - 1) + probeY(a)) / 2 : 0;
  long hi = seeHi ? (probeY(b) + probeY(b + 1)) / 2 : 0;
  if (seeLo && seeHi && abs(hi - lo - GAP_CM) > SWEEP_STEP_CM) return NO_GAP;
  long c = seeLo && seeHi ? (lo + hi) / 2 : seeLo ? lo + GAP_CM / 2 : hi - GAP_CM / 2;
  return abs(c) <= 30 - GAP_CM / 2 + 2 ? c : NO_GAP;
}

// Probes the line ahead across the corridor, near side first, and returns gapMiddle(). Once it has seen line after gap,
// both edges are known and the rest must be line, so it stops there.
// LED per probe: red = line, green = gap. Serial: "sweep LL......LLL -> c" (first char = leftmost probe).
long sweep(long &y) {
  bool line[N_PROBES], up = y < 0, gapSeen = false;
  byte j = 0;
  for (; j < N_PROBES; j++) {
    byte i = up ? j : N_PROBES - 1 - j;
    strafeTo(probeY(i), y);
    y = probeY(i);
    line[i] = probe();
    displayColor(line[i] ? "RED" : "GREEN");
    if (!line[i]) gapSeen = true;
    else if (gapSeen) break;
  }
  for (byte k = j + 1; k < N_PROBES; k++) line[up ? k : N_PROBES - 1 - k] = true;
  long c = gapMiddle(line);
  Serial.print("sweep ");
  for (byte i = 0; i < N_PROBES; i++) Serial.print(line[i] ? 'L' : '.');
  Serial.print(" -> ");
  Serial.println(c);
  return c;
}

// Strafes to the gap's middle c, then creeps slowly until the sensor is 3 cm past the line's center, watching the floor,
// then drives the rest of the tile, ending PROBE_START_CM before the next line.
// False = it saw the line while crossing: it backed up, and the line must be swept again.
bool cross(long c, long &y) {
  strafeTo(c, y);
  y = c;
  const int N = (PROBE_START_CM + 3) * 10;
  int moved = 0, back = 0;
  while (moved < N && slow(1, N - moved, true, moved))
    if (offSafe()) { slow(-1, moved, false, back); return false; }
  moveCm(0, 30 - N / 10);
  return true;
}

// The three lines. Starts with the sensor about PROBE_START_CM before line 1, ends the same before the checkpoint 2 edge.
// If no probe reached the line, the line is farther than PROBE_REACH_CM at every y (it's straight), so creeping that far
// less 1 cm is safe: done once per line to fix a short start. A sweep that doesn't make sense, or a crossing that sees the
// line, is retried. After 3 failed tries on one line the robot stops for good with the LED red: it never guesses through.
void crossLines() {
  long y = sideY(0);
  for (byte k = 0; k < 3; k++) {
    bool advanced = false;
    for (byte tries = 0; ; ) {
      long c = sweep(y);
      if (c == NO_LINE && !advanced) {
        int m = 0;
        if (slow(1, (PROBE_REACH_CM - 1) * 10, true, m) && offSafe()) slow(-1, (PROBE_START_CM - 1) * 10, false, m);
        advanced = true;
        continue;
      }
      if (c < NO_LINE && cross(c, y)) break;
      if (++tries == 3) { displayColor("RED"); setAll(0, 0, 0, 0); while (true) {} }
    }
  }
}

// Drives forward until the color sensor has left the red tile: 2 fresh reads in a row that are a color but not RED.
// Then the sensor is just past the section's edge, a known spot. Gives up after maxMs. True if it found the edge.
// The color board is asked every 5 ms, like slow().
bool leaveRed(unsigned maxMs) {
  getColor();
  byte last = colorCount, n = 0;
  unsigned long t0 = millis(), lastAsk = 0;
  bool found = false;
  while (!found && millis() - t0 < maxMs) {
    steer(FWD);
    if (millis() - lastAsk < 5) continue;
    lastAsk = millis();
    const char *c = getColor();
    if (colorCount == last) continue;
    last = colorCount;
    if (!strcmp(c, "RED")) n = 0;
    else if (strcmp(c, "?") && strcmp(c, "NO SIGNAL")) found = ++n >= 2;
  }
  setAll(0, 0, 0, 0);
  imuWait(MS_SETTLE);
  return found;
}

// Moves forward or back until the back sensor reads target cm (within CENTER_TOL_CM), nudging by the measured error.
// At most 6 tries. Like centerAxis(): nudges at NUDGE_PCT, and it stops if the back sensor sees nothing or the
// error stops shrinking.
void backTo(int target) {
  int prev = 1000;
  for (byte k = 0; k < 6; k++) {
    int d = readCm3(3);
    int e = d - target;   // + = too far forward
    if (!d || abs(e) <= CENTER_TOL_CM || abs(e) >= prev) return;
    prev = abs(e);
    const int8_t *w = MOVE[e > 0 ? 2 : 0];
    driveAt(w, max((unsigned long)NUDGE_MS, abs(e) * MS_STRAIGHT / 30), NUDGE_PCT, 4);
  }
}

// From checkpoint 1 into column 1, ending with the color sensor PROBE_START_CM before line 1.
// 1. Forward until the sensor leaves red (the section's edge), then to column 1's middle. Lines lie on tile borders, so
//    there the sensor is still 15 - COLOR_AHEAD_CM before line 1: the entry can't reach it.
// 2. Checkpoint 1 is in line with one row, so the other row has the section's start wall behind column 1. It strafes to
//    that row (up or down, whichever it isn't in), checking the wall is there, else it tries the first row.
// 3. Backs or advances until the back sensor reads the distance that puts the color sensor PROBE_START_CM before line 1
//    (a centered robot reads BACK_CENTER_CM), so the start doesn't depend on how far the entry went.
void enterSection2() {
  if (leaveRed(MS_STRAIGHT)) moveCm(0, 15 + COLOR_AHEAD_CM - 1);   // Sensor ~1 cm past the edge when it stops
  long y = sideY(0);
  long row = y >= 0 ? -GAP_HALF : GAP_HALF;   // The row it isn't in
  strafeTo(row, y);
  if (!isWall(3, readCm3(3))) { strafeTo(-row, row); row = -row; }
  backTo(BACK_CENTER_CM + 15 - PROBE_START_CM - COLOR_AHEAD_CM);
  strafeTo(row, row);   // backTo() drives straight only; square up sideways again
  settleHeading();
}

// From checkpoint 1: enterSection2(), the three lines.
// In column 4, checkpoint 2 lines up with one row and the other row has its end wall right ahead: it picks the open row,
// crosses the white strip (not a penalty line) and drives onto red.
void section2() {
  enterSection2();
  crossLines();
  long y = sideY(0);
  strafeTo(GAP_HALF, y);
  y = GAP_HALF;
  int f = readCm(1);
  if (f && f < CP2_WALL_CM) strafeTo(-GAP_HALF, y);
  long t = 0;
  while (true) {
    const char *c = creep(1, MS_STRAIGHT, t);
    if (!strcmp(c, "RED")) { if (isRed()) break; continue; }
    if (!strcmp(c, "WHITE")) { fwdMs(MS_CROSS); t += MS_CROSS; continue; }
    break;
  }
  fwdMs(MS_ONTO_CP2);
}

// Call once from setup(): creeps slowly for 4 s. Measure how far it went in mm and set SLOW_MM_PER_S = mm / 4.
void testSlow() {
  int m = 0;
  slow(1, SLOW_MM_PER_S * 4, false, m);
}
