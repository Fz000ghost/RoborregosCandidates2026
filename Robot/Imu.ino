// IMU (MPU-6050). The gyro's Z axis measures turning; the accelerometer only tells if the robot is tilted (ramp).
//
// The gyro samples itself at GYRO_HZ into its own FIFO, and updateYaw() adds up every sample with its exact time step.
// So no rotation is missed while the Uno is busy (ultrasonic pulseIn, color reads, claw), and a long gap between
// calls doesn't multiply one noisy reading by a long time, as reading one value per call did.

const byte MPU = 0x68;
const byte GYRO_HZ = 200;                // FIFO sample rate. 1024-byte FIFO / 2 bytes = 2.5 s of samples without a read
const float DEG_PER_RAW = 1.0 / 131.0 / GYRO_HZ;   // One raw sample (131 = 1 deg/s at +-250 deg/s) times its time step
const int STILL_RAW = 131;               // Wheels off and turning under 1 deg/s = the robot is still
const unsigned STILL_AFTER_MS = 300;     // Wheels off at least this long before trusting "still" (after MS_SETTLE)
const float BIAS_RATE = 0.002;           // How fast the offset follows the gyro while still (~2.5 s time constant)
const int CAL_SPREAD_RAW = 100;          // Calibration readings spread more than this = the robot was touched: redo it

float lastRate = 0;                      // Previous sample, offset removed, for the trapezoid rule
bool haveLast = false;

// Writes one MPU-6050 register.
void mpuWrite(byte reg, byte val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Empties the FIFO and starts filling it again.
void fifoReset() {
  mpuWrite(0x6A, 0x04);   // FIFO off, reset
  mpuWrite(0x6A, 0x40);   // FIFO on
  haveLast = false;
}

// Reads up to 15 gyro Z samples (raw) from the FIFO into z[]. Returns how many (0 = none yet).
// If the count read fails, nothing left the FIFO: the samples are read next time. If the data read fails halfway,
// the FIFO would be out of step (high and low bytes swapped), so it is reset: those few ms are lost, not misread.
byte fifoRead(int z[]) {
  Wire.beginTransmission(MPU);
  Wire.write(0x72);
  if (Wire.endTransmission(false) || Wire.requestFrom(MPU, (byte)2) != 2) return 0;
  unsigned count = Wire.read() << 8;
  count |= Wire.read();
  if (count >= 1000) { Serial.println("gyro FIFO full, reset"); fifoReset(); return 0; }
  byte n = min(count / 2, 15u);   // The Wire buffer holds 32 bytes
  if (!n) return 0;
  Wire.beginTransmission(MPU);
  Wire.write(0x74);
  if (Wire.endTransmission(false) || Wire.requestFrom(MPU, (byte)(2 * n)) != 2 * n) { fifoReset(); return 0; }
  for (byte i = 0; i < n; i++) {
    int hi = Wire.read();
    z[i] = (int16_t)((hi << 8) | Wire.read());
  }
  return n;
}

// Starts I2C, wakes the MPU-6050 and starts the gyro Z FIFO.
void imuBegin() {
  Wire.begin();
  Wire.setWireTimeout(3000, true);   // A motor-noise glitch on the bus must not freeze the robot
  mpuWrite(0x6B, 0x01);              // Wake up, clocked by the gyro (steadier than the internal clock)
  delay(100);
  mpuWrite(0x1A, 0x03);              // ~44 Hz low-pass: cuts motor vibration (gyro runs at 1 kHz inside)
  mpuWrite(0x1B, 0x00);              // +-250 deg/s: the finest resolution
  mpuWrite(0x19, 1000 / GYRO_HZ - 1);  // Sample rate divider: 1 kHz / 5 = 200 Hz
  mpuWrite(0x23, 0x10);              // Only gyro Z goes into the FIFO
  fifoReset();
}

// True when the wheels have been off long enough that the robot can't be turning.
bool still() {
  return !wheelsOn && millis() - lastWheelMs > STILL_AFTER_MS;
}

// Measures the gyro's offset with the robot still: 2 s of samples. If they spread more than CAL_SPREAD_RAW
// (the robot was touched), it measures again, up to 5 times. Zeroes the heading.
void imuCalibrate() {
  for (byte t = 0; t < 5; t++) {
    fifoReset();
    long sum = 0;
    int n = 0, lo = 32767, hi = -32767, z[15];
    unsigned long t0 = millis();
    while (n < 2 * GYRO_HZ && millis() - t0 < 3000) {
      byte k = fifoRead(z);
      for (byte i = 0; i < k; i++) { sum += z[i]; n++; lo = min(lo, z[i]); hi = max(hi, z[i]); }
    }
    gyroBias = n ? (float)sum / n : 0;
    if (n && hi - lo <= CAL_SPREAD_RAW) break;
    Serial.println("gyro calibration: robot moved, again");
  }
  fifoReset();
  yaw = yawTarget = 0;
}

// Adds every gyro sample waiting in the FIFO to yaw (trapezoid rule, exact 1/GYRO_HZ steps).
// While the robot is still: the offset slowly follows the reading (it drifts as the chip warms up), and nothing
// is added to yaw, so the heading can't drift during scans, color reads or the claw.
void updateYaw() {
  int z[15];
  byte n;
  do {
    n = fifoRead(z);
    for (byte i = 0; i < n; i++) {
      float r = z[i] - gyroBias;
      if (still() && fabs(r) < STILL_RAW) { gyroBias += r * BIAS_RATE; lastRate = 0; haveLast = true; continue; }
      if (!haveLast) lastRate = r;
      yaw += GYRO_SIGN * GYRO_SCALE * (r + lastRate) * 0.5 * DEG_PER_RAW;
      lastRate = r;
      haveLast = true;
    }
  } while (n == 15);
}

// Waits ms milliseconds, keeping the heading up to date.
void imuWait(unsigned ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) updateYaw();
}

// True if the accelerometer says the robot is tilted (on the ramp).
bool tilted() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) || Wire.requestFrom(MPU, (byte)4) != 4) return false;
  int hi = Wire.read();
  int ax = (int16_t)((hi << 8) | Wire.read());
  hi = Wire.read();
  int ay = (int16_t)((hi << 8) | Wire.read());
  return abs(ax) > TILT_RAW || abs(ay) > TILT_RAW;
}
