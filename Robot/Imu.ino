// IMU (MPU-6050). The gyro's Z axis measures turning; the accelerometer only tells if the robot is tilted (ramp).

const byte MPU = 0x68;

// Writes one MPU-6050 register.
void mpuWrite(byte reg, byte val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Raw gyro Z into z (131 = 1 deg/s at the default +-250 deg/s range). False if the I2C read failed.
bool gyroZ(int &z) {
  Wire.beginTransmission(MPU);
  Wire.write(0x47);
  if (Wire.endTransmission(false) || Wire.requestFrom(MPU, (byte)2) != 2) return false;
  int hi = Wire.read();
  z = (int16_t)((hi << 8) | Wire.read());
  return true;
}

// Starts I2C and wakes the MPU-6050.
void imuBegin() {
  Wire.begin();
  Wire.setWireTimeout(3000, true);   // A motor-noise glitch on the bus must not freeze the robot
  mpuWrite(0x6B, 0x01);              // Wake up, clocked by the gyro (steadier than the internal clock)
  mpuWrite(0x1A, 0x03);              // ~44 Hz low-pass: cuts motor vibration
}

// Averages the gyro for ~1 s with the robot still; that offset is removed from every read. Zeroes the heading.
void imuCalibrate() {
  long sum = 0;
  int n = 0, z;
  for (int i = 0; i < 400; i++) {
    if (gyroZ(z)) { sum += z; n++; }
    delay(2);
  }
  gyroBias = n ? (float)sum / n : 0;
  yaw = yawTarget = 0;
  lastYawUs = micros();
}

void updateYaw() {
  unsigned long now = micros();
  float dt = (now - lastYawUs) * 1e-6;
  lastYawUs = now;
  int z;
  if (gyroZ(z)) yaw += GYRO_SIGN * (z - gyroBias) / 131.0 * dt;
}

void imuWait(unsigned ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) updateYaw();
}


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
