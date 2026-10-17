// IMU (MPU-6050). THE GYRO'S Z AXIS MEASURES TURNING; THE ACCELEROMETER ONLY TELLS IF THE ROBOT IS TILTED (RAMP).

const byte MPU = 0x68;

// WRITES ONE MPU-6050 REGISTER.
void mpuWrite(byte reg, byte val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// RAW GYRO Z INTO z (131 = 1 DEG/S AT THE DEFAULT +-250 DEG/S RANGE). FALSE IF THE I2C READ FAILED.
bool gyroZ(int &z) {
  Wire.beginTransmission(MPU);
  Wire.write(0x47);
  if (Wire.endTransmission(false) || Wire.requestFrom(MPU, (byte)2) != 2) return false;
  int hi = Wire.read();
  z = (int16_t)((hi << 8) | Wire.read());
  return true;
}

// STARTS I2C AND WAKES THE MPU-6050.
void imuBegin() {
  Wire.begin();
  Wire.setWireTimeout(3000, true);   // A MOTOR-NOISE GLITCH ON THE BUS MUST NOT FREEZE THE ROBOT
  mpuWrite(0x6B, 0x01);              // WAKE UP, CLOCKED BY THE GYRO (STEADIER THAN THE INTERNAL CLOCK)
  mpuWrite(0x1A, 0x03);              // ~44 HZ LOW-PASS: CUTS MOTOR VIBRATION
}

// AVERAGES THE GYRO FOR ~1 S WITH THE ROBOT STILL; THAT OFFSET IS REMOVED FROM EVERY READ. ZEROES THE HEADING.
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
