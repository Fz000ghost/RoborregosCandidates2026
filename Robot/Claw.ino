#include <Servo.h>

// CLAW SERVO

const byte CLAW_PIN = A5;
const byte CLAW_OPEN = 90;
const byte CLAW_CLOSED = 0;
const unsigned MS_CLAW = 500;
Servo claw;
bool clawOpen = true;

void toggleClaw() {
  clawOpen = !clawOpen;
  claw.write(clawOpen ? CLAW_OPEN : CLAW_CLOSED);
  delay(MS_CLAW);
}

void setMotor(byte i, int8_t s) {
  s *= m[i].dir;
  digitalWrite(m[i].a, s > 0);
  digitalWrite(m[i].b, s < 0);
  analogWrite(m[i].en, s ? SPEED : 255);
}
