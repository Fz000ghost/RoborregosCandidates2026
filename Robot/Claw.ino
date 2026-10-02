
// CLAW SERVO

void toggleClaw() {
  clawOpen = !clawOpen;
  claw.write(clawOpen ? CLAW_OPEN : CLAW_CLOSED);
  delay(MS_CLAW);
}

