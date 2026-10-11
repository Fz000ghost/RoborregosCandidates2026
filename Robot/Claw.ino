// CLAW SERVO

// Opens the claw if closed, closes it if open, and waits MS_CLAW for it to finish moving.
void toggleClaw() {
  clawOpen = !clawOpen;
  claw.write(clawOpen ? CLAW_OPEN : CLAW_CLOSED);
  delay(MS_CLAW);
}
