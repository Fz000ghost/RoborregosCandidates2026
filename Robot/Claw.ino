// Claw servo

// Opens (true) or closes (false) the claw and waits MS_CLAW for it to move. Does nothing if it's already there.
void setClaw(bool open) {
  if (open == clawOpen) return;
  clawOpen = open;
  claw.write(clawOpen ? CLAW_OPEN : CLAW_CLOSED);
  delay(MS_CLAW);
}

// Opens the claw if closed, closes it if open.
void toggleClaw() { setClaw(!clawOpen); }
