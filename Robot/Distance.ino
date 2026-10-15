// DISTANCE SENSORS

// Reads the three sensors (0 L, 1 F, 2 R) into dist[]: 1 = wall seen.
void readSensors() {
  for (byte i = 0; i < 3; i++) dist[i] = digitalRead(SENSOR_PIN[i]) == LOW;
}
