// DISTANCE SENSORS

void readSensors() {
  for (byte i = 0; i < 3; i++) dist[i] = digitalRead(SENSOR_PIN[i]) == LOW;
}



