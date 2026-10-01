// DISTANCE SENSORS

const byte SENSOR_PIN[3] = {2, A4, A3}; 
int dist[3];

void readSensors() {
  for (byte i = 0; i < 3; i++) dist[i] = digitalRead(SENSOR_PIN[i]) == LOW;
}

// ON DISTANCE DETECTION STOP SETUP

const byte FRONT = 1; 
const int8_t FWD[4] = {1, -1, 1, 1};

