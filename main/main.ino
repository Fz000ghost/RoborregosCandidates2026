#include <Servo.h>


// WHEEL MOTORS

struct Motor { byte en, a, b; int8_t dir; };

Motor m[4] = {
  {6, 11, 12, 1},  
  {9, 13, 0, 1}, 
  {3,  4,  7, 1},  
  {5,  8, 10, 1},   
};

const byte SPEED = 255; 
const unsigned MS_STRAIGHT = 1000;
const unsigned MS_STRAFE = 1500; 
const unsigned MS_TURN90 = 1000; 
const bool WHEEL_TEST = false;

void drive(int8_t fl, int8_t fr, int8_t rl, int8_t rr, unsigned ms) {
  int8_t s[4] = {fl, fr, rl, rr};
  for (byte i = 0; i < 4; i++) setMotor(i, s[i]);
  delay(ms);
  for (byte i = 0; i < 4; i++) setMotor(i, 0);
}

void forward30()     { drive( 1,  -1,  1,  1, MS_STRAIGHT); }
void back30()        { drive(-1, 1, -1, -1, MS_STRAIGHT); }
void right30()       { drive( 1, 1, 1,  -1, MS_STRAFE); }
void left30()        { drive(-1,  -1,  -1, 1, MS_STRAFE); }
void rotateRight90()  { drive(-1,  -1, 1,  -1, MS_TURN90); }
void rotateLeft90() { drive( 1, 1,  -1, 1, MS_TURN90); }

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

// DISTANCE SENSORS

const byte SENSOR_PIN[3] = {2, A4, A3}; 
int dist[3];

void readSensors() {
  for (byte i = 0; i < 3; i++) dist[i] = digitalRead(SENSOR_PIN[i]) == LOW;
}

// ON DISTANCE DETECTION STOP SETUP

const byte FRONT = 1; 
const int8_t FWD[4] = {1, -1, 1, 1};

// COLOR SENSOR
const byte S2 = A1, S3 = A0, COLOR_OUT = A2;
int rgb[3]; 

int readFilter(byte s2, byte s3) {
  digitalWrite(S2, s2);
  digitalWrite(S3, s3);
  return pulseIn(COLOR_OUT, LOW, 20000);
}

void readColor() {
  rgb[0] = readFilter(LOW, LOW); 
  rgb[1] = readFilter(HIGH, HIGH);
  rgb[2] = readFilter(LOW, HIGH);
}


struct Ref { const char *name; int r, g, b; };
Ref COLORS[] = {
  {"WHITE",   20,  20,  18}, 
  {"BLACK",  200, 210, 170},
  {"RED",     30, 110,  80},
  {"GREEN",   90,  60,  80},
  {"BLUE",   100,  80,  40},
  {"YELLOW",  25,  35,  70},
};
const byte N_COLORS = sizeof(COLORS) / sizeof(COLORS[0]);


const char *colorName() {
  const char *best = "?";
  long bestD = 2147483647L;
  for (byte i = 0; i < N_COLORS; i++) {
    long d = sq((long)rgb[0] - COLORS[i].r) + sq((long)rgb[1] - COLORS[i].g) + sq((long)rgb[2] - COLORS[i].b);
    if (d < bestD) { bestD = d; best = COLORS[i].name; }
  }
  return best;
}

// ON INITIALIZATION

void setup() {
  Serial.begin(9600); 
  //UCSR0B &= ~_BV(RXEN0);

  /*for (byte i = 0; i < 4; i++) { 
    pinMode(m[i].en, OUTPUT);
    pinMode(m[i].a, OUTPUT);
    pinMode(m[i].b, OUTPUT);
  }*/
  

  //claw.attach(CLAW_PIN);
  //claw.write(CLAW_OPEN);
  
  //pinMode(S2, OUTPUT);
  //pinMode(S3, OUTPUT);

  //delay(3000);

  /*if (WHEEL_TEST) { 
    for (byte i = 0; i < 4; i++) {
      setMotor(i, 1); delay(1000);
      setMotor(i, 0); delay(1000);
    }
    return;
  }*/

  //forward30();     delay(1000);
  //back30();        delay(1000);
  //right30();       delay(1000);
  //left30();        delay(1000);
  //rotateRight90(); delay(1000);
  //rotateLeft90();  delay(1000);
  //toggleClaw();    delay(1000); 

  //setMotor(1, 1); delay(1000);

  //toggleClaw();   delay(1000);

  //setMotor(3,0);

}

// ON LOOP

void loop() {
  /*readSensors();
  Serial.print("L F R: ");
  for (byte i = 0; i < 3; i++) { Serial.print(dist[i]); Serial.print(' '); }
  Serial.println();

  readColor();
  Serial.print("R G B: ");
  for (byte i = 0; i < 3; i++) { Serial.print(rgb[i]); Serial.print(' '); }
  Serial.print("-> ");
  Serial.println(colorName());
  
  delay(200);*/

  /*readSensors();
  for (byte i = 0; i < 4; i++) setMotor(i, dist[FRONT] ? 0 : FWD[i]); 

  Serial.print("L F R: ");
  for (byte i = 0; i < 3; i++) { Serial.print(dist[i]); Serial.print(' '); }
  Serial.println();
  delay(20);*/
}
