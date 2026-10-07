#include <Servo.h>

// CLAW
const byte CLAW_PIN = A5;
const byte CLAW_OPEN = 90;
const byte CLAW_CLOSED = 0;
const unsigned MS_CLAW = 500;
Servo claw;
bool clawOpen = true;

// COLOR
const byte S2 = A1, S3 = A0, COLOR_OUT = A2;
int rgb[3]; 

//DISTANCE
const byte SENSOR_PIN[3] = {2, A4, A3}; 
int dist[3];

//MOTORS
struct Motor { byte a, b; int8_t dir; };

Motor m[4] = {
  {13, 10, 1},
  {12,  11, 1},
  { 6,  7, 1},
  { 8, 9, 1},
};

const unsigned MS_STRAIGHT = 1500;
const unsigned MS_STRAFE = 1500; 
const unsigned MS_TURN90 = 1850; 
const bool WHEEL_TEST = false;

// MAZE
const unsigned MS_SETTLE = 200;
const int8_t DX[4] = {0, 1, 0, -1};
const int8_t DY[4] = {1, 0, -1, 0};
const byte GRID = 9;  
const byte NO_SIDE = 255;
bool visited[GRID][GRID];
bool freeSide[4]; 
byte posX = GRID / 2, posY = GRID / 2, heading = 0;
byte path[GRID * GRID], pathLen = 0; 
byte hist[2 * GRID * GRID], histLen = 0; 

// ON INITIALIZATION
void setup() {
  Serial.begin(9600);

  for (byte i = 0; i < 4; i++) {
    pinMode(m[i].a, OUTPUT);
    pinMode(m[i].b, OUTPUT);
  }
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);

  //delay(3000);      

  if (WHEEL_TEST) {
    for (byte i = 0; i < 4; i++) {
      setMotor(i, 1); delay(1000);
      setMotor(i, 0); delay(1000);
    }
    return;
  }

  /*forward30();     delay(1000);
  back30();        delay(1000);
  right30();       delay(1000);
  left30();        delay(1000);
  rotateRight90(); delay(1000);  
  rotateLeft90();  delay(1000);*/
  //toggleClaw();    delay(1000);
  

  //explore();

}

// ON LOOP

void loop() {
  /*readSensors();
  Serial.print("L F R: ");
  for (byte i = 0; i < 3; i++) { Serial.print(dist[i]); Serial.print(' '); }
  Serial.println();*/

  /*readColor();
  Serial.print("R G B: ");
  for (byte i = 0; i < 3; i++) { Serial.print(rgb[i]); Serial.print(' '); }
  Serial.print("-> ");
  Serial.println(colorName());*/
  
  delay(200);
  
  
}
