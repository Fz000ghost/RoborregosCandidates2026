#include <Servo.h>

// CLAW

const byte CLAW_PIN = A5;
const byte CLAW_OPEN = 90;
const byte CLAW_CLOSED = 0;
const unsigned MS_CLAW = 500;
Servo claw;
bool clawOpen = true;

// COLOR SENSOR

const byte S2 = A1, S3 = A0, COLOR_OUT = A2;
int rgb[3];

// RGB LED

const int LED_R = 3, LED_G = 6, LED_B = 5;

// DISTANCE SENSORS

const byte SENSOR_PIN[3] = {2, A4, A3};
int dist[3];

// MOTORS

struct Motor { byte a, b; int8_t dir; };

Motor m[4] = {
  {13, 10, 1},
  {12, 11, 1},
  { 4,  7, 1},
  { 8,  9, 1},
};

// Wheel signs (FL, FR, RL, RR) for each direction.
#define FWD    1, -1,  -1, -1
#define BACK  -1,  1, 1,  1
#define RIGHT -1, -1,  -1,  1
#define LEFT   1,  1, 1, -1

const unsigned MS_STRAIGHT = 2450;
const unsigned MS_STRAFE = 2450;
const unsigned MS_TURN90 = 1670;
const unsigned MS_SETTLE = 200;

// MAZE (Pista A)

const int8_t DX[4] = {0, 1, 0, -1};
const int8_t DY[4] = {1, 0, -1, 0};
const byte GRID = 9;
const byte NO_SIDE = 255;
bool visited[GRID][GRID];
bool freeSide[4];
byte posX = GRID / 2, posY = GRID / 2, heading = 0;
byte path[GRID * GRID], pathLen = 0;
byte hist[2 * GRID * GRID], histLen = 0;

// LEVELS (Pista B)

const bool TRACK_B = false;

// INICIO changes per round: set it in calibration. Cells: 3x3 = 1..3 (x east, y north), CP1 at (4,2).
// START_HEADING = direction the robot faces, into the 3x3: 0 N, 1 E, 2 S, 3 W. Drawn layout: (1, 0) facing N.
const byte START_X = 1, START_Y = 0, START_HEADING = 0;

const unsigned MS_GRAB_BACK = MS_STRAIGHT / 2;
const long MS_PAST = MS_STRAIGHT / 15;
const long MS_TO_GREEN = MS_STRAIGHT / 4;

// Robot width
const byte ROBOT_W_CM = 19;

const long Y_LIM = (30L - ROBOT_W_CM / 2 - 2) * MS_STRAFE / 30;
const long GAP_HALF = MS_STRAFE / 2;
const unsigned MS_CROSS = 300;
const unsigned MS_ONTO_CP2 = MS_STRAIGHT / 2;

// MAIN

// Runs once at power-up: sets every pin, opens the claw, waits 3 s to place the robot on its tile, 
// then runs Pista B (levels) or Pista A (explore) depending on TRACK_B.
//To run a test, call it from Tests.ino here instead.
void setup() {
  Serial.begin(9600);

  for (byte i = 0; i < 4; i++) {
    pinMode(m[i].a, OUTPUT);
    pinMode(m[i].b, OUTPUT);
  }
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  claw.attach(CLAW_PIN);
  claw.write(CLAW_OPEN);

  delay(3000);

  
  if (TRACK_B) levels();
  else explore();

  
}

void loop() {testColor();}
