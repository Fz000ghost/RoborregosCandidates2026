#include <Servo.h>
#include <Wire.h>

// Claw

const byte CLAW_PIN = A2;
const byte CLAW_OPEN = 0;
const byte CLAW_CLOSED = 60;
const unsigned MS_CLAW = 1000;
Servo claw;
bool clawOpen = false;   // Starts closed

// Color sensor and RGB LED (second Uno).

const byte COLOR_ADDR = 0x08;

// Distance sensors

const byte US_TRIG[4] = {2, 5, A0, 5};
const byte US_ECHO[4] = {3, 6, A1, A3};
const unsigned long US_TIMEOUT_US = 12000;
const int US_MAX_CM = 200;
const int SIDE_CENTER_CM = 8;
const int FRONT_CENTER_CM = 5;
const int BACK_CENTER_CM = 4;
const int CENTER_CM[4] = {SIDE_CENTER_CM, FRONT_CENTER_CM, SIDE_CENTER_CM, BACK_CENTER_CM};
const int WALL_MARGIN_CM = 9;
const int CENTER_TOL_CM = 1;
const int JUMP_CM = 5;
int dist[4];

// Motors

struct Motor { byte a, b; int8_t dir; };

Motor m[4] = {
  { 4,  7, 1},
  { 8,  9, 1},
  {13, 12, 1},
  {11, 10, 1},
};

// Wheel signs (FL, FR, RL, RR) for each direction.
#define FWD    1,  1,  1,  1
#define BACK  -1, -1, -1, -1
#define RIGHT 1,  -1,  -1, 1
#define LEFT   -1,  1,  1, -1
#define ROT_R  1, -1,  1, -1 

const unsigned MS_STRAIGHT = 1700;
const unsigned MS_STRAFE = 1700;
const unsigned MS_TURN90 = 1670;
const unsigned MS_SETTLE = 200;

// IMU

const int8_t GYRO_SIGN = 1;
const float HOLD_DEG = 1.0; 
const float TURN_LEAD_DEG = 6.0;
const float TURN_TOL_DEG = 1.5;
const unsigned NUDGE_MS = 40;
const int TILT_RAW = 2000;
float yaw = 0, yawTarget = 0, gyroBias = 0;
unsigned long lastYawUs = 0;

// Maze (Pista A)

const int8_t DX[4] = {0, 1, 0, -1};
const int8_t DY[4] = {1, 0, -1, 0};
const byte GRID = 9;
const byte NO_SIDE = 255;
bool visited[GRID][GRID];
bool freeSide[4];
byte posX = GRID / 2, posY = GRID / 2, heading = 0;
byte path[GRID * GRID], pathLen = 0;
byte hist[2 * GRID * GRID], histLen = 0;

// Levels (Pista B)

const bool TRACK_B = false;

const byte START_X = 1, START_Y = 0, START_HEADING = 0;

const int GRAB1_CM = 25;
const int GRAB2_CM = 1;

const long MS_PAST = MS_STRAIGHT / 15;
const long MS_TO_GREEN = MS_STRAIGHT / 4;

const byte ROBOT_W_CM = 18.5;

const long Y_LIM = 30L - ROBOT_W_CM / 2 - 2;
const long GAP_HALF = 15;
const int END_WALL_CM = 45;
const long COLOR_AHEAD_CM = 10;
const unsigned MS_CROSS = 300;
const unsigned MS_ONTO_CP2 = MS_STRAIGHT / 2;

// Main

void setup() {
  Serial.begin(9600);
  Serial.println("start");


  for (byte i = 0; i < 4; i++) {
    pinMode(m[i].a, OUTPUT);
    pinMode(m[i].b, OUTPUT);
  }
  for (byte i = 0; i < 4; i++) {
    pinMode(US_TRIG[i], OUTPUT);
    pinMode(US_ECHO[i], INPUT);
  }

  claw.write(CLAW_CLOSED); 
  claw.attach(CLAW_PIN);

  imuBegin();
  delay(2000);
  imuCalibrate();

  
  if (TRACK_B) levels();
  else explore();
  

}

void loop() {
//testColor();
//testDistance();
//testImu();
}
