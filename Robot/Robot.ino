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
const int CENTER_STEP_CM = 4;   // center() nudges at most this far per try
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
const unsigned MS_STRAFE = 1675;
const unsigned MS_TURN90 = 1670;
const unsigned MS_SETTLE = 200;

// Speeds in % of full speed (software PWM)
const int MIN_PCT = 25;           // Slowest speed that still moves the robot. Find it with testSpeed()
const int APPROACH_PCT = 35;      // Measured moves slow to this for their last APPROACH_CM
const int APPROACH_CM = 6;
const int NUDGE_PCT = 30;         // Centering and end-of-move nudges
const int TURN_SLOW_PCT = 35;     // Last TURN_SLOW_DEG of a turn; heading nudges start here
const float TURN_SLOW_DEG = 20;
const unsigned RAMP_MS = 150;     // Moves ramp up from MIN_PCT over this long
const float TURN_KP = 20;         // Heading hold: % of turn per degree off
const int TURN_MAX_PCT = 60;

// Side guard while moving (guard())
const int GUARD_CM = 2;           // Push away from a wall once this much closer than centered
const int GUARD_KP = 15;          // % sideways per cm past that
const int GUARD_MAX_PCT = 50;
const float POST_CM = 2;          // timedTile(): a side reading changes this far from the border (beam width + read lag)

// Squaring on a wall: ahead, or left (squareOn(), Pista A)
const int SQUARE_PCT = 40;
const int SQUARE_TOUCH_CM = 3;
const int LEFT_TOUCH_CM = 2;      // Left reading with the left panels touching a wall. Check with testDistance()
const byte SQUARE_EVERY = 3;      // Left squaring only if the last heading reset was at least this many tiles ago    // Front reading with the robot's front touching the wall. Check with testDistance()
const unsigned SQUARE_PUSH_MS = 300;
const float SQUARE_MAX_DEG = 8;   // Off by more than this = not flush: the heading is kept

// IMU

const int8_t GYRO_SIGN = 1;
const float GYRO_SCALE = 1.0*360/345*360/370.3;     // Each chip reads up to 3% off (3° per 90° turn). Calibrate with testGyroScale()
const float TURN_LEAD_DEG = 3.0;
const float TURN_TOL_DEG = 1.0;
const unsigned NUDGE_MS = 40;
const int TILT_RAW = 2000;
float yaw = 0, yawTarget = 0, gyroBias = 0;
byte wheelsOn = 0;                // One bit per wheel that is on
unsigned long lastWheelMs = 0;    // Last time any wheel was on

// Moves measured by a wall (moveCm)

const int MEASURE_MAX_CM = 120;   // Walls farther than this aren't used: the beam is too wide to hit only the wall
const int STOP_LEAD_CM = 1;       // The robot coasts about this far after the motors stop, so it stops this early
const int MOVE_TOL_CM = 1;        // After a measured move it nudges until within this of the target
const int SNAP_MAX_CM = 8;        // A tile move snaps to the tile grid of the wall only if that changes it by this or less
const int GRID_TOL_CM = 5;        // center() uses a wall one tile beyond only if it reads within this of its grid line
const int TRUST_SLACK_CM = 4;     // Wall progress may differ from timed progress by this + 1/8 before the wall is dropped

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

const bool TRACK_B = true;

const byte START_X = 1, START_Y = 0, START_HEADING = 0;

const int GRAB1_CM = 25;
const int GRAB2_CM = 1;

const long MS_PAST = MS_STRAIGHT / 15;
const long MS_TO_GREEN = MS_STRAIGHT / 4;

const byte ROBOT_W_CM = 18.5;

const long Y_LIM = 30L - ROBOT_W_CM / 2 - 2;
const long GAP_HALF = 15;
const int END_WALL_CM = 45;
const int COLOR_AHEAD_CM = 10;   // How far the color sensor is ahead of the robot's center. Measure it on the robot
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
  pwmBegin();
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
//testGyroScale();
}
