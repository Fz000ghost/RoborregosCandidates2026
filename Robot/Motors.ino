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