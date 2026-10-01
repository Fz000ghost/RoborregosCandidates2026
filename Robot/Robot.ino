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
