// Useful links:
// IR Obstacle Collision Detection Module
// http://henrysbench.capnfatz.com/henrys-bench/arduino-sensors-and-input/arduino-ir-obstacle-sensor-tutorial-and-manual/
// http://qqtrading.com.my/ir-infrared-obstacle-detaction-sensor-module-fc-5
// Motor
// https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library
// https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino?view=all
//


// Motor
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
// Create the motor shield object with the default I2C address:
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Select which 'port' M1, M2, M3 or M4:
Adafruit_DCMotor *Motor1 = AFMS.getMotor(1); /* rear left */ 
Adafruit_DCMotor *Motor2 = AFMS.getMotor(2); /* rear right */
Adafruit_DCMotor *Motor3 = AFMS.getMotor(3); /* Front right */
Adafruit_DCMotor *Motor4 = AFMS.getMotor(4); /* Front left */
// Speed variables: fr=front_right ; fl=front_left ; rr=rear_right ; rl=rear_left
int rl, rr, fr, fl;
int ii = 1;
int gostraight = 1;
int trig_R = 1;
int trig_L = 1;

// IR Sensor
int LED = 13; // Use the onboard Uno LED
int isObstaclePin_Left = 27;  // This is our input pin - left position
int isObstaclePin_Right = 26;  // This is our input pin - right position
// LOW = NO OBSTACLE == GO == WHITE; HIGH = YES OBSTACLE == TURN == BLACK
int isObstacle_Left = LOW;  // HIGH = YES OBSTACLE ON THE LEFT == TURN LEFT; 
int isObstacle_Right = LOW;  // HIGH = YES OBSTACLE ON THE RIGHT == TURN RIGHT;

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(isObstaclePin_Left, INPUT);
  pinMode(isObstaclePin_Right, INPUT);
  
  AFMS.begin();

  Serial.begin(9600);

}


// isObstacle ==low there is obstacle infront of sensor
// using serial monitor we can see this output
void loop() {
  isObstacle_Left = digitalRead(isObstaclePin_Left);
  isObstacle_Right = digitalRead(isObstaclePin_Right);
  
  if(trig_L == 1 && trig_R == 1 && gostraight == 1) { 
    // 1st straight
    rl = 50;
    rr = 50;
    fr = 50;
    fl = 50;
    go_motion(rl,rr,fr,fl);
    delay(4000);   
    gostraight = 2;
  }
      
  if (trig_L == 1 && trig_R == 1) { 
    // 1st turn right
    rl = 70;
    rr = 20;
    fr = 20;
    fl = 70;
    go_motion(rl,rr,fr,fl);
    delay(20);
    ii = ii+1;
    if (isObstacle_Left == HIGH && isObstacle_Right == LOW) {
      trig_L = 2;
      trig_R = 2;    
      delay(20);
    }
  }

  if (trig_L == 2 && trig_R == 2) { 
    // go straight after turn right
    rl = 50;
    rr = 50;
    fr = 50;
    fl = 50;
    go_motion(rl,rr,fr,fl);
    delay(20);
    if (isObstacle_Right == HIGH && isObstacle_Left == LOW) {
      trig_L = 300;
      trig_R = 300;
      delay(20);
    }
  }  

  if (trig_L == 300 && trig_R == 300) {
    // turn left after straight
    rl = 20;
    rr = 70;
    fr = 70;
    fl = 20;
    go_motion(rl,rr,fr,fl);
    delay(20*ii);
    trig_L = 3;
    trig_R = 3;
  } 

  if (trig_L == 3 && trig_R == 3) {
    // 2nd straight
    rl = 50;
    rr = 50;
    fr = 50;
    fl = 50;
    go_motion(rl,rr,fr,fl);
    delay(3000); 
    trig_L = 600;
    trig_R = 600;
  }
  
  if (trig_L == 600 && trig_R == 600) {
    stop_motion();
    delay(10000);
  }

  
}

void go_motion(int rl, int rr, int fr, int fl) {
  Motor1->setSpeed(rl);
  Motor2->setSpeed(rr);
  Motor3->setSpeed(fr);
  Motor4->setSpeed(fl);
  Motor1->run(FORWARD);
  Motor2->run(FORWARD);
  Motor3->run(FORWARD);
  Motor4->run(FORWARD);
}

void go_motion_left(int rl, int rr, int fr, int fl) {
  Motor1->setSpeed(rl);
  Motor2->setSpeed(rr);
  Motor3->setSpeed(fr);
  Motor4->setSpeed(fl);
  Motor1->run(BACKWARD);
  Motor2->run(FORWARD);
  Motor3->run(FORWARD);
  Motor4->run(BACKWARD);
}

void go_motion_right(int rl, int rr, int fr, int fl) {
  Motor1->setSpeed(rl);
  Motor2->setSpeed(rr);
  Motor3->setSpeed(fr);
  Motor4->setSpeed(fl);
  Motor1->run(FORWARD);
  Motor2->run(BACKWARD);
  Motor3->run(BACKWARD);
  Motor4->run(FORWARD);
}

void stop_motion() {
  Motor1->run(RELEASE);
  Motor2->run(RELEASE);
  Motor3->run(RELEASE);
  Motor4->run(RELEASE);
}