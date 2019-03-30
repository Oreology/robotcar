
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *RearLeftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *RearRightMotor = AFMS.getMotor(2);
Adafruit_DCMotor *FrontRightMotor = AFMS.getMotor(3);
Adafruit_DCMotor *FrontLeftMotor = AFMS.getMotor(4);


const int LeftLED = 22;
const int ChangeLeftLED = 23;
const int RightLED = 41;  
const int ChangeRightLED = 53;

const int InitialLaneChangeDelay = 4000;   // In seconds.
const int SecondLaneChangeDelay = 3000;
const int TurningDelay = 1500;
const int LastDetectStraightDelay = 400;
const int OppositeTriggerTurnDelay = 50;
const int OppositeTriggerStraightDelay = 50;
const int Mode1RefreshDelay = 50;
const int TurningForward = 80;
const int TurningBackward = 80;
const int TurningStraightDelay = 500;

const int LaneDelay = 1400;
const int LaneDetect = 1000;

// HIGH = BLACK
// LOW = WHITE
const int ObstaclePinLeft = 25;  // This is our input pin - left position
const int ObstaclePinRight = 24;  // This is our input pin - right position
int ObstacleLeft = LOW;  // LOW = nothing, go.
int ObstacleRight = LOW;  // HIGH = black, turn / stop

int Mode = 1;
int ChangeCounter = 0;
int AlreadyChangedLane = 0;
int x = 0;

void setup() {
    pinMode(LeftLED, OUTPUT);
    pinMode(RightLED, OUTPUT);
    pinMode(ChangeLeftLED, OUTPUT);
    pinMode(ChangeRightLED, OUTPUT);
    
    pinMode(ObstaclePinLeft, INPUT);
    pinMode(ObstaclePinRight, INPUT);

    AFMS.begin();

    StopMotor();
    Mode = 1;
    ChangeCounter = 0;
    AlreadyChangedLane = 0;
    x = 0;
    delay(1000);
}

void loop() {

    if (ObstacleLeft==HIGH) {
        digitalWrite(LeftLED, HIGH);
    } else if (ObstacleRight==HIGH) {
        digitalWrite(RightLED,HIGH);
    }

    if (Mode==1) {  /* Lane Keeping */
        delay(Mode1RefreshDelay);
        ObstacleLeft = digitalRead(ObstaclePinLeft);
        ObstacleRight = digitalRead(ObstaclePinRight);
        ChangeCounter = ChangeCounter + Mode1RefreshDelay;
        if (ChangeCounter>=InitialLaneChangeDelay && AlreadyChangedLane==0) {
            Mode = 3;
            AlreadyChangedLane = 1;
        } else if (ChangeCounter>=InitialLaneChangeDelay && AlreadyChangedLane==1) {
            Mode = 2;
            AlreadyChangedLane = 0;
        }
        if (ObstacleLeft==LOW && ObstacleRight==LOW) {
            FrontRightMotor->setSpeed(TurningForward);
            FrontLeftMotor->setSpeed(TurningForward);
            RearRightMotor->setSpeed(TurningForward);
            RearLeftMotor->setSpeed(TurningForward);
            FrontRightMotor->run(FORWARD);
            FrontLeftMotor->run(FORWARD);
            RearRightMotor->run(FORWARD);
            RearLeftMotor->run(FORWARD); 
        } else if (ObstacleLeft==HIGH && ObstacleRight==LOW) {
            FrontRightMotor->setSpeed(TurningBackward);
            FrontLeftMotor->setSpeed(TurningForward);
            RearRightMotor->setSpeed(TurningBackward);
            RearLeftMotor->setSpeed(TurningForward);
            FrontRightMotor->run(BACKWARD);
            FrontLeftMotor->run(FORWARD);
            RearRightMotor->run(BACKWARD);
            RearLeftMotor->run(FORWARD); 
        } else if (ObstacleLeft==LOW && ObstacleRight==HIGH) {
            FrontRightMotor->setSpeed(TurningForward);
            FrontLeftMotor->setSpeed(TurningBackward);
            RearRightMotor->setSpeed(TurningForward);
            RearLeftMotor->setSpeed(TurningBackward);
            FrontRightMotor->run(FORWARD);
            FrontLeftMotor->run(BACKWARD);
            RearRightMotor->run(FORWARD);
            RearLeftMotor->run(BACKWARD);
        }
    } else if (Mode==2) {   /* Change Right */
        /*
        digitalWrite(ChangeRightLED, HIGH);
        ObstacleLeft = digitalRead(ObstaclePinLeft);
        ObstacleRight = digitalRead(ObstaclePinRight);

        FrontRightMotor->setSpeed(TurningBackward);
        FrontLeftMotor->setSpeed(TurningForward);
        RearRightMotor->setSpeed(TurningBackward);
        RearLeftMotor->setSpeed(TurningForward);
        FrontRightMotor->run(BACKWARD);
        FrontLeftMotor->run(FORWARD);
        RearRightMotor->run(BACKWARD);
        RearLeftMotor->run(FORWARD);

        delay(TurningDelay);

        FrontRightMotor->setSpeed(TurningForward);
        FrontLeftMotor->setSpeed(TurningForward);
        RearRightMotor->setSpeed(TurningForward);
        RearLeftMotor->setSpeed(TurningForward);
        FrontRightMotor->run(FORWARD);
        FrontLeftMotor->run(FORWARD);
        RearRightMotor->run(FORWARD);
        RearLeftMotor->run(FORWARD);

        delay(TurningStraightDelay);

        if (ObstacleRight==HIGH) {
            FrontRightMotor->setSpeed(TurningForward);
            FrontLeftMotor->setSpeed(TurningBackward);
            RearRightMotor->setSpeed(TurningForward);
            RearLeftMotor->setSpeed(TurningBackward);
            FrontRightMotor->run(FORWARD);
            FrontLeftMotor->run(BACKWARD);
            RearRightMotor->run(FORWARD);
            RearLeftMotor->run(BACKWARD);
            delay(OppositeTriggerTurnDelay);
            FrontRightMotor->setSpeed(TurningForward);
            FrontLeftMotor->setSpeed(TurningForward);
            RearRightMotor->setSpeed(TurningForward);
            RearLeftMotor->setSpeed(TurningForward);
            FrontRightMotor->run(FORWARD);
            FrontLeftMotor->run(FORWARD);
            RearRightMotor->run(FORWARD);
            RearLeftMotor->run(FORWARD);
            delay(OppositeTriggerStraightDelay);
        }

        if (ObstacleLeft==HIGH) {
            delay(LastDetectStraightDelay);
            FrontRightMotor->setSpeed(TurningForward);
            FrontLeftMotor->setSpeed(TurningBackward);
            RearRightMotor->setSpeed(TurningForward);
            RearLeftMotor->setSpeed(TurningBackward);
            FrontRightMotor->run(FORWARD);
            FrontLeftMotor->run(BACKWARD);
            RearRightMotor->run(FORWARD);
            RearLeftMotor->run(BACKWARD);
            delay(OppositeTriggerTurnDelay);
            Mode = 1;
            digitalWrite(ChangeRightLED, LOW);
            ChangeCounter = 0;
        }
        */
        digitalWrite(ChangeRightLED, HIGH);
        FrontRightMotor->setSpeed(TurningForward);
        FrontLeftMotor->setSpeed(TurningBackward);
        RearRightMotor->setSpeed(TurningForward);
        RearLeftMotor->setSpeed(TurningBackward);
        FrontRightMotor->run(BACKWARD);
        FrontLeftMotor->run(FORWARD);
        RearRightMotor->run(BACKWARD);
        RearLeftMotor->run(FORWARD);
        delay(TurningDelay);
        FrontRightMotor->setSpeed(TurningForward);
        FrontLeftMotor->setSpeed(TurningBackward);
        RearRightMotor->setSpeed(TurningForward);
        RearLeftMotor->setSpeed(TurningBackward);
        FrontRightMotor->run(FORWARD);
        FrontLeftMotor->run(FORWARD);
        RearRightMotor->run(FORWARD);
        RearLeftMotor->run(FORWARD);
        delay(LaneDelay);
        for (x=0; x<=LaneDetect; x ++) {
            ObstacleLeft = digitalRead(ObstaclePinLeft);
            ObstacleRight = digitalRead(ObstaclePinRight);
            if (ObstacleLeft==HIGH) {
                digitalWrite(LeftLED, HIGH);
            } else if (ObstacleRight==HIGH) {
                digitalWrite(RightLED,HIGH);
            }
            if (ObstacleRight==HIGH) {
            FrontRightMotor->setSpeed(TurningForward);
            FrontLeftMotor->setSpeed(TurningBackward);
            RearRightMotor->setSpeed(TurningForward);
            RearLeftMotor->setSpeed(TurningBackward);
            FrontRightMotor->run(FORWARD);
            FrontLeftMotor->run(BACKWARD);
            RearRightMotor->run(FORWARD);
            RearLeftMotor->run(BACKWARD);
            delay(TurningDelay);
            Mode = 1;
            digitalWrite(ChangeRightLED, LOW);
            ChangeCounter = 0;
            break;
            }
            delay(100);
        }
    } else if (Mode==3) {   /* Change Left */
    /*
        digitalWrite(ChangeLeftLED, HIGH);
        ObstacleLeft = digitalRead(ObstaclePinLeft);
        ObstacleRight = digitalRead(ObstaclePinRight);

        FrontRightMotor->setSpeed(TurningForward);
        FrontLeftMotor->setSpeed(TurningBackward);
        RearRightMotor->setSpeed(TurningForward);
        RearLeftMotor->setSpeed(TurningBackward);
        FrontRightMotor->run(FORWARD);
        FrontLeftMotor->run(BACKWARD);
        RearRightMotor->run(FORWARD);
        RearLeftMotor->run(BACKWARD);

        delay(TurningDelay);

        FrontRightMotor->setSpeed(TurningForward);
        FrontLeftMotor->setSpeed(TurningForward);
        RearRightMotor->setSpeed(TurningForward);
        RearLeftMotor->setSpeed(TurningForward);
        FrontRightMotor->run(FORWARD);
        FrontLeftMotor->run(FORWARD);
        RearRightMotor->run(FORWARD);
        RearLeftMotor->run(FORWARD);

        delay(TurningStraightDelay);

        if (ObstacleLeft==HIGH) {
            FrontRightMotor->setSpeed(TurningBackward);
            FrontLeftMotor->setSpeed(TurningForward);
            RearRightMotor->setSpeed(TurningBackward);
            RearLeftMotor->setSpeed(TurningForward);
            FrontRightMotor->run(BACKWARD);
            FrontLeftMotor->run(FORWARD);
            RearRightMotor->run(BACKWARD);
            RearLeftMotor->run(FORWARD);
            delay(OppositeTriggerTurnDelay);
            FrontRightMotor->setSpeed(TurningForward);
            FrontLeftMotor->setSpeed(TurningForward);
            RearRightMotor->setSpeed(TurningForward);
            RearLeftMotor->setSpeed(TurningForward);
            FrontRightMotor->run(FORWARD);
            FrontLeftMotor->run(FORWARD);
            RearRightMotor->run(FORWARD);
            RearLeftMotor->run(FORWARD);
            delay(OppositeTriggerStraightDelay);
        }

        if (ObstacleRight==HIGH) {
            delay(LastDetectStraightDelay);
            FrontRightMotor->setSpeed(TurningBackward);
            FrontLeftMotor->setSpeed(TurningForward);
            RearRightMotor->setSpeed(TurningBackward);
            RearLeftMotor->setSpeed(TurningForward);
            FrontRightMotor->run(BACKWARD);
            FrontLeftMotor->run(FORWARD);
            RearRightMotor->run(BACKWARD);
            RearLeftMotor->run(FORWARD);
            delay(OppositeTriggerTurnDelay);
            Mode = 1;
            digitalWrite(ChangeLeftLED, LOW);
            ChangeCounter = 0;
        }
    */
        digitalWrite(ChangeLeftLED, HIGH);
        FrontRightMotor->setSpeed(TurningForward);
        FrontLeftMotor->setSpeed(TurningBackward);
        RearRightMotor->setSpeed(TurningForward);
        RearLeftMotor->setSpeed(TurningBackward);
        FrontRightMotor->run(FORWARD);
        FrontLeftMotor->run(BACKWARD);
        RearRightMotor->run(FORWARD);
        RearLeftMotor->run(BACKWARD);
        delay(TurningDelay);
        FrontRightMotor->setSpeed(TurningForward);
        FrontLeftMotor->setSpeed(TurningBackward);
        RearRightMotor->setSpeed(TurningForward);
        RearLeftMotor->setSpeed(TurningBackward);
        FrontRightMotor->run(FORWARD);
        FrontLeftMotor->run(FORWARD);
        RearRightMotor->run(FORWARD);
        RearLeftMotor->run(FORWARD);
        delay(LaneDelay);
        for (x=0; x<=LaneDetect; x ++) {
            ObstacleLeft = digitalRead(ObstaclePinLeft);
            ObstacleRight = digitalRead(ObstaclePinRight);
            if (ObstacleLeft==HIGH) {
                digitalWrite(LeftLED, HIGH);
            } else if (ObstacleRight==HIGH) {
                digitalWrite(RightLED,HIGH);
            }
            if (ObstacleLeft==HIGH) {
            FrontRightMotor->setSpeed(TurningForward);
            FrontLeftMotor->setSpeed(TurningBackward);
            RearRightMotor->setSpeed(TurningForward);
            RearLeftMotor->setSpeed(TurningBackward);
            FrontRightMotor->run(BACKWARD);
            FrontLeftMotor->run(FORWARD);
            RearRightMotor->run(BACKWARD);
            RearLeftMotor->run(FORWARD);
            delay(TurningDelay);
            Mode = 1;
            digitalWrite(ChangeLeftLED, LOW);
            ChangeCounter = 0;
            break;
            }
            delay(100);
        }
    }
}

void StopMotor() {
    FrontRightMotor->run(RELEASE);
    FrontLeftMotor->run(RELEASE);
    RearRightMotor->run(RELEASE);
    RearLeftMotor->run(RELEASE);
}