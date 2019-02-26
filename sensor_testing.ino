/*

    Robotic Systems Class, Ultrasonic Set
    Hafidh Satyanto

    Will use 6 ultrasonic sensors.
    Each ultrasonic sensor is connected by:
        Sensor GND = Board GND
        Sensor VCC = Board 5V
        Sensor Echo = Digital I/O
        Sensor Trig = Digital I/O

    Direction Control - Want to keep at 0.
                        If turning right, then Direction + 1.
                        If turning left, then Direction - 1.

    Has different modes: 
        1. Drive Straight   -   will drive straight until front sensor triggers at 10cm
        2. Turning Right    -   will use only left motors to turn at near 45 degree angle until
                                RightFront reaches 10cm. Once RightFront reaches 10cm, continue until
                                RightRear also reaches 10cm, then moves to Right Tracking mode.
        3. Turning Left     -   same as turning right but left.
        4. Tracking Right   -   will drive straight as long as the RightFront and RightRear are less
                                than ±2cm deviating from 10cm. Both motor sides will move, but the front
                                and rear will have differing speeds to adjust direction while maintaining
                                speed.
        5. Tracking Left    -   same as tracking right but left.
        6. Reverse          -   

*/

/* Import Motor Library */
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *RearLeftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *RearRightMotor = AFMS.getMotor(2);
Adafruit_DCMotor *FrontRightMotor = AFMS.getMotor(3);
Adafruit_DCMotor *FrontLeftMotor = AFMS.getMotor(4);

/* Set speed goes from 0 to 255 */

/* Front Ultrasonic */
const int FrontTrig = 31;
const int FrontEcho = 30;
int FrontDistance;

/* Rear Ultrasonic */
const int RearTrig = 43;
const int RearEcho = 42;
int RearDistance;

/* Right Front Ultrasonic */

const int RightFrontTrig = 51;
const int RightFrontEcho = 50;
int RightFrontDistance;

/* Right Rear Ultrasonic */
const int RightRearTrig = 47;
const int RightRearEcho = 46;
int RightRearDistance;

/* Left Front Ultrasonic */
const int LeftFrontTrig = 35;
const int LeftFrontEcho = 34;
int LeftFrontDistance;

/* Left Rear Ultrasonic */
const int LeftRearTrig = 39;
const int LeftRearEcho = 38;
int LeftRearDistance;

int Mode1LED = 22;
int Mode2LED = 23;
int Mode3LED = 41;
int Mode4LED = 45;
int Mode5LED = 53;

int FrontDetectionDistance = 25;    /* Used by straight mode and tracking mode for obstacle detection */
int TurningInPlaceDistance = 20;    /* Used by turning mode to reverse wheels and not advance */
int TrackingStartDistance = 15;     /* Used by RightRear & LeftRear to start tracking mode out of turning mode */
int FrontTrackingStartAngleDistance = 20;   /* Used by turning mode to make sure that the front angle is not too close and PREVENT START TRACKING THE CORNER */
int TrackingDropOffDistance = 30;   /* Used by tracking mode to detect end of wall (or obstacle) */
int TurningPreventTrackingDistance = 35;    /* While turning, if the FrontDistance is less than this value, then DONT START TRACKING THE CORNER */ 
int DropOffTurnDelay = 2400;
int DropOffStraightDelay = 1000;

int SonarDistance, SonarDuration;

int Mode = 1;
int Direction = 0;

void setup() {
    Serial.begin(9600);

    AFMS.begin();

    pinMode(FrontTrig, OUTPUT);
    pinMode(FrontEcho, INPUT);

    pinMode(RightFrontTrig, OUTPUT);
    pinMode(RightFrontEcho, INPUT);

    pinMode(RightRearTrig, OUTPUT);
    pinMode(RightRearEcho, INPUT);

    pinMode(LeftFrontTrig, OUTPUT);
    pinMode(LeftFrontEcho, INPUT);

    pinMode(LeftRearTrig, OUTPUT);
    pinMode(LeftRearEcho, INPUT);

    //pinMode(RearTrig, OUTPUT);
    //pinMode(RearEcho, INPUT);
    
    pinMode(Mode1LED, OUTPUT);
    pinMode(Mode2LED, OUTPUT);
    pinMode(Mode3LED, OUTPUT);
    pinMode(Mode4LED, OUTPUT);
    pinMode(Mode5LED, OUTPUT);

    StopMotor();
    Mode = 1;
    Direction = 0;
    delay(1000);
    Mode = 1;
    Direction = 0;
    delay(200);

}

void loop() {

    Serial.print("Front Distance: ");
    Serial.println(FrontDistance);
    Serial.print("Rear Distance: ");
    //Serial.println(RearDistance);
    //Serial.print("RightFront Distance: ");
    Serial.println(RightFrontDistance);
    Serial.print("RightRear Distance: ");
    Serial.println(RightRearDistance);
    Serial.print("LeftFront Distance: ");
    Serial.println(LeftFrontDistance);
    Serial.print("RightRear Distance: ");
    Serial.println(RightRearDistance);
    
    if (Mode==1) {      /* Driving Straight Forward */
        digitalWrite(Mode1LED, HIGH);
        SonarSensor(FrontTrig, FrontEcho);
        FrontDistance = SonarDistance;
        SonarSensor(RightFrontTrig, RightFrontEcho);
        RightFrontDistance = SonarDistance;
        SonarSensor(LeftFrontTrig, LeftFrontEcho);
        LeftFrontDistance = SonarDistance;

        if (FrontDistance<=FrontDetectionDistance) {
            if (RightFrontDistance < LeftFrontDistance) {
                Mode = 3;   /* If RightFrontDistance is closer than LeftFrontDistance, turn Left. */
                digitalWrite(Mode1LED, LOW);
            }
            else if (RightFrontDistance > LeftFrontDistance) {
                Mode = 2;   /* If Left is closer than Right, turn Right. */
                digitalWrite(Mode1LED, LOW);
            }
        }
        else {
            FrontRightMotor->setSpeed(64);
            FrontLeftMotor->setSpeed(64);
            RearRightMotor->setSpeed(64);
            RearLeftMotor->setSpeed(64);
            FrontRightMotor->run(FORWARD);
            FrontLeftMotor->run(FORWARD);
            RearRightMotor->run(FORWARD);
            RearLeftMotor->run(FORWARD);
        }
    }
    else if (Mode==2) {  /* Turning Right */
        digitalWrite(Mode2LED, HIGH);
        SonarSensor(FrontTrig, FrontEcho);
        FrontDistance = SonarDistance;
        SonarSensor(LeftFrontTrig, LeftFrontEcho);
        LeftFrontDistance = SonarDistance;
        SonarSensor(LeftRearTrig, LeftRearEcho);
        LeftRearDistance = SonarDistance;

        Direction = Direction + 1;
        if (LeftFrontDistance<=TurningInPlaceDistance) {
            FrontRightMotor->setSpeed(85);
            FrontLeftMotor->setSpeed(85);
            RearRightMotor->setSpeed(85);
            RearLeftMotor->setSpeed(85);
            FrontRightMotor->run(BACKWARD);
            FrontLeftMotor->run(FORWARD);
            RearRightMotor->run(BACKWARD);
            RearLeftMotor->run(FORWARD);
            delay(250);
        }
        else if (LeftFrontDistance>TurningInPlaceDistance) {
            FrontRightMotor->setSpeed(10);
            FrontLeftMotor->setSpeed(85);
            RearRightMotor->setSpeed(10);
            RearLeftMotor->setSpeed(85);
            FrontRightMotor->run(BACKWARD);
            FrontLeftMotor->run(FORWARD);
            RearRightMotor->run(BACKWARD);
            RearLeftMotor->run(FORWARD);
            delay(100);
        }
        if (LeftRearDistance<=TrackingStartDistance && FrontDistance>=TurningPreventTrackingDistance && LeftFrontDistance<=FrontTrackingStartAngleDistance) {
            digitalWrite(Mode2LED, LOW);
            Mode = 5;   /* After RearDistance is correct distance to the left wall, track left. */
        }
    }
    else if (Mode==3) {  /* Turning Left */
        digitalWrite(Mode3LED, HIGH);
        SonarSensor(FrontTrig, FrontEcho);
        FrontDistance = SonarDistance;
        SonarSensor(RightFrontTrig, RightFrontEcho);
        RightFrontDistance = SonarDistance;
        SonarSensor(RightRearTrig, RightRearEcho);
        RightRearDistance = SonarDistance;

        Direction = Direction - 1;
        if (RightFrontDistance<=TurningInPlaceDistance) {
            FrontRightMotor->setSpeed(85);
            FrontLeftMotor->setSpeed(85);
            RearRightMotor->setSpeed(85);
            RearLeftMotor->setSpeed(85);
            FrontRightMotor->run(FORWARD);
            FrontLeftMotor->run(BACKWARD);
            RearRightMotor->run(FORWARD);
            RearLeftMotor->run(BACKWARD);
            delay(250);
        }
        else if (RightFrontDistance>TurningInPlaceDistance) {
            FrontRightMotor->setSpeed(85);
            FrontLeftMotor->setSpeed(10);
            RearRightMotor->setSpeed(85);
            RearLeftMotor->setSpeed(10);
            FrontRightMotor->run(FORWARD);
            FrontLeftMotor->run(BACKWARD);
            RearRightMotor->run(FORWARD);
            RearLeftMotor->run(BACKWARD);
            delay(100);
        } 
        if (RightRearDistance<=TrackingStartDistance && FrontDistance>=TurningPreventTrackingDistance && RightFrontDistance<=FrontTrackingStartAngleDistance) {
            digitalWrite(Mode3LED, LOW);
            Mode = 4;   /* After RearDistance is correct distance to the right wall, track right. */
        } 
    }
    else if (Mode==4) {  /* Tracking right */
        digitalWrite(Mode4LED, HIGH);
        SonarSensor(FrontTrig, FrontEcho);
        FrontDistance = SonarDistance;
        SonarSensor(RightFrontTrig, RightFrontEcho);
        RightFrontDistance = SonarDistance;
        SonarSensor(RightRearTrig, RightRearEcho);
        RightRearDistance = SonarDistance;

        if (Direction>0) {
            Direction = 1;
        } else if (Direction<0) {
            Direction = -1;
        }

        int RightAngle;
        RightAngle = RightFrontDistance-RightRearDistance;

        if (RightFrontDistance>=TrackingDropOffDistance) {    /* Detected a drop-off */
            digitalWrite(Mode4LED, LOW);
            if (Direction > 0) {
                //Mode = 3;
                FrontRightMotor->setSpeed(85);
                FrontLeftMotor->setSpeed(40);
                RearRightMotor->setSpeed(85);
                RearLeftMotor->setSpeed(40);
                FrontRightMotor->run(FORWARD);
                FrontLeftMotor->run(BACKWARD);
                RearRightMotor->run(FORWARD);
                RearLeftMotor->run(BACKWARD);
                delay(DropOffTurnDelay);
                FrontRightMotor->setSpeed(85);
                FrontLeftMotor->setSpeed(85);
                RearRightMotor->setSpeed(85);
                RearLeftMotor->setSpeed(85);
                FrontRightMotor->run(FORWARD);
                FrontLeftMotor->run(FORWARD);
                RearRightMotor->run(FORWARD);
                RearLeftMotor->run(FORWARD);
                delay(DropOffStraightDelay);
                Direction = Direction - 1;
                Mode = 5;
            } else if (Direction < 0) {
                //Mode = 2;
                FrontRightMotor->setSpeed(40);
                FrontLeftMotor->setSpeed(85);
                RearRightMotor->setSpeed(40);
                RearLeftMotor->setSpeed(85);
                FrontRightMotor->run(BACKWARD);
                FrontLeftMotor->run(FORWARD);
                RearRightMotor->run(BACKWARD);
                RearLeftMotor->run(FORWARD);
                delay(DropOffTurnDelay);
                FrontRightMotor->setSpeed(85);
                FrontLeftMotor->setSpeed(85);
                RearRightMotor->setSpeed(85);
                RearLeftMotor->setSpeed(85);
                FrontRightMotor->run(FORWARD);
                FrontLeftMotor->run(FORWARD);
                RearRightMotor->run(FORWARD);
                RearLeftMotor->run(FORWARD);
                delay(DropOffStraightDelay);
                Direction = Direction + 1;
                Mode = 4;
            } else if (Direction == 0) {
                Mode = 1;
            }
        }
        else if (FrontDistance<=FrontDetectionDistance) {
            digitalWrite(Mode4LED, LOW);
            Mode = 1;
        } else if (FrontDistance>FrontDetectionDistance) {
            if (RightAngle>=1) {
                FrontRightMotor->setSpeed(0);
                FrontLeftMotor->setSpeed(64);
                RearRightMotor->setSpeed(0);
                RearLeftMotor->setSpeed(64);
                FrontRightMotor->run(RELEASE);
                FrontLeftMotor->run(FORWARD);
                RearRightMotor->run(RELEASE);
                RearLeftMotor->run(FORWARD);
            }
            else if (RightAngle<=-1) {
                FrontRightMotor->setSpeed(64);
                FrontLeftMotor->setSpeed(0);
                RearRightMotor->setSpeed(64);
                RearLeftMotor->setSpeed(0);
                FrontRightMotor->run(FORWARD);
                FrontLeftMotor->run(RELEASE);
                RearRightMotor->run(FORWARD);
                RearLeftMotor->run(RELEASE);
            }
            else {
                FrontRightMotor->setSpeed(64);
                FrontLeftMotor->setSpeed(64);
                RearRightMotor->setSpeed(64);
                RearLeftMotor->setSpeed(64);
                FrontRightMotor->run(FORWARD);
                FrontLeftMotor->run(FORWARD);
                RearRightMotor->run(FORWARD);
                RearLeftMotor->run(FORWARD);
            }
        }
    }
    else if (Mode==5) { /* Tracking left */
        digitalWrite(Mode5LED, HIGH);
        SonarSensor(FrontTrig, FrontEcho);
        FrontDistance = SonarDistance;
        SonarSensor(LeftFrontTrig, LeftFrontEcho);
        LeftFrontDistance = SonarDistance;
        SonarSensor(LeftRearTrig, LeftRearEcho);
        LeftRearDistance = SonarDistance;

        if (Direction>0) {
            Direction = 1;
        } else if (Direction<0) {
            Direction = -1;
        }

        int LeftAngle;
        LeftAngle = LeftFrontDistance-LeftRearDistance;

        if (LeftFrontDistance>=TrackingDropOffDistance) {    /* Detected a drop-off while tracking */
            digitalWrite(Mode5LED, LOW);
            if (Direction > 0) {
                //Mode = 3;
                FrontRightMotor->setSpeed(85);
                FrontLeftMotor->setSpeed(40);
                RearRightMotor->setSpeed(85);
                RearLeftMotor->setSpeed(40);
                FrontRightMotor->run(FORWARD);
                FrontLeftMotor->run(BACKWARD);
                RearRightMotor->run(FORWARD);
                RearLeftMotor->run(BACKWARD);
                delay(DropOffTurnDelay);
                FrontRightMotor->setSpeed(85);
                FrontLeftMotor->setSpeed(85);
                RearRightMotor->setSpeed(85);
                RearLeftMotor->setSpeed(85);
                FrontRightMotor->run(FORWARD);
                FrontLeftMotor->run(FORWARD);
                RearRightMotor->run(FORWARD);
                RearLeftMotor->run(FORWARD);
                delay(DropOffStraightDelay);
                Direction = Direction - 1;
                Mode = 5;
            } else if (Direction < 0) {
                //Mode = 2;
                FrontRightMotor->setSpeed(40);
                FrontLeftMotor->setSpeed(85);
                RearRightMotor->setSpeed(40);
                RearLeftMotor->setSpeed(85);
                FrontRightMotor->run(BACKWARD);
                FrontLeftMotor->run(FORWARD);
                RearRightMotor->run(BACKWARD);
                RearLeftMotor->run(FORWARD);
                delay(DropOffTurnDelay);
                FrontRightMotor->setSpeed(85);
                FrontLeftMotor->setSpeed(85);
                RearRightMotor->setSpeed(85);
                RearLeftMotor->setSpeed(85);
                FrontRightMotor->run(FORWARD);
                FrontLeftMotor->run(FORWARD);
                RearRightMotor->run(FORWARD);
                RearLeftMotor->run(FORWARD);
                delay(DropOffStraightDelay);
                Direction = Direction + 1;
                Mode = 4;
            } else if (Direction == 0) {
                Mode = 1;
            }
        }
        else if (FrontDistance<=FrontDetectionDistance) {   /* Detected an obstacle while tracking */
            digitalWrite(Mode5LED, LOW);
            Mode = 1;
        } else if (FrontDistance>FrontDetectionDistance) {
            if (LeftAngle>=1) {
                FrontRightMotor->setSpeed(64);
                FrontLeftMotor->setSpeed(0);
                RearRightMotor->setSpeed(64);
                RearLeftMotor->setSpeed(0);
                FrontRightMotor->run(FORWARD);
                FrontLeftMotor->run(RELEASE);
                RearRightMotor->run(FORWARD);
                RearLeftMotor->run(RELEASE);
            }
            else if (LeftAngle<=-1) {
                FrontRightMotor->setSpeed(0);
                FrontLeftMotor->setSpeed(64);
                RearRightMotor->setSpeed(0);
                RearLeftMotor->setSpeed(64);
                FrontRightMotor->run(RELEASE);
                FrontLeftMotor->run(FORWARD);
                RearRightMotor->run(RELEASE);
                RearLeftMotor->run(FORWARD);
            }
            else {
                FrontRightMotor->setSpeed(64);
                FrontLeftMotor->setSpeed(64);
                RearRightMotor->setSpeed(64);
                RearLeftMotor->setSpeed(64);
                FrontRightMotor->run(FORWARD);
                FrontLeftMotor->run(FORWARD);
                RearRightMotor->run(FORWARD);
                RearLeftMotor->run(FORWARD);
            }
        }
    }

}

void StopMotor() {
    FrontRightMotor->run(RELEASE);
    FrontLeftMotor->run(RELEASE);
    RearRightMotor->run(RELEASE);
    RearLeftMotor->run(RELEASE);
}

void SonarSensor(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    SonarDuration = pulseIn(echoPin, HIGH);
    SonarDistance = (SonarDuration/2) / 29.1;
}