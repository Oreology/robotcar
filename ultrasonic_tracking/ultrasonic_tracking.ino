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

/* Direction */
int Direction = 0;          

/* Front Ultrasonic */
const int FrontTrig = 31;
const int FrontEcho = 30;
long FrontDuration;
int FrontDistance;

/* Right Front Ultrasonic */

const int RightFrontTrig = 51;
const int RightFrontEcho = 50;
long RightFrontDuration;
int RightFrontDistance;

/* Right Rear Ultrasonic */
const int RightRearTrig = 47;
const int RightRearEcho = 46;
long RightRearDuration;
int RightRearDistance;

/* Left Front Ultrasonic */
const int LeftFrontTrig = 35;
const int LeftFrontEcho = 34;
long LeftFrontDuration;
int LeftFrontDistance;


/* Left Rear Ultrasonic */
const int LeftRearTrig = 39;
const int LeftRearEcho = 38;
long LeftRearDuration;
int LeftRearDistance;

/* Rear Ultrasonic */
const int RearTrig = 43;
const int RearEcho = 42;
long RearDuration;
int RearDistance;

int Mode = 1;
/*
    1 = Drive Straight
    2 = Turning Right
    3 = Turning Left
    4 = Tracking Right
    5 = Tracking Left
    6 = Reverse 

*/


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

    pinMode(RearTrig, OUTPUT);
    pinMode(RearEcho, INPUT);
}


void loop() {
    long FrontDuration, FrontDistance;
    long RightFrontDuration, RightFrontDistance;
    long RightRearDuration, RightRearDistance;
    long LeftFrontDuration, LeftFrontDistance;
    long LeftRearDuration, LeftRearDistance;
    long RearDuration, RearDistance;

    /* Set Trig Pin to LOW and then HIGH */
    digitalWrite(FrontTrig, LOW);
    digitalWrite(RightFrontTrig, LOW);
    digitalWrite(RightRearTrig, LOW);
    digitalWrite(LeftFrontTrig, LOW);
    digitalWrite(LeftRearTrig, LOW);
    digitalWrite(RearTrig, LOW);
    delayMicroseconds(2);
    digitalWrite(FrontTrig, HIGH);
    digitalWrite(RightFrontTrig, HIGH);
    digitalWrite(RightRearTrig, HIGH);
    digitalWrite(LeftFrontTrig, HIGH);
    digitalWrite(LeftRearTrig, HIGH);
    digitalWrite(RearTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(FrontTrig, LOW);
    digitalWrite(RightFrontTrig, LOW);
    digitalWrite(RightRearTrig, LOW);
    digitalWrite(LeftFrontTrig, LOW);
    digitalWrite(LeftRearTrig, LOW);

    /* Measuring Duration from the Echo pins */
    FrontDuration = pulseIn(FrontEcho, HIGH);
    RightFrontDuration = pulseIn(RightFrontEcho, HIGH);
    RightRearDuration = pulseIn(RightRearEcho, HIGH);
    LeftFrontDuration = pulseIn(LeftFrontEcho, HIGH);
    LeftRearDuration = pulseIn(LeftRearEcho, HIGH);
    RearDuration = pulseIn(RearEcho, HIGH);

    /* Calculate Distance from Duration */
    FrontDistance = (FrontDuration/2)/29.1;
    RightFrontDistance = (RightFrontDuration/2)/29.1;
    RightRearDistance = (RightRearDuration/2)/29.1;
    LeftFrontDistance = (LeftFrontDistance/2)/29.1;
    LeftRearDistance = (LeftRearDistance/2)/29.1;
    RearDistance = (RearDuration/2)/29.1;

    /* Print Distance for Testing Purposes (all in cm)*/
    Serial.print("Front Distance: ");
    Serial.println(FrontDistance);
    Serial.print("RightFront Distance: ");
    Serial.println(RightFrontDistance);
    Serial.print("RightRear Distance: ");
    Serial.print(RightRearDistance);
    Serial.print("LeftFront Distance: ");
    Serial.println(LeftFrontDistance);
    Serial.print("LeftRear Distance: ");
    Serial.println(LeftRearDistance);
    Serial.print("Rear Distance: ");
    Serial.println(RearDistance);

    if (Mode==1) {      /* Driving Straight Forward */
        FrontRightMotor->setSpeed(255);
        FrontLeftMotor->setSpeed(255);
        RearRightMotor->setSpeed(255);
        RearLeftMotor->setSpeed(255);
        FrontRightMotor->run(FORWARD);
        FrontLeftMotor->run(FORWARD);
        RearRightMotor->run(FORWARD);
        RearLeftMotor->run(FORWARD);
        if (FrontDistance<=13) {
            if (RightFrontDistance < LeftFrontDistance) {
                Mode = 3;   /* If RightFrontDistance is closer than LeftFrontDistance, turn Left. */
            }
            else {
                Mode = 2;   /* If Left is closer than Right, turn Right. */
            }
        }
    }
    else if (Mode==2) {  /* Turning Right */
        Direction = Direction + 1;
        if (LeftFrontDistance<=11) {
            FrontRightMotor->setSpeed(255);
            FrontLeftMotor->setSpeed(255);
            RearRightMotor->setSpeed(255);
            RearLeftMotor->setSpeed(255);
            FrontRightMotor->run(BACKWARD);
            FrontLeftMotor->run(FORWARD);
            RearRightMotor->run(BACKWARD);
            RearLeftMotor->run(FORWARD);
        }
        else if (LeftFrontDistance>11) {
            FrontRightMotor->setSpeed(20);
            FrontLeftMotor->setSpeed(255);
            RearRightMotor->setSpeed(20);
            RearLeftMotor->setSpeed(255);
            FrontRightMotor->run(FORWARD);
            FrontLeftMotor->run(FORWARD);
            RearRightMotor->run(FORWARD);
            RearLeftMotor->run(FORWARD);
        }
        if (LeftRearDistance<=11) {
            Mode = 1;   /* After RearDistance is correct distance to the left wall, track left. */
        }
    }
    else if (Mode==3) {  /* Turning Left */
        Direction = Direction - 1;
        if (RightFrontDistance<=11) {
            FrontRightMotor->setSpeed(255);
            FrontLeftMotor->setSpeed(255);
            RearRightMotor->setSpeed(255);
            RearLeftMotor->setSpeed(255);
            FrontRightMotor->run(FORWARD);
            FrontLeftMotor->run(BACKWARD);
            RearRightMotor->run(FORWARD);
            RearLeftMotor->run(BACKWARD);
        }
        else if (RightFrontDistance>11) {
            FrontRightMotor->setSpeed(255);
            FrontLeftMotor->setSpeed(20);
            RearRightMotor->setSpeed(255);
            RearLeftMotor->setSpeed(20);
            FrontRightMotor->run(FORWARD);
            FrontLeftMotor->run(FORWARD);
            RearRightMotor->run(FORWARD);
            RearLeftMotor->run(FORWARD);
        }
        if (RightRearDistance<=11) {
            Mode = 1;   /* After RearDistance is correct distance to the right wall, track right. */
        }
    }
    else if (Mode==4) {  /* Tracking right */
        int RightAngle;
        RightAngle = RightFrontDistance-RightRearDistance;
        if (RightAngle>=0.5) {
            FrontRightMotor->setSpeed(0);
            FrontLeftMotor->setSpeed(255);
            RearRightMotor->setSpeed(20);
            RearLeftMotor->setSpeed(255);
            FrontRightMotor->run(FORWARD);
            FrontLeftMotor->run(FORWARD);
            RearRightMotor->run(BACKWARD);
            RearLeftMotor->run(FORWARD);
        }
        else if (RightAngle<=-0.5) {
            FrontRightMotor->setSpeed(255);
            FrontLeftMotor->setSpeed(0);
            RearRightMotor->setSpeed(255);
            RearLeftMotor->setSpeed(20);
            FrontRightMotor->run(FORWARD);
            FrontLeftMotor->run(FORWARD);
            RearRightMotor->run(FORWARD);
            RearLeftMotor->run(BACKWARD);
        }
        else {
            FrontRightMotor->setSpeed(255);
            FrontLeftMotor->setSpeed(255);
            RearRightMotor->setSpeed(255);
            RearLeftMotor->setSpeed(255);
            FrontRightMotor->run(FORWARD);
            FrontLeftMotor->run(FORWARD);
            RearRightMotor->run(FORWARD);
            RearLeftMotor->run(FORWARD);
        }
        if (FrontDistance>=50) {    /* Detected a drop-off */
            Mode = 1;
        }
        else if (FrontDistance<=12) {
            Mode = 1;
        }
    }
    else if (Mode==5) { /* Tracking left */
        int LeftAngle;
        LeftAngle = LeftFrontDistance-LeftRearDistance;
        if (LeftAngle>=0.5) {
            FrontRightMotor->setSpeed(255);
            FrontLeftMotor->setSpeed(0);
            RearRightMotor->setSpeed(255);
            RearLeftMotor->setSpeed(20);
            FrontRightMotor->run(FORWARD);
            FrontLeftMotor->run(FORWARD);
            RearRightMotor->run(FORWARD);
            RearLeftMotor->run(BACKWARD);
        }
        else if (LeftAngle<=-0.5) {
            FrontRightMotor->setSpeed(0);
            FrontLeftMotor->setSpeed(255);
            RearRightMotor->setSpeed(20);
            RearLeftMotor->setSpeed(255);
            FrontRightMotor->run(FORWARD);
            FrontLeftMotor->run(FORWARD);
            RearRightMotor->run(BACKWARD);
            RearLeftMotor->run(FORWARD);
        }
        else {
            FrontRightMotor->setSpeed(255);
            FrontLeftMotor->setSpeed(255);
            RearRightMotor->setSpeed(255);
            RearLeftMotor->setSpeed(255);
            FrontRightMotor->run(FORWARD);
            FrontLeftMotor->run(FORWARD);
            RearRightMotor->run(FORWARD);
            RearLeftMotor->run(FORWARD);
        }
        if (FrontDistance>=50) {    /* Detected a drop-off while tracking */
            Mode = 1;
        }
        else if (FrontDistance<=12) {   /* Detected an obstacle while tracking */
            Mode = 1;
        }
    }

    delay(50); /* Half a second update time */


}
