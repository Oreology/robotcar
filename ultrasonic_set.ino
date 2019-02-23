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
#include <Adafruit_Motorshield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_Motorshield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *RearLeftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *RearRightMotor = AFMS.getMotor(2);
Adafruit_DCMotor *FrontRightMotor = AFMS.getMotor(3);
Adafruit_DCMotor *FrontLeftMotor = AFMS.getMore(4);

/* Set speed goes from 0 to 255 */

/* Direction */
int Direction = 0;          

/* Front Ultrasonic */
const int FrontTrig = 22;
const int FrontEcho = 23;
long FrontDuration;
int FrontDistance;

/* Right Front Ultrasonic */

const int RightFrontTrig = 3;
const int RightFrontEcho = 4;
long RightFrontDuration;
int RightFrontDistance;

/* Right Rear Ultrasonic */
const int RightRearTrig = 5;
const int RightRearEcho = 6;
long RightRearDuration;
int RightRearDistance;

/* Left Front Ultrasonic */
const int LeftFrontTrig = 7;
const int LeftFrontEcho = 8;
long LeftFrontDuration;
int LeftFrontDistance;


/* Left Rear Ultrasonic */
const int LeftRearTrig = 9;
const int LeftRearEcho = 10;
long LeftRearDuration;
int LeftRearDistance;

/* Rear Ultrasonic */
const int RearTrig = 11;
const int RearEcho = 12;
long RearDuration;
int RearDistance;

const int Mode = 1;
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
    Serial.print(lnRightRearDistance);
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
        if (FrontDistance<=13) {
            if (RightFrontDistance < LeftFrontDistance) {
                Mode = 3;   /* If RightFrontDistance is closer than LeftFrontDistance, turn Left. */
            }
            else {
                Mode = 2;   /* If Left is closer than Right, turn Right. */
            }
        }
    elseif (Mode==2) {  /* Turning Right */
        Direction = Direction + 1;
        if (LeftFrontDistance<=11) {
            FrontRightMotor->setSpeed(-255);
            FrontLeftMotor->setSpeed(255);
            RearRightMotor->setSpeed(-255);
            RearLeftMotor->setSpeed(255);
        }
        elseif (LeftFrontDistance>11) {
            FrontRightMotor->setSpeed(20);
            FrontLeftMotor->setSpeed(255);
            RearRightMotor->setSpeed(20);
            RearLeftMotor->setSpeed(255);
        }
        if (LeftRearDistance<=11) {
            Mode = 5;   /* After RearDistance is correct distance to the left wall, track left. */
        }
    }
    elseif (Mode==3) {  /* Turning Left */
        Direction = Direction - 1;
        if (RightFrontDistance<=11) {
            FrontRightMotor->setSpeed(255);
            FrontLeftMotor->setSpeed(-255);
            RearRightMotor->setSpeed(255);
            RearLeftMotor->setSpeed(-255);
        }
        elseif (RightFrontDistance>11) {
            FrontRightMotor->setSpeed(255);
            FrontLeftMotor->setSpeed(20);
            RearRightMotor->setSpeed(255);
            RearLeftMotor->setSpeed(20);
        }
        if (RightRearDistance<=11) {
            Mode = 4;   /* After RearDistance is correct distance to the right wall, track right. */
        }
    }
    elseif (Mode==4) {  /* Tracking right */
        if (FrontDistance<=11 && FrontDistance>9 && RearDistance<=11 && RearDistance>9) {
            FrontRightMotor->setSpeed(255);
            FrontLeftMotor->setSpeed(255);
            RearRightMotor->setSpeed(255);
            RearLeftMotor->setSpeed(255);
        }
        elseif (FrontDistance>11 && RearDistance<=11) {
            FrontRightMotor->setSpeed()
        } 
    }
    else {

    }

    delay(250); /* Half a second update time */


}