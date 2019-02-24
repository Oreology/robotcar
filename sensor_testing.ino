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
const int FrontTrig = 51;
const int FrontEcho = 50;
int FrontDistance;

/* Rear Ultrasonic */
const int RearTrig = 43;
const int RearEcho = 42;
int RearDistance;

/* Right Front Ultrasonic */

const int RightFrontTrig = 31;
const int RightFrontEcho = 30;
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

int SonarDistance, SonarDuration;

void setup() {
    Serial.begin(9600);

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
    long RearDuration, RearDistance;
    
    SonarSensor(FrontTrig, FrontEcho);
    FrontDistance = SonarDistance;
    SonarSensor(RearTrig, RearEcho);
    RearDistance = SonarDistance;
    SonarSensor(RightFrontTrig, RightFrontEcho);
    RightFrontDistance = SonarDistance;
    SonarSensor(RightRearTrig, RightRearEcho);
    RightRearDistance = SonarDistance;
    SonarSensor(LeftFrontTrig, LeftFrontEcho);
    LeftFrontDistance = SonarDistance;
    SonarSensor(LeftRearTrig, LeftRearTrig);
    LeftRearDistance = SonarDistance;

    Serial.print("Front Distance: ");
    Serial.println(FrontDistance);
    Serial.print("Rear Distance: ");
    Serial.println(RearDistance);
    Serial.print("RightFront Distance: ");
    Serial.println(RightFrontDistance);
    Serial.print("RightRear Distance: ");
    Serial.println(RightRearDistance);
    Serial.print("LeftFront Distance: ");
    Serial.println(LeftFrontDistance);
    Serial.print("RightRear Distance: ");
    Serial.println(RightRearDistance);
    
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