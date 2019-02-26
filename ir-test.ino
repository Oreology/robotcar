const int ObstaclePinLeft = 27;  // This is our input pin - left position
const int ObstaclePinRight = 26;  // This is our input pin - right position
int ObstacleLeft = LOW;  // LOW = nothing, go.
int ObstacleRight = LOW;  // HIGH = black, turn / stop

const int LeftLED = 22;
const int ChangeLeftLED = 23;
const int RightLED = 41;  
const int ChangeRightLED = 45;

void setup() {
    pinMode(LeftLED, OUTPUT);
    pinMode(RightLED, OUTPUT);
    pinMode(ChangeLeftLED, OUTPUT);
    pinMode(ChangeRightLED, OUTPUT);
    
    pinMode(ObstaclePinLeft, INPUT);
    pinMode(ObstaclePinRight, INPUT);

    delay(1000);
    digitalWrite(ChangeLeftLED, HIGH);
}

void loop() {
  ObstacleLeft = digitalRead(ObstaclePinLeft);
  ObstacleRight = digitalRead(ObstaclePinRight);

    if (ObstacleLeft==HIGH) {
        digitalWrite(LeftLED, HIGH);
    } else if (ObstacleLeft==LOW) {
        digitalWrite(LeftLED,LOW);
    }
    if (ObstacleRight==HIGH) {
        digitalWrite(RightLED, HIGH);
    } else if (ObstacleRight==LOW) {
        digitalWrite(RightLED,LOW);
    }

    delay(500);
}
