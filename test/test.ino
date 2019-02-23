/* Front Ultrasonic */
const int FrontTrig = 52;
const int FrontEcho = 53;
long FrontDuration;
int FrontDistance;


void setup() {
    Serial.begin(9600);

    pinMode(FrontTrig, OUTPUT);
    pinMode(FrontEcho, INPUT);
}


void loop() {
    long FrontDuration, FrontDistance;
    digitalWrite(FrontTrig, LOW);
    delayMicroseconds(2);
    digitalWrite(FrontTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(FrontTrig, LOW);

    FrontDuration = pulseIn(FrontEcho, HIGH);
    FrontDistance = (FrontDuration/2)/29.1;

    Serial.println("Front Distance: ");
    Serial.print(FrontDistance);

}
