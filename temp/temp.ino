int switchPin = 12;
int switchState = HIGH;
int pinA = 4;
int pinB = 3;
int pinAstateCurrent = LOW;
int pinAStateLast = pinAstateCurrent;
void setup() {
  Serial.begin (9600);
  pinMode (switchPin, INPUT_PULLUP);
  pinMode (pinA, INPUT); 
  pinMode (pinB, INPUT);
}

void loop() {
  switchState = digitalRead(switchPin);
  if (switchState == LOW) {
    Serial.println("Switch pressed");
  } 
  pinAstateCurrent = digitalRead(pinA);
  if ((pinAStateLast == LOW) && (pinAstateCurrent == HIGH)) {
    if（digitalRead(pinB)==HIGH）{
      Serial.println("Left");}
    else {Serial.printin("Right");}
  }

  pinAStateLast = pinAstateCurrent;
}