/*
 * Bas on Tech - Rotary Encoder and Interrupts
 * This course is part of the courses on https://arduino-tutorials.net
 *  
 * (c) Copyright 2018-2019 - Bas van Dijk / Bas on Tech
 * This code and course is copyrighted. It is not allowed to use these courses commerically
 * without explicit written approval
 * 
 * YouTube:    https://www.youtube.com/c/BasOnTech
 * Facebook:   https://www.facebook.com/BasOnTechChannel
 * Instagram:  https://www.instagram.com/BasOnTech
 * Twitter:    https://twitter.com/BasOnTech
 * 
 * ---------------------------------------------------------------------------
 *
 * 
 * More info about the Rotary Encoder: 
 * https://en.wikipedia.org/wiki/Rotary_encoder
 * 
 * PIN CONNECTIONS
 *
 * GND --> GND black
 *  +  --> 5V  red
 * SW  --> 12  yellow
 * DT  --> 3   green (data)
 * CLK --> 4   blue (clock)
 *
 */

int switchPin = 12;                        // button pin
int switchState = HIGH;                    // button value

static const int buzzerPin = 13;                          // buzzer pin
int buzzerHold=0;                                //

int vabriPin=2;                  //vabriation pin

int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor

int pinA = 4;                              // Rotary encoder Pin A
int pinB = 3;                              // Rotary encoder Pin B
int pinAstateCurrent = LOW;                // Current state of Pin A
int pinAStateLast = pinAstateCurrent;      // Last read value of Pin A
int numberA=0; //initial the count 
int numberB=0; //initial the count 
int numberC=0; //initial the count 
int numberD=0; //initial the count 

char sliderArea;



void setup() {

  int sensorPin = A0;    // select the input pin for the potentiometer
  int ledPin = 13;      // select the pin for the LED
  int sensorValue = 0;  // variable to store the value coming from the sensor
 
 // declare the ledPin, buzzerPin as an OUTPUT:
  pinMode(buzzerPin,OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(vabriPin,OUTPUT);

  Serial.begin (9600);                     // Initialise the serial monitor

  pinMode (switchPin, INPUT_PULLUP);       // Enable the switchPin as input with a PULLUP resistor
  
  pinMode (pinA, INPUT);                   // Set PinA as input
  pinMode (pinB, INPUT);                   // Set PinB as input
  
}

void loop() {
   // SLIDER
  sensorValue = analogRead(sensorPin);
  if(sensorValue<256){
    //Serial.println('A'); 
    sliderArea='A';}
  else if(sensorValue>=256 && sensorValue<512){
    //Serial.println('B');
    sliderArea='B';}
  else if(sensorValue>=512&& sensorValue<768){
    //Serial.println('C');
    sliderArea='C';}
  else{
    //Serial.println('D');
    sliderArea='D';}
  //Serial.println(sensorValue);
  // stop the program for <sensorValue> milliseconds:
  //Serial.println(sliderArea); 


  // BUTTON
  switchState = digitalRead(switchPin);    // Read the digital value of the switch (LOW/HIGH)
  // If the switch is pressed (LOW), print message
  if (switchState == LOW) {
      Serial.println("Switch pressed");
  }
  

 // ROTATION DIRECTION
  pinAstateCurrent = digitalRead(pinA);    // Read the current state of Pin A
  
  if(sliderArea=='A'){
    if ((pinAStateLast == LOW) && (pinAstateCurrent == HIGH)) {
    
      if (digitalRead(pinB) == HIGH) {      // If Pin B is HIGH
      numberA++;
      Serial.println("Left");             // Print on screen
      //Serial.println(number);             // Print on screen
      } else {
      numberA--;
      Serial.println("Right");            // Print on screen
      //Serial.println(number);             // Print on screen
      }
    }
  }  
  // If there is a minimal movement of 1 step

   if(sliderArea=='B'){
    if ((pinAStateLast == LOW) && (pinAstateCurrent == HIGH)) {
    
      if (digitalRead(pinB) == HIGH) {      // If Pin B is HIGH
      numberB++;
      Serial.println("Left");             // Print on screen
      //Serial.println(number);             // Print on screen
      } else {
      numberB--;
      Serial.println("Right");            // Print on screen
      //Serial.println(number);             // Print on screen
      }
    }
  }
  // If there is a minimal movement of 1 step

  if(sliderArea=='C'){
    if ((pinAStateLast == LOW) && (pinAstateCurrent == HIGH)) {
    
      if (digitalRead(pinB) == HIGH) {      // If Pin B is HIGH
      numberC++;
      Serial.println("Left");             // Print on screen
      //Serial.println(number);             // Print on screen
      } else {
      numberC--;
      Serial.println("Right");            // Print on screen
      //Serial.println(number);             // Print on screen
      }
    }
  }
  // If there is a minimal movement of 1 step


  if ((pinAStateLast == LOW) && (pinAstateCurrent == HIGH)) {
    
    if (digitalRead(pinB) == HIGH) {      // If Pin B is HIGH
      numberD++;
      Serial.println("Left");             // Print on screen
      Serial.println(numberD);             // Print on screen
      } else {
      numberD--;
      Serial.println("Right");            // Print on screen
      Serial.println(numberD);             // Print on screen
      }
    }
  
  // If there is a minimal movement of 1 step
  
  pinAStateLast = pinAstateCurrent;        // Store the latest read value in the currect state variable

  if(sliderArea=='A'){Serial.print("A:");            // Print on screen
      Serial.println(numberA); 
      }
  else if(sliderArea=='B'){Serial.print("B:");            // Print on screen
      Serial.println(numberB); 
      }
  else if(sliderArea=='C'){Serial.print("C:");            // Print on screen
      Serial.println(numberC); 
      }
  else{Serial.print("D:");            // Print on screen
      Serial.println(numberD); 
      }  

  
  
  //buzzer:
  if(sensorValue==125|| sensorValue==384 || sensorValue==640 || sensorValue==896){
    beep();
    
  }   
}

void beep(){
  //if(sensorValue==buzzerHold)
  digitalWrite(vabriPin,HIGH);
  delay(50);
  digitalWrite(vabriPin,LOW);
  delay(50);
  tone(buzzerPin,196,200);//give a tone to the buzzerPin, at 960Hz for 200ms.
  buzzerHold=sensorValue;   
}