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

int pinA = 4;                              // Rotary encoder Pin A
int pinB = 3;                              // Rotary encoder Pin B
int pinAstateCurrent = LOW;                // Current state of Pin A
int pinAStateLast = pinAstateCurrent;      // Last read value of Pin A
int number=0; //initial the count 

void setup() {
  Serial.begin (9600);                     // Initialise the serial monitor

  pinMode (switchPin, INPUT_PULLUP);       // Enable the switchPin as input with a PULLUP resistor
  
  pinMode (pinA, INPUT);                   // Set PinA as input
  pinMode (pinB, INPUT);                   // Set PinB as input
  
}

void loop() {

  // BUTTON
  switchState = digitalRead(switchPin);    // Read the digital value of the switch (LOW/HIGH)

  // If the switch is pressed (LOW), print message
  if (switchState == LOW) {
      Serial.println("Switch pressed");
  }
  

  // ROTATION DIRECTION
  pinAstateCurrent = digitalRead(pinA);    // Read the current state of Pin A
  
  // If there is a minimal movement of 1 step
  if ((pinAStateLast == LOW) && (pinAstateCurrent == HIGH)) {
    
    if (digitalRead(pinB) == HIGH) {      // If Pin B is HIGH
      number++;
      Serial.println("Left");             // Print on screen
      Serial.println(number);             // Print on screen
    } else {
      number--;
      Serial.println("Right");            // Print on screen
      Serial.println(number);             // Print on screen
    }
    
  }
  
  pinAStateLast = pinAstateCurrent;        // Store the latest read value in the currect state variable
  
}