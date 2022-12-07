/*
This is HORINZON project in Final Design Studio 2022-2023, Digital and Interaction Design, Politecnico di Milano. 
The code can be divided into four parts:
   1.Grab:sharp sensor & motor- slider knob can slightly move forward driver's hand.
   2.Slide: potentiometer-slide the slider potentiometer into different area
   3.Rotate: rotary potentiometer-increase and decrease by rotating
   4.Press:knob pressed-change the status
Key words: Arduino Uno, Sharp sensor, motored rotary potentiometer, buzzer, vabiration, L298N motor drive, Protopie communication.
Contributors: Gustav Moorhouse, Jan Ostrówka, Madeleine Kiær, Mojca Fortunat, Ruiyi Liu, Umnah Aslam, Weijian Xu, Xiyu Li.
If you want more information, feel free to contact email: 978439441@qq.com
*/

//Press button
int switchPin = 12;                        // button pin
int switchState = HIGH;                    // button value
int pressCount=0;                          // count the button pressed
int pressChange=0;                         //detect the change of press button (from 1 to 0)

//Feedback
static const int buzzerPin = 13;           // buzzer pin
int vabriPin=2;                            //vabriation pin

//Slider Area
int sensorPin = A0;                        // select the input pin for the potentiometer
int sensorValue = 0;                       // variable to store the value coming from the sensor
int currentSlider=0;                       //var for slider current number.
int sliderArea;                            //multistage for slider， 1=A，2=B，3=C

//Rotate button
int pinA = 4;                              // Rotary encoder Pin A
int pinB = 3;                              // Rotary encoder Pin B
int pinAstateCurrent = LOW;                // Current state of Pin A
char rotateStatus;                         // for serialprint L/0 or R/1
int pinAStateLast = pinAstateCurrent;      // Last read value of Pin A
int numberA=0;                             //initial the count 
int numberB=0;                             //initial the count 
int numberC=0;                             //initial the count 

//For motor in potentiometer:
int in1 = 8;
int in2 = 7;

// For sharp sensor.
const int sharpVoPin = A1;                 // Arduino analog pin A1 connect to sensor Vo.
float sharpvalue[20];
int i=0;
float c ;
float sum ;
int sharpState=0;
int sharpChange=0;
int mappingvaluetosharp;


//for communication for protopie
String message="1";                        // serialprint for position on slider
String temp="0";                           // for sending only once
String message1="1";                       // Swrialprint for press
String temp1="0";                          // for sending only once
String message2="1";                       // Swrialprint for rotate
String temp2="0";                          // for sending only once

// Arduino setup function.
void setup() {
  //get the value from potentiometer
  int sensorPin = A0;                       // select the input pin for the potentiometer
  int sensorValue = 0;                      // variable to store the value coming from the sensor
 
  pinMode(buzzerPin,OUTPUT);                // declare the buzzerPin as an OUTPUT:
  pinMode(vabriPin,OUTPUT);                 // declare the vabirationPin as an OUTPUT:

  Serial.begin (9600);                      // Initialise the serial monitor

  pinMode (switchPin, INPUT_PULLUP);        // Enable the switchPin as input with a PULLUP resistor
  
  pinMode (pinA, INPUT);                    // Set PinA as input,rotatory 
  pinMode (pinB, INPUT);                    // Set PinB as input, rotatory

  // for motor in potentiometer
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	// Turn off motors - Initial state
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
  
}

void loop() {

  // SLIDER
  sensorValue = analogRead(sensorPin);
  //Serial.println(sensorValue);
  if(sensorValue<341){
    sliderArea=1;
    }
  else if(sensorValue>=341 && sensorValue<682){
    sliderArea=2;
    
    }
  else{
    sliderArea=3;
    
    }

  // BUTTON
  switchState = digitalRead(switchPin);    // Read the digital value of the switch (LOW/HIGH)
  if (switchState == LOW ) {
      if(pressChange==HIGH){
        pressCount=1;
        }
      beepBuzz();
  }else{
    pressCount=0;
    }
  pressChange=switchState;
  //communication with protopie
   message1="press||"+String(pressCount);
  if(message1!=temp1) { 
    Serial.println(message1);
    temp1=message1;
    }
  

 // ROTATION DIRECTION
  pinAstateCurrent = digitalRead(pinA);    // Read the current state of Pin A
  
  if(sliderArea==1){
    if ((pinAStateLast == LOW) && (pinAstateCurrent == HIGH)) {
      if (digitalRead(pinB) == HIGH) {      // If Pin B is HIGH
      numberA++;
      rotateStatus='0';                     // 0 is rotate left
      } else {
      numberA--;                            // count the rotate
      rotateStatus='1';                     // 0 is rotate right
      }
      //Serial print for protopie
      message2="rotation||"+String(rotateStatus);
        Serial.println(message2);
    }
  }  

   if(sliderArea==2){
    if ((pinAStateLast == LOW) && (pinAstateCurrent == HIGH)) {
      if (digitalRead(pinB) == HIGH) {      // If Pin B is HIGH
      numberB++;
      rotateStatus='0';
      } else {
      numberB--;
      rotateStatus='1';
      }
      //Serial print for protopie
      message2="rotation||"+String(rotateStatus);
        Serial.println(message2);
    }
  }

  if(sliderArea==3){
    if ((pinAStateLast == LOW) && (pinAstateCurrent == HIGH)) {
      if (digitalRead(pinB) == HIGH) {      // If Pin B is HIGH
      numberC++;
      rotateStatus='0';
      } else {
      numberC--;
      rotateStatus='1';
      }
    }
  }
  pinAStateLast = pinAstateCurrent;        // Store the latest read value in the currect state variable  

  // judge logic for vabrition:
  if((sensorValue>155 && sensorValue<180)|| (sensorValue>497 && sensorValue<527 ) || (sensorValue>838 && sensorValue<868) ){
    if(currentSlider!=sensorValue){ 
      beepVabri();
      currentSlider=sensorValue;
    }
  }

  //for sharp sensor
  int VoRaw = sharpValue();                    // Read the output voltage.
  //printValue("VoRaw", VoRaw);
  float Vo = (5.0 * VoRaw) / 1024.0;          // Compute the output voltage in Volts.
  //printFValue("Vo", Vo, "V");

  /*
  Convert to Distance in units of mm
  // by approximating datasheet graph
  // using equation of form: y = a/x + b
  // and two (x,y) points on the graph:
  // (60mm, 2.02V) and (300mm, 0.435V)
  */
  const float a = 2076;                       //find a,b from internet by model
  const float b = 11;
  float dist = 0;
  if ( VoRaw > b ) {
    dist = a / (VoRaw - b);
  }
  //Serial.println(dist);

  if(dist>4 && dist<9){
    sharpState=1;                              //in the area of sharp working
  }else{
    sharpState=0;
  }

  if (sharpState == 0 ) {
      if(sharpChange==1){
        //Serial.println("motor activated!");
        directionControl();
        /*mappingvaluetosharp=(sensorValue,0,1024,8,4);
        if (mappingvaluetosharp<VoRaw){
          directionControl() ;
        }else {
          directionControl() ;
        }*/
      }
  }
  sharpChange=sharpState;
  


  /*
  //communicate with Unity by Serialprint
  if(sliderArea=='A'){
      //Serial.print("A:");            // Print on screen
      //Serial.println(numberA); 
      Serial.println(String("")+sliderArea+",P"+pressCount+","+rotateStatus+abs(numberA));
      }
  else if(sliderArea=='B'){
      //Serial.print("B:");            // Print on screen
      //Serial.println(numberB); 
      Serial.println(String("")+sliderArea+",P"+pressCount+","+rotateStatus+abs(numberB));
      }
  else if(sliderArea=='C'){
      //Serial.print("C:");            // Print on screen
      //Serial.println(numberC); 
      Serial.println(String("")+sliderArea+",P"+pressCount+","+rotateStatus+abs(numberC));
      }  
    */

 //communication for protopie
  message="position||"+String(sliderArea);
 if(message!=temp) { 
   Serial.println(message);
   temp=message;
   }
}
 


//This funcion is for vabrition when sliding into specific area on slider.
void beepVabri(){
  digitalWrite(vabriPin,HIGH);
  delay(50);
  digitalWrite(vabriPin,LOW);
  delay(50);  
}

//This function is for buzzer when button pressed
void beepBuzz(){
    digitalWrite(buzzerPin,HIGH);
    tone(buzzerPin,196,200);//give a tone to the buzzerPin, at 960Hz for 20ms.
    delay(50);
    digitalWrite(buzzerPin,LOW);
    delay(50);

}

// This function is for stable calculation of sharp signal
float sharpValue(){
  int b=analogRead(sharpVoPin) ;
  if (i<20){
    sharpvalue[i]=b;
    i+=1;
  }else{
    sum=0;
    for (i=0;i<20;i++){
      sum+=sharpvalue[i];
    }
    c=sum/20;
    i=0;
  }
  return c;
}

// This function lets you control spinning direction of motors
void directionControl() {
	// Turn on motor
    digitalWrite(in1, HIGH);
	  digitalWrite(in2, LOW);
	  delay(500);
	// Now change motor directions
    digitalWrite(in1, LOW);
	  digitalWrite(in2, HIGH);
	  delay(500);
	// Turn off motors
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
}
