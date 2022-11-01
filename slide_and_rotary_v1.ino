#include <BfButton.h>

#define LED 13
#define PIN_SLIDE A0  

#define BUTTON_PIN 3 //GPIO #3-Push button on encoder
#define DT 4 //GPIO #4-DT on encoder (Output B)
#define CLK 5 //GPIO #5-CLK on encoder (Output A)
BfButton btn(BfButton::STANDALONE_DIGITAL, BUTTON_PIN, true, LOW);
 
int counter = 0;
int angle = 0; 
int aState;
int aLastState;  
int buttonState = 0;         // variable for reading the pushbutton status

//Button press handling function
void pressHandler (BfButton *btn, BfButton::press_pattern_t pattern) {
  switch (pattern) {
    case BfButton::SINGLE_PRESS:
      Serial.println("Single push");
      break;
      
    case BfButton::DOUBLE_PRESS:
      Serial.println("Double push");
      break;
      
    case BfButton::LONG_PRESS:
      Serial.println("Long push");
      break;
  }
}


void setup() {
  Serial.begin(9600);
  Serial.println(angle);
  
  pinMode(BUTTON, INPUT);
  pinMode(PIN_SLIDE, INPUT );
  pinMode(CLK,INPUT_PULLUP);
  pinMode(DT,INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  
  aLastState = digitalRead(CLK);

  //Button settings
  btn.onPress(pressHandler)
  .onDoublePress(pressHandler) // default timeout
  .onPressFor(pressHandler, 1000); // custom timeout for 1 second
}

void loop() {

  //SLIDE
  int value_slide_pot_a = analogRead(PIN_SLIDE);
  Serial.print("Slide Pot value: ");
  Serial.println(value_slide_pot_a);

  //ROTARY SWITCH
  //Wait for button press to execute commands
  btn.read();
  
  aState = digitalRead(CLK);
 
  //Encoder rotation tracking
  if (aState != aLastState){     
     if (digitalRead(DT) != aState) { 
       counter ++;
       angle ++;
     }
     else {
       counter--;
       angle --;
     }
     if (counter >=100 ) {
       counter =100;
     }
     if (counter <=-100 ) {
       counter =-100;
     }
     Serial.println(counter); 
  }   
  aLastState = aState;
}
