/*
          TB6560                  Arduino
           CLK+                     5
           CW+                      2
           EN+                      8 
           CLK-                    GND
           CW-                     GND
           EN-                     GND

  A+ A- B+ B- Connect to Stepper Motor 

  Voltage : 10-35VDC.

  Button: Button 1: 22, Button 2: 26, Button: 
  
 */
const int stepPin = 5;
const int dirPin = 2;
const int enPin = 8;

const int buttonPin1 = 24;
const int buttonPin2 = 26;

int buttonState1 = 0;
int buttonState2 = 0;
int lastButtonState1 = 0;
int lastButtonState2 = 0;

void step_start(int cycle, bool dir);

void setup() {
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, HIGH);

  Serial.begin(9600);
}

void loop() {
  buttonState1 = digitalRead(buttonPin1);
  buttonState2 = digitalRead(buttonPin2);

  if(buttonState1 != lastButtonState1){
    if (buttonState1 == LOW){
      step_start(10, 1);
    }
  }

  if(buttonState2 != lastButtonState2){
    if (buttonState2 == LOW){
      step_start(10, 0);
    }
  }
  
  lastButtonState1 = buttonState1;
  lastButtonState2 = buttonState2;
}

void step_start(int cycle, bool dir) {
  if (dir == 1) {
    digitalWrite(enPin, LOW);
    digitalWrite(dirPin, HIGH);
    for (int x = 0; x < 400 * cycle; x++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
    }
  }

  if (dir == 0) {
    digitalWrite(enPin, LOW);
    digitalWrite(dirPin, LOW);
    for (int x = 0; x < 400 * cycle; x++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
    }
  }
}
