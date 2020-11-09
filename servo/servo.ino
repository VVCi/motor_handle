#include <Encoder.h>
#include "BTS7960.h"
int pin = 7;
unsigned long duration;

int32_t start_angle, end_angle;


/* Define Mode
  0: Locked Mode
  1: Auto Handling Mode
  2: Remode Handling Mode
*/

/* Define DIR */
/* DIR: Run Forward: R_PWM - Run Reverse: L_PWM */
const uint8_t EN = 11;
const uint8_t L_PWM = 9;
const uint8_t R_PWM = 10;

const int buttonPin1 = 24;
const int buttonPin2 = 26;

int buttonState1 = 0;
int buttonState2 = 0;
int lastButtonState1 = 0;
int lastButtonState2 = 0;

BTS7960 motorController(EN, L_PWM, R_PWM);

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(2, 8);
//   avoid using pins with LEDs attached
long oldPosition  = -999;
long newPosition;
void setup() {
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
  
}

void loop() {
  newPosition = myEnc.read();
  end_angle = newPosition / 1000;

  buttonState1 = digitalRead(buttonPin1);
  buttonState2 = digitalRead(buttonPin2);

  if(buttonState1 != lastButtonState1){
    if (buttonState1 == LOW){
      servo_handle(end_angle, 0, 22, 255);
      end_angle = 0;
    }
  }

  if(buttonState2 != lastButtonState2){
    if (buttonState2 == LOW){
      servo_handle(end_angle, 0, -22, 255);
      end_angle = 0;
    }
  }
  
  lastButtonState1 = buttonState1;
  lastButtonState2 = buttonState2; 
}

void motor_run( uint32_t pulse, uint32_t dir ) {
  motorController.Enable();
  if (dir == 1) {
    motorController.TurnLeft(pulse);
  }
  else if (dir == 0) {
    motorController.TurnRight(pulse);
  }
}

void servo_handle(int32_t end_angle, int32_t start, int32_t ended, uint32_t pulse) {
  uint32_t dir;
  if (ended - start > 0) {
    //dir = 1;
    if (end_angle >= start && end_angle < ended) {
      motor_run(pulse, 1);
    }

    if (end_angle >= ended){
      motor_run (0, 1);
    }  
  }

  if (ended - start < 0) {
    //dir = 0;
    if (end_angle <= start && end_angle > ended) {
      motor_run(pulse, 0);
    }

    if (end_angle <= ended){
      motor_run(0, 0);
    }

    end_angle = 0;
  }

  
}
