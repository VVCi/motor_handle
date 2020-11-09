// PID motor position control.
// Thanks to Brett Beauregard for his nice PID library http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

#include <PinChangeInterrupt.h>
#include "BTS7960.h"
#include <PID_v1.h>
#define encodPinA1      2                       // Quadrature encoder A pin
#define encodPinB1      8                       // Quadrature encoder B pin

const uint8_t EN = 11;

/* Define Mode
  0: Locked Mode
  1: Auto Handling Mode
  2: Remode Handling Mode
*/

/* Define DIR */
/* DIR: Run Forward: R_PWM - Run Reverse: L_PWM */
const uint8_t L_PWM = 9;
const uint8_t R_PWM = 10;

BTS7960 motorController(EN, L_PWM, R_PWM);

double kp = 2000 , ki = 0 , kd = 0.09;             // modify for optimal performance
double input = 0, output = 0, setpoint = 0;
long temp;
volatile long encoderPos = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'

void setup() {
  pinMode(encodPinA1, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);                  // quadrature encoder input B
  attachInterrupt(0, encoder, FALLING);               // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);
  Serial.begin (115200);                              // for debugging
}

void loop() {
  setpoint = analogRead(A0;                       // modify to fit motor and encoder characteristics, potmeter connected to A0
  Serial.println(setpoint);
  input = encoderPos ;                                // data from encoder
  Serial.println(encoderPos);                      // monitor motor position
  myPID.Compute();                                    // calculate new output
  pwmOut(output);                                     // drive L298N H-Bridge module
  Serial.println(output);
}

void pwmOut(int32_t out) {                                // to H-Bridge board
  if (out > 0) {
    motor_run(out, 1);
  }
  else {
    motor_run(abs(out), 0);                       // drive motor CCW
  }
}

void motor_run(int32_t pulse, uint32_t dir ) {
  motorController.Enable();
  if (dir == 1) {
    motorController.TurnLeft(pulse);
  }
  else if (dir == 0) {
    motorController.TurnRight(pulse);
  }
}

void encoder()  {                                     // pulse and direction, direct port reading to save cycles
  if (PINB & 0b00000001)    encoderPos++;             // if(digitalRead(encodPinB1)==HIGH)   count ++;
  else                      encoderPos--;             // if(digitalRead(encodPinB1)==LOW)   count --;
}
