#include "BTS7960.h"
#include "PinChangeInterrupt.h"
#include "PID_v1.h"

#define encodPinA1  11
#define encodPinB1  12


/* Define Mode
  0: Locked Mode
  1: Auto Handling Mode
  2: Remode Handling Mode
*/

/* Define DIR */
/* DIR: Run Forward: R_PWM - Run Reverse: L_PWM */
const uint8_t EN = 8;
const uint8_t L_PWM = 9;
const uint8_t R_PWM = 10;

/* Define Servo
   A0: Channel A Feedback
   A1: Channel B Feedback
*/

BTS7960 motorController(EN, L_PWM, R_PWM);
int pin = 7;
unsigned long duration;
uint32_t pulse1, pulse2;
/* Setup PID Parameter */

double Kp = 5, Ki = 1, Kd = 0.01;
double input = 0, output = 0, setpoint = 0;
long temp;
volatile long encoderPos = 0;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  setup_servo();
  Serial.begin(115200);
}

void loop() {
  duration = pulseIn(pin, HIGH);
  setpoint = analogRead(0) * 5;
  input = encoderPos;
  myPID.Compute();
  motor_run(output);
}

void motor_run( int pulse) {
  motorController.Enable();
  if (pulse > 0){
    motorController.TurnLeft(pulse);
  }
  else {
    motorController.TurnRight(abs(pulse));
  }
}

void motor_stop() {
  motorController.Disable();
}

void setup_servo() {
  pinMode(encodPinA1, INPUT_PULLUP);
  pinMode(encodPinB1, INPUT_PULLUP);
  attachInterrupt(0, encoder, FALLING);
  TCCR1B = TCCR1B & 0b11111000 | 1;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);
}

void encoder(){
  if( PINB & 0b00000001){
    encoderPos++;
  }
  else{
    encoderPos--;
  }
}

void signal_handle(){
  if (duration >= 1050 && duration <= 1470) {
    pulse1 = map (duration, 1470, 1050, 0, 255);
    Serial.println(pulse1);
  }

  if (duration >= 1490 && duration <= 1910) {
    pulse2 = map (duration, 1490, 1910, 0, 255);
    Serial.println(pulse2);
  }

  else{
    motorController.Disable();
  }
}
