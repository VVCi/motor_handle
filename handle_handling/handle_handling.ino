#include "BTS7960.h"
const uint8_t EN = 8;

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
int pin = 7;
unsigned long duration;
uint32_t pulse1, pulse2;

void setup() {
  Serial.begin(9600);
  pinMode(pin, INPUT);
}

void loop() {
  /* Min: 1063 Max: 1890*/
  duration = pulseIn(pin, HIGH);
  if (duration >= 1050 && duration <= 1470) {
    pulse1 = map (duration, 1470, 1050, 0, 255);
    Serial.println(pulse1);
    motor_run(pulse1, 1);
  }

  else if (duration >= 1490 && duration <= 1910) {
    pulse2 = map (duration, 1490, 1910, 0, 255);
    Serial.println(pulse2);
    motor_run(pulse2, 0);
  }

  else motorController.Disable();
  

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

void signal_handle() {
  if (duration >= 1050 && duration <= 1470) {
    pulse1 = map (duration, 1470, 1050, 0, 255);
    Serial.println(pulse1);
    motor_run(pulse1, 1);
  }

  if (duration >= 1490 && duration <= 1910) {
    pulse2 = map (duration, 1490, 1910, 0, 255);
    Serial.println(pulse2);
    motor_run(pulse2, 0);
  }

  else{
    motorController.Disable();
  }
}
