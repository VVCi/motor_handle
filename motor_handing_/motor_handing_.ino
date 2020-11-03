#include "BTS7960.h"
int pin = 7;
unsigned long duration;
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

void setup() 
{
  
}

void loop() 
if

}

void handle(uint16_t mode, uin32_t pulse){
  if (mode == 0){
    motor_stop();
  }

  if (mode == 1){
    motor_start(255, 0);
    motor_run(255, 0);
  }

  if (mode == 2){

  }
}

void motor_start( uint32_t pulse, bool dir){
  motorController.Enable();
  if(dir == 1){
    for(int pulse = 0 ; pulse < 255; pulse+=10){
	    motorController.TurnLeft(pulse);
  	  delay(100);
    }
    break;
  } 
  else if(dir == 0){
    for(int pulse = 0 ; pulse < 255; pulse+=10){
	    motorController.TurnRight(pulse);
  	  delay(100);
    }
    break;
  }
  else{
    break;
  }
}

void motor_run( uint32_t pulse, uint32_t dir ){
  motorController.Enable();
  if(dir == 1){
    motorController.TurnLeft(pulse);
  }
  else if (dir == 0){
    motorController.TurnRight(pulse);
  }
  else{
    break;
  }
}

void motor_stop(){
  motorController.Disable();
}
