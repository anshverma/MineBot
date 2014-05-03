#include "Arduino.h"
#include "utility.h"

#define R_PWM 6
#define R2 A1
#define R1 7
//#define STBY A0
#define L1 4
#define L2 A0
#define L_PWM 5

#define PWM_MAX 255
#define PWM_MIN 0
#define ACCEL_DELAY 5

// Using this code, control functions are
// forward(pwm speed)
// reverse(pwm speed)
// turnLeft()
// turnRight()
// Stop();


boolean movingForward;
int motorSpeed = 0;

int forward_L1 = HIGH;
int forward_L2 = LOW;
int reverse_L1 = forward_L2;
int reverse_L2 = forward_L1;
int forward_R1 = reverse_L1;
int forward_R2 = reverse_L2;
int reverse_R1 = forward_R2;
int reverse_R2 = forward_R1;


void motor_init(){
  pinMode(L_PWM, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(R2, OUTPUT);
//  pinMode(STBY, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  
//  digitalWrite(STBY, HIGH);
}

void forward(int pwm){
  if(pwm > PWM_MAX) pwm = PWM_MAX;
  if(pwm < PWM_MIN) pwm = PWM_MIN;
  //  if(!movingForward){
  //    for(int i = motorSpeed; i >= 0; i--){
  //      analogWrite(L_PWM, i);
  //      analogWrite(R_PWM, i);
  //      delay(ACCEL_DELAY);
  //    }
  //    motorSpeed = 0;
  //  }
  digitalWrite(L1, forward_L1);
  digitalWrite(L2, forward_L2);
  digitalWrite(R1, forward_R1);
  digitalWrite(R2, forward_R2);
  //  if(movingForward){
  //    if(pwm > motorSpeed){
  //      for(int i = motorSpeed; i <= pwm; i++){
  //        analogWrite(L_PWM, i);
  //        analogWrite(R_PWM, i);
  //        delay(ACCEL_DELAY);
  //      }
  //    }
  //    else if(pwm < motorSpeed){
  //      for(int i = motorSpeed; i >=pwm; i--){
  //        analogWrite(L_PWM, i);
  //        analogWrite(R_PWM, i);
  //        delay(ACCEL_DELAY);
  //      }
  //    }
  //  }
  //  motorSpeed = pwm;
  //  movingForward = true;
  analogWrite(L_PWM, pwm);
  analogWrite(R_PWM, pwm);
}


void reverse(int pwm){
  if(pwm > PWM_MAX) pwm = PWM_MAX;
  if(pwm < PWM_MIN) pwm = PWM_MIN;
//  if(movingForward){
//    for(int i = motorSpeed; i >= 0; i--){
//      analogWrite(L_PWM, i);
//      analogWrite(R_PWM, i);
//      delay(ACCEL_DELAY);
//    }
//    motorSpeed = 0;
//  }
  digitalWrite(L1, reverse_L1);
  digitalWrite(L2, reverse_L2);
  digitalWrite(R1, reverse_R1);
  digitalWrite(R2, reverse_R2);
//  if(!movingForward){
//    if(pwm > motorSpeed){
//      for(int i = motorSpeed; i <= pwm; i++){
//        analogWrite(L_PWM, i);
//        analogWrite(R_PWM, i);
//        delay(ACCEL_DELAY);
//      }
//    }
//    else if(pwm < motorSpeed){
//      for(int i = motorSpeed; i >=pwm; i--){
//        analogWrite(L_PWM, i);
//        analogWrite(R_PWM, i);
//        delay(ACCEL_DELAY);
//      }
//    }
//  }
//  motorSpeed = pwm;
//  movingForward = false;
  analogWrite(L_PWM, pwm);
  analogWrite(R_PWM, pwm);
}


void turnLeft(int pwm){
  if(pwm > PWM_MAX) pwm = PWM_MAX;
  if(pwm < PWM_MIN) pwm = PWM_MIN;
  digitalWrite(L1, reverse_L1);
  digitalWrite(L2, reverse_L2);
  digitalWrite(R1, forward_R1);
  digitalWrite(R2, forward_R2);
  analogWrite(L_PWM, 0.5*pwm);
  analogWrite(R_PWM, pwm);
}


void turnRight(int pwm){
  if(pwm > PWM_MAX) pwm = PWM_MAX;
  if(pwm < PWM_MIN) pwm = PWM_MIN;
  digitalWrite(L1, forward_L1);
  digitalWrite(L2, forward_L2);
  digitalWrite(R1, reverse_R1);
  digitalWrite(R2, reverse_R2);
  analogWrite(R_PWM, 0.5*pwm);
  analogWrite(L_PWM, pwm);
}


void Stop(){
  digitalWrite(L1, reverse_L1);
  digitalWrite(L2, reverse_L2);
  digitalWrite(R1, reverse_R1);
  digitalWrite(R2, reverse_R2);
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);
}

void alignment(int pwm_L,int pwm_R){
  if(pwm_L > PWM_MAX) pwm_L = PWM_MAX;
  if(pwm_L < -PWM_MAX) pwm_L = -PWM_MAX;
  if (pwm_L >=0){
    digitalWrite(L1, forward_L1);
    digitalWrite(L2, forward_L2);
  }
  else{
    pwm_L = -pwm_L;
    digitalWrite(L1, reverse_L1);
    digitalWrite(L2, reverse_L2);
  }
    
  if(pwm_R > PWM_MAX) pwm_R = PWM_MAX;  
  if(pwm_R < -PWM_MAX) pwm_R = -PWM_MAX;
  if (pwm_R >=0){
    digitalWrite(R1, forward_R1);
    digitalWrite(R2, forward_R2);
  }
  else{
    pwm_R = -pwm_R;
    digitalWrite(R1, reverse_R1);
    digitalWrite(R2, reverse_R2);
  }  

  
  analogWrite(L_PWM, pwm_L);
  analogWrite(R_PWM, pwm_R);
}

