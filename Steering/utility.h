#ifndef utility_h
#define utility_h
#include "Wire.h"

void motor_init();
void forward(int pwm);
void reverse(int pwm);
void turnLeft(int pwm);
void turnRight(int pwm);
void alignment(int pwm_L,int pwm_R);
void Stop();

class ultrasonic
{
  int _trigPin;
  int _echoPin;
  public:
    void _init(int trigPin, int echoPin);
    float dist();
};
#endif

void mpr121_init();
int* readTouchInputs();
int num_of_mine_detected(int* readings);
float* mine_position(int* readings, int num);
float* fwd_distance(int *readings, int num);
void mpr121_setup();
void set_register(int address, unsigned char r, unsigned char v);




