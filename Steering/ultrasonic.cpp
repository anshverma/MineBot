#include "Arduino.h"
#include "utility.h"

void ultrasonic::_init(int trigPin, int echoPin){
//  Serial.begin(9600);
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  _trigPin=trigPin;
  _echoPin=echoPin;
}

float ultrasonic::dist(){
  long duration,distance;
  digitalWrite(_trigPin,LOW); //make sure the trigPin starts with a LOW and no wrong reading will be received for every loop
  delayMicroseconds(2);
  digitalWrite(_trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigPin,LOW);
  duration =  pulseIn(_echoPin,HIGH,80000); //Returns the length of the pulse (high level time) in microseconds
  if (duration==0){
    distance = 1000;
  }
  else{
    distance = (double)duration/1000000*340/2*100;  //range = high level time * velocity (340m/s)/2 (in cm)
  }
  
//  Serial.print("distance = ");
//  Serial.print(distance);
//  Serial.println(" cm.");
  return distance;
  
  //delay(150);
}

