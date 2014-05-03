#include "Arduino.h"
#include "utility.h"
#include "mpr121.h"

#define irqpin 2  // Digital 2
boolean touchStates[12]; //to keep track of the previous touch states
static float pos[]={-28.6,-23.9,-20.1,-14.3,-9.5,-4.7,0,4.8,9.8,14.6,19.4,24.2}; //cm
static float pos_for_offset[] = {5.89,16.78,21.18,25.46,27.61,28.82,29.20,28.20,27.51,25.29,21.82,16.34}; //cm



void mpr121_init(){
  pinMode(irqpin, INPUT);
  digitalWrite(irqpin, HIGH); //enable pullup resistor
  
  //Serial.begin(9600);
  Wire.begin();

  mpr121_setup();
}



int* readTouchInputs(){
  static int a[12];  //array to store individual binary codes for sensing probes

  float x;     //position of the mine ( origin at the center of robot, a[i] equally spaced)
  int n=0; //number of 1s (number of postitions detecting mine)
  float sum=0; //sum of the coordinates of a[i]
    
  Wire.requestFrom(0x5A,2); 
    
    
  byte LSB = Wire.read();
  byte MSB = Wire.read();
    
   
  uint16_t touched = ((MSB << 8) | LSB); 

  for (int i=0; i < 12; i++){ 
    if(touched & (1<<i)) a[i]=1;
    else a[i]=0;
  }   
  return a;    
}


int num_of_mine_detected(int *readings){
  int num=0;     //number of mines detected (effective if spacing >=4)
  int i=0;
  int j=0;
  
  while (i<12){
    if (*(readings+i)==1){
      num=num+1;
      i=i+5;
    }
    else{
      i++;
    }
  }
  
  return num;

}


float* mine_position(int *readings, int num){
  static float x[12];  //position array of the mine (-5~6, origin at the center of robot, a[i] equally spaced, spacing =1 and readings[0] is at x=6)
  int i=0;
  int index=0;
  int n;   //number of 1s (number of postitions detecting mine)
  float sum;  //sum of the coordinates of a[i]
  
  
  while (i<12){
    if (*(readings+i)==1){
      n=0;
      sum=0;
      for (int j=i;j<i+5 && j<12;j++){
        n=n+*(readings+j);
        sum=sum+(*(pos+j))*(*(readings+j));
        //sum=sum+(6-j)*(*(readings+j)); 
      }
      x[index]=sum/n;
      index++;
      i=i+5;
    }
    else{
      i++;
    }
  }  
  return x;
}
  

float* fwd_distance(int *readings, int num){
  static float y[12];  //forward distance array of the mine
  int i=0;
  int index=0;
  int n;   //number of 1s (number of postitions detecting mine)
  float sum;  //sum of the coordinates of a[i]
  
  
  while (i<12){
    if (*(readings+i)==1){
      n=0;
      sum=0;
      for (int j=i;j<i+5 && j<12;j++){
        n=n+*(readings+j);
        sum=sum+(*(pos_for_offset+j))*(*(readings+j));
      }
      y[index]=sum/n;
      index++;
      i=i+5;
    }
    else{
      i++;
    }
  }  
  return y;
}





  
  

void mpr121_setup(){

  set_register(0x5A, ELE_CFG, 0x00); 
  
  // Section A - Controls filtering when data is > baseline.
  set_register(0x5A, MHD_R, 0x01);
  set_register(0x5A, NHD_R, 0x01);
  set_register(0x5A, NCL_R, 0x00);
  set_register(0x5A, FDL_R, 0x00);

  // Section B - Controls filtering when data is < baseline.
  set_register(0x5A, MHD_F, 0x01);
  set_register(0x5A, NHD_F, 0x01);
  set_register(0x5A, NCL_F, 0xFF);
  set_register(0x5A, FDL_F, 0x02);
  
  // Section C - Sets touch and release thresholds for each electrode
  set_register(0x5A, ELE0_T, TOU_THRESH);
  set_register(0x5A, ELE0_R, REL_THRESH);
 
  set_register(0x5A, ELE1_T, TOU_THRESH);
  set_register(0x5A, ELE1_R, REL_THRESH);
  
  set_register(0x5A, ELE2_T, TOU_THRESH);
  set_register(0x5A, ELE2_R, REL_THRESH);
  
  set_register(0x5A, ELE3_T, TOU_THRESH);
  set_register(0x5A, ELE3_R, REL_THRESH);
  
  set_register(0x5A, ELE4_T, TOU_THRESH);
  set_register(0x5A, ELE4_R, REL_THRESH);
  
  set_register(0x5A, ELE5_T, TOU_THRESH);
  set_register(0x5A, ELE5_R, REL_THRESH);
  
  set_register(0x5A, ELE6_T, TOU_THRESH);
  set_register(0x5A, ELE6_R, REL_THRESH);
  
  set_register(0x5A, ELE7_T, TOU_THRESH);
  set_register(0x5A, ELE7_R, REL_THRESH);
  
  set_register(0x5A, ELE8_T, TOU_THRESH);
  set_register(0x5A, ELE8_R, REL_THRESH);
  
  set_register(0x5A, ELE9_T, TOU_THRESH);
  set_register(0x5A, ELE9_R, REL_THRESH);
  
  set_register(0x5A, ELE10_T, TOU_THRESH);
  set_register(0x5A, ELE10_R, REL_THRESH);
  
  set_register(0x5A, ELE11_T, TOU_THRESH);
  set_register(0x5A, ELE11_R, REL_THRESH);
  
  // Section D
  // Set the Filter Configuration
  // Set ESI2
  set_register(0x5A, FIL_CFG, 0x04);
  
  // Section E
  // Electrode Configuration
  // Set ELE_CFG to 0x00 to return to standby mode
  set_register(0x5A, ELE_CFG, 0x0C);  // Enables all 12 Electrodes
  
  
  // Section F
  // Enable Auto Config and auto Reconfig
  /*set_register(0x5A, ATO_CFG0, 0x0B);
  set_register(0x5A, ATO_CFGU, 0xC9);  // USL = (Vdd-0.7)/vdd*256 = 0xC9 @3.3V   set_register(0x5A, ATO_CFGL, 0x82);  // LSL = 0.65*USL = 0x82 @3.3V
  set_register(0x5A, ATO_CFGT, 0xB5);*/  // Target = 0.9*USL = 0xB5 @3.3V
  
  set_register(0x5A, ELE_CFG, 0x0C);
  
}



void set_register(int address, unsigned char r, unsigned char v){
    Wire.beginTransmission(address);
    Wire.write(r);
    Wire.write(v);
    Wire.endTransmission();
}
