//
//This code was authored by Ansh Verma, Ryan Clark, Tyler Helmus and Jiajun Cao
//for the feeder mechanism, MPR 121 mine detection and Flagging Arm Servo of the 
//Mine-robot for the course completion of ME 588 at Purdue University. 
// 

#include "mpr121.h"         //Import capacitive touch sensor library
#include <Wire.h>
#include <Servo.h>          // Import servo library

#define ARM_SPEED 10        // arm speed
#define ARM_HOME 71         // arm electromagnet-magnet pickup position
#define PICKUP_POS 80       // flag dispensor servo position for electromagnet-magnet pickup
#define RETREIVE_POS 0      // flag dispensor servo position of the magnet reservoir
#define FLAG_SPEED 1        // flag dispensor servo speed
#define MAG_LOAD 1700       // delay for magnet to drop into slot
#define MAG_LIFT_DELAY 1500 // ms delay which ensures magnet is lifted prior to arm movin
#define WARN_LIGHT 7        // define pins and states with arbitrary value
#define MAGNET 6            //Define pin for magnet 
#define IRQ_PIN 4           // Define pin for IRQ on capacitive touch
#define ARM_SERVO 9         //Define pin for PWM signal to arm servo
#define FLAG_SERVO 10       //Define pin for PWM signal to flagging servo
#define ARM_MOVE 500        //Define delay for arm movement to complete
#define BUTTON 8            //Define pin for start up sensing
#define MINE_DETECT 1000    //Define delay for robot to be in position prior to lifting magnet
#define COMM A0             //Define pin for turning communication from drive arduino: stops flagging when turning

Servo armServo, flagServo; //Name servo

int armPos = ARM_HOME;
boolean touchStates[12]; //to keep track of the previous touch states

void setup() {
  pinMode(IRQ_PIN, INPUT); //enable IRQ pin
  digitalWrite(IRQ_PIN, HIGH); //enable pullup resistor

  //Ryan Insert Begin---- Declare Pins
  armServo.attach (ARM_SERVO);//Main Servo
  flagServo.attach (FLAG_SERVO);//Mag Feed Servo
  pinMode(MAGNET, OUTPUT);//Enable magnet pin
  pinMode(BUTTON, INPUT);//enable start button
  pinMode(COMM, INPUT);//Define pin for turning communication
  digitalWrite(MAGNET, LOW);//Define starting LOW state for the magnet

  Serial.begin(9600);
  Wire.begin();

  armServo.write(ARM_HOME);//Start with servo in the home position
  getNextFlag();//Function to actuate magnet feeder servo to get the next flag
  mpr121_setup();//setput the capacitive touch sensor
  while(!digitalRead(BUTTON)){//Look for when the start button is pushed
    delay(5);
  }
  delay(2500);
  deployWings();//deploy the wings in the start sequence
  
}

void loop()
{
  if(digitalRead(COMM)){ //If the robot is not turning, continue on with the switch case and subsequent flagging procedure
  if(digitalRead(BUTTON)){ //Read startup button
    while(true){
      delay(10);
    }
  }

  Wire.requestFrom(0x5A, 2);//printing touch state from the capacitive touch sensor
  byte LSB = Wire.read();
  byte MSB = Wire.read();
  uint16_t touched = ((MSB << 8) | LSB);
  Serial.println(touched);


  //BEGIN SWITCH CASE------------------------------
  switch (touched) {
  case 1:
    Serial.print("a");//check touch state to see which wire was touched
    delay(MINE_DETECT);//Delay for wheels to stop rotating
    digitalWrite(MAGNET, HIGH);//Turn electromagnet on
    delay(MAG_LIFT_DELAY);//Delay to ensure the magnet has been lifted up to the electromagnet
    armPosition(168);//Move the arm servo to the necessary location
    delay(ARM_MOVE);//Wait for the servo to reach this location
    digitalWrite(MAGNET, LOW);//Turn off the electromagnet to release the flag
    delay(MAG_LIFT_DELAY);//Ensure the magnet has been dropped
    armReturn();//Return the arm to the home position
    getNextFlag();//Actuate the magnet feeder servo
    break;
  case 2:
    Serial.print("b");
    delay(MINE_DETECT);
    digitalWrite(MAGNET, HIGH);
    delay(MAG_LIFT_DELAY);
    armPosition(145);
        delay(ARM_MOVE);
    digitalWrite(MAGNET, LOW);
    delay(MAG_LIFT_DELAY);
    armReturn();
    getNextFlag();
    break;
  case 4:
    Serial.print("c");
    delay(MINE_DETECT);
    digitalWrite(MAGNET, HIGH);
    delay(MAG_LIFT_DELAY);
    armPosition(134);
        delay(ARM_MOVE);
    digitalWrite(MAGNET, LOW);
    delay(MAG_LIFT_DELAY);
    armReturn();
    getNextFlag();
    break;
  case 8:
    Serial.print("d");
    delay(MINE_DETECT);
    digitalWrite(MAGNET, HIGH);
    delay(MAG_LIFT_DELAY);
    armPosition(119);
    delay(ARM_MOVE);
    digitalWrite(MAGNET, LOW);
    delay(MAG_LIFT_DELAY);
    armReturn();
    getNextFlag();
    break;
  case 16:
    Serial.print("e");
    delay(MINE_DETECT);
    digitalWrite(MAGNET, HIGH);
    delay(MAG_LIFT_DELAY);
    armPosition(109);
    delay(ARM_MOVE);
    digitalWrite(MAGNET, LOW);
    delay(MAG_LIFT_DELAY);
    armReturn();
    getNextFlag();
    break;
  case 32:
    Serial.print("f");
    delay(MINE_DETECT);
    digitalWrite(MAGNET, HIGH);
    delay(MAG_LIFT_DELAY);
    armPosition(99);
    delay(ARM_MOVE);
    digitalWrite(MAGNET, LOW);
    delay(MAG_LIFT_DELAY);
    armReturn();
    getNextFlag();
    break;
  case 64:
    Serial.print("g");
    delay(MINE_DETECT);
    digitalWrite(MAGNET, HIGH);
    delay(MAG_LIFT_DELAY);
    armPosition(90);
    delay(ARM_MOVE);
    digitalWrite(MAGNET, LOW);
    delay(MAG_LIFT_DELAY);
    armReturn();
    getNextFlag();
    break;
  case 128:
    Serial.print("h");
    delay(MINE_DETECT);
    digitalWrite(MAGNET, HIGH);
    delay(MAG_LIFT_DELAY);
    armPosition(85);
    delay(ARM_MOVE);
    digitalWrite(MAGNET, LOW);
    delay(MAG_LIFT_DELAY);
    armReturn();
    getNextFlag();
    break;
  case 256:
    Serial.print("i");
    delay(MINE_DETECT);
    digitalWrite(MAGNET, HIGH);
    delay(MAG_LIFT_DELAY);
    armPosition(65);
    delay(ARM_MOVE);
    digitalWrite(MAGNET, LOW);
    delay(MAG_LIFT_DELAY);
    armReturn();
    getNextFlag();
    break;
  case 512:
    Serial.print("j");
    delay(MINE_DETECT);
    digitalWrite(MAGNET, HIGH);
    delay(MAG_LIFT_DELAY);
    armPosition(60);
    delay(ARM_MOVE);
    digitalWrite(MAGNET, LOW);
    delay(MAG_LIFT_DELAY);
    armReturn();
    getNextFlag();
    break;
  case 1024:
    Serial.print("k");
    delay(MINE_DETECT);
    digitalWrite(MAGNET, HIGH);
    delay(MAG_LIFT_DELAY);
    armPosition(48);
    delay(ARM_MOVE);
    digitalWrite(MAGNET, LOW);
    delay(MAG_LIFT_DELAY);
    armReturn();
    getNextFlag();
    break;
  case 2048:
    Serial.print("l");
    delay(MINE_DETECT);
    digitalWrite(MAGNET, HIGH);
    delay(MAG_LIFT_DELAY);
    armPosition(34);
    delay(ARM_MOVE);
    digitalWrite(MAGNET, LOW);
    delay(MAG_LIFT_DELAY);
    armReturn();
    getNextFlag();
    break;
  case 0:
    Serial.print("Default");
    digitalWrite(MAGNET, LOW);//Default position with magnet off
    break;
  }

  }
}


void mpr121_setup(void) {

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


//boolean checkInterrupt(void) {
//  return digitalRead(IRQ_PIN);
//}


void set_register(int address, unsigned char r, unsigned char v) {
  Wire.beginTransmission(address);
  Wire.write(r);
  Wire.write(v);
  Wire.endTransmission();
}

void armPosition(int target) {//Code to move the arm after a mine is detected
  int pos;//position pulled from switch case
  armPos = target;//current position of arm so we can see which direction arm must turn
  if (armPos > ARM_HOME) {//Turn counterclockwise
    for (pos = ARM_HOME; pos <= armPos; pos++) {//velocity control for servo to move to desired position
      armServo.write (pos);
      delay(ARM_SPEED);//Delay which dictates the arm speed
    }
  }
  if (armPos < ARM_HOME) {//Same as above except for clockwise movement
    for (pos = ARM_HOME; pos >= armPos; pos--) {
      armServo.write (pos);
      delay(ARM_SPEED);
    }
  }
}

void armReturn() {//Speed controlled return function after flag is dropped
  int pos;
  if (armPos > ARM_HOME) {
    for (pos = armPos; pos >= ARM_HOME; pos--) {
      armServo.write(pos);
      delay(ARM_SPEED);
    }
  }
  if (armPos < ARM_HOME) {
    for (pos = armPos; pos <= ARM_HOME; pos++) {
      armServo.write(pos);
      delay(ARM_SPEED);
    }
  }
}

void getNextFlag() {//Function to actuate the magnet feeder

  flagServo.write(RETREIVE_POS);//Move to get a magnet
  delay(MAG_LOAD);//Delay to get in position
  flagServo.write(PICKUP_POS);//Return to the home position with a new flag
}

void deployWings(){//startup function to delploy the wings
  armPosition(34);//drop right wing
  delay(200);
  armReturn();
  armPosition(110);//drop left wing
  delay(200);
  armReturn();//return arm to home position
}

