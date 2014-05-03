#include "utility.h"
#include "Wire.h"
//#include "math.h"

//define states
#define FORWARD     0    //S0(FW): robot goes forward
//#define REVERSE     5    //S5 (RV): robot goes backward
#define TURNING     1    //S1(TN): robot turns clockwise
#define ALIGNMENT   2    //S2(AL): robot aligns itself to go straight
#define MARKING     3    //S3(MK): robot deploys marking procedure (call subroutine mark()<---several inputs subject to change)
#define STOP        4    //S4(ST): robot comes to a stop

//define inputs
int ST = 0;       //straght (1)/not straight (0)       
int NW = 0;       //near wall (1)/not near wall (0)
int CS = 0;       //mine detected (1)/mine not detected (0)
int EOT = 0;       //turning completed (1)/turning not completed (0)
int CT = 0;       //count of mines = 3 (1)/count of mines < 3 (0)
int ER = 0;      //error occurred (1)/no error (0)


//initialize the states
int state = FORWARD;  //start from S0(FW)

//define on/off button state
int button_state = LOW; //button default OFF 


//define pins         >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>TBD
#define start_button_pin A2
#define indicator_LED_pin A3

#define US1_trigPin 13
#define US1_echoPin 12
#define US2_trigPin 11
#define US2_echoPin 10
#define US3_trigPin 9
#define US3_echoPin 8                                                                                                                                                                                                                                                           


//define instances for ultrasonic sensors
ultrasonic US1;      //front ultrasonic sensors
ultrasonic US2;      //side-front ultrasonic sensor
ultrasonic US3;      //side-rear ultrasonic sensor

//define other parameters  >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>TBD
float t = 0.5;     //tolerance for discrepancy of readings of two ultrasonic sensors (US2&US3) (in cm)
float offset = 0; //offset distance of ultrasonice sensors 2 &3 (theretically, US2.dist()-3=US3.dist())
int dtw=22;       //distance to wall where the robot turns, increase after every turn   >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>algorithm TBD (especially when a turing a completed during marking)
int bwd=18;    //backward distance to ensure a safe turn
int scl,fcl;    //safety clearance to side wall/front wall
int RPM=200;         //'RPM' of motor represented in PWM
int RPM_change; //'change of 'RPM' for alignment purpose
int RPM_change_max = 3*RPM; //maximum allowable adjustment of RPM
float k_align=13;   //alignment RPM adjustment coefficent: RPM_change=k_align*(US2.dist()-US3.dist())
int turn_count=0;  //count of turn per cycle around perimeter
int pass_count=0; //count of passes around the perimeter

int mine_count=0;       //count of successive marking

float us1_dist_last=60;
float us2_dist_last=60;
float us3_dist_last=60;

int *cap_pin_reading;
int num_of_mine;  //number of mine detected instantaneously
int num_of_mine_last;
float *x_mine; //array of mine positions
float l=29.2; //length of the beam
float *fwd_dis; //forward distance for marking
float current_dis; //initial distance to wall at the start of marking

long previousMillis_turn = 0;
long previousMillis_mark = 0;

int start_flag = 1;









void setup(){
  pinMode(start_button_pin,INPUT);
  pinMode(indicator_LED_pin,OUTPUT);
  
  Serial.begin(9600);
  
  motor_init();
  
  US1._init(US1_trigPin,US1_echoPin);
  US2._init(US2_trigPin,US2_echoPin);
  US3._init(US3_trigPin,US3_echoPin);
  
  mpr121_init();
  
  digitalWrite(indicator_LED_pin,LOW);
  
  
}







void loop(){
  if (digitalRead(start_button_pin)){                              //start/shut button
    button_state = !button_state;
    while (digitalRead(start_button_pin)) delayMicroseconds(1);
  }
    
  if (button_state){
    
    digitalWrite(indicator_LED_pin,HIGH);
    
    if (start_flag ==1){
      forward(RPM);
      delay(800);
      turnRight(RPM);
      delay(1000);
      Stop();
      delay(3500);
      start_flag =0;
    }
    
    
    float us1_dist = US1.dist();
    float us2_dist = US2.dist()-offset;
    float us3_dist = US3.dist();
    
    //if (us1_dist>2000) us1_dist = us1_dist_last;
    if (us2_dist>1000) us2_dist = us2_dist_last;
    if (us3_dist>1000) us3_dist = us3_dist_last;
    
    us1_dist_last = us1_dist;
    us2_dist_last = us2_dist;
    us2_dist_last = us2_dist;
    
    //Serial.print("Front distance = ");
    //Serial.print(us1_dist);
    //Serial.print(" cm.   ");
    //Serial.print("Side front distance = ");
    //Serial.print(us2_dist);
    //Serial.print(" cm.   ");
    //Serial.print("Side back distance = ");
    //Serial.print(us3_dist);  
    //Serial.println(" cm.");
   
    //forward(RPM);
    //Serial.println(pass_count);
    
    cap_pin_reading = readTouchInputs();
    num_of_mine_last = num_of_mine;
    num_of_mine = num_of_mine_detected(cap_pin_reading);
    for (int i=0;i<12;i++){
      Serial.print("pin");
      Serial.print(i);
      Serial.print("=");
      Serial.print(*(cap_pin_reading+i));
      Serial.print("   ");    
    }
    
    
    //read inputs
    if (num_of_mine==1 &&  (millis()-previousMillis_mark>2000)){
      CS=1;
      x_mine = mine_position(cap_pin_reading, num_of_mine);
      fwd_dis=fwd_distance(cap_pin_reading, num_of_mine);
      Serial.print("Mine position:  ");
      for (int i=0;i<num_of_mine;i++){
        Serial.print(*(x_mine+i));
        Serial.println("  ");
        Serial.print("forward distance:  ");
        Serial.print(*(fwd_dis+i));
        Serial.println("  ");     
      }
    }
    else{
      CS = 0;
      //Serial.println("no mine detected");
      Serial.println();
    }
  
  

  
    if (abs(us2_dist-us3_dist)<t) ST=1;
    else ST=0;
    
    if (us1_dist<=dtw && (millis()-previousMillis_turn>2000)) NW=1;
    else NW=0;
  /*  
    if (1) CS=1;     //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>TBD
    else CS=0;
    
    if (state==TURNING){
      if (1) EOT =1;  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>TBD
      else EOT =0;
    }
    else EOT=EOT;   //EOT should be subject to change during the marking procedure mark(), mark from FW/TN when EOT=1/0
    
    if (mine_count==3) CT=1;
    else CT =0;
    
    if (US1.dist()<fcl || US2.dist()<scl || US3.dist()<scl) ER=1;
    else ER=0;
    
  */
    //in-state algorithm
    switch(state){
      case FORWARD:
        forward(RPM);
        if (!NW && ST && !CS){
          state=FORWARD;
        }
        else if (!NW && !ST && !CS){
          state=ALIGNMENT;
        }
        else if (NW && !CS ){
          state=TURNING;
        }
        else{
          state=MARKING;
        }
  /*
        if (ST && !NW && !CS && !CT && !ER){
          state = FORWARD;
        }
        else if (NW && !CS && !CT && !ER){
          state = TURNING;
        }
        else if (!ST && !NW && !CS && !CT && !ER){
          state = ALIGNMENT;
        }
        else if (CS && !CT && !ER){
          state = MARKING;
        }
        else{
          state = STOP;
        }
  */
        break;
  
      case TURNING:   //turn right in this case
         digitalWrite(indicator_LED_pin,LOW);
         reverse(RPM);
         while(US1.dist()<dtw+bwd && button_state){
             if (digitalRead(start_button_pin)){     //start/shut button
               button_state = !button_state;
               while (digitalRead(start_button_pin)) delayMicroseconds(1);
             }
             Serial.print("U1_distance = ");
             Serial.print(US1.dist());
             Serial.println(" cm.   ");
         };
         if (button_state){
           turnLeft(RPM);
           delay(900);
           turn_count = turn_count +1;
           if ((turn_count == 3)  && (pass_count<3)){
            if (pass_count == 0){
              dtw =dtw +50;
            }
            else{
              dtw = dtw +40;
            }
            bwd = 0;
            turn_count = -1;
            pass_count = pass_count +1;
           }
         }
         previousMillis_turn = millis();
         if (pass_count >=2){
           state = STOP;
         }
         else state=FORWARD;
         digitalWrite(indicator_LED_pin,HIGH);
   /*      
         if (!CS && EOT && !CT && !ER){
           state = FORWARD;
         }
         else if (!CS && !EOT && !CT && !ER){
           state = TURNING;
         }
         else if (CS && !CT && !ER){
           state = MARKING;
         }
         else{
           state = STOP;
         }     
   */

        break;
       
  
      
      case ALIGNMENT:
        /*
        if (US2.dist()<US3.dist()){   //sensor on the left of the vehicle     >>>>>>>>>>>>>>>>>subject to change 'RPM_change' can be changed according to US2.dist()<US3.dist())
          alignment(RPM-RPM_change,RPM+RPM_change);
        }
        else if (US2.dist()>US3.dist()){
          alignment(RPM+RPM_change,RPM-RPM_change);
        }
        */
        RPM_change = k_align*(us2_dist-us3_dist); 
        if (RPM_change >RPM_change_max) RPM_change = RPM_change_max;
        if (RPM_change <-RPM_change_max) RPM_change = -RPM_change_max;
        //Serial.print(RPM_change);
        //Serial.print("  ");
        //Serial.print(RPM+RPM_change);
        //Serial.print("  ");
        //Serial.println(RPM-RPM_change);
        alignment(RPM+RPM_change,RPM-RPM_change);   
        if (!NW && ST && !CS){
          state=FORWARD;
        }
        else if (!NW && !ST && !CS){
          state=ALIGNMENT;
        }
        else if(NW && !CS ){
          state=TURNING;
        }
        else{
          state = MARKING;
        }
  /* 
        if (ST && !NW && !CS && !CT && !ER){
          state = FORWARD;
          
        }
        else if (NW && !CS && !CT && !ER){
          state = TURNING;
        }
        else if (!ST && !NW && !CS && !CT && !ER){
          state = ALIGNMENT;
        }
        else if (CS && !CT && !ER){
          state = MARKING;
        }
        else{
          state = STOP;
        }
  */      
        break;
  
  
     
      case MARKING:
        Serial.println("detected");
        Serial.println(num_of_mine_last);
        current_dis = us1_dist;
        float fwd_dis_seq[num_of_mine_last];
        float temp;
        for (int i=0;i<num_of_mine_last;i++){
          fwd_dis_seq[i]=*(fwd_dis+i);
        }
        for (int i=0;i<num_of_mine_last;i++){
          for (int j=i;j<num_of_mine_last;j++){
            if (*(fwd_dis_seq+i)>*(fwd_dis_seq+j)){
              temp = *(fwd_dis_seq+i);
              fwd_dis_seq[i]=*(fwd_dis_seq+j);
              fwd_dis_seq[j]=temp;
            }
          }
        }      
        
        for (int i=0;i<num_of_mine_last;i++){
          while (US1.dist() > (current_dis - *(fwd_dis_seq+i))+5){
            forward(0.5*RPM);
          }
          Serial.println("marking");
          //forward(0.5*RPM);
          //delay(*(fwd_dis_seq+i)*50);
          Stop();
          delay(5000);
        }
        state = FORWARD;
        previousMillis_mark = millis();
        //mark();        //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>TBD
  /*
        //codes to change necessary inputs (including NW EOT CT.......)
        if (ST && !NW && !CS && EOT && !CT && !ER){
          state = FORWARD;
        }
        else if ((!CS && !EOT && !CT && !ER)||(NW && !CS && EOT && !CT && !ER)){
          state = TURNING;
        }
        else if (!ST && !NW && !CS && EOT && !CT && !ER){
          state = ALIGNMENT;
        }
        else if (CS && !CT && !ER){
          state = MARKING;
        }
        else{
          state = STOP;
        }
  */  
        break;
     
      case STOP:
        Stop();
        state = STOP;
        break;
      
    }
  } //end of operational program
  
  
  else{
    digitalWrite(indicator_LED_pin,LOW);
    Stop();       //shut the motor
  }
}
