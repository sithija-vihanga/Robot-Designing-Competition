#include <HCSR04.h>




#define COLLISION_DISTANCE 48
#define MAX_SPEED 180
#define REFERENCE_DISTANCE 30



#define MAX_SENSOR_VALUE 4000
#define MAX_OF_SENSOR 3000
#define DIST_PROPORTIONAL_CONST 0.034/2
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

float box_distance1,box_distance2,box_distance3,box_distance4,box_distance5,box_front_errorD,box_back_errorD,box_errorD1,box_errorD2,box_derivative1,box_derivative2,box_integral1=0.0,box_integral2=0.0,box_previousErrorD1=0.0,box_previousErrorD2=0.0,box_outputD1,box_outputD2,MIN_SPEED = 40;
float Kp_d=0.9,Ki_d=0,Kd_d=0.9;
int speedL,speedR,basespeed=110;

bool box_enter = true;

//right motor pins
const int in1 = A0;
const int in2 = A1;
const int enA = 3;

//left motor pins
const int in3 = A3;
const int in4 = A2;
const int enB = 10;

const int ProxSensor_left=A9;     // 3 left
const int ProxSensor_1=A15;       // 2 left
const int ProxSensor_2=A14;       // 1 left
const int ProxSensor_3=A13;       // 0 middle
const int ProxSensor_4=A12;       // 1 right
const int ProxSensor_5=A11;       // 2 right
const int ProxSensor_right=A10;   // 3 right

//sensor 
const int trigPin1 = 4;//box_distance1
const int echoPin1 = 5;
const int trigPin2 = 26;//box_distance2
const int echoPin2 = 27;
const int trigPin3 = 14;//box_distance3
const int echoPin3 = 15;
const int trigPin4 = 16;//box_distance4
const int echoPin4 = 17;
const int trigPin5 = 22;//box_distance5
const int echoPin5 = 23;

int error_list[5] = {0,0,0,0,0};
float kp = 0.6;
float kd = 0.;
float ki = 0.0000;
int inputVal[7] = {0,0,0,0,0,0,0};
int error = 0;
int d_error = 0;
int  i_error = 0;
int allsensors = 0;

void turnright(){
  analogWrite(enA, 200);
  analogWrite(enB, 200);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW) ;
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH) ; 
  delay(1000);

 } 

void turnleft(){
  analogWrite(enA, 200);
  analogWrite(enB, 200);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH) ;
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW) ; 
  delay(1000);

 }

void stopwall(){
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH) ;
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW) ; 
  delay(1000);

 } 


void reverse(){
  analogWrite(enA, 110);
  analogWrite(enB, 110);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW) ;
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW) ; 
  delay(1000);

 } 

bool lineDetected(){
  inputVal[0]  = digitalRead(ProxSensor_left);
  inputVal[1]  = digitalRead(ProxSensor_1);
  inputVal[2]  = digitalRead(ProxSensor_2);
  inputVal[3]  = digitalRead(ProxSensor_3);
  inputVal[4]  = digitalRead(ProxSensor_4);
  inputVal[5]  = digitalRead(ProxSensor_5);
  inputVal[6]  = digitalRead(ProxSensor_right);

  allsensors = inputVal[0] + inputVal[1] + inputVal[3] +inputVal[4] +inputVal[5] + inputVal[6] + inputVal[7];
  
  

  if (allsensors > 0){
    return true;
  }
  
}

bool isleftidle(){
  HCSR04 hc_left_front(trigPin1, echoPin1);
  box_distance1=hc_left_front.dist();
  HCSR04 hc_left_back(trigPin3, echoPin3);
  box_distance3=hc_left_back.dist();
  HCSR04 hc_right_back(trigPin4, echoPin4);
  box_distance4=hc_right_back.dist();
  HCSR04 hc_right_front(trigPin2, echoPin2);
  box_distance2=hc_right_front.dist();
  HCSR04 front(trigPin5, echoPin5);
  box_distance5=front.dist();

 if  ((box_distance1==0)||(box_distance1>40))
  {
    box_distance1=40;
  }
  
  if  ((box_distance2==0)||(box_distance2>40))
  {
    box_distance2=40;
  }


  if (box_distance1 = 40){
    return true;
  }

  else{
    return false;
  }


}
  
bool isfrontblocked(){
  
  HCSR04 hc_left_front(trigPin1, echoPin1);
  box_distance1=hc_left_front.dist();
  HCSR04 hc_left_back(trigPin3, echoPin3);
  box_distance3=hc_left_back.dist();
  HCSR04 hc_right_back(trigPin4, echoPin4);
  box_distance4=hc_right_back.dist();
  HCSR04 hc_right_front(trigPin2, echoPin2);
  box_distance2=hc_right_front.dist();
  HCSR04 front(trigPin5, echoPin5);
  box_distance5=front.dist();

 if  ((box_distance1==0)||(box_distance1>40))
  {
    box_distance1=40;
  }
  
  if  ((box_distance2==0)||(box_distance2>40))
  {
    box_distance2=40;
  }


  if (box_distance5 < 35) {
    return true;
  }

  else{
    return false;
  }


}


void wall_following(){



  HCSR04 hc_left_front(trigPin1, echoPin1);
  box_distance1=hc_left_front.dist();
  HCSR04 hc_left_back(trigPin3, echoPin3);
  box_distance3=hc_left_back.dist();
  HCSR04 hc_right_back(trigPin4, echoPin4);
  box_distance4=hc_right_back.dist();
  HCSR04 hc_right_front(trigPin2, echoPin2);
  box_distance2=hc_right_front.dist();
  HCSR04 front(trigPin5, echoPin5);
  box_distance5=front.dist();

 if  ((box_distance1==0)||(box_distance1>40))
  {
    box_distance1=40;
  }
  
  if  ((box_distance2==0)||(box_distance2>40))
  {
    box_distance2=40;
  }
   if  ((box_distance3==0)||(box_distance3>40))
  {
    box_distance3=40;
  }
   if  ((box_distance4==0)||(box_distance4>40))
  {
    box_distance4=40;
  }
  if  ((box_distance5==0)||(box_distance5>50))
  {
    box_distance5=50;
  }
 
    
    
    
   
  
  if ((box_distance1+box_distance3)<=(box_distance2+box_distance4)){
    
    if (box_distance1==40){
      box_errorD1=(REFERENCE_DISTANCE-(box_distance1*2));
      
      
      }
      else{
        box_errorD1=(REFERENCE_DISTANCE-(box_distance1*0.8+box_distance3)/2);
      }
      
      box_derivative1=box_errorD1-box_previousErrorD1;
      box_integral1+=box_errorD1;

    box_previousErrorD1=box_errorD1;
    
    box_outputD1=(Kp_d*box_errorD1+Kd_d*box_derivative1+Ki_d*box_integral1)/1.6;

    speedL=basespeed+box_outputD1;
    speedR=basespeed-box_outputD1;

      
    
    
    }
    
  else{
    
    if (box_distance2==40){
      box_errorD2=(REFERENCE_DISTANCE-(box_distance2*2));
      
      
      }
      else{
    box_errorD2=REFERENCE_DISTANCE-((box_distance2*0.8+box_distance4)/2);
      }
      
    box_derivative2=box_errorD2-box_previousErrorD2;
    box_integral2+=box_errorD2;

    box_previousErrorD2=box_errorD2;
    
    box_outputD2=(Kp_d*box_errorD2+Kd_d*box_derivative2+Ki_d*box_integral2)/1.6;

    speedL=basespeed-box_outputD2;
    speedR=basespeed+box_outputD2;
    
    
    }
    
    Serial.print("control_Error = "+String(box_outputD1));
    Serial.print(" ");
    Serial.print(box_distance1);
    Serial.print(" ");
    Serial.print(box_distance3);
    Serial.print(" ");
    Serial.print(box_distance2);
    Serial.print(" ");
    Serial.print(box_distance4);
    Serial.println();
    
    
    
    //int ctrl=round(control_signal);
    //int boundedctrl=constrain(ctrl,-10,10);

    

    if (speedL>=MAX_SPEED){
      speedL=MAX_SPEED;
    }
    if (speedL<=MIN_SPEED){
      speedL=MIN_SPEED;
    }
    if (speedR>=MAX_SPEED){
      speedR=MAX_SPEED;
    }
    if (speedR<=MIN_SPEED){
      speedR=MIN_SPEED;
      
      }
    analogWrite(enB,speedL);
    analogWrite(enA,speedR);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    }





bool check_collision(){
  HCSR04 front(trigPin5, echoPin5);
  box_distance5=front.dist();
  
   if  ((box_distance5==0)||(box_distance5>50))
  {
    box_distance5=50;
  }
 
    
  if(box_distance5<=COLLISION_DISTANCE){
    return true;
  }else{
    return false;
  }
 }


void linefollow(){ 

  inputVal[0]  = digitalRead(ProxSensor_left);
  inputVal[1]  = digitalRead(ProxSensor_1);
  inputVal[2]  = digitalRead(ProxSensor_2);
  inputVal[3]  = digitalRead(ProxSensor_3);
  inputVal[4]  = digitalRead(ProxSensor_4);
  inputVal[5]  = digitalRead(ProxSensor_5);
  inputVal[6]  = digitalRead(ProxSensor_right);
  
  error = (-inputVal[0]*3000 -inputVal[1]*300 - inputVal[2]*200 + inputVal[4]*200 +inputVal[5]*300 + inputVal[6]*3000)/(8*(inputVal[0]+inputVal[1]+inputVal[2]+inputVal[3]+inputVal[4]+inputVal[5]+inputVal[6]));

  for (int i = 0; i<4 ; i++){
    error_list[i] = error_list[i+1];
  }

  error_list[4] = error;
  d_error = error_list[4] - error_list[3];

  i_error = i_error + error;
  int pid = kp*error + kd*d_error + ki*i_error;  

  oled.clearDisplay();
  oled.setTextSize(1);       
  oled.setTextColor(WHITE);
  oled.setCursor(0, 40);    
  oled.print(inputVal[0]);
  oled.print(inputVal[1]);
  oled.print(inputVal[2]);
  oled.print(inputVal[3]);
  oled.print(inputVal[4]);
  oled.print(inputVal[5]);  
  oled.println(inputVal[6]);
  
  oled.display();

int base_speed = 80;  
 int plus_speed = 80;
 int min_speed = 80;


  base_speed = 75;  
  plus_speed = base_speed + pid;
  min_speed = base_speed - pid;

  if (base_speed + pid > 100){
    plus_speed =100;    
  }

  else if(base_speed - pid > 100){
    min_speed = 100;
  }

  if (base_speed + pid < 50){
    plus_speed = 50;
  }

  else if (base_speed - pid < 50){
    min_speed =50;  
  }
  

  oled.setTextSize(1);       
  oled.setTextColor(WHITE);
  oled.setCursor(0, 50);     
  oled.print(pid);
  oled.print("    ");
  oled.print(plus_speed);
  oled.print("    ");
  oled.println(min_speed);
  oled.display();

  analogWrite(enA, plus_speed);
  analogWrite(enB, min_speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW) ;
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW) ;    


}

 





void setup() {
  Serial.begin(9600);
  // initialize OLED display with address 0x3C for 128x64
   if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }  


  //////line sensors//////

  pinMode(ProxSensor_left,INPUT);
  pinMode(ProxSensor_1,INPUT);    
  pinMode(ProxSensor_2,INPUT);    
  pinMode(ProxSensor_3,INPUT);    
  pinMode(ProxSensor_4,INPUT);
  pinMode(ProxSensor_5,INPUT);
  pinMode(ProxSensor_right,INPUT);

  // Set all the motor control pins to outputs
	pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
	
	// Turn off motors - Initial state
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);



  
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(enB,OUTPUT);
  pinMode(enA,OUTPUT); 
  
  pinMode(trigPin1,OUTPUT);
  pinMode(echoPin1,INPUT);
  pinMode(trigPin2,OUTPUT);
  pinMode(echoPin2,INPUT);
  pinMode(trigPin3,OUTPUT);
  pinMode(echoPin3,INPUT);
  pinMode(trigPin4,OUTPUT);
  pinMode(echoPin4,INPUT);
  pinMode(trigPin5,OUTPUT);
  pinMode(echoPin5,INPUT);

  // Turn off motors - Initial state
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);

  
  oled.clearDisplay(); // clear display

  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 10);        // position to display
  oled.println("TEAM SPECTRO"); // text to display
  oled.display();               // show on OLED  

}

void loop() {


  HCSR04 hc_left_front(trigPin1, echoPin1);
  box_distance1=hc_left_front.dist();
  HCSR04 hc_left_back(trigPin3, echoPin3);
  box_distance3=hc_left_back.dist();
  HCSR04 hc_right_back(trigPin4, echoPin4);
  box_distance4=hc_right_back.dist();
  HCSR04 hc_right_front(trigPin2, echoPin2);
  box_distance2=hc_right_front.dist();
  HCSR04 front(trigPin5, echoPin5);
  box_distance5=front.dist();

  if ((box_distance1==0)||(box_distance1>40)){
    box_distance1=40;
  }
  
  if  ((box_distance2==0)||(box_distance2>40)){
    box_distance2=40;
  }

  if ((box_distance3==0)||(box_distance3>40)){
    box_distance3=40;
  }

  if  ((box_distance4==0)||(box_distance4>40)){
    box_distance4=40;
  }

  if  ((box_distance5==0)||(box_distance5>50)){
    box_distance5=50;
  } 

  if (box_enter){

    if (isleftidle()){
    stopwall();
    if (isleftidle()){

      turnleft();
      box_enter = false;
      }
    }
  }

  if (lineDetected()){
    linefollow();

  }

  if (isleftidle()){
    stopwall();
    if (isleftidle()){

      turnleft();
      if (lineDetected()){
        linefollow();
      }


      else{

        turnright();
        reverse();
        turnright();
      }

    }


  } 

                                                                                                                                                                                                                       



  if (check_collision()){
    if (isleftidle()){
      turnleft();
      if (lineDetected()){
        linefollow();
      }


      else{

        turnright();
        reverse();
        turnright();
      }


  }
    

    else{
      turnright();
    }

    }
  oled.clearDisplay();
  oled.setTextSize(1);      
  oled.setTextColor(WHITE);
  oled.setCursor(0, 10);   
  oled.println("Team Spectro");
  oled.setCursor(0, 30);    
  oled.print(speedL);
  oled.print("  ");
  oled.print(speedR);
  oled.print("  ");
  oled.print(box_distance1);
  oled.print("  ");
  oled.print(box_distance3);
  oled.print("  ");
  oled.print(box_distance2);
  oled.print("  ");
  oled.print(box_distance4);
  oled.print("  ");
  oled.print(box_distance5);
  oled.print("  ");
  oled.print(box_outputD1);
  oled.print("  ");
  oled.print(box_outputD2);  
  oled.display();

  
}



