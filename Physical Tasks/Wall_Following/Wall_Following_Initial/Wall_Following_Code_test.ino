#include <HCSR04.h>

#define COLLISION_DISTANCE 85
#define MAX_SPEED 220
#define  REFERENCE_DISTANCE 30

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

float distance1,distance2,distance3,distance4,distance5,front_errorD,back_errorD,errorD1,errorD2,derivative1,derivative2,integral1=0.0,integral2=0.0,previousErrorD1=0.0,previousErrorD2=0.0,outputD1,outputD2,MIN_SPEED=40;
float Kp_d=1,Ki_d=0,Kd_d=1.5;
int speedL,speedR,basespeed=110;

//right motor pins
const int in1R = A0;
const int in2R = A1;
const int enR = 3;

//left motor pins
const int in1L = A3;
const int in2L = A2;
const int enL = 10;

//sensor 
const int trigPin1 = 4;//distance1
const int echoPin1 = 5;
const int trigPin2 = 26;//distance2
const int echoPin2 = 27;
const int trigPin3 = 14;//distance3
const int echoPin3 = 15;
const int trigPin4 = 16;//distance4
const int echoPin4 = 17;
const int trigPin5 = 22;//distance5
const int echoPin5 = 23;



void wall_following(){
  HCSR04 hc_left_front(trigPin1, echoPin1);
  distance1=hc_left_front.dist();
  HCSR04 hc_left_back(trigPin3, echoPin3);
  distance3=hc_left_back.dist();
  HCSR04 hc_right_back(trigPin4, echoPin4);
  distance4=hc_right_back.dist();
  HCSR04 hc_right_front(trigPin2, echoPin2);
  distance2=hc_right_front.dist();
  HCSR04 front(trigPin5, echoPin5);
  distance5=front.dist();
 if ((distance1==0.00) || (distance1>100))
  {
    distance1=0.0;
  }
  
  if ((distance2==0.00) || (distance2>100))
  {
    distance2=0.0;
  }
   if ((distance3==0.00) || (distance3>100))
  {
    distance3=0.0;
  }
   if ((distance4==0.00) || (distance4>100))
  {
    distance4=0.0;
  }
  if (((distance5==0.00) || (distance5>100)))
  {
    distance5=100.0;
  }
 
 if  ((distance1+distance3)==(distance2+distance4)){
    speedL=basespeed;
    speedR=basespeed;
    
    
    }
  
    
else if ((distance1+distance3)>(distance2+distance4)){
  
   if (distance1==0.0){
    errorD1=(REFERENCE_DISTANCE-(distance1*3+distance3*3));
    
    
    }
    else{
  errorD1=REFERENCE_DISTANCE-((distance1*0.8+distance3)/2);
    }
    
     derivative1=errorD1-previousErrorD1;
   integral1+=errorD1;

  previousErrorD1=errorD1;
  
  outputD1=(Kp_d*errorD1+Kd_d*derivative1+Ki_d*integral1)/1.8;

   speedL=basespeed+outputD1;
  speedR=basespeed-outputD1;

    
  
  
  }
else if ((distance1+distance3)>(distance2+distance4)){
  
   if (distance2==0.0){
    errorD1=(REFERENCE_DISTANCE-(distance1*2+distance3*2));
    
    
    }
    else{
  errorD1=REFERENCE_DISTANCE-((distance1*0.8+distance3)/2);
    }
    
     derivative1=errorD1-previousErrorD1;
   integral1+=errorD1;

  previousErrorD1=errorD1;
  
  outputD1=(Kp_d*errorD1+Kd_d*derivative1+Ki_d*integral1)/1.8;

   speedL=basespeed+outputD1;
  speedR=basespeed-outputD1;

    
  
  
  }


   
      
      
      
      
      
      
      
      
      
      




  
  
  
  

 // front_errorD=REFERENCE_DISTANCE-distance1;
 // back_errorD=REFERENCE_DISTANCE-distance3;
  //float error2=d_reference-d_left_back;
 // errorD=(1.5*front_errorD+1.5*back_errorD)/8;
//  Serial.println("Error = "+String(error));

  
 Serial.print("control_Error = "+String(outputD1));
  Serial.print(" ");
  Serial.print(distance1);
  Serial.print(" ");
  Serial.print(distance3);
  Serial.print(" ");
  Serial.print(distance2);
  Serial.print(" ");
  Serial.print(distance4);
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
  analogWrite(enL,speedL);
  analogWrite(enR,speedR);
  digitalWrite(in1L, HIGH);
  digitalWrite(in2L, LOW);
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW);
  }

/*
void reach_distance(){
  distance1 = sensor_output(trigPin1, echoPin1);
  distance2 = sensor_output(trigPin2, echoPin2);
  distance3 = sensor_output(trigPin3, echoPin3);
  distance4 = sensor_output(trigPin4, echoPin4);
  if ((distance1+distance3)<(distance2+distance4)){
    front_errorD = (REFERENCE_DISTANCE-distance1);
   back_errorD=(REFERENCE_DISTANCE-distance3);
    }
  else{
    front_errorD = (REFERENCE_DISTANCE-distance2);
  back_errorD=(REFERENCE_DISTANCE-distance3);
    
    }
 
  errorD=(back_errorD*1.5)/2;
  derivative = errorD - previousErrorD;
  integral += errorD;
  outputD = Kp_d*errorD + Ki_d*integral + Kd_d*derivative;
  previousErrorD = errorD;
  
  if ((distance1+distance3)<(distance2+distance4)){
    speedL = SPEED + (int)outputD;
    speedR = SPEED - (int)outputD;
    
    
    }
 else{
  speedL = SPEED - (int)outputD;
  speedR = SPEED + (int)outputD;
  
  
  }
  
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
  analogWrite(enL,speedL);
  analogWrite(enR,speedR);
  digitalWrite(in1L, HIGH);
  digitalWrite(in2L, LOW);
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW);

  
  Serial.print("left speed: ");
  Serial.print(speedL);
  Serial.print("  ");
  Serial.print("right speed: ");
  Serial.println(speedR);
  
  
  
  }
  */

bool check_collision(){
  HCSR04 front(trigPin5, echoPin5);
  distance5=front.dist();
   if (((distance5==0.00) || (distance5>100)))
  {
    distance5=100.0;
  }
 
    
  if(distance5<=COLLISION_DISTANCE){
    return true;
  }else{
    return false;
  }
}



void setup() {
Serial.begin(9600);
// initialize OLED display with address 0x3C for 128x64
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }  
  
pinMode(in1L,OUTPUT);
pinMode(in2L,OUTPUT);
pinMode(in1R,OUTPUT);
pinMode(in2R,OUTPUT);
pinMode(enL,OUTPUT);
pinMode(enR,OUTPUT); 
 
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
  digitalWrite(in1L, LOW);
  digitalWrite(in2L, LOW);
  digitalWrite(in1R, LOW);
  digitalWrite(in2R, LOW);

 
oled.clearDisplay(); // clear display

  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 10);        // position to display
  oled.println("TEAM SPECTRO"); // text to display
  oled.display();               // show on OLED  

}

void loop() {

/*
distance1 = sensor_output(trigPin1, echoPin1);
distance2 = sensor_output(trigPin2, echoPin2);
distance3 = sensor_output(trigPin3, echoPin3);
distance4 = sensor_output(trigPin4, echoPin4);
distance5 = sensor_output(trigPin5, echoPin5);

Serial.print(distance1);
Serial.print("  ");
Serial.print(distance2);
Serial.print("  ");
Serial.print(distance3);
Serial.print("  ");
Serial.print(distance4);
Serial.print("  ");
Serial.print(distance5);
Serial.println();
8*/


if (check_collision()){
  speedL=basespeed-10;
  speedR=basespeed-10;
  
  if (distance5<70){
    if ((distance3+distance1)<(distance4+distance2)){
      speedL=basespeed+30;
      speedR=basespeed-40;
      
     }
     else{
      speedL=basespeed-40;
      speedR=basespeed+30;
      
      }
  analogWrite(enL,speedL);
  analogWrite(enR,speedR);
  digitalWrite(in1L, HIGH);
  digitalWrite(in2L, LOW);
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW); 
    
  }
  else{
    basespeed=90;
  }
   wall_following();
  }
    
else{
  
 // SPEED=130;
 // MIN_SPEED=80;
 basespeed=110;
  wall_following();
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
  oled.print(distance1);
   oled.print("  ");
  oled.print(distance3);
  oled.print("  ");
  oled.print(distance2);
  oled.print("  ");
  oled.print(distance4);
  oled.print("  ");
  oled.print(distance5);
  oled.print("  ");
  oled.print(outputD1);
  oled.print("  ");
  oled.print(outputD2);
  


   

  
  oled.display();


/*if(SPEED > 100){
  SPEED = 100;
}
if(SPEED < 50){
  SPEED = 50;
} */

  
}


