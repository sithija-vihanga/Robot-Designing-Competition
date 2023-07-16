#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HCSR04.h>

#define COLLISION_DISTANCE 52
#define MAX_SPEED 180
#define  REFERENCE_DISTANCE 30



#define MAX_SENSOR_VALUE 4000
#define MAX_OF_SENSOR 3000
#define DIST_PROPORTIONAL_CONST 0.034/2



#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
const int ProxSensor_left=A9;
const int ProxSensor_1=A15;
const int ProxSensor_2=A14;
const int ProxSensor_3=A13;
const int ProxSensor_4=A12;
const int ProxSensor_5=A11;
const int ProxSensor_right=A10;
int inputVal[7] = {0,0,0,0,0,0,0};
int strtrted=0;
int stage=0;

//////////////////////////////////////////////////////ranuka//////
float distance1,distance2,distance3,distance4,distance5,fronterrorD1,fronterrorD2,backerrorD1,backerrorD2,errorD1,errorD2,derivative1,derivative2,integral1=0.0,integral2=0.0,previousErrorD1=0.0,previousErrorD2=0.0,outputD1,outputD2,MIN_SPEED=40;
float Kp_d=0.9,Ki_d=0,Kd_d=0.9;
int speedL,speedR,basespeed=110;

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

/////////////////////////////////////////////////////

// Motor A connections
int enA = 3;
int in1 = A0;
int in2 = A1;
// Motor B connections
int enB = 10;
int in3 = A3;
int in4 = A2;

int error_list[5] = {0,0,0,0,0};
float kp = 1.1;
float kd = 0.5;
float ki = 0.001;

int error = 0;
int d_error = 0;
int  i_error = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int LFSensor[7]={0, 0, 0, 0, 0, 0, 0};
int i=0;
int part = 0;// if maze solved this will be 2 and yet to be sloved, this will be 1 (used in the loop)
char path[100] = " ";
char mode = ' ';
int pid;
unsigned char pathLength = 0; // the length of the path
int pathIndex = 0; // used to reach an specific array element.

unsigned int stat = 0; // solving = 0; reach Maze End = 1

//Function for sorting an array
void sort(int a[], int len) {
  for (int i = 0; i < (len - 1); i++) {
    bool flag = true;
    for (int o = 0; o < (len - (i + 1)); o++) {
      if (a[o] > a[o + 1]) {
        int t = a[o];
        a[o] = a[o + 1];
        a[o + 1] = t;
        flag = false;
      }
    }
    if (flag)break;
  }
}
void motorStop(){
   //stop the wheels
  //analogWrite(enA, x);
  //analogWrite(enB, y);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(700);

}
void go(int x, int y) {   //right-x left-y
  
  //go forward in a curved path
  analogWrite(enA, x);
  analogWrite(enB, y);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW) ;
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW) ;

  if(x>y){
    Serial.println("Left");
  }
  else if(x<y){
    Serial.println("Right");
  }
  else{
    Serial.println("Forward");
  }
 delay(200);
}


void runExtraInch(void)
{
  go(80,80);//to move extra length need to change the delay
  motorStop();//need to define
}

void goAndTurn( int Degrees){
  if( Degrees==270){
  analogWrite(enA, 200);
  analogWrite(enB, 200);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW) ;
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH) ; 
  delay(300);
  }
  if( Degrees==180){
      analogWrite(enA, 200);
  analogWrite(enB, 200);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW) ;
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH) ; 
  delay(825);
  }
  if( Degrees==90){
  analogWrite(enA, 200);
  analogWrite(enB, 200);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH) ;
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW) ; 
  delay(350);
  }
}

void mazeTurn (char dir) 
{
       
  if (dir=='L')
  {
      // Turn Left
       goAndTurn (270);
       //lineFollow();      
  }   
    
   else if(dir=='R'){ // Turn Right
       goAndTurn (90);
       //lineFollow();     
   }   
       
   else if(dir== 'B'){ // Turn Back
       goAndTurn (180);
       //lineFollow();     
   }   
       
    else if(dir== 'S'){ // Go Straight
       runExtraInch();
      // lineFollow(); 
    }
  }



void mazeEnd(void)
{
  motorStop();

       oled.setTextSize(1);       
       oled.setTextColor(WHITE);
       oled.setCursor(0, 40);    
       oled.print(6);
       oled.print(mode);
       oled.display();
  
  Serial.print("  pathLenght == ");
  Serial.println(pathLength);
  stat = 1;
  delay(1000);
  //mode = "AE";
}
void simplifyPath()
{
  // only simplify the path if the second-to-last turn was a 'B'
  if(pathLength < 3 || path[pathLength-2] != 'B'){
    delay(10);
  }
  else{
  int totalAngle = 0;
  int i;
  for(i=1;i<=3;i++)
  {
   
     if((path[pathLength-i])=='R'){
      
        totalAngle += 90;
     }
      else if((path[pathLength-i])=='L'){
      
        totalAngle += 270;
     }
       else if((path[pathLength-i])=='B'){
      
        totalAngle += 180;
     }
  }
  
  

  // Get the angle as a number between 0 and 360 degrees.
  totalAngle = totalAngle % 360;

  // Replace all of those turns with a single one.
 if((totalAngle)==0)
  {
    
  path[pathLength - 3] = 'S';
  }
  else if((totalAngle)==90){
  path[pathLength - 3] = 'R';
  }
  else if((totalAngle)==180){
  path[pathLength - 3] = 'B';
  }
  else if((totalAngle)==270){
  path[pathLength - 3] = 'L';
  
  }

  // The path is now two steps shorter.
  pathLength -= 2;
  pathIndex=pathLength-1;//index of last element in path
  

  //change the oder and derections of the path for reverse
  for (int j=0; j<=pathLength-1;j++){
    if (path[j]=='R'){
      path[j]='L';
      j++;
    }
    else if(path[j]=='L'){
      path[j]='R';
      j++;
    }
    else if(path[j]=='S'){
      path[j]='S';
      j++;
    }
  }
  
}
}







void readLFSsensors()
{
  LFSensor[0] = inputVal[0];
  LFSensor[1] = inputVal[1];
  LFSensor[2] = inputVal[2];
  LFSensor[3] = inputVal[3];
  LFSensor[4] = inputVal[4];
  LFSensor[5] = inputVal[5];
  LFSensor[6] = inputVal[6];
  
  //farRightSensor = LFSensor[0]
  //farLeftSensor =  LFSensor[6] 
  
  if     ((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 )&&(LFSensor[5]== 1 )&&(LFSensor[6]== 1 ))  {mode = 'n'; }//n= no line
  else if((LFSensor[0]== 0)&&(LFSensor[1]== 0 )&&(LFSensor[5]== 1 )&&(LFSensor[6]== 1 )) {mode = 'l';}//left turn
  else if((LFSensor[0]== 1)&&(LFSensor[1]== 1 )&&(LFSensor[5]== 0 )&&(LFSensor[6]== 0 )) {mode = 'r';}//right turn
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )&&(LFSensor[5]== 0 )&&(LFSensor[6]== 0 ))  {mode = 'c'; }// cont line
 // else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = 'l'; }//after the new 2 sensor used we dont want to use this
 // else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = 'r'; }
  else {mode = 'f';}//following line
  
 /* Serial.print (farLeftSensor);
  Serial.print (" <== LEFT  RIGH==> ");
  Serial.print (farRightSensor);
  Serial.print ("  mode: ");
  Serial.print (mode);
  Serial.print ("  error:");
  Serial.println (error);*/
    
}


void mazeOptimization (void)// check the syntax
{
  //while (!stat)
  //{
    //readLFSsensors();
       oled.setTextSize(1);       
       oled.setTextColor(WHITE);
       oled.setCursor(0, 40);    
       oled.print(4);
       oled.print(mode);
       oled.display();  
      
      //go(95,100);
      if (mode== 'f'){
        linefollow();
      }  
      else if(mode== 'c'){
        if (pathIndex <= 0) {mazeEnd (); }
        else {mazeTurn (path[pathIndex]); pathIndex=pathIndex-1;}
      }  
      else if(mode== 'l'){
        if (pathIndex <= 0) {mazeEnd ();} 
        else {mazeTurn (path[pathIndex]); pathIndex=pathIndex-1;}
      }  
      else if(mode=='r'){
        if (pathIndex <= 0) {mazeEnd ();} 
        else {mazeTurn (path[pathIndex]); pathIndex=pathIndex-1;}
      }  
    }    
   
//}




void recIntersection(char Direction)
{
  path[pathLength] = Direction; // Store the intersection in the path variable.
  pathLength ++;
  simplifyPath(); // Simplify the learned path.
             oled.setTextSize(1);       
            oled.setTextColor(WHITE);
            oled.setCursor(0, 50);    
            oled.print(5);
            oled.print(mode);
            oled.display();
}

void linefollow(){ 

  inputVal[0]  = digitalRead(ProxSensor_left);
  inputVal[1]  = digitalRead(ProxSensor_1);
  inputVal[2]  = digitalRead(ProxSensor_2);
  inputVal[3]  = digitalRead(ProxSensor_3);
  inputVal[4]  = digitalRead(ProxSensor_4);
  inputVal[5]  = digitalRead(ProxSensor_5);
  inputVal[6]  = digitalRead(ProxSensor_right);
  
  error = (-inputVal[0]*3000 -inputVal[1]*800 - inputVal[2]*200 + inputVal[4]*200 +inputVal[5]*800 + inputVal[6]*3000)/(8*(inputVal[0]+inputVal[1]+inputVal[2]+inputVal[3]+inputVal[4]+inputVal[5]+inputVal[6]));

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





  base_speed = 80;  
  plus_speed = base_speed + pid;
  min_speed = base_speed - pid;


//Minus Speed
//   if (base_speed + pid > 110){
//     plus_speed =110;    
//   }

//   else if(base_speed - pid > 110){
//     min_speed = 110;
//   }

//   if (base_speed + pid < 50){
//     plus_speed = plus_speed - 100;
//   }

//   else if (base_speed - pid < 50){
//     min_speed = min_speed - 100;  
//   }


//   if (min_speed < 0){
//   analogWrite(enA, plus_speed);
//   analogWrite(enB, -min_speed);
//   digitalWrite(in1, HIGH);
//   digitalWrite(in2, LOW) ;
//   digitalWrite(in3, LOW);
//   digitalWrite(in4, HIGH) ; 
    
//   }

//   else if (plus_speed < 0){
//   analogWrite(enA, -plus_speed);
//   analogWrite(enB, min_speed);
//   digitalWrite(in1, LOW);
//   digitalWrite(in2, HIGH) ;
//   digitalWrite(in3, HIGH);
//   digitalWrite(in4, LOW) ; 
    
//   }


//   else{

//   analogWrite(enA, plus_speed);
//   analogWrite(enB, min_speed);
//   digitalWrite(in1, HIGH);
//   digitalWrite(in2, LOW) ;
//   digitalWrite(in3, HIGH);
//   digitalWrite(in4, LOW) ;

//   }



 




// Base Speeds for Ranges
  // if ((error > 10) || (error < -10)){

  // base_speed = 70;  
  // plus_speed = base_speed + pid;
  // min_speed = base_speed - pid;

  // if (base_speed + pid > 100){
  //   plus_speed =100;    
  // }

  // else if(base_speed - pid > 100){
  //   min_speed = 100;
  // }

  // if (base_speed + pid < 40){
  //   plus_speed = 40;
  // }

  // else if (base_speed - pid < 40){
  //   min_speed = 40;  
  // }
 

  // }

  


  // else{

  // base_speed = 80;  
  // plus_speed = base_speed + pid;
  // min_speed = base_speed - pid;

  // if (base_speed + pid > 130){
  //   plus_speed =130;    
  // }

  // else if(base_speed - pid > 130){
  //   min_speed = 130;
  // }

  // if (base_speed + pid < 50){
  //   plus_speed = 50;
  // }

  // else if (base_speed - pid < 50){
  //   min_speed =50;  
  // }
  // }
  

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

void mazeSolve(void)
{           oled.setTextSize(1);       
            oled.setTextColor(WHITE);
            oled.setCursor(0, 40);    
            oled.print(1);
            oled.print(mode);
            oled.display(); 
   // while (!stat)
    //{
        //readLFSsensors();
             
       if (mode=='n')
        {   

            motorStop();
            goAndTurn (180);
            //lineFollow();

            recIntersection('B');
        }
            
          
         else if(mode== 'c'){
                       
            motorStop();
            //runExtraInch();
            go(80,80);
            int tempInputValue_left = digitalRead(ProxSensor_left);
            int tempInputValue_1 = digitalRead(ProxSensor_1);
            int tempInputValue_2 = digitalRead(ProxSensor_2);
            int tempInputValue_3 = digitalRead(ProxSensor_3);
            int tempInputValue_4 = digitalRead(ProxSensor_4);
            int tempInputValue_5 = digitalRead(ProxSensor_5);
            int tempInputValue_right = digitalRead(ProxSensor_right);

            inputVal[0] = tempInputValue_left;
            inputVal[1] = tempInputValue_1;
            inputVal[2] = tempInputValue_2;
            inputVal[3] = tempInputValue_3;
            inputVal[4] = tempInputValue_4;
            inputVal[5] = tempInputValue_5;
            inputVal[6] = tempInputValue_right;

            LFSensor[0] = inputVal[0];
            LFSensor[1] = inputVal[1];
            LFSensor[2] = inputVal[2];
            LFSensor[3] = inputVal[3];
            LFSensor[4] = inputVal[4];
            LFSensor[5] = inputVal[5];
            LFSensor[6] = inputVal[6];
  
  //farRightSensor = LFSensor[0]
  //farLeftSensor =  LFSensor[6] 
  
            if     ((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 )&&(LFSensor[5]== 1 )&&(LFSensor[6]== 1 ))  {mode = 'n'; }//n= no line
            else if((LFSensor[0]== 0)&&(LFSensor[1]== 0 )&&(LFSensor[5]== 1 )&&(LFSensor[6]== 1 )) {mode = 'l';}//left turn
            else if((LFSensor[0]== 1)&&(LFSensor[1]== 1 )&&(LFSensor[5]== 0 )&&(LFSensor[6]== 0 )) {mode = 'r';}//right turn
            else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )&&(LFSensor[5]== 0 )&&(LFSensor[6]== 0 ))  {mode = 'c'; }// cont line
            // else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = 'l'; }//after the new 2 sensor used we dont want to use this
            // else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = 'r'; }
            else {mode = 'f';}//following line
          if(mode!='c'){
            if ((strtrted==0)){
              go(95,100);
              go(95,100);
              go(95,100);
              strtrted++;
              
            }
            else
            {goAndTurn (270); recIntersection('L');} // or it is a "T" or "Cross"). In both cases, goes to LEFT
          }
            else mazeEnd();} 
         
            
         else if(mode== 'r'){

 

             motorStop();
             go(80,80);
             //runExtraInch();
             //motorStop();
             int tempInputValue_left = digitalRead(ProxSensor_left);
             int tempInputValue_1 = digitalRead(ProxSensor_1);
             int tempInputValue_2 = digitalRead(ProxSensor_2);
             int tempInputValue_3 = digitalRead(ProxSensor_3);
             int tempInputValue_4 = digitalRead(ProxSensor_4);
             int tempInputValue_5 = digitalRead(ProxSensor_5);
             int tempInputValue_right = digitalRead(ProxSensor_right);

            inputVal[0] = tempInputValue_left;
            inputVal[1] = tempInputValue_1;
            inputVal[2] = tempInputValue_2;
            inputVal[3] = tempInputValue_3;
            inputVal[4] = tempInputValue_4;
            inputVal[5] = tempInputValue_5;
            inputVal[6] = tempInputValue_right;

            LFSensor[0] = inputVal[0];
            LFSensor[1] = inputVal[1];
            LFSensor[2] = inputVal[2];
            LFSensor[3] = inputVal[3];
            LFSensor[4] = inputVal[4];
            LFSensor[5] = inputVal[5];
            LFSensor[6] = inputVal[6];


  
  //farRightSensor = LFSensor[0]
  //farLeftSensor =  LFSensor[6] 
  
            if     ((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 )&&(LFSensor[5]== 1 )&&(LFSensor[6]== 1 ))  {mode = 'n'; }//n= no line
            else if((LFSensor[0]== 0)&&(LFSensor[1]== 0 )&&(LFSensor[5]== 1 )&&(LFSensor[6]== 1 )) {mode = 'l';}//left turn
            else if((LFSensor[0]== 1)&&(LFSensor[1]== 1 )&&(LFSensor[5]== 0 )&&(LFSensor[6]== 0 )) {mode = 'r';}//right turn
            else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )&&(LFSensor[5]== 0 )&&(LFSensor[6]== 0 ))  {mode = 'c'; }// cont line
            // else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = 'l'; }//after the new 2 sensor used we dont want to use this
            // else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = 'r'; }
            else {mode = 'f';}//following line
           
            if (mode == 'n') {goAndTurn (90); recIntersection('R');}
            else recIntersection('S');
         }  
            
          else if( 'l'){
            motorStop();
            goAndTurn (270);
            //lineFollow(); 
            recIntersection('L');
          }   
         
         else if(mode== 'f'){
            linefollow();
                
        
         }
}

//}




void setup() 
{    
  Serial.begin(9600);           

   // initialize OLED display with address 0x3C for 128x64
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }  
  
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

  delay(2000);         // wait for initializing
  oled.clearDisplay(); // clear display

  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 10);        // position to display
  oled.println("TEAM SPECTRO"); // text to display
  oled.display();               // show on OLED

  int count = 0;
  
}
//int error_list[5] = {0,0,0,0,0};
//float kp = 0.6;
//float kd = 0.;
//float ki = 0.0000;

//int error = 0;
//int d_error = 0;
//int  i_error = 0;


void loop(){ 
  int tempInputValue_left[5] = {10,10,10,10,10};
  int tempInputValue_1[5] = {10,10,10,10,10};
  int tempInputValue_2[5] = {10,10,10,10,10};
  int tempInputValue_3[5] = {10,10,10,10,10};
  int tempInputValue_4[5] = {10,10,10,10,10};
  int tempInputValue_5[5] = {10,10,10,10,10};
  int tempInputValue_right[5] = {10,10,10,10,10};

  for (int i = 0 ; i < 5 ; i++) {
    tempInputValue_left[i] = digitalRead(ProxSensor_left);
    tempInputValue_1[i] = digitalRead(ProxSensor_1);
    tempInputValue_2[i] = digitalRead(ProxSensor_2);
    tempInputValue_3[i] = digitalRead(ProxSensor_3);
    tempInputValue_4[i] = digitalRead(ProxSensor_4);
    tempInputValue_5[i] = digitalRead(ProxSensor_5);
    tempInputValue_right[i] = digitalRead(ProxSensor_right);
  }
  sort( tempInputValue_left,5);
  sort(tempInputValue_1,5);
  sort(tempInputValue_2,5);
  sort(tempInputValue_3,5);
  sort(tempInputValue_4,5);
  sort(tempInputValue_5,5);
  sort(tempInputValue_right,5);

  inputVal[0] = tempInputValue_left[2];
  inputVal[1] = tempInputValue_1[2];
  inputVal[2] = tempInputValue_2[2];
  inputVal[3] = tempInputValue_3[2];
  inputVal[4] = tempInputValue_4[2];
  inputVal[5] = tempInputValue_5[2];
  inputVal[6] = tempInputValue_right[2];
  
  oled.clearDisplay();
  oled.setTextSize(1);      
  oled.setTextColor(WHITE);
  oled.setCursor(0, 10);   
  oled.println("Team Spectro");
  oled.setCursor(0, 30);    
  oled.print(mode);  

  
  oled.display();
  //////////////////////////////////////////////////////////////////////////////////////  
  error = (-inputVal[0]*3000 -inputVal[1]*800 - inputVal[2]*300 + inputVal[4]*300 +inputVal[5]*800 + inputVal[6]*3000)/(8*(inputVal[0]+inputVal[1]+inputVal[2]+inputVal[3]+inputVal[4]+inputVal[5]+inputVal[6]));

  for (int i = 0; i<4 ; i++){
    error_list[i] = error_list[i+1];
  }

  error_list[4] = error;
  d_error = error_list[4] - error_list[3];

  i_error = i_error + error;
  int pid = kp*error + kd*d_error + ki*i_error;  

  
///////////////////////////////////////////////////////////////////////////////////
if (stage==0){
  readLFSsensors(); 
  if (mode=='c'||mode=='r'||mode=='l'||mode=='n'){
    //motorStop();
    go(95,100);
    //go(95,100);
    //go(100,100);
    i=i+1;
    //if (i>=2){
       
      if((part==0)&& (mode=='n')){
        motorStop();
        i=0;
      }
      if ((part==0)&&(mode=='c')){
            go(95,100);
            go(95,100);
            //go(100,100);
            
            motorStop();
            int tempInputValue_left = digitalRead(ProxSensor_left);
            int tempInputValue_1 = digitalRead(ProxSensor_1);
            int tempInputValue_2 = digitalRead(ProxSensor_2);
            int tempInputValue_3 = digitalRead(ProxSensor_3);
            int tempInputValue_4 = digitalRead(ProxSensor_4);
            int tempInputValue_5 = digitalRead(ProxSensor_5);
            int tempInputValue_right = digitalRead(ProxSensor_right);

            inputVal[0] = tempInputValue_left;
            inputVal[1] = tempInputValue_1;
            inputVal[2] = tempInputValue_2;
            inputVal[3] = tempInputValue_3;
            inputVal[4] = tempInputValue_4;
            inputVal[5] = tempInputValue_5;
            inputVal[6] = tempInputValue_right;

            LFSensor[0] = inputVal[0];
            LFSensor[1] = inputVal[1];
            LFSensor[2] = inputVal[2];
            LFSensor[3] = inputVal[3];
            LFSensor[4] = inputVal[4];
            LFSensor[5] = inputVal[5];
            LFSensor[6] = inputVal[6];
  
  //farRightSensor = LFSensor[0]
  //farLeftSensor =  LFSensor[6] 
  
            if     ((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 )&&(LFSensor[5]== 1 )&&(LFSensor[6]== 1 ))  {mode = 'n'; }//n= no line
            else if((LFSensor[0]== 0)&&(LFSensor[6]== 1 )) {mode = 'l';}//left turn
            else if((LFSensor[0]== 1)&&(LFSensor[6]== 0 )) {mode = 'r';}//right turn
            else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )&&(LFSensor[5]== 0 )&&(LFSensor[6]== 0 ))  {mode = 'c'; }// cont line
            // else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = 'l'; }//after the new 2 sensor used we dont want to use this
            // else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = 'r'; }
            else {mode = 'f';}//following line
            if(mode=='c'){
              go(95,100);
              go(95,100);
              go(95,100);
              
              part=1;//maze started
              }
             i=0;
               }
       if ((part==1)&&(stat==0)){
        mazeSolve();
       oled.setTextSize(1);       
       oled.setTextColor(WHITE);
       oled.setCursor(0, 40);    
       oled.print(2);
       oled.print(path);
       oled.display();
     //     i=0;// First pass to solve the maze
      }

       if ((stat==1)&&(part==1)){
         
         goAndTurn(180);
         go(95,100);
         go(95,100);
         go(95,100);
         //pathIndex = 0;
         stat = 0;
         part=2;
         i=0;
       }
      if (part==2){
        mazeOptimization();
        i=0;

        // Second Pass: run the maze as fast as possible
        //mode = STOPPED;
        }
       if ((stat==1)&&(part==2)){
         //pathIndex = 0;
         //wallfollow()
         stage=1;
         stat = 0;
         part=0;
         i=0;
       }
   //}
  }
  else{
 linefollow();

  }
}
if (stage==1){
  //wallfolow
  //stage=2
}
if (stage==2){
  //blindbox()
  //stage=3
}

if (stage==3){
  //linefollow();
  linefollowlast();
}
}


/////////////////////////////////////////////////////////////////////////////////////////////////
void forward(int lSpeed,int rSpeed){
  analogWrite(enA, lSpeed);
  analogWrite(enB, rSpeed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW) ;
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW) ;
}

void turnLeft(){
  analogWrite(enA, 150);
  analogWrite(enB, 150);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW) ;
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH) ;  
}

void turnRight(){
  analogWrite(enA, 150);
  analogWrite(enB, 150);
   digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH) ;
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW) ;  
}

void linefollowlast(){
  inputVal[0]  = digitalRead(ProxSensor_left);
  inputVal[1]  = digitalRead(ProxSensor_1);
  inputVal[2]  = digitalRead(ProxSensor_2);
  inputVal[3]  = digitalRead(ProxSensor_3);
  inputVal[4]  = digitalRead(ProxSensor_4);
  inputVal[5]  = digitalRead(ProxSensor_5);
  inputVal[6]  = digitalRead(ProxSensor_right);
  
  error = (-inputVal[0]*3000 -inputVal[1]*1000 - inputVal[2]*200 + inputVal[4]*200 +inputVal[5]*1000 + inputVal[6]*3000)/(2*(inputVal[0]+inputVal[1]+inputVal[2]+inputVal[3]+inputVal[4]+inputVal[5]+inputVal[6]));

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





  base_speed = 80;  
  plus_speed = base_speed + pid;
  min_speed = base_speed - pid;


//Minus Speed
//   if (base_speed + pid > 110){
//     plus_speed =110;    
//   }

//   else if(base_speed - pid > 110){
//     min_speed = 110;
//   }

//   if (base_speed + pid < 50){
//     plus_speed = plus_speed - 100;
//   }

//   else if (base_speed - pid < 50){
//     min_speed = min_speed - 100;  
//   }


//   if (min_speed < 0){
//   analogWrite(enA, plus_speed);
//   analogWrite(enB, -min_speed);
//   digitalWrite(in1, HIGH);
//   digitalWrite(in2, LOW) ;
//   digitalWrite(in3, LOW);
//   digitalWrite(in4, HIGH) ; 
    
//   }

//   else if (plus_speed < 0){
//   analogWrite(enA, -plus_speed);
//   analogWrite(enB, min_speed);
//   digitalWrite(in1, LOW);
//   digitalWrite(in2, HIGH) ;
//   digitalWrite(in3, HIGH);
//   digitalWrite(in4, LOW) ; 
    
//   }


//   else{

//   analogWrite(enA, plus_speed);
//   analogWrite(enB, min_speed);
//   digitalWrite(in1, HIGH);
//   digitalWrite(in2, LOW) ;
//   digitalWrite(in3, HIGH);
//   digitalWrite(in4, LOW) ;

//   }



 




// Base Speeds for Ranges
  // if ((error > 10) || (error < -10)){

  // base_speed = 70;  
  // plus_speed = base_speed + pid;
  // min_speed = base_speed - pid;

  // if (base_speed + pid > 100){
  //   plus_speed =100;    
  // }

  // else if(base_speed - pid > 100){
  //   min_speed = 100;
  // }

  // if (base_speed + pid < 40){
  //   plus_speed = 40;
  // }

  // else if (base_speed - pid < 40){
  //   min_speed = 40;  
  // }
 

  // }

  


  // else{

  // base_speed = 80;  
  // plus_speed = base_speed + pid;
  // min_speed = base_speed - pid;

  // if (base_speed + pid > 130){
  //   plus_speed =130;    
  // }

  // else if(base_speed - pid > 130){
  //   min_speed = 130;
  // }

  // if (base_speed + pid < 50){
  //   plus_speed = 50;
  // }

  // else if (base_speed - pid < 50){
  //   min_speed =50;  
  // }
  // }
  if(pid>100){
    turnLeft();
    delay(100);
  }
  else if(pid<-100){
    turnRight();
    delay(100);
  }
  else{
  base_speed = 120;  
  plus_speed = base_speed + pid;
  min_speed = base_speed - pid;

  if (base_speed + pid > 250){
    plus_speed =250;    
  }

  else if(base_speed - pid > 250){
    min_speed = 250;
  }

  if (base_speed + pid < 50){
    plus_speed = 50;
  }

  else if (base_speed - pid < 50){
    min_speed =50;  
  }
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
  forward(plus_speed,min_speed);

}
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
 if  ((distance1==0)||(distance1>50))
  {
    distance1=50;
  }
  
  if  ((distance2==0)||(distance2>50))
  {
    distance2=50;
  }
   if  ((distance3==0)||(distance3>50))
  {
    distance3=50;
  }
   if  ((distance4==0)||(distance4>50))
  {
    distance4=50;
  }
  if  ((distance5==0)||(distance5>60))
  {
    distance5=60;
  }
 
    
    
    
   
  
if ((distance1+distance3)<(distance2+distance4)){
  
   if (distance1==50){
    errorD1=(REFERENCE_DISTANCE-(distance1*2));
    
    
    }
    else{
  fronterrorD1=(REFERENCE_DISTANCE-distance1*0.8);
      backerrorD1=(REFERENCE_DISTANCE-distance3);
      errorD1=((fronterrorD1+backerrorD1*10)/2);
    }
    
     derivative1=errorD1-previousErrorD1;
   integral1+=errorD1;

  previousErrorD1=errorD1;
  
  outputD1=(Kp_d*errorD1+Kd_d*derivative1+Ki_d*integral1)/5;

   speedL=basespeed+outputD1;
  speedR=(basespeed-5)-outputD1;

    
  
  
  }
  
 else{
  
  if (distance2==50){
    errorD2=(REFERENCE_DISTANCE-(distance2*2));
    
    
    }
    else{
  fronterrorD2=(REFERENCE_DISTANCE-distance2*0.8);
      backerrorD2=(REFERENCE_DISTANCE-distance4);
      errorD2=(fronterrorD2+backerrorD2*2)/2;                                          
  
    }
    
   derivative2=errorD2-previousErrorD2;
   integral2+=errorD2;

  previousErrorD2=errorD2;
  
  outputD2=(Kp_d*errorD2+Kd_d*derivative2+Ki_d*integral2)/1.6;

   speedL=basespeed-outputD2;
  speedR=basespeed+outputD2;
  
  
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
  analogWrite(enB,speedL);
  analogWrite(enA,speedR);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  }
bool check_collision(){
  

if(distance5<=COLLISION_DISTANCE){
    return true;
  }else{
    return false;
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
