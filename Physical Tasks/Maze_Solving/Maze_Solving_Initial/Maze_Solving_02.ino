#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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

// Motor A connections
int enA = 3;
int in1 = A0;
int in2 = A1;
// Motor B connections
int enB = 10;
int in3 = A3;
int in4 = A2;

int error_list[5] = {0,0,0,0,0};
float kp = 1.0;
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
  error = (-inputVal[0]*250 -inputVal[1]*100  + inputVal[3]*100 + inputVal[4]*250)/(inputVal[0]+inputVal[1]+inputVal[2]+inputVal[2]+inputVal[3]+inputVal[4]);

  for (int i = 0; i<4 ; i++){
    error_list[i] = error_list[i+1];
  }

  error_list[4] = error;
  d_error = error_list[4] - error_list[3];

  i_error = i_error + error;
  int pid = kp*error + kd*d_error + ki*i_error;  

  
///////////////////////////////////////////////////////////////////////////////////
  readLFSsensors(); 
  if (mode=='c'||mode=='r'||mode=='l'||mode=='n'){
    motorStop();
    i=i+1;
    if (i>=2){
       
      if((part==0)&& (mode=='n')){
        motorStop();
        i=0;
      }
      if ((part==0)&&(mode=='c')){
            runExtraInch();
            
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
            else if((LFSensor[0]== 0)&&(LFSensor[1]== 0 )&&(LFSensor[5]== 1 )&&(LFSensor[6]== 1 )) {mode = 'l';}//left turn
            else if((LFSensor[0]== 1)&&(LFSensor[1]== 1 )&&(LFSensor[5]== 0 )&&(LFSensor[6]== 0 )) {mode = 'r';}//right turn
            else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )&&(LFSensor[5]== 0 )&&(LFSensor[6]== 0 ))  {mode = 'c'; }// cont line
            // else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = 'l'; }//after the new 2 sensor used we dont want to use this
            // else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = 'r'; }
            else {mode = 'f';}//following line
            if(mode=='c'){
              part=1;//maze started
              }
             i=0;
               }
       if (part==1){
        mazeSolve();
       oled.setTextSize(1);       
       oled.setTextColor(WHITE);
       oled.setCursor(0, 40);    
       oled.print(part);
       oled.print(mode);
       oled.display();
          i=0;// First pass to solve the maze
      }

       if ((stat==1)&&(part==1)){
         
         goAndTurn(180);
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
         stat = 0;
         part=0;
         i=0;
       }
   }
  }
  else{
 lineFollow();

  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void mazeTurn (char dir) 
{
       
  switch(dir)
  {
    case 'L': // Turn Left
       goAndTurn (270);
       //lineFollow();      
       break;   
    
    case 'R': // Turn Right
       goAndTurn (90);
       //lineFollow();     
       break;   
       
    case 'B': // Turn Back
       goAndTurn (180);
       //lineFollow();     
       break;   
       
    case 'S': // Go Straight
       runExtraInch();
      // lineFollow(); 
       break;
  }
}

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
  delay(1000);

}

void goAndTurn( int Degrees){
  if( Degrees==270){
  analogWrite(enA, 110);
  analogWrite(enB, 110);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW) ;
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH) ; 
  delay(1000);
  }
  if( Degrees==180){
      analogWrite(enA, 110);
  analogWrite(enB, 110);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW) ;
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH) ; 
  delay(2000);
  }
  if( Degrees==90){
  analogWrite(enA, 110);
  analogWrite(enB, 110);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH) ;
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW) ; 
  delay(1000);
  }
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
 delay(500);
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
       oled.print(345);
       oled.print(mode);
       oled.display();  
    switch (mode)
    {
      case 'f':
        lineFollow();
        break;    
      case 'c':
        if (pathIndex <= 0) {mazeEnd (); }
        else {mazeTurn (path[pathIndex]); pathIndex=pathIndex-1;}
        break;  
      case 'l':
        if (pathIndex <= 0) {mazeEnd ();} 
        else {mazeTurn (path[pathIndex]); pathIndex=pathIndex-1;}
        break;  
      case 'r':
        if (pathIndex <= 0) {mazeEnd ();} 
        else {mazeTurn (path[pathIndex]); pathIndex=pathIndex-1;}
        break;   
    }    
   }  
//}

void lineFollow(){
  int base_speed = 80 ;  
  int plus_speed = base_speed + pid;
  int min_speed = base_speed - pid;
  if (base_speed + pid > 255){
    plus_speed =255;    
  }

  else if(base_speed - pid > 255){
    min_speed = 255;
  }

  if (base_speed + pid < 50){
    plus_speed = 50;
  }

  else if (base_speed - pid < 50){
    min_speed =50;  
  }

  analogWrite(enA, plus_speed);
  analogWrite(enB, min_speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW) ;
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW) ;

   delay(500);
}
void runExtraInch(void)
{
  go(80,80);//to move extra length need to change the delay
  motorStop();//need to define
}
void mazeEnd(void)
{
  motorStop();

       oled.setTextSize(1);       
       oled.setTextColor(WHITE);
       oled.setCursor(0, 40);    
       oled.print(257);
       oled.print(mode);
       oled.display();
  
  Serial.print("  pathLenght == ");
  Serial.println(pathLength);
  stat = 1;
  //mode = "AE";
}


void mazeSolve(void)
{           oled.setTextSize(1);       
            oled.setTextColor(WHITE);
            oled.setCursor(0, 40);    
            oled.print(200);
            oled.print(mode);
            oled.display(); 
   // while (!stat)
    //{
        //readLFSsensors();
             
        switch (mode) 
        {   
         case 'n': 
          
            motorStop();
            goAndTurn (180);
            //lineFollow();
            recIntersection('B');
            break;
          
          case 'c':
                       
            motorStop();
            runExtraInch();
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

            if (mode != 'c') {goAndTurn (270); recIntersection('L');} // or it is a "T" or "Cross"). In both cases, goes to LEFT
            else mazeEnd(); 
            break;
            
         case 'r':

          oled.setTextSize(1);       
            oled.setTextColor(WHITE);
            oled.setCursor(0, 40);    
            oled.print(250);
            oled.print(part);
            oled.display(); 

             motorStop();
             runExtraInch();
             motorStop();
             tempInputValue_left = digitalRead(ProxSensor_left);
             tempInputValue_1 = digitalRead(ProxSensor_1);
             tempInputValue_2 = digitalRead(ProxSensor_2);
             tempInputValue_3 = digitalRead(ProxSensor_3);
             tempInputValue_4 = digitalRead(ProxSensor_4);
             tempInputValue_5 = digitalRead(ProxSensor_5);
             tempInputValue_right = digitalRead(ProxSensor_right);

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
            break;   
            
         case 'l':
            motorStop();
            goAndTurn (270);
            //lineFollow(); 
            recIntersection('L');
            break;   
         
         case 'f': 
            lineFollow();
            break;      
        
         }
    }
//}

void recIntersection(char Direction)
{
  path[pathLength] = Direction; // Store the intersection in the path variable.
  pathLength ++;
  simplifyPath(); // Simplify the learned path.
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
    switch(path[pathLength-i])
    {
      case 'R':
        totalAngle += 90;
        break;
      case 'L':
        totalAngle += 270;
        break;
      case 'B':
        totalAngle += 180;
        break;
    }
  }
  

  // Get the angle as a number between 0 and 360 degrees.
  totalAngle = totalAngle % 360;

  // Replace all of those turns with a single one.
  switch(totalAngle)
  {
    case 0:
  path[pathLength - 3] = 'S';
  break;
    case 90:
  path[pathLength - 3] = 'R';
  break;
    case 180:
  path[pathLength - 3] = 'B';
  break;
    case 270:
  path[pathLength - 3] = 'L';
  break;
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







//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
