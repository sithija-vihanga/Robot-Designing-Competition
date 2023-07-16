#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


// Motor A connections
int enA = 3;
int in1 = A0;
int in2 = A1;
// Motor B connections
int enB = 10;
int in3 = A3;
int in4 = A2;

double pos;
double error;
double lineError[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
double Derivative;
double line_Kp = 1;
double line_Kd = 1;
double lineControlSignal;
int velocity = 120;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*const int lineFollowSensor0 = 12; //Using Digital input
const int lineFollowSensor1 = 18; //Using Analog Pin A4 as Digital input
const int lineFollowSensor2 = 17; //Using Analog Pin A3 as Digital input
const int lineFollowSensor3 = 16; //Using Analog Pin A2 as Digital input
const int lineFollowSensor4 = 19; //Using Analog Pin A5 as Digital input
const int farRightSensorPin = 0;  //Analog Pin A0
const int farLeftSensorPin = 1;   //Analog Pin A1*/

int LFSensor[5]={0, 0, 0, 0, 0};

char path[100] = " ";
char mode[8] = " ";


unsigned char pathLength = 0; // the length of the path
int pathIndex = 0; // used to reach an specific array element.

unsigned int status = 0; // solving = 0; reach Maze End = 1


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
  LFSensor[0] = inputVal[0]
  LFSensor[1] = inputVal[1]
  LFSensor[2] = inputVal[2]
  LFSensor[3] = inputVal[3]
  LFSensor[4] = inputVal[4]
  
  //farRightSensor = analogRead(farRightSensorPin);
  //farLeftSensor = analogRead(farLeftSensorPin);
  
  if     ((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = "MC"; error = 0;}
  else if(LFSensor[0]== 0 ) {mode = "MR"; error = 0;}
  else if(LFSensor[4]== 0 ) {mode = "ML"; error = 0;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = "MN"; error = 0;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1 ))  {mode = "FOLLOW"; error = 4;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))  {mode = "FOLLOW"; error = 3;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 ))  {mode = "FOLLOW"; error = 2;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 ))  {mode = "FOLLOW"; error = 1;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = "FOLLOW"_LINE; error = 0;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = "FOLLOW"; error =- 1;}
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = "FOLLOW"; error = -2;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = "FOLLOW"; error = -3;}
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))  {mode = "FOLLOW"; error = -4;}
  
 /* Serial.print (farLeftSensor);
  Serial.print (" <== LEFT  RIGH==> ");
  Serial.print (farRightSensor);
  Serial.print ("  mode: ");
  Serial.print (mode);
  Serial.print ("  error:");
  Serial.println (error);*/
    
}


void runExtraInch(void)
{
  go(2,2)//to move extra length need to change the delay
  motorStop();//need to define
}

void goAndTurn(int direction, int degrees)//need to define this
{
  motorPIDcontrol();
  delay(adjGoAndTurn);
  motorTurn(direction, degrees);
}


void mazeSolve(void)
{
    while (!status)
    {
        readLFSsensors();  
        switch (mode)
        {   
          case "MN":  
            motorStop();
            goAndTurn (LEFT, 180);
            recIntersection('B');
            break;
          
          case "MC": 
            runExtraInch();
            readLFSsensors();
            if (mode != "MC") {goAndTurn (LEFT, 90); recIntersection('L');} // or it is a "T" or "Cross"). In both cases, goes to LEFT
            else mazeEnd(); 
            break;
            
         case "MR": 
            runExtraInch();
            readLFSsensors();
            if (mode == "MN") {goAndTurn (RIGHT, 90); recIntersection('R');}
            else recIntersection('S');
            break;   
            
         case "ML": 
            goAndTurn (LEFT, 90); 
            recIntersection('L');
            break;   
         
         case "FOLLOW": 
            followingLine();
            break;      
        
         }
    }
}

void recIntersection(char direction)
{
  path[pathLength] = direction; // Store the intersection in the path variable.
  pathLength ++;
  simplifyPath(); // Simplify the learned path.
}

void simplifyPath()
{
  // only simplify the path if the second-to-last turn was a 'B'
  if(pathLength < 3 || path[pathLength-2] != 'B')
    return;

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
  
}

void mazeOptimization (void)
{
  while (!status)
  {
    readLFSsensors();  
    switch (mode)
    {
      case "FOLLOW":
        followingLine();
        break;    
      case "MC":
        if (pathIndex >= pathLength) mazeEnd (); 
        else {mazeTurn (path[pathIndex]); pathIndex++;}
        break;  
      case "ML":
        if (pathIndex >= pathLength) mazeEnd (); 
        else {mazeTurn (path[pathIndex]); pathIndex++;}
        break;  
      case "MR":
        if (pathIndex >= pathLength) mazeEnd (); 
        else {mazeTurn (path[pathIndex]); pathIndex++;}
        break;   
    }    
   }  
}

void mazeTurn (char dir) 
{
  switch(dir)
  {
    case 'L': // Turn Left
       goAndTurn (LEFT, 90);      
       break;   
    
    case 'R': // Turn Right
       goAndTurn (RIGHT, 90);     
       break;   
       
    case 'B': // Turn Back
       goAndTurn (RIGHT, 800);     
       break;   
       
    case 'S': // Go Straight
       runExtraInch(); 
       break;
  }
}

void mazeEnd(void)
{
  motorStop();
  
  Serial.print("  pathLenght ==> ");
  Serial.println(pathLength);
  status = 1;
  mode = "STOPPED";
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
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

 // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){2, 4, 5, 6, 7, 8, 9, 11}, SensorCount);
  qtr.setEmitterPin(12);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void loop() {
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  pos= position/1000;
  error=pos-3.5;
    for (int i = 0; i < 4 ; i++) {                        // Storing 5 previous lineError values
    lineError[i] = lineError[i + 1];
  }

   if (lineError[3] > lineError[2]) {
    if (lineError[3] < lineError[2] + 4) {

    }
    else {
      lineError[3] = lineError[2];
    }
  }
  else if (lineError[3] < lineError[2]) {
    if (4 + lineError[3] > lineError[2]) {

    }
    else {
      lineError[3] = lineError[2];
    }
  }
  
 lineError[4] = error;
 Derivative = (lineError[3] - lineError[1]);     //derivative
 
 lineControlSignal = 0.8 * lineError[4] + Derivative;
 lineControlSignal = 87 * lineControlSignal/4;//  lineControlSignal maped from (-4_4)->(-87_+87)


  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  
  Serial.println(position);

  delay(250);
  go(velocity+lineControlSignal,velocity-lineControlSignal);
}
