//TEAM SPECTRO

#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Motor.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <webots/DistanceSensor.hpp>
#include <string>
#include <vector>


using namespace webots;
using namespace cv;
using namespace std;

//////////////////////////////// Functions //////////////////////////////////
////////// Vision Controls ////////

void getContours(Mat imgIn, Mat imgOut);

////// Basic motion controls /////

void forward();
void reverse();
void turnRight();
void turnLeft();
void stop();
void skidDrive(double Lspeed, double Rspeed);
void readLineSensors();
int  compareLines( double values);
void pidLineFollower();
double speedControl(double speed);
void floorPattern();
int compare(const void* a, const void* b);
void search180();
void imagePointFollowing();
void grabTheBox();
void readWallSensors();
void controlForward(int n); 
void liftTheBox();
void brokenBridge();   //To correct the position of the arm
void bridgeNavigator();
bool delaySteps(int n);
void wallFollowing();

//////////////////////////////////////////////////////////////////////////////  

/////////////////////////////// Definitions //////////////////////////////////
 //Motor *base; 
 Motor *camControl;
 Motor *wheels[4];  

 DistanceSensor *DS[8];  //IR sensors
 DistanceSensor *US[7];  //Ultrasonic sensors
 
 Motor *armMotors[6]; 
 

 //////////////////////////////////////////////////////////////////////////////

// box gripper control variables
double rotateBase = 0.15;
double upAndDown = 0.0;
double armrotate = -2.2;
double catchRight = 0.5;
double catchLeft = -0.22;
int stage = 0;
int gripperCounter = 0;

//line following variables
int counter = 0;
int wallSpeed = 5;
int rawSensorReadings[8];  //Store raw values
int sensorReadings[8];
double lineControlSignal = 0.0;
double lineErrorSum = 0.0;
double lineError[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
int leftSpeed = 0;
int rightSpeed = 0;

//line following PID  variables 
double line_Kp = 1;
double line_Kd = 1;
double line_Ki = 0.01;
double lineErrorPredict = 0.0;

//floor pattern variables 
string floorColor = "Black";
string floorLocation= "Unknown";
int floorErrorControl[5] = {0,0,0,0,0};
int sortedFloorError[5];
int hardTurnCounter = 0;
int whiteLineCounter = 0;
int blackLineCounter = 0;
bool T_Junction = false;
int turnRightCounter = 0;
int turnLeftCounter = 0;
bool rightTurn = false;
bool leftTurn = false;


//Box Arena Variables
bool boxSearch = true;
int searchCounter = 0;
int xBoxPoints[5]= {0,0,0,0,0};  //Sorted x points of the box
int xPointUnsorted[5]= {0,0,0,0,0};  //unsorted xpoints
bool sendForward = true;   //send the robot forward for some time to reduce errors
bool firstHop = false;
bool initiateHop = true;
int searchRange = 5;
int leftWallCounter = 0;
int rightWallCounter = 0;
bool rightWall = false;
bool leftWall = false;
int noWall = 0;
  
// Vision control variables
int key;          //Keyboard input
int pixelSum = 0; //Sum of pixels in given region
string color;     //Store detected color
double distance = 10;  //measured distance.  Not zero anywhere
int boxSize = 5;
int snapCounter = 0;   //Count iterations
bool takeSnap = true;  //Take image
bool camEnable = true;  //enable or diable camera


//Image point Follwoing Variables
int turnLeftControl = 0;
int turnRightControl = 0;
bool reached = false;
bool orientation = false;

//Wall following
int sonicSensorReadings[7] = {0,0,0,0,0,0,0};

//Control forward
int iter = 0;
bool goingForward = false;

//Broken bridge
string ramp = "Not_Detected";
int rampDetector = 0;
string rampPosition = "none";
bool armPosition = false;
int imageSensorReadings[32];
int imageSensorLineY = 50;
int followingXPoint = 42;
int brokenBridgeError = 0;
bool horizontalSurface = false;
int horizontalCounter = 0;
//int countToHoles = 0;
bool holeDetected = false;
bool reachedHole = false;
int placingStage = 0;

//Box handling
bool oneBoxDown = false;
bool overRideBrokenBridge = false;

// delay step function
int delayCounter = 0;

//Double wall following

double segmentedWallError = 0.0;
int segmentedBaseSpeed = 7;
double segmentedLeftSpeed;  
double segmentedRightSpeed; 
/*int wall_speed=7;
double error,previous_error1=0,previous_error2=0,integral1,integral2,derivative1,derivative2,PID_error1,PID_error2;
  double wallKp=0.5;
  double wallKi=0.01;
  double wallKd=0.01;
  double max_speed=30.0;
  double min_speed=30.0;
  int leftmotor_speed,rightmotor_speed;
  double error1,error2;
  int REFERENCE_DISTANCE=800;  */

  // Box navigator
  int wallCounter = 0;
  bool turnOverRide = false;
  int sensorSumCount = 0;




//******************************** Control Commands *****************************************
string location = "Line_Following";


//*******************************************************************************************
int main() {
  /////////////////////////////////////////// Initialize //////////////////////////////////////////
  //Initialize the robot//
  Robot *robot = new Robot();
  const int timeStep = robot->getBasicTimeStep();

  //Initialize the camera//
  Camera *camera = robot->getCamera("cam");
  camera->enable(timeStep);
  const int width = camera->getWidth();
  const int height = camera->getHeight();

  
  char ultraSonicNames[7][12] = {"Front","FrontRight","FrontLeft","Right_1","Right_2","Left_1","Left_2"};
  char armMotorNames[6][20] = {"linear motor","rotational motor1","cmBase","camMotor","rotational motor2","rotational motor3"};
  char wheels_names[4][15] = {"leftTopMotor", "leftRearMotor","rightTopMotor", "rightRearMotor"};                                //store names of motors
  char dsNames[8][8] = {"IR1","IR2","IR3","IR4","IR5","IR6","IR7","IR8"}; 
  
  for (int i = 0; i < 4; i++) {                                      //Initializing motors using a for loop
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY); 
  }
  
    for (int j = 0; j<8; j++){                                                    //Enabling distance sensors using for a loop
     DS[j] = robot->getDistanceSensor(dsNames[j]);                 
     DS[j]->enable(timeStep);                                      
 }
 
  for (int j = 0; j<7; j++){                                                    //Enabling distance sensors using for a loop
     US[j] = robot->getDistanceSensor(ultraSonicNames[j]);                 
     US[j]->enable(timeStep);                                      
 }
 
 for (int i = 0; i < 6; i++) {                                      //Initializing motors of the arm using a for loop
      armMotors[i] = robot->getMotor(armMotorNames[i]);
 } 
armMotors[2]->setPosition(rotateBase);  // Set camera position
 
  ////////////////////////// Image processing Initialization //////////////////////////////
  int grayImage[width][height];
  cout<<"width is :"<<width<<" height is :"<<height<<endl;  //Output image size
  Display *display = robot->getDisplay("display");
  Mat imageGray = Mat::zeros(Size(width,height),CV_8UC1);
  Mat imageMat = Mat(Size(width, height),  CV_8UC4);     //generate empty image

  /////////////////////////////////////////////////////////////////////////////////////////


  while (robot->step(timeStep) != -1) {
    cout<<"location is: "<<location<<endl;
    cout<<rampDetector<<endl;
    cout<<"ramp location :"<<rampPosition<<endl;
    cout<<" bridge error: "<<brokenBridgeError<<endl;
    cout<<"broken bridge :"<<brokenBridgeError<<endl;
    //////////////////////////////// Basic motion ////////////////////////////////////////
    if (location=="Line_Following"){
        leftSpeed = speedControl(wallSpeed+lineControlSignal);
        rightSpeed = speedControl(wallSpeed-lineControlSignal);
             
             //****************** CHANGE THIS ************************
            if(floorLocation=="T_Junction"){   //Decision at first junction
               rightTurn = true;
             }
             //*******************************************************

             if(rightTurn){
                 turnRight();
                 turnRightCounter++;
                 if((turnRightCounter>30) and (turnRightCounter<45)){
                      forward();    
                 }
                 if(turnRightCounter>45) {
                      turnRightCounter = 0;
                      rightTurn = false;
                 }
                 
             }
             
             else if(leftTurn){
                 turnLeft();
                 turnLeftCounter++;
                 if((turnLeftCounter>30) and (turnLeftCounter<45)){
                      forward();    
                 }
                 if(turnLeftCounter>45) {
                      turnLeftCounter = 0;
                      leftTurn = false;
                 }
             }
             else{
             skidDrive(leftSpeed,rightSpeed);
             }
             counter+=1;
             readLineSensors();
             readWallSensors();
             floorPattern();
             pidLineFollower();
    }
    else{
            stop();  ///CHANGE THIS
    }
    ///////////////////////////////////////////////////////////////////////////////////////////    
    
    

    //////////////////////////////// Image Processing ///////////////////////////////////////

    const unsigned char *data = camera->getImage();     //Taking a picture from camera

    if (data) {
      if(((snapCounter == 0) or takeSnap) and camEnable){          // Take discrete snaps
            imageMat.data = (unsigned char *)data;                         //convert const unsigned char to unsigned char

            for (int i = 0 ; i < width ; i++){
              for ( int j = 0 ; j< height ; j++){
                  int value = camera->imageGetGray( data,width, i, j);
                  grayImage[j][i] = value+100;    //Change color ranges
                  
              }
            }
      }
      
      ////////////////////////////////// Chess Board Image Processing /////////////////////////////////////////
      
      if (location == "Chess_Board"){
        if(snapCounter == 0){          // Take discrete snaps
    
          for(int i = (31-boxSize); i<(31+boxSize); i++){             //Calculate pixel values to identify black or white
            for(int j = (31-boxSize); j<(31+boxSize); j++){           //Change the box with distance
                pixelSum = pixelSum+grayImage[j][i];
                  
            } 
          }
          pixelSum = pixelSum/((2*boxSize)^2);    //Check for white or black 

          if(pixelSum >= 2700){   //Threshhold for white
            color = "White";
            cout<<color<<endl; 
          }
          else if(pixelSum<2700){
            color = "Black";
            cout<<color<<endl;
          } 
          rectangle(imageMat, Point((31-boxSize),(31-boxSize)),Point((31+boxSize),(31+boxSize)),Scalar(0, 255, 0), 1 , LINE_8);
        }
      }
      ///////////////////////////////////////////////Box Arena vision //////////////////////////////////
      
      if (location == "Box_Arena"){
          if(((snapCounter == 0) or takeSnap) and camEnable){          // Take discrete snaps
            Mat Img = Mat(width, height , CV_8UC4, &grayImage);
            GaussianBlur(Img, Img, Size(5, 5),0); 
            Mat imageCanny; 
            Canny(Img, imageCanny, 100, 300,3);
            Mat kernel = getStructuringElement(MORPH_RECT, Size(3,3));
            dilate(imageCanny,imageCanny,kernel); 
            getContours(imageCanny,imageMat);  //Get contours
          }  
            //Turn left and right for box search
            if (boxSearch){
            search180();  
            }
            if ( not boxSearch){
            
            readWallSensors();
            imagePointFollowing();
            }
      } 
    
    // After reaching top of the broken bridge

    if (location == "Broken_Bridge"){
        camEnable = true;
        brokenBridge();
        // first go to right side of the bridge
       if (oneBoxDown and overRideBrokenBridge){     // Change the box location
           /// write this code
           followingXPoint = 12;
           overRideBrokenBridge = true; // initiate the previous box placing algorithm again
         }
       if( (not oneBoxDown) or ( overRideBrokenBridge)){  // Go write of the broken bridge

            line(imageMat,Point(followingXPoint,0),Point(followingXPoint,64),Scalar(155,255,100),1);
            line(imageMat,Point(0,imageSensorLineY),Point(64,imageSensorLineY),Scalar(155,255,100),1);
            for(int j =0; j<32 ; j++){
                imageSensorReadings[j] =  grayImage[imageSensorLineY][2*j];
                if (imageSensorReadings[j] > 250){
                  imageSensorReadings[j] = 1;  
                }
               else{
                   imageSensorReadings[j] = 0;
               }
               cout<<imageSensorReadings[j]<<"";
            }
            cout<<" "<<endl;
            brokenBridgeError = 0;  //Initialize to 0
            for(int i = 0; i <32; i++) {   // Iterate over measured 32 values
                if (2*i<= followingXPoint){  //Give negative errors
                    brokenBridgeError += -(followingXPoint - 2*i)*imageSensorReadings[i];
                }
                else{  // give positive erros
                    brokenBridgeError += (2*i-followingXPoint)*imageSensorReadings[i];
                }
            }
            cout<<"    "<<brokenBridgeError<<endl;
            
            readWallSensors();
            bridgeNavigator();
           
            
            cout<<sonicSensorReadings[0]<<endl;
            //cout<<sensorReadings[0]<<sensorReadings[1]<<sensorReadings[2]<<sensorReadings[3]<<sensorReadings[4]<<endl;
            
        }
       
       }
        
        
        if(location == "oneBoxComplete"){    //Turn right when detected first box has correctly placed
            if(delaySteps(250)){
                 location = "Line_Following";
                 oneBoxDown = true;
            }
            else{
                turnRight();
               
            } 
         }   

        if(location == "Wall_Following"){
            wallFollowing();
         }

        if(location == "Box_Navigator"){
          readLineSensors();
          int sensorSum = sensorReadings[0]+sensorReadings[1]+sensorReadings[2]+sensorReadings[3]+sensorReadings[4]+sensorReadings[5]+sensorReadings[6];
          cout<<" line sensor sum: "<<sensorSum<<endl;
          if(sensorSum>0){
            sensorSumCount ++;
          }
          if(sensorSumCount>5){
            location = "Line_Following";
            sensorSumCount = 0;
;          }
          wallFollowing();
        }
     /*   ////////////////////////////////////// remove this only for testing //////////////////    
       
       if (oneBoxDown){
            if(delaySteps(250)){
               location = "oneBoxComplete";
            }
            else{
              location = "Line_Following";
            }
        }
        
      /////////////////////////////////////////////////////////////////////////////////////   */ 
          
         

      ImageRef *ir = display->imageNew(width, height, imageMat.data , Display::BGRA);     //Generating output image
      display->imagePaste(ir, 0, 0, false);
      display->imageDelete(ir);
      
      
    }
    
   
      snapCounter++;
      if(snapCounter>3){          //Take snaps after every 10 iterations
        snapCounter=0;
      }
      
      
  }

  delete robot;
  return 0;
}


 
void getContours(Mat imgIn, Mat imgOut){
  vector<vector<Point>>contours;
  vector<Vec4i>hierarchy;
  
  findContours(imgIn,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
  int contSize =  contours.size();
  for (int i = 0; i< contSize; i++){
        
        int pointNum = 0;             //Initialize x points to 0
        for (int j = 0; j<5; j++){
          xPointUnsorted[j] = 0;
        }
        int area = contourArea(contours[i]);
        //cout<< "area: "<<area << endl;
        //cout<< sizeof(contours[i])<<endl;
        //vector<vector<Point>> conPoly(contours.size());
        if ((area > 80) and (sizeof(contours[i])<30)){     //Change threshhold values for better callibration
            //float peri = arcLength(contours[i], true);
            //approxPolyDP(contours[i], conPoly[i], 0.02*peri,true);
            drawContours(imgOut, contours, i, Scalar(255,0,255), 1);
            //cout<<contours[i][2]+contours[i][3]<<endl;
            int xPoint = contours[i][0].x;
            int yPoint = contours[i][0].y;
            
            
            
            if (xPoint<64 and yPoint <64 and yPoint>15){
                 circle(imgOut,Point(xPoint,yPoint),3,Scalar(0,69,255),FILLED);
                 line(imgOut,Point(0,15),Point(64,15),Scalar(155,255,100),2);
                 line(imgOut,Point(32,0),Point(32,64),Scalar(0,255,0),1);
                 boxSearch = false;                    //To stop 180 search
                 xPointUnsorted[pointNum] = xPoint;        //Upload new x points
                 pointNum++;
                 qsort(xPointUnsorted,5,sizeof(int),compare);
                 for(int k = 0;k<5;k++){
                     xBoxPoints[k] = xPointUnsorted[k];
                   }
                 
                 /* if(sendForward){
                      cout<<"working"<<endl;
                      controlForward(200);
                  }
                  sendForward = false; */
                 //cout<<Point(xPoint,yPoint)<<endl;
                 //imagePointFollowing(xBoxPoints);
                 
                 //cout<<xBoxPoints[0]<<"  "<<xBoxPoints[1]<<" "<<xBoxPoints[2]<<"  "<<xBoxPoints[3]<<"  "<<xBoxPoints[4]<<endl;
                 }
            /*
            int xPoint = 0;
            int yPoint = 0;
            for (int j =0 ; j<sizeof(contours[i]); j++){
                contours[i][0] += contours[i][j];
              }
             xPoint = contours[i][0].x/sizeof(contours[i]);
             yPoint = contours[i][0].y/sizeof(contours[i]);
             cout<<Point(xPoint,yPoint)<<endl;
             if (xPoint<64 and yPoint <64 and yPoint>15){
                 circle(imgOut,Point(xPoint,yPoint),3,Scalar(0,69,255),FILLED);
                } cout<<Point(xPoint,yPoint)<<endl;
                 */
              
             
             
            //cout<< "area: "<<area <<"points :"<<conPoly.size()<<endl;
            
            
       }
        
  }
  
  

}  

 void forward(){
               wheels[0]->setVelocity(wallSpeed);
               wheels[1]->setVelocity(wallSpeed);
               wheels[2]->setVelocity(wallSpeed);
               wheels[3]->setVelocity(wallSpeed);
 }
void reverse(){
               wheels[0]->setVelocity(-wallSpeed);
               wheels[1]->setVelocity(-wallSpeed);
               wheels[2]->setVelocity(-wallSpeed);
               wheels[3]->setVelocity(-wallSpeed);
 }
 
 void turnRight(){
               wheels[0]->setVelocity(wallSpeed);
               wheels[1]->setVelocity(wallSpeed);
               wheels[2]->setVelocity(-wallSpeed);
               wheels[3]->setVelocity(-wallSpeed);
 }

 void turnLeft(){
               wheels[0]->setVelocity(-wallSpeed);
               wheels[1]->setVelocity(-wallSpeed);
               wheels[2]->setVelocity(wallSpeed);
               wheels[3]->setVelocity(wallSpeed);
 }
 
 void stop(){
               wheels[0]->setVelocity(0);
               wheels[1]->setVelocity(0);
               wheels[2]->setVelocity(0);
               wheels[3]->setVelocity(0);
 }

 void skidDrive(double Lspeed , double Rspeed){                //Differential drive
               wheels[0]->setVelocity(Lspeed);
               wheels[1]->setVelocity(Lspeed);
               wheels[2]->setVelocity(Rspeed);
               wheels[3]->setVelocity(Rspeed);
 }
 
 void readLineSensors(){
               
                for (int j = 0; j <8; j++){
                   rawSensorReadings[j] = DS[j]->getValue();                                                    //Enabling distance sensors using for a loop
                   sensorReadings[j] = compareLines( rawSensorReadings[j]);              
                   cout<<rawSensorReadings[j]<<"  ";                        
                   }
                   cout<<" "<<endl;
               
                   
 }
 
 void readWallSensors(){
            for (int j = 0; j <7; j++){
                   sonicSensorReadings[j] = US[j]->getValue();                                                    //Enabling distance sensors using for a loop           
                                              
                   }
  }
 
 int compareLines(double value){    //Edit the threshhold values
 
                //cout<< value <<endl;                   // Extracting colors
                if (value<100){                        
                  return 1;                         //White
                }
                else if((value>100) and (value <150)){   //Red
                  return 1;
                }
                else if(value>600 and value <700){     //Blue
                  return 1;
                }
                else if(value>700){                 //Black
                  return 0;
                }
                else {
                  return 1;
                }
 }

 /*double maxSpeedControl(double lineControlSignal){
                double speed = maxSpeed;
                if(lineControlSignal > 4){
                   speed = maxSpeed - lineControlSignal;
                }
                 return speed; 
 } */

double speedControl(double speed){
              if (speed>10){
                speed = 10;
              }
              else if (speed<-10){
                speed = -10;
              }
              return speed;
 }

 void pidLineFollower(){
                cout<<sonicSensorReadings[0]<<endl;
                for (int i = 0; i<4 ; i++){                           // Storing 5 previous lineError values
                  lineError[i] = lineError[i+1];
                }

              
                if (lineError[3]>lineError[2]){                          
                  if (lineError[3]<lineError[2]+8){                      
                      
                  }
                  else{
                    lineError[3] = lineError[2];
                  }
                }
                else if(lineError[3]<lineError[2]){
                  if (8+lineError[3]>lineError[2]){                      
                      
                  }
                   else{
                    lineError[3] = lineError[2];
                  }
                }
                
                lineErrorSum += lineError[3];                    // Get sum of the errors
                

          
                lineError[4] = 3*(sensorReadings[0]-sensorReadings[7])+2*(sensorReadings[1]-sensorReadings[6])+(sensorReadings[2]-sensorReadings[5]);      // Error algorithm

               
                lineErrorPredict = (lineError[3]-lineError[1]);       // Derivative
                      
                /////////////////////////////////    PID control signal  ////////////////////////////

                lineControlSignal = 0.8*lineError[4] + 0.5*line_Ki*lineErrorSum + lineErrorPredict;    

                ///////////////////////////////////////////////////////////////////////////////////  
                
                // Defining boundaries
                
                if ((lineErrorSum > 15) or (lineErrorSum < 15)){                      //Regulating the max speed
                     wallSpeed = 3;
                  }
                else if ((lineErrorPredict>8)){                      //Regulating the max speed
                     wallSpeed = 2;
                  }

                if (lineErrorSum>30){                               //Regulating max lineError
                  lineErrorSum = 30;
                } 
                else if (lineErrorSum<-30){                               //Regulating max lineError
                  lineErrorSum = -30;
                }

                if((sonicSensorReadings[0]<750) and (rampPosition == "none")){  // Detect bottom of the ramp
                   rampDetector++;
                   if (rampDetector>5){
                      rampPosition = "Bottom";
                      rampDetector = 0;
                   }
                   }
                
                
                else if((sonicSensorReadings[0]>980) and (rampPosition == "Bottom")){  // Detect bottom of the ramp
                   rampDetector++;
                   if (rampDetector>250){
                      rampDetector = 0;
                      rampPosition = "Top";
                      location = "Broken_Bridge";
                   }
                   }
                else{
                  rampDetector = 0;
                }
                // To get the arm to front  
                if(rampPosition == "Bottom"){
                //cout<<"arm"<<endl;
                  if(armrotate <-0.8){
                        armrotate += 0.05;
                              }  
                  else {
                        armrotate +=0;
                        //stage++; 
                        } 
                 armMotors[1]->setPosition(armrotate);
                }
                
           

 }
 
 void floorPattern(){

  //***************************** CHANGE THIS **************************
      //Convert sensor readings to string
     string groundScan = to_string(sensorReadings[0]) + to_string(sensorReadings[1]) + to_string(sensorReadings[2]) + to_string(sensorReadings[3]) + to_string(sensorReadings[4]) + to_string(sensorReadings[5]) + to_string(sensorReadings[6]) + to_string(sensorReadings[7]) ;

     int onesCount = sensorReadings[0]+sensorReadings[1]+sensorReadings[2]+sensorReadings[3]+sensorReadings[4]+sensorReadings[5]+sensorReadings[6]+sensorReadings[7];

     // Store 5 values of above strings 
     for (int j = 0; j<=3; j++){
       floorErrorControl[j] = floorErrorControl[j+1];   
     }
     floorErrorControl[4] = onesCount;
    
    
        //Sort the list to reduce errors
        for(int j =0; j<=4;j++){
            sortedFloorError[j] = floorErrorControl[j];
          }
        qsort(sortedFloorError, 5,sizeof(int),compare);
      //*********************************************************************
 
  

     //floorErrorControl.sort();
     cout<< groundScan <<"    " <<onesCount <<endl;
     //cout<< floorErrorControl[0] << floorErrorControl[1] << floorErrorControl[2] << floorErrorControl[3] << floorErrorControl[4]  <<endl;
     cout<< floorLocation  << endl;
     //cout<< sortedFloorError[2]  << endl;
     
     if ((not rightTurn) and (not leftTurn) and((sortedFloorError[2] == 2) or (sortedFloorError[2] == 3))){  //Line following
           hardTurnCounter = 0;
           whiteLineCounter = 0;
           blackLineCounter = 0;
           floorLocation = "Line_Fllowing";
       }
      
    // Hard Turn
     else if ((not rightTurn) and (not leftTurn) and (sortedFloorError[2] >=5) and (sortedFloorError[2]<=7)){
           hardTurnCounter ++;
           if (hardTurnCounter >=5){
             floorLocation = "Hard_Turn";
             whiteLineCounter = 0;
             blackLineCounter = 0;
             T_Junction = false;
           }
          }
  
          
    //White box     
     else if((not rightTurn) and (not leftTurn) and (sortedFloorError[2] == 8)){
           whiteLineCounter ++;
           hardTurnCounter = 0;
           blackLineCounter = 0;
           if(whiteLineCounter>5){
             T_Junction = true;
           }
           
           if (whiteLineCounter>=20){
             floorLocation = "White_Box";
             T_Junction = false;
             whiteLineCounter = 0;
           }   
     }
     
     // Check whether robot is out of track
     else if ((not rightTurn) and (not leftTurn) and(sortedFloorError[2] == 0)){
           hardTurnCounter = 0;
           whiteLineCounter = 0;
           blackLineCounter++;
           if (blackLineCounter >10){   // Change the threshhold
           floorLocation = "Lost";
           }
       }

    //After detecting white box
    if(floorLocation == "White_Box"){
        controlForward(500);
        //cout<<"forward"<<endl;
    } 
    
    //T Junction
    if((not rightTurn) and (not leftTurn) and T_Junction and (sortedFloorError[2] == 0)){
             floorLocation = "T_Junction";
              hardTurnCounter = 0;
              whiteLineCounter = 0;
              blackLineCounter = 0;
              T_Junction = false;
             
       }
       
    if (floorLocation == "Hard_Turn"){
        if((not rightTurn) and (sensorReadings[4]== 1) and (sensorReadings[5]== 1) and (sensorReadings[6]== 1) and (sensorReadings[7]== 1)){
              floorLocation = "Hard_Turn_Left";
              leftTurn = true;
              /*turnLeft();
              turnLeftCounter++;
              if (turnLeftCounter > 30){
                  leftTurn = false;
                  turnLeftCounter = 0;
                  cout<<"complete"<<endl; 
              } */
        }
        else if((not leftTurn) and(sensorReadings[0]== 1) and (sensorReadings[1]== 1) and (sensorReadings[2]== 1) and (sensorReadings[3]== 1)){
              floorLocation = "Hard_Turn_Right";
              rightTurn = true;
             /* turnRight();
              turnRightCounter++;
              if (turnRightCounter > 30){
                  rightTurn = false;
                  turnRightCounter = 0;
                  cout<<"complete"<<endl;
              } */
        }
    }
     
           
       
 }
 
 int compare(const void* a , const void* b){
    const int* x = (int*)a;
    const int* y = (int*)b;
    
    if (*x>*y)
        return 1;
    else if(*x<*y)
        return -1;
    return 0;
 }
 
void search180(){
     if(searchCounter<120){
        turnLeft();
        searchCounter++;
        }
      else if(searchCounter<340){
        turnRight();
        searchCounter++;
      }
      
      
}

void imagePointFollowing(){
    //cout<<"front sensor: "<<sonicSensorReadings[0]<<endl;
    //armMotors[2]->setPosition(0.1);   //Set camera position
    if ((xBoxPoints[4] <31- searchRange) and not reached and (not firstHop)){ //Have to turn Left
       turnLeftControl = 32-xBoxPoints[4];  // take the box to the middle
       if(turnLeftControl>0){
         //cout<<"turningLeft"<<endl;
         turnLeft();
       }
      
    }
    else if ((xBoxPoints[4] >33 + searchRange) and not reached and (not firstHop)){  //Have to turn right
      turnRightControl = xBoxPoints[4] -32;  //To take the box to the middle
       if(turnRightControl>0){
       //cout<<"turningRight"<<endl;
         turnRight();
       }
       }
    else{
       //cout<<sonicSensorReadings[0]<<endl;
       if(sonicSensorReadings[0]>890){    //front sensor detected object if <950
         if(initiateHop){
              firstHop = true;
              //forward();
              if(delaySteps(100)){
                firstHop = false;
                initiateHop = false;
                searchRange = 1;
              }
              else{
                  forward();
              }
              cout<<"first hop"<<endl;
         }
         else{
          forward();
         }
        
       }
       else{
         reached = true;
         camEnable = false;  // Disable the camera
       }
      //check for left and right walls
      if((sonicSensorReadings[2]>sonicSensorReadings[1] +10) and ((sonicSensorReadings[1]<900) or(sonicSensorReadings[2]<900))){
          rightWallCounter++;
          
          if(rightWallCounter>40){
                rightWall = true;
                noWall = 0;
          }
      }

      if((sonicSensorReadings[1]>sonicSensorReadings[2] +10) and ((sonicSensorReadings[1]<900) or(sonicSensorReadings[2]<900))){
          leftWallCounter++;
          if(leftWallCounter>40){
                leftWall = true;
                noWall = 0;
          }
      }
      else{
        rightWall = false;
          leftWall = false;
        noWall++;
        if(noWall> 10){
          noWall = 0;
          leftWallCounter = 0;
          rightWallCounter = 0;
          
        }
      }


      cout<< "left wall: "<<leftWall<<"  right wall: "<<rightWall<<endl;

      if((reached and (leftWall or rightWall)) and (not orientation)){
        if(leftWall){
          turnRight();
        }
        else if(rightWall){
          turnLeft();
        }
      }
      else{
       if ((sonicSensorReadings[1]>sonicSensorReadings[2] +10) and reached and (not orientation)){  //check side sensors with 10 gap to get correct orientation
          
          turnLeft();
         
         cout<<" turning left *********" <<endl;
         
       }
       else if((sonicSensorReadings[2] >sonicSensorReadings[1] +10) and reached and (not orientation)){
         
          turnRight();
        
         cout<<" turning right ************" <<endl;
         
       }
       else if (reached){
         stop();
         orientation = true;
         if (stage<3){
         grabTheBox();
         }
         if(stage>=3 ){
         liftTheBox();
         }
         if(stage>15){
          //cout<<"stage 5"<<endl;
          reverse();
          stage++;
         } 
         if(stage>30){
          location = "Box_Navigator";
         }
          cout<<stage<<endl;
        
       }
       //grabTheBox();
      }  
    }
    
    }

void grabTheBox(){
            //Get the arm to front

         if(stage == 0){        //Stage 01
            if(armrotate <0){
              armrotate += 0.1;
              }  
            else {
              armrotate +=0;
              stage++; 
            } 
            armMotors[1]->setPosition(armrotate);
          }
          if(stage==1){   //Stage 2 open the grippers
           /*if(catchRight <0.5 && catchLeft >-0.22){
              catchRight += 0.1;
              catchLeft -= 0.1;
              } */
             
            if(catchRight >-0.8 && catchLeft <0.6){     //catchRight> 0     catchLeft <0.4
              catchRight += -0.1; 
              catchLeft -= -0.1; 
              }
            else {
              catchRight +=0; 
              catchLeft +=0;
              stage++;
            }
            armMotors[4]->setPosition(catchRight);
            armMotors[5]->setPosition(catchLeft);
            }
           if(stage==2){
             controlForward(150);   //Move forward to correctly grab the box
             if (not goingForward){
               stage++;
             }
           }
          /*if(stage==3){
            forward();
            gripperCounter++;
            if(gripperCounter>20){
              stop();
              stage++; 
            }
          }  */
            
            
}

void liftTheBox(){

      if(stage == 3){  //stage 04
             
            if( catchRight <0.6 && catchLeft >-0.32){   // grab the box
              catchRight += 0.1;
              catchLeft -= 0.1;
              }
            else {
              catchRight +=0; 
              catchLeft +=0;
              stage++;
            }
            armMotors[4]->setPosition(catchRight);
            armMotors[5]->setPosition(catchLeft);
           } 
      else{
          if(stage<15){
           stage++;     //wait some iterations before lifting
           }
      }
      if(stage == 15){        //Stage 05           //lift the box
          
            if( armrotate >-2.2){
              armrotate += -0.05; 
              }
            else {
              armrotate +=0; 
              stage++;
            }
            armMotors[1]->setPosition(armrotate);
            }
        
          }
    


void controlForward(int n){
      goingForward = true;
       if (iter<n){
          forward();
          iter++;
      }
      else{
        stop();
        goingForward = false;
      }
  }
  
void brokenBridge(){

    // To get the box to the top
  if(not armPosition){

    if( armrotate >-2.2){
              armrotate += -0.025; 
              }
    else {
              armrotate +=0; 
              armPosition = true;  //Stop this loop after reaching required arm position
            }
            armMotors[1]->setPosition(armrotate);
    
  }
  
  // Camera line following
  //  camEnable = true;  // enable the camera
    
}

void bridgeNavigator(){
         //cout<<" count: "<<countToHoles<<endl;
         
         cout<<horizontalSurface<<endl;
         readLineSensors();
        if((brokenBridgeError<-2) and (not reachedHole)){
            turnLeft();
            
        }
        
        else if((brokenBridgeError>2) and (not reachedHole) ){
            turnRight();
        }
        else{
          if((not holeDetected )or ( not horizontalSurface)){    //Change countTOHoles decision
                forward();
                //countToHoles++;      
          }
          else if((holeDetected) and (not reachedHole)){
                
                if((delaySteps(30)) or  (reachedHole)){
                    stop();
                    reachedHole = true;
                }
                else{
                    reverse();
                }
          }
          else {
            stop();  //Change this
            //followingXPoint = 37;
           if(placingStage == 0){    //Get arm to front
                 if(armrotate <0){
                    armrotate += 0.025;
                    }  
                  else {
                    armrotate +=0;
                    placingStage++;
                    
                  } 
                  armMotors[1]->setPosition(armrotate); 
             }
             
          if(placingStage == 1){    //Open up the gripper
               if(catchRight >0 && catchLeft <0.4){
                catchRight += -0.1; 
                catchLeft -= -0.1; 
                }
              else {
                catchRight +=0; 
                catchLeft +=0;
                placingStage++;
              }
              armMotors[4]->setPosition(catchRight);
              armMotors[5]->setPosition(catchLeft);
             }
         
          if(placingStage ==2){   //
                if(catchRight >0 && catchLeft <0.4){
                    catchRight += -0.1; 
                    catchLeft -= -0.1; 
                    }
                else {
                    catchRight +=0; 
                    catchLeft +=0;
                    placingStage++;
                  }
                armMotors[4]->setPosition(catchRight);
                armMotors[5]->setPosition(catchLeft);
            }   
            
          if(placingStage == 3){   //Lift the arm
                  if( armrotate >-1.8){
                        armrotate += -0.05; 
                  }
                  else {
                        armrotate +=0; 
                        placingStage++;
                        }
                   armMotors[1]->setPosition(armrotate);
                     
                }
            
           if(placingStage == 4){
               if(not oneBoxDown){
                 location = "oneBoxComplete";
                 }
               else{
                 location = "LineFollowing";
               }
                 /* if(delaySteps(250)){
                     location = "Line_Following";
                 }
                 else{
                     cout<<"************************************ Turning Right ********************************"<<endl;
                     turnRight();
                 } */
           }
           cout<<"placing stage is: "<<placingStage<<endl;
          }
             
             }
            
            
            
            
            
  
          if(sonicSensorReadings[0]<960){  //Detect horizontal surface
            horizontalCounter++;
            }
          else{
            horizontalCounter=0;
          }
          if(horizontalCounter>10){    //Count up to 3 values to reduce errors
             horizontalSurface = true;
           }
          
          if(followingXPoint>32){
              if((horizontalSurface) and (rawSensorReadings[7]>980) and (rawSensorReadings[6]>980) and (rawSensorReadings[5]>980)){
                  holeDetected =  true;
              }
          }
          
          else{
              if((horizontalSurface) and (rawSensorReadings[0]>980) and (rawSensorReadings[1]>980) and (rawSensorReadings[2]>980)){
                  holeDetected = true;
              }
          }
        }
        
        
bool delaySteps(int n){
      cout<<"delay : "<<delayCounter<<endl;
      if (delayCounter<n){
          delayCounter++;
          return false;
      }
      else{
          delayCounter = 0;
          return true;
      }
}

void wallFollowing(){
   readWallSensors();
   //Right direction is positive
   
   if(turnOverRide){
      wallCounter++;
      if(wallCounter>60){
        turnOverRide = false;
        wallCounter = 0;
      }
      if (segmentedRightSpeed>segmentedLeftSpeed){
      turnLeft();
      }
      else{
        turnRight();
      }
   }
   else{
          if(sonicSensorReadings[0]<890){
                segmentedBaseSpeed = 0;
                //if(segmentedLeftSpeed <0.5 and segmentedRightSpeed <0.5){
                  turnOverRide = true;
                //}

          }
          else{
                segmentedBaseSpeed = 7;
          }
          segmentedLeftSpeed  = segmentedBaseSpeed+segmentedWallError;
          segmentedRightSpeed = segmentedBaseSpeed-segmentedWallError;
          if(sonicSensorReadings[0]<890){
                segmentedLeftSpeed = 10*segmentedLeftSpeed;
                segmentedRightSpeed = 10*segmentedRightSpeed;

          }
          if(segmentedLeftSpeed>10 ){
            segmentedLeftSpeed = 10;
          }
          if(segmentedLeftSpeed<-10 ){
            segmentedLeftSpeed = -10;
          } 
          if(segmentedRightSpeed>10 ){
            segmentedRightSpeed = 10;
          }
          if(segmentedRightSpeed<-10 ){
            segmentedRightSpeed = -10;
          }
   }        
 
   
   segmentedWallError = (4*(sonicSensorReadings[1]-sonicSensorReadings[2])+0.8*(sonicSensorReadings[3]-sonicSensorReadings[4])+0.6*(sonicSensorReadings[5]-sonicSensorReadings[6]))/70;
   skidDrive(segmentedLeftSpeed,segmentedRightSpeed);
   cout<<" wall error: "<<segmentedWallError<<endl;
   cout<<sonicSensorReadings[0]<<endl;
   cout<<segmentedLeftSpeed<<"   "<<segmentedRightSpeed<<endl;
}