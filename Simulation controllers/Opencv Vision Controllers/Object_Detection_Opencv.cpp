#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Motor.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <string>
#include <vector>


using namespace webots;
using namespace cv;
using namespace std;

void keyBoardControls();
void getContours(Mat imgIn, Mat imgOut);


//////////////// camera motors /////////////////////
 Motor *base; 
 Motor *camControl;

double rotateBase = 0.0;
double upAndDown = 0.0; 
int key;          //Keyboard input
int pixelSum = 0; //Sum of pixels in given region
string color;     //Store detected color
double distance = 10;  //measured distance.  Not zero anywhere
int boxSize = 5;
string location = "Chess_Board";


int main() {

  //Initialize the robot//
  Robot *robot = new Robot();
  const int timeStep = robot->getBasicTimeStep();

  //Initialize the camera//
  Camera *camera = robot->getCamera("cam");
  camera->enable(timeStep);
  const int width = camera->getWidth();
  const int height = camera->getHeight();
  
  int grayImage[width][height];
  //int cannyImage[width][height];
  
  cout<<"width is :"<<width<<" height is :"<<height<<endl;

  //Initialize the keyboard//
  Keyboard kb;            
  kb.enable(timeStep);


  Display *display = robot->getDisplay("display");
  
  base=robot->getMotor("cmBase");
  camControl=robot->getMotor("camMotor");
  
  Mat imageGray = Mat::zeros(Size(width,height),CV_8UC1);
  //Mat imageCanny = Mat::zeros(Size(width,height),CV_8UC1);
  Mat imageMat = Mat(Size(width, height),  CV_8UC4);     //generate empty image
 // Mat proImg = Mat(Size(width, height),  CV_8UC4);     //generate empty image


  while (robot->step(timeStep) != -1) {
    key= kb.getKey();                                   //Read keyboard inputs
    const unsigned char *data = camera->getImage();     //Taking a picture from camera

    if (data) {
      
      //int pixel =  camera->imageGetGray( data,width, 10, 10);
      //cout <<pixel<<endl;
      
      imageMat.data = (unsigned char *)data;                         //convert const unsigned char to unsigned char
      //GaussianBlur(imageMat, proImg, Size(9, 9),0);
      //GaussianBlur(imageMat, imageGray, Size(9, 9),0);                //Gaussian blur
      //rectangle(imageMat, Point(20,20),Point(30,30),Scalar(255,255,255),FILLED);
      
      //cvtColor(imageMat, imageGray, COLOR_BGR2GRAY);
      
      //cout<<"start uploading"<<endl;
      for (int i = 0 ; i < width ; i++){
        for ( int j = 0 ; j< height ; j++){
             int value = camera->imageGetGray( data,width, i, j);
             grayImage[j][i] = value+100;    //Change color ranges
             //cout<<value<<endl;
            
        }
      }
      
      ///////////////////////////////////////////////////////////// Chess Board  ///////////////////////////////////////////////////////////
      
      if (location == "Chess_Board"){
      
      
      
      
      for(int i = (31-boxSize); i<(31+boxSize); i++){             //Calculate pixel values to identify black or white
        for(int j = (31-boxSize); j<(31+boxSize); j++){           //Change the box with distance
            pixelSum = pixelSum+grayImage[j][i];
               
        } 
      }
      
      pixelSum = pixelSum/144;    //Check for white or black 
      //cout<<pixelSum<<endl;
      if(pixelSum >= 70){   //Threshhold for white
        color = "White";
        cout<<color<<endl;
        
      }
      else if(pixelSum<70){
        color = "Black";
        cout<<color<<endl;
      }
      
      
      //Mat grayMat(width, height, CV_64F, grayImage);
      //cout <<" Image uploaded"<<endl;  
      
      rectangle(imageMat, Point((31-boxSize),(31-boxSize)),Point((31+boxSize),(31+boxSize)),Scalar(0, 255, 0), 1 , LINE_8);
      //rectangle(imageMat, Point(10,10),Point(20,20),Scalar(255,255,255),FILLED);
      //cout<< pixel<<endl;
      
      }
      /////////////////////////////////////////////////////////////////////////////////////////////////////////
      
      //Mat greyImg = Mat(width, height , CV_8U, &grayImage);
      Mat Img = Mat(width, height , CV_8UC4, &grayImage);
      GaussianBlur(Img, Img, Size(5, 5),0); 
      
      
      Mat imageCanny; 
     
      Canny(Img, imageCanny, 100, 300,3);
      Mat kernel = getStructuringElement(MORPH_RECT, Size(3,3));
      dilate(imageCanny,imageCanny,kernel); 
      //cvtColor(imageCanny,imageGray,COLOR_BGRA2GRAY);  
      
      //Converting to gray scale

      //  
      
      getContours(imageCanny,imageMat);  
      
      
      
      ImageRef *ir = display->imageNew(width, height, imageMat.data , Display::BGRA);     //Generating output image
      display->imagePaste(ir, 0, 0, false);
      display->imageDelete(ir);
      
      }
    keyBoardControls();
  }

  delete robot;
  return 0;
}



void keyBoardControls(){
        ////////////////// rotateBase the camera ///////////////// 
            if(key == 65 && rotateBase <1.57){             //Turn left and Right
              rotateBase += 0.1;
              }
            else if(key == 68 && rotateBase >-1.57){
              rotateBase += -0.1; 
              }
            else {
              rotateBase +=0; 
            }
            base->setPosition(rotateBase);
            
            if(key == 87 && upAndDown <1.57){               //Turn up and Down
              upAndDown += 0.1;
              }
            else if(key == 83 && upAndDown >-1.57){
              upAndDown += -0.1; 
              }
            else {
              upAndDown +=0; 
            }
            camControl->setPosition(upAndDown);
          
            //std::cout<<key<<std::endl;
       ////////////////////////////////////////////////////

}
 
void getContours(Mat imgIn, Mat imgOut){
  vector<vector<Point>>contours;
  vector<Vec4i>hierarchy;
 
  
  findContours(imgIn,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
  drawContours(imgOut , contours , -1,Scalar(255,0,255),2);
  
  

}  
