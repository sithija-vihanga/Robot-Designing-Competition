#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Motor.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace webots;
using namespace cv;
using namespace std;

void keyBoardControls();

//////////////// camera motors /////////////////////
 Motor *base; 
 Motor *camControl;

double rotateBase = 0.0;
double upAndDown = 0.0; 
int key;

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
  
  cout<<"width is :"<<width<<" height is :"<<height<<endl;

  //Initialize the keyboard//
  Keyboard kb;            
  kb.enable(timeStep);


  Display *display = robot->getDisplay("display");
  
  base=robot->getMotor("cmBase");
  camControl=robot->getMotor("camMotor");
  
  Mat imageGray = Mat::zeros(Size(width,height),CV_8UC1);
  Mat imageMat = Mat(Size(width, height),  CV_8UC4);     //generate empty image


  while (robot->step(timeStep) != -1) {
    key= kb.getKey();                                   //Read keyboard inputs
    const unsigned char *data = camera->getImage();     //Taking a picture from camera

    if (data) {
      
      //int pixel =  camera->imageGetGray( data,width, 10, 10);
      //cout <<pixel<<endl;
      imageMat.data = (unsigned char *)data;                         //convert const unsigned char to unsigned char
      //GaussianBlur(imageMat, imageGray, Size(9, 9),0);                //Gaussian blur
      //rectangle(imageMat, Point(20,20),Point(30,30),Scalar(255,255,255),FILLED);
      
      
      
      
      //cvtColor(imageMat, imageGray, COLOR_BGR2GRAY);
      
      //cout<<"start uploading"<<endl;
      for (int i = 0 ; i < width ; i++){
        for ( int j = 0 ; j< height ; j++){
            grayImage[j][i] =  camera->imageGetGray( data,width, i, j);
            
        }
      }
      
      Mat grayMat(64, 64, CV_64F, grayImage);
      //cout <<" Image uploaded"<<endl;  
      
      //cout<< pixel<<endl;
      ImageRef *ir = display->imageNew(width, height, grayMat.data, Display::BGRA);     //Generating output image
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