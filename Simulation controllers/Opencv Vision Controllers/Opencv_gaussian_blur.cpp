#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Motor.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace webots;
using namespace cv;

 Motor *base; 
 Motor *camControl; 

int main() {
  Robot *robot = new Robot();
  const int timeStep = robot->getBasicTimeStep();

  Camera *camera = robot->getCamera("cam");
  camera->enable(timeStep);
  const int width = camera->getWidth();
  const int height = camera->getHeight();
  Keyboard kb;
  Display *display = robot->getDisplay("display");
  
  base=robot->getMotor("cmBase");
  camControl=robot->getMotor("camMotor");
  
  double rotate = 0.0;
  double upAndDown = 0.0;
  
  kb.enable(timeStep);
  
  
  Mat imageMat = Mat(Size(width, height), CV_8UC4);

  while (robot->step(timeStep) != -1) {
    int key= kb.getKey();
    const unsigned char *data = camera->getImage();
    if (data) {
    
      imageMat.data = (unsigned char *)data;
      GaussianBlur(imageMat, imageMat, Size(9, 9),0);
      
      
      
      
      ImageRef *ir = display->imageNew(width, height, imageMat.data, Display::BGRA);
      display->imagePaste(ir, 0, 0, false);
      display->imageDelete(ir);
      
      
     ////////////////// Rotate the camera ///////////////// 
                  if(key == 65 && rotate <1.57){
              rotate += 0.1;
              }
            else if(key == 68 && rotate >-1.57){
              rotate += -0.1; 
              }
            else {
              rotate +=0; 
            }
            base->setPosition(rotate);
            
            
        
            
            if(key == 87 && upAndDown <1.57){
              upAndDown += 0.1;
              }
            else if(key == 83 && upAndDown >-1.57){
              upAndDown += -0.1; 
              }
            else {
              upAndDown +=0; 
            }
            camControl->setPosition(upAndDown);
            
            
            std::cout<<key<<std::endl;
       ////////////////////////////////////////////////////
    }
  }

  delete robot;
  return 0;
}