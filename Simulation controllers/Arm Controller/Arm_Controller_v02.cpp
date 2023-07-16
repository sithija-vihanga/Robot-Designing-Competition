//Team Name :- Team Spectro
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Camera.hpp>
//#include <webots/LED.hpp>
//#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#define TIME_STEP 32
using namespace webots;
  //Motor *wheels[4];
  
  
  Motor *armMotors[6]; 
  
    
  //DistanceSensor *DS[8]; 
  //Motor *base; 
  //Motor *camControl; 

  double rotate = 0.0;
  double upAndDown = 0.0;
  double armrotate = -2.2;
  double catchRight = 0.5;
  double catchLeft = -0.22;

int main(int argc, char **argv) {

  Robot *robot = new Robot();
  
  Keyboard kb;
  
  Camera *cm;
  cm= robot->getCamera("cam");
  cm->enable(TIME_STEP);
  
  char armMotorNames[6][20] = {"linear motor","rotational motor1","cmBase","camMotor","rotational motor2","rotational motor3"};
  
 
 
   for (int i = 0; i < 6; i++) {                                      //Initializing motors using a for loop
      armMotors[i] = robot->getMotor(armMotorNames[i]);
      //armMotors[i]->setPosition(INFINITY);
 }
   kb.enable(TIME_STEP);

  
  

   while (robot->step(TIME_STEP) != -1) {
           int key= kb.getKey();
       
            /////////////// Rotate cam base ////////////////////////
            if(key == 65 && rotate <1.57){
              rotate += 0.1;
              }
            else if(key == 68 && rotate >-1.57){
              rotate += -0.1; 
              }
            else {
              rotate +=0; 
            }
            armMotors[2]->setPosition(rotate);
            
            
           /////////////////// Rotate cam up and down /////////////////////
            
            if(key == 87 && upAndDown <1.57){
              upAndDown += 0.1;
              }
            else if(key == 83 && upAndDown >-1.57){
              upAndDown += -0.1; 
              }
            else {
              upAndDown +=0; 
            }
             armMotors[3]->setPosition(upAndDown);
             
            
          //////////////////////// Rotate arm up and down ///////////////////////
             
            if(key == 70 && armrotate <0){
              armrotate += 0.1;
              }
            else if(key == 82 && armrotate >-2.2){
              armrotate += -0.1; 
              }
            else {
              armrotate +=0; 
            }
            armMotors[1]->setPosition(armrotate);
            
             //////////////////////// Rotate right gripper ///////////////////////
             
            if(key == 84 && catchRight <0.5 && catchLeft >-0.22){
              catchRight += 0.1;
              catchLeft -= 0.1;
              }
            else if(key == 71 && catchRight >0 && catchLeft <0.4){
              catchRight += -0.1; 
              catchLeft -= -0.1; 
              }
            else {
              catchRight +=0; 
              catchLeft +=0;
            }
            armMotors[4]->setPosition(catchRight);
            armMotors[5]->setPosition(catchLeft);
            
            std::cout<<"left: "<<catchLeft<<"    "<<"Right: "<<catchRight<<std::endl;
           //std::cout<<armrotate<<std::endl;
            
             //////////////////////// Rotate left gripper///////////////////////
           /*  
            if(key == 89 && catchLeft <1.57){
              catchLeft += 0.1;
              }
            else if(key == 72 && catchLeft >-1.57){
              catchLeft += -0.1; 
              }
            else {
              catchLeft +=0; 
            }
            armMotors[5]->setPosition(catchLeft);*/
            
            
            
            
            
            
            
            std::cout<<key<<std::endl;
            
            
            
            
            
            
            
            //std::cout<<speed<<std::endl;
  }  
}

 