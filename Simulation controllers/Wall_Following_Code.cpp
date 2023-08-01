#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#define TIME_STEP 16

using namespace webots;
  int value;
  int speed=7;
  double error,previous_error1=0,previous_error2=0,integral1,integral2,derivative1,derivative2,PID_error1,PID_error2;
  double Kp=1.5;
  double Ki=0.01;
  double Kd=0.5;
  double max_speed=10.0;
  double min_speed=10.0;
  int leftmotor_speed,rightmotor_speed;
  double error1,error2;
  int REFERENCE_DISTANCE=900;
  int REFERENCE_DISTANCE1=900;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  
  
  
  // Get the front proximity sensor
  DistanceSensor *frontSensor = robot->getDistanceSensor("Front");
  frontSensor->enable(TIME_STEP);
  
  
  // Get the side proximity sensors
  DistanceSensor *leftSensor1 = robot->getDistanceSensor("Left_1");
  leftSensor1->enable(TIME_STEP);
  DistanceSensor *rightSensor1 = robot->getDistanceSensor("Right_1");
  rightSensor1->enable(TIME_STEP);
  DistanceSensor *leftSensor2=robot->getDistanceSensor("Left_2");
  leftSensor2-> enable(TIME_STEP);
  DistanceSensor *rightSensor2 =robot->getDistanceSensor("Right_2");
  rightSensor2-> enable(TIME_STEP);
   DistanceSensor *frontleftSensor=robot->getDistanceSensor("FrontLeft");
  frontleftSensor-> enable(TIME_STEP);
  DistanceSensor *frontrightSensor =robot->getDistanceSensor("FrontRight");
  frontrightSensor-> enable(TIME_STEP);
  // Get the wheel motors
  Motor *leftWheel1 = robot->getMotor("leftTopMotor");
  Motor *leftWheel2 = robot->getMotor("leftRearMotor");
  Motor *rightWheel1 = robot->getMotor("rightTopMotor");
  Motor *rightWheel2 = robot->getMotor("rightRearMotor");
  leftWheel1->setPosition(INFINITY);
  leftWheel2->setPosition(INFINITY);
  rightWheel1->setPosition(INFINITY);
  rightWheel2->setPosition(INFINITY);
  leftWheel1->setVelocity(0.0);
  leftWheel2->setVelocity(0.0);
  rightWheel1->setVelocity(0.0);
  rightWheel2->setVelocity(0.0);
  
  while (robot->step(TIME_STEP) != -1) {
    double frontValue = frontSensor->getValue();
    double leftValue1 =  leftSensor1 ->getValue();
    double leftValue2 =  leftSensor2->getValue();
    double rightValue1 = rightSensor1->getValue();
    double rightValue2=  rightSensor2->getValue();
    double frontrightValue = frontrightSensor->getValue();
    double frontleftValue= frontleftSensor-> getValue();
    std::cout<<leftValue1<<" "<<leftValue2<<" "<<frontleftValue<<" "<<frontValue<<" "<<frontrightValue<<" "<<rightValue1<<" "<<rightValue2<<std::endl;
   
   
    if (frontValue<900){
   leftmotor_speed=4;
      rightmotor_speed=4;
      
      if((frontleftValue)==(frontrightValue)){
      
     if((rightValue2+rightValue1)>(leftValue2+leftValue1)){
     leftmotor_speed=10.0;
      rightmotor_speed=-10.0;
    
    }
    if((rightValue2+rightValue1)<(leftValue2+leftValue1)){
    
      leftmotor_speed=-10.0;
     rightmotor_speed=10.0;
    
    }
    }
    if (((frontleftValue)==(frontrightValue))&&(rightValue1==leftValue1)){
       if (leftValue2>rightValue2)
    {
    rightmotor_speed=10;
    leftmotor_speed=-10;
    
    }
    else if (leftValue2<rightValue2){
    
     rightmotor_speed=-10;
    leftmotor_speed=10;
    
    }
   
    
    }
    
    
    
    
    
    
     
     
   
if((rightValue1+rightValue2+frontrightValue)>(leftValue1+leftValue2+frontleftValue)){
        leftmotor_speed=10.0;
      rightmotor_speed=-10.0;
      }
      else if((rightValue1+rightValue2+frontrightValue)<(leftValue1+leftValue2+frontleftValue)){
      leftmotor_speed=-10.0;
     rightmotor_speed=10.0;
     }


   }
   
 
 
 
 
 
if(frontValue>900){

   
    
    
   if ((leftValue1+leftValue2+frontrightValue)==(rightValue1+rightValue2+frontleftValue)){
   leftmotor_speed=speed;
   rightmotor_speed=speed;
   
   
   
   
   }
    
    //error2=(rightValue2-leftValue2);
    //error=(error1*2+error2*1.5)/80;
     if ((leftValue1+leftValue2+frontleftValue)<(rightValue1+rightValue2+frontrightValue)){
          if ((leftValue1==1000)){
           error1=(REFERENCE_DISTANCE-leftValue1)/50;
          
          }
          
          else{
          error1=(REFERENCE_DISTANCE-(leftValue1+frontleftValue)/2)/30;
          
          }
    
    
      
      integral1+=error1;
    derivative1=(error1-previous_error1);
    PID_error1=(Kp*error1+Ki*integral1+Kd*derivative1);
    previous_error1=error1;
     
    leftmotor_speed=speed+PID_error1;
    rightmotor_speed=speed-PID_error1;
    }
   
   
     if ((leftValue1+leftValue2+frontleftValue)>(rightValue1+rightValue2+frontrightValue)){
    
    if (rightValue1==1000){
     error2=(REFERENCE_DISTANCE1-rightValue1)/50;
    
    }
    else{
      error2=(REFERENCE_DISTANCE1-(rightValue1+frontrightValue)/2)/30;
      
    
    }
    
    integral2+=error2;
    derivative2=(error2-previous_error2);
    PID_error2=(Kp*error2+Ki*integral2+Kd*derivative2);
    previous_error2=error2;
    leftmotor_speed=speed-PID_error2;
    rightmotor_speed=speed+PID_error2;
  
 
 }
 }
 
 if (leftmotor_speed>=max_speed){
 leftmotor_speed=max_speed;
}
 if( rightmotor_speed>=max_speed){
 rightmotor_speed=max_speed;
 
 
 }
 if (leftmotor_speed>=min_speed){
 leftmotor_speed=min_speed;
}
 if( rightmotor_speed>=min_speed){
 rightmotor_speed=min_speed;
 
 
 }
 leftWheel1->setVelocity(leftmotor_speed);
 leftWheel2->setVelocity(leftmotor_speed);
 rightWheel1->setVelocity(rightmotor_speed);
 rightWheel2->setVelocity(rightmotor_speed);
 std::cout<<leftmotor_speed<<" "<<rightmotor_speed<<" "<<error2<<std::endl;
 }
 }
      