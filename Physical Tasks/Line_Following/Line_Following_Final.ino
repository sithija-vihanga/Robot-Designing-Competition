//Team Name :- Team Spectro


//______Variable names________
int counter = 0;
int speed = 128;
int sensorReadings[8];
double lineControlSignal = 0.0;
double lineErrorSum = 0.0;
double lineError[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
double line_Kp = 1;
double line_Kd = 1;
double line_Ki = 0.01;

double lineErrorPredict = 0.0;

char wheels_names[4][15] = {"leftTopMotor", "leftRearMotor", "rightTopMotor", "rightRearMotor"};                               //store names of motors
char dsNames[8][8] = {"IR1", "IR2", "IR3", "IR4", "IR5", "IR6", "IR7", "IR8"};

//________PinNames_________

int irPin1 = 1;                 // IR sensor array pins
int irPin2 = 2;
int irPin3 = 3;
int irPin4 = 4;
int irPin5 = 5;
int irPin6 = 6;
int irPin7 = 7;
int irPin8 = 8;


/*________functions________

  void forward();
  void reverse();
  void turnRight();
  void turnLeft();
  void stop();
  void skidDrive(double Lspeed, double Rspeed);
  void readLineSensors();
  int  compareLines( double values);
  void pidLineFollower();
  double maxSpeedControl(double lineControlSignal); */




void setup() {
  // put your setup code here, to run once:
  // Insert PinModes
  pinMode(A0, OUTPUT);      //we used A0-A3 pins as digital output pins
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);




}


//_______Functions____________

void forward() {
  digitalWrite(A1, HIGH);
  digitalWrite(A2, HIGH);
  digitalWrite(A0, LOW);
  digitalWrite(A3, LOW);
}
void reverse() {
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A0, HIGH);
  digitalWrite(A3, HIGH);
}

void turnRight() {
  digitalWrite(A0, HIGH);
  digitalWrite(A1, LOW);
  digitalWrite(A2, HIGH);
  digitalWrite(A3, LOW);
}

void turnLeft() {
  digitalWrite(A0, LOW);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, LOW);
  digitalWrite(A3, HIGH);
}

void stop() {
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
}

void skidDrive(double Lspeed , double Rspeed) {               //Differential drive    //Lspeed and Rspeed is between 0 255
  analogWrite(A1, Lspeed);
  analogWrite(A2, Rspeed);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, HIGH);
  digitalWrite(A0, LOW);
  digitalWrite(A3, LOW);
}

void readLineSensors() {

  for (int j = 0; j < 8; j++) {
    sensorReadings[j] = digitalRead(1 + j); //Read sensor values and upload to a array

  }
}

/* int compareLines(double value){                    // Extracting colors
               if (value<100){
                 return 1;
               }
               else if((value>100) and (value <150)){
                 return 2;
               }
               else if(value>600 and value <700){
                 return 3;
               }
               else if(value>700){
                 return 0;
               }
               else {
                 return 5;
               }
  } */

/*double maxSpeedControl(double lineControlSignal){
               double speed = maxSpeed;
               if(lineControlSignal > 4){
                  speed = maxSpeed - lineControlSignal;
               }
                return speed;
  } */

void pidLineFollower() {
  for (int i = 0; i < 4 ; i++) {                        // Storing 5 previous lineError values
    lineError[i] = lineError[i + 1];
  }


  if (lineError[3] > lineError[2]) {
    if (lineError[3] < lineError[2] + 8) {

    }
    else {
      lineError[3] = lineError[2];
    }
  }
  else if (lineError[3] < lineError[2]) {
    if (8 + lineError[3] > lineError[2]) {

    }
    else {
      lineError[3] = lineError[2];
    }
  }

  lineErrorSum += lineError[3];                    // Get sum of the errors

  //lineError[4] = 2*(sensorReadings[7]-sensorReadings[5])+2*(1-sensorReadings[7])(1-sensorReadings[5])(sensorReadings[4]-sensorReadings[0])+(sensorReadings[4]-sensorReadings[1]);      // Error algorithm
  lineError[4] = 3 * (sensorReadings[0] - sensorReadings[7]) + 2 * (sensorReadings[1] - sensorReadings[6]) + (sensorReadings[2] - sensorReadings[5]); // Error algorithm


  lineErrorPredict = (lineError[3] - lineError[1]);     // Derivative


  //lineErrorPredict = ((x-2)(x-3)(x-4)/(-6)(lineError[0])) + ((x-1)(x-3)(x-4)/(2)(lineError[1])) + ((x-1)(x-2)(x-4)/(-2)(lineError[2])) + ((x-1)(x-2)(x-3)/(6)(lineError[3]))/100;

  /////////////////////////////////    PID control signal  ////////////////////////////

  lineControlSignal = 0.8 * lineError[4] + 0.5 * line_Ki * lineErrorSum + lineErrorPredict;
  lineControlSignal = 255 * lineControlSignal/10;//  lineControlSignal maped from 0-10 to 0-255

  ///////////////////////////////////////////////////////////////////////////////////

  // Defining boundaries

  if ((lineErrorSum > 380) or (lineErrorSum < -380)) {                     //Regulating the max speed
    speed = 75;           //max speed 10
  }
  else if ((lineErrorPredict > 8)) {                   //Regulating the max speed
    speed = 50;
  }

  if (lineErrorSum > 510) {                            //Regulating max lineError
    lineErrorSum = 510;
  }
  else if (lineErrorSum < -510) {                            //Regulating max lineError
    lineErrorSum = -510;
  }


}

void loop() {
  // put your main code here, to run repeatedly:
  skidDrive(speed + lineControlSignal, speed - lineControlSignal);
  counter += 1;
  readLineSensors();
  pidLineFollower();

}


//_________________________________

//int main(int argc, char **argv) {
//
//  Robot *robot = new Robot();
//
//
//
//  for (int i = 0; i < 4; i++) {                                      //Initializing motors using a for loop
//    wheels[i] = robot->getMotor(wheels_names[i]);
//    wheels[i]->setPosition(INFINITY);
//
//  }
//
//  //Initialize distance sensors
//  for (int j = 0; j < 8; j++) {                                                 //Enabling distance sensors using for a loop
//    DS[j] = robot->getDistanceSensor(dsNames[j]);
//    DS[j]->enable(TIME_STEP);
//  }
//
//
//
//
//  while (robot->step(TIME_STEP) != -1) {
//    //speed = maxSpeedControl(lineControlSignal);
//    skidDrive(speed + lineControlSignal, speed - lineControlSignal);
//    counter += 1;
//    readLineSensors();
//    pidLineFollower();
//
//    //std::cout <<"Color codes "<< DS[0]->getValue() <<"  " << DS[1]->getValue() <<"  "<< DS[2]->getValue() <<"  "<< DS[3]->getValue() <<"  "<< DS[4]->getValue() <<"  " <<"Front motors " <<DS[5]->getValue() <<"  " << DS[6]->getValue() <<"  " << DS[7]->getValue() <<std::endl;
//    //std::cout << sensorReadings[0] <<" " << sensorReadings[1] <<" " << sensorReadings[2] <<" " << sensorReadings[3] <<" " << sensorReadings[4] <<" " << sensorReadings[5] <<" " << sensorReadings[6] <<" " << sensorReadings[7] <<" "  <<std::endl;
//    //std::cout <<lineErrorPredict <<" "<< lineError[3] <<" " <<lineErrorSum <<std::endl;
//    //std::cout << lineError[0] <<" " << lineError[1] <<" " << lineError[2] <<" " << lineError[3] <<" " << lineError[4] <<" " <<lineErrorSum<<std::endl;
//    //std::cout<<lineControlSignal<<std::endl;
//    std::cout << speed << std::endl;
//  }
//}
