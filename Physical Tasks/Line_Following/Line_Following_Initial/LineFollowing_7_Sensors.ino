#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const int ProxSensor_left=A9;     // 3 left
const int ProxSensor_1=A15;       // 2 left
const int ProxSensor_2=A14;       // 1 left
const int ProxSensor_3=A13;       // 0 middle
const int ProxSensor_4=A12;       // 1 right
const int ProxSensor_5=A11;       // 2 right
const int ProxSensor_right=A10;   // 3 right


int inputVal[7] = {0,0,0,0,0,0,0};

// Motor A connections
int enA = 3;
int in1 = A0;
int in2 = A1;
// Motor B connections
int enB = 10;
int in3 = A3;
int in4 = A2;



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

int error_list[5] = {0,0,0,0,0};
float kp = 0.6;
float kd = 0.;
float ki = 0.0000;

int error = 0;
int d_error = 0;
int  i_error = 0;

void loop(){
  linefollow();


}


void linefollow(){ 

  inputVal[0]  = digitalRead(ProxSensor_left);
  inputVal[1]  = digitalRead(ProxSensor_1);
  inputVal[2]  = digitalRead(ProxSensor_2);
  inputVal[3]  = digitalRead(ProxSensor_3);
  inputVal[4]  = digitalRead(ProxSensor_4);
  inputVal[5]  = digitalRead(ProxSensor_5);
  inputVal[6]  = digitalRead(ProxSensor_right);
  
  error = (-inputVal[0]*3000 -inputVal[1]*300 - inputVal[2]*200 + inputVal[4]*200 +inputVal[5]*300 + inputVal[6]*3000)/(8*(inputVal[0]+inputVal[1]+inputVal[2]+inputVal[3]+inputVal[4]+inputVal[5]+inputVal[6]));

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
