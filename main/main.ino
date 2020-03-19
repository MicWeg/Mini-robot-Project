//Add library for BT and create new serial object
#include <SoftwareSerial.h>
// RX,TX
SoftwareSerial my_serial(,);

//Setting baud rate for serial data transmission
void Baud_setup(){
  Serial.begin(9600);
  my_serial.begin(9600);
}

//Define which pins to use for Motors
//First Motor
#define on_first 
#define FMotor_Forward 
#define FMotor_Backward 

//Second Motor
#define on_second 
#define SMotor_Forward 
#define SMotor_Backward 

//Motors pins configuration
void Motor_setup() {
  pinMode(on_first, OUTPUT);
  pinMode(FMotor_Forward, OUTPUT);
  pinMode(FMotor_Backward, OUTPUT);
  
  pinMode(on_second,OUTPUT);
  pinMode(SMotor_Forward,OUTPUT);
  pinMode(SMotor_Backward,OUTPUT);
}

//Define which pins to use for sensor
#define Trigger
#define Echo

//Sensor pins configuration
void Sensor_setup() {
  pinMode(Trigger, OUTPUT);
  pinMode(Echo, INPUT);
}



void setup() {
  
  Baud_setup();
  Motor_setup();
  Sensor_setup();

}

void loop() {

  

}
