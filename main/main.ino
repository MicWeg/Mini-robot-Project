//Add library for BT and create new serial object
#include <SoftwareSerial.h>
// RX,TX
SoftwareSerial my_serial(,);
//String variable for sending msg through BT
String my_string = "Distance: ";

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

int Sensor(){
  int distance;
  
    //Set the Trigger to LOW for 2 microseconds
  digitalWrite(Trigger, LOW);
  delayMicroseconds(2);

  //Set the Trigger to HIGH for 10 microseconds to emit 40kHz sound pulse
  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigger, LOW);

  //Read the lenght of the pulse in microseconds and calculate the distance in centimeters
  distance = pulseIn(Echo, HIGH) / 58;

  delay(10);

  if(my_serial.available()){
    
     //Send distance through BT -- check write if there is a problem
     my_serial.print(my_string + distance);
     
  }
  
  return distance;
  
}

void Move(int distance){

  //Move backward if distance between robot and obstacle is equal or less than 2 centimeters 
  if(distance <= 2){
    //Enable in case the robot was turned on in front of an obstacle
    digitalWrite(on_first,HIGH);
    digitalWrite(on_second,HIGH);

    digitalWrite(FMotor_Forward,LOW);
    digitalWrite(FMotor_Backward,HIGH);

    digitalWrite(SMotor_Forward,LOW);
    digitalWrite(SMotor_Backward,HIGH);

    delay(300);

    //Move right or left depending on pseudo-random number generator
    if(random(2) == 1){
      
      digitalWrite(FMotor_Forward,LOW);
      digitalWrite(FMotor_Backward,HIGH);
  
      digitalWrite(SMotor_Forward,HIGH);
      digitalWrite(SMotor_Backward,LOW);

      delay(300);
    }
    else{
      
      digitalWrite(FMotor_Forward,HIGH);
      digitalWrite(FMotor_Backward,LOW);
  
      digitalWrite(SMotor_Forward,LOW);
      digitalWrite(SMotor_Backward,HIGH);

      delay(300);
    }
  }
  //Move forward
  digitalWrite(on_first,HIGH);
  digitalWrite(on_second,HIGH);

  digitalWrite(FMotor_Forward,HIGH);
  digitalWrite(FMotor_Backward,LOW);

  digitalWrite(SMotor_Forward,HIGH);
  digitalWrite(SMotor_Backward,LOW);

  delay(30);
}

void setup() {

  randomSeed(analogRead(0));
  Baud_setup();
  Motor_setup();
  Sensor_setup();

}

void loop() {

  //First test another solution
  //Send_msg_BT(Sensor());
  Move(Sensor());
  
}
