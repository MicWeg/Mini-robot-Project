#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

//Define which pins to use for sensor
#define Trigger 2
#define Echo 3

//Define which pins to use for Motors
//First Motor
#define on_first 6
#define FMotor_Forward 7
#define FMotor_Backward 8

//Second Motor
#define on_second 9
#define SMotor_Forward 10
#define SMotor_Backward 11

//Create handler
ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
std_msgs::String str_msg;
std_msgs::Float64 left_wheel;
std_msgs::Float64 right_wheel;

ros::Publisher pub_range( "/ultrasound", &range_msg);
ros::Publisher pub_str("/move", &str_msg);
ros::Publisher pub_lw("/mybot/leftWheel_effort_controller/command", &left_wheel);
ros::Publisher pub_rw("/mybot/rightWheel_effort_controller/command", &right_wheel);

char frameid[] = "/ultrasound";

//Motors pins configuration
void motor_setup() {
  pinMode(on_first, OUTPUT);
  pinMode(FMotor_Forward, OUTPUT);
  pinMode(FMotor_Backward, OUTPUT);
  
  pinMode(on_second,OUTPUT);
  pinMode(SMotor_Forward,OUTPUT);
  pinMode(SMotor_Backward,OUTPUT);
}

//Sensor pins configuration
void sensor_setup() {
  pinMode(Trigger, OUTPUT);
  pinMode(Echo, INPUT);
}

//Declare detection range in cm
const int max_range = 200;
const int min_range = 2;

//Measue 50ms interval for sensor
long range_time;


float getRange_Ultrasound(){
  int distance;

    //Set the Trigger to LOW for 2 microseconds
    digitalWrite(Trigger, LOW);
    delayMicroseconds(2);
  
    //Set the Trigger to HIGH for 10 microseconds to emit 40kHz sound pulse
    digitalWrite(Trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trigger, LOW);
  
    //Read the lenght of the pulse in microseconds and add to variable 
    distance = pulseIn(Echo, HIGH) ;

  //Calculate  distance in cm (4*58.2)
  return distance/58.2;
}

void move(int distance){

  //Move backward if distance between robot and obstacle is equal or less than 2 centimeters 
  if(distance <= 2 && distance >=0){
    //Enable in case the robot was turned on in front of an obstacle
    digitalWrite(on_first,HIGH);
    digitalWrite(on_second,HIGH);

    digitalWrite(FMotor_Forward,LOW);
    digitalWrite(FMotor_Backward,HIGH);

    digitalWrite(SMotor_Forward,LOW);
    digitalWrite(SMotor_Backward,HIGH);

    str_msg.data = "Backward";
    pub_str.publish(&str_msg);

    left_wheel.data = -7.0;
    right_wheel.data = -7.0;
    pub_lw.publish(&left_wheel);
    pub_rw.publish(&right_wheel);

    delay(500);

    //Move right or left depending on pseudo-random number generator
    if(random(2) == 1){
      
      digitalWrite(FMotor_Forward,LOW);
      digitalWrite(FMotor_Backward,HIGH);
  
      digitalWrite(SMotor_Forward,HIGH);
      digitalWrite(SMotor_Backward,LOW);

      str_msg.data = "Right";
      pub_str.publish(&str_msg);

      left_wheel.data = 0.0;
      right_wheel.data = 7.0;
      pub_lw.publish(&left_wheel);
      pub_rw.publish(&right_wheel);
    }
    else{
      
      digitalWrite(FMotor_Forward,HIGH);
      digitalWrite(FMotor_Backward,LOW);
  
      digitalWrite(SMotor_Forward,LOW);
      digitalWrite(SMotor_Backward,HIGH);

      str_msg.data = "Left";
      pub_str.publish(&str_msg);

      left_wheel.data = 7.0;
      right_wheel.data = 0.0;
      pub_lw.publish(&left_wheel);
      pub_rw.publish(&right_wheel);

    }
    delay(1000);
  }
  //Move forward
  digitalWrite(on_first,HIGH);
  digitalWrite(on_second,HIGH);

  digitalWrite(FMotor_Forward,HIGH);
  digitalWrite(FMotor_Backward,LOW);

  digitalWrite(SMotor_Forward,HIGH);
  digitalWrite(SMotor_Backward,LOW);

  str_msg.data = "Forward";
  pub_str.publish(&str_msg);
  
  left_wheel.data = 7.0;
  right_wheel.data = 7.0;
  pub_lw.publish(&left_wheel);
  pub_rw.publish(&right_wheel);
  delay(50);
}

void ros_sensor(){
  
  if ( (millis()-range_time) > 50 ){
  
      range_msg.range = getRange_Ultrasound();
      range_msg.header.stamp = nh.now();
      pub_range.publish(&range_msg);
      range_time =  millis() + 50;
    }
}


void setup(){
  nh.initNode();
  nh.advertise(pub_range);
  nh.advertise(pub_str);
  nh.advertise(pub_lw);
  nh.advertise(pub_rw);
   
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.26;  
  range_msg.min_range = min_range;
  range_msg.max_range = max_range;
  
  sensor_setup();
  motor_setup();
}

void loop()
{
  randomSeed(analogRead(0));
  ros_sensor();
  move(getRange_Ultrasound());
  nh.spinOnce();
}
