/*------------------------------------------------------------------------------
  24/05/2020
  Author: KFP
  Platforms: ArduinoNano/ESP8266
  Language: C++/Arduino
  File: nano_firmware.ino
  ------------------------------------------------------------------------------*/
#include <ArduinoJson.h>
#include <ros.h>
#include <ros/time.h>

//Define which pins to use for sensor's input and output
#define Trigger 4
#define Echo    5

//Define which pins to use for Motors
//First Motor
#define on_first 6
#define FMotor_Forward 7
#define FMotor_Backward 8

//Second Motor
#define on_second 9
#define SMotor_Forward 10
#define SMotor_Backward 11

//Define pwm for Motors
int pwm = 180;

//Define variable for distance
int distance;

String str_msg;
float left_wheel;
float right_wheel;

//Define variable for comunication
String message = "";
bool messageReady = false;

void send_data() {
  if (Serial.available()) {
    DynamicJsonDocument doc(512); //doc(1024);

    doc["type"] = "response";
    // Get data
    doc["distance"] = distance;
    doc["move"] = str_msg;
    doc["lw"] = left_wheel;
    doc["rw"] = right_wheel;
    serializeJson(doc, Serial);
    Serial.println();
  }
}



//Motors pins configuration
void motor_setup() {
  pinMode(on_first, OUTPUT);
  pinMode(FMotor_Forward, OUTPUT);
  pinMode(FMotor_Backward, OUTPUT);

  pinMode(on_second, OUTPUT);
  pinMode(SMotor_Forward, OUTPUT);
  pinMode(SMotor_Backward, OUTPUT);
}

//Sensor pins configuration
void sensor_setup() {
  pinMode(Trigger, OUTPUT);
  pinMode(Echo, INPUT);
}

float getRange_Ultrasound() {
  int pom_distance;

  //Set the Trigger to LOW for 2 microseconds
  digitalWrite(Trigger, LOW);
  delayMicroseconds(2);

  //Set the Trigger to HIGH for 10 microseconds to emit 40kHz sound pulse
  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigger, LOW);

  //Read the lenght of the pulse in microseconds and add to variable
  pom_distance = pulseIn(Echo, HIGH) / 58.2 ;
  distance = pom_distance;

  //Calculate  distance in cm (4*58.2)
  return pom_distance;
}

void move(int pom_distance) {

  //Move backward if distance between robot and obstacle is equal or less than 2 centimeters
  if (pom_distance <= 5 && pom_distance >= 0) {
    //Enable in case the robot was turned on in front of an obstacle
    analogWrite(on_first, 0);
    analogWrite(on_second, 0);

    digitalWrite(FMotor_Forward, LOW);
    digitalWrite(FMotor_Backward, HIGH);

    digitalWrite(SMotor_Forward, LOW);
    digitalWrite(SMotor_Backward, HIGH);

    analogWrite(on_first, pwm);
    analogWrite(on_second, pwm);

    str_msg = "Backward";

    left_wheel = -7.0;
    right_wheel = -7.0;
    send_data();

    delay(1000);

    //Move right or left depending on pseudo-random number generator
    if (random(2) == 1) {

      analogWrite(on_first, 0);
      analogWrite(on_second, 0);

      digitalWrite(FMotor_Forward, LOW);
      digitalWrite(FMotor_Backward, HIGH);

      digitalWrite(SMotor_Forward, HIGH);
      digitalWrite(SMotor_Backward, LOW);

      analogWrite(on_first, pwm);
      analogWrite(on_second, pwm);

      str_msg = "Right";

      left_wheel = -7.0;
      right_wheel = 7.0;
      send_data();

    }
    else {

      analogWrite(on_first, 0);
      analogWrite(on_second, 0);

      digitalWrite(FMotor_Forward, HIGH);
      digitalWrite(FMotor_Backward, LOW);

      digitalWrite(SMotor_Forward, LOW);
      digitalWrite(SMotor_Backward, HIGH);

      analogWrite(on_first, pwm);
      analogWrite(on_second, pwm);

      str_msg = "Left";

      left_wheel = 7.0;
      right_wheel = -7.0;
      send_data();

    }
    delay(1000);
  }
  //Move forward
  analogWrite(on_first, 0);
  analogWrite(on_second, 0);

  digitalWrite(FMotor_Forward, HIGH);
  digitalWrite(FMotor_Backward, LOW);

  digitalWrite(SMotor_Forward, HIGH);
  digitalWrite(SMotor_Backward, LOW);

  analogWrite(on_first, pwm);
  analogWrite(on_second, pwm);

  str_msg = "Forward";

  left_wheel = 7.0;
  right_wheel = 7.0;
  send_data();

  delay(50);
}


void setup() {
  //Set the baud rate for serial data transmission
  Serial.begin(57600);

  sensor_setup();
  motor_setup();
}

void loop() {
  move(getRange_Ultrasound());
}
