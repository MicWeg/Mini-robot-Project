/*------------------------------------------------------------------------------
  17/06/2020
  Author: KFP
  Platforms: ESP8266
  Language: C++/Arduino
  File: ESP8266_rosserial.ino
  ------------------------------------------------------------------------------*/

#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>
#include <ros/time.h>
#include <tf/tf.h>


const char* SSID = "";//type your ssid
const char* password = "";//type your password
IPAddress server(0,0,0,0);      // Set the rosserial socket ROSCORE SERVER IP address
const uint16_t serverPort = 11411;    // Set the rosserial socket server port

//Create handler
ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
std_msgs::String str_msg;
std_msgs::Float64 left_wheel;
std_msgs::Float64 right_wheel;

ros::Publisher pub_range( "/ultrasound", &range_msg);
ros::Publisher pub_str("/move", &str_msg);
ros::Publisher pub_lw("/robot/leftWheel_effort_controller/command", &left_wheel);
ros::Publisher pub_rw("/robot/rightWheel_effort_controller/command", &right_wheel);

char frameid[] = "/ultrasound";

// Functions definitions //

void setupWiFi() {

  WiFi.begin(SSID, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void setup() {
  //if(DEBUG) Serial.begin(19200);
  Serial.begin(57600);
  setupWiFi();
  delay(2000);


// Ros objects constructors
  nh.getHardware()->setConnection(server, serverPort);
  delay(10000);

  nh.initNode();
  nh.advertise(pub_range);
  nh.advertise(pub_str);
  nh.advertise(pub_lw);
  nh.advertise(pub_rw);

  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.26;
  range_msg.min_range = 2;
  range_msg.max_range = 200;

}

void loop(){

  if ( nh.connected() )
  {

    DynamicJsonDocument doc(512); //doc(1024);

    String message = "";
    message = Serial.readString();

    // Attempt to deserialize the JSON-formatted message
    DeserializationError error = deserializeJson(doc,message);
    if(error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      return;
    }
    range_msg.range = doc["distance"];
    str_msg.data =  doc["move"];
    left_wheel.data =  doc["lw"];
    right_wheel.data =   doc["rw"];

    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    pub_lw.publish(&left_wheel);
    pub_rw.publish(&right_wheel);
    pub_str.publish(&str_msg);

  }
  nh.spinOnce();
  delay(1);
}
