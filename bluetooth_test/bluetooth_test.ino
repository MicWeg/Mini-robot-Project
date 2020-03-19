#include <SoftwareSerial.h>
// RX,TX
SoftwareSerial my_serial(,);

void setup() {
  //Set baud rate for serial data transmission 
  Serial.begin(9600);
  my_serial.begin(9600);
  Serial.println("1234");
}

void loop() {
  //Send data from BT to Terminal
  if(my_serial.available())
    Serial.write(my_serial.read());

  //Send data from Terminal to BT
  if(Serial.available())
    my_serial.write(Serial.read());
}
