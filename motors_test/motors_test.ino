//Define which pins to use for Motors
//First Motor
#define on_first 
#define FMotor_Forward 
#define FMotor_Backward 

//Second Motor
#define on_second 
#define SMotor_Forward 
#define SMotor_Backward 

void setup() {
//Set the baud rate for serial data transmission  
Serial.begin(9600)  
//Pins configuration
pinMode(on_first, OUTPUT);
pinMode(FMotor_Forward, OUTPUT);
pinMode(FMotor_Backward, OUTPUT);

pinMode(on_second,OUTPUT);
pinMode(SMotor_Forward,OUTPUT);
pinMode(SMotor_Backward,OUTPUT);

}

void loop() {
//Starting Motors
digitalWrite(on_first, HIGH);
digitalWrite(on_second, HIGH);
delay(1000);

//Move backwards
digitalWrite(FMotor_Backward, HIGH);
digitalWrite(FMotor_Forward, LOW);

digitalWrite(SMotor_Backward, HIGH);
digitalWrite(SMotor_Forward, LOW);

delay(1000);

//Move forward
digitalWrite(FMotor_Backward, LOW);
digitalWrite(FMotor_Forward, HIGH);

digitalWrite(SMotor_Backward, LOW);
digitalWrite(SMotor_Forward, HIGH);

delay(1000);
//Stop motors
digitalWrite(on_first, LOW);
digitalWrite(on_second, LOW);
Serial.println("Stopped motors");
delay(5000);
}
