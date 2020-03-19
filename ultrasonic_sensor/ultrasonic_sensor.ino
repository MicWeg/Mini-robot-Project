//Define which pins to use for sensor's input and output
#define Trigger
#define Echo

//Define variable for distance
int distance;

void setup() {
  //Set the baud rate for serial data transmission
  Serial.begin(9600);
  //Pins configuration
  pinMode(Trigger, OUTPUT);
  pinMode(Echo, INPUT);
}

void loop() {
  //Set the Trigger to LOW for 2 microseconds
  digitalWrite(Trigger, LOW);
  delayMicroseconds(2);

  //Set the Trigger to HIGH for 10 microseconds to emit 40kHz sound pulse
  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigger, LOW);

  //Read the lenght of the pulse in microseconds and calculate the distance in centimeters
  distance = pulseIn(Echo, HIGH) / 58;

  //Print the distance in centimeters
  Serial.println(distance);

  delay(100);
}
