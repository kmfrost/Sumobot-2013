#include <Servo.h>
#define DEBUG

//This is the main code for BRUTUS the sumobot
//Written by Kayla Frost and Ben Straub

int motorPin = 3;     //Set motor pin - must be PWM
int buttonPin = 4;    //Set the start button pin - doesn't matter what kind of pin
int upPos = 0;        //Raised position for the plate
int downPos = 110;    //Lowered position for the plate
int mcLEnPin = 0;     //Left motor controller enable pin - Must be PWM
int mcLCPin = 0;      //Left motor controller "C" pin
int mcLDPin = 0;      //Left motor controller "D" pin
int mcREnPin = 0;     //Right motor controller enable pin - Must be PWM
int mcRCPin = 0;      //Right motor controller "C" pin
int mcRDPin = 0;      //Right motor controller "D" pin 
int scanvel = 0;    //Ideal vel for scanning with sensors

Servo motor;          //Instantiate the motor


void setup()
{
  #ifdef DEBUG               //opem serial connection if in debug mode
    Serial.begin(9600);
  #endif
  motor.attach(motorPin);    //assign the motor object to the defined motor pin
  pinMode(buttonPin, INPUT); //set the button pin as an input  
  
  pinMode(mcLEnPin, OUTPUT);
  pinMode(mcLCPin, OUTPUT);
  pinMode(mcLDPin, OUTPUT);
  pinMode(mcREnPin, OUTPUT);
  pinMode(mcRCPin, OUTPUT);
  pinMode(mcRDPin, OUTPUT);
}

void loop()
{
  motor.write(upPos);  //Have the robot hold the plate up initially
  while(!digitalRead(buttonPin)) { } //Wait for the button to be pressed
  #ifdef DEBUG
    Serial.print("Button pressed!");
  #endif
  motor.write(downPos); //Drop the plate
  delay(5000)
  
  //Somehow calibrate the reflective sensors  
  
  driveForward(100);
  delay(5000);
  driveBackward(100);
  delay(5000);
  driveTurn(100, "LEFT");
  delay(5000);
  driveTurn(100, "RIGHT");
  delay(5000);
  driveStop(100);
  delay(1000000000);
}

void driveForward(int vel) //vel (velocity) is of the range 0-255 and is a measure of how fast to turn the motors
{
   analogWrite(mcLEnPin, vel);    //Turn on the enable pin for both motor controller sides at the given velocity
   analogWrite(mcREnPin, vel);
   
   digitalWrite(mcLCPin, HIGH);   //Driving forward calls for setting pin C high and pin D low
   digitalWrite(mcLDPin, LOW);
   digitalWrite(mcRCPin, HIGH);
   digitalWrite(mcRDPin, LOW);
}

void driveBackward(int vel) //vel (velocity) is of the range 0-255 and is a measure of how fast to turn the motors
{
   analogWrite(mcLEnPin, vel);  //Turn on the enable pin for both motor controller sides at the given speed 
   analogWrite(mcREnPin, vel);
   
   digitalWrite(mcLCPin, LOW);   //Driving backward calls for setting pin C low and pin D high
   digitalWrite(mcLDPin, HIGH);
   digitalWrite(mcRCPin, LOW);
   digitalWrite(mcRDPin, HIGH);
}

void driveTurn(int vel, String direct) //vel (velocity) is of the range 0-255 and is a measure of how fast to turn the motors
{                              //direct  (direction) dictates which way to turn, "LEFT" or "RIGHT"
  analogWrite(mcLEnPin, vel);  //Turn on the enable pin for both motor controller sides at the given speed 
  analogWrite(mcREnPin, vel);     

  if (direct == "LEFT")
  {
   digitalWrite(mcLCPin, LOW);    //Stop the left wheel from turning - set pin C and pin D to the same value
   digitalWrite(mcLDPin, LOW);
   digitalWrite(mcRCPin, HIGH);   //Drive the right wheel forward - setting pin C high and pin D low
   digitalWrite(mcRDPin, LOW);
  }
  else if (direct == "RIGHT")
  {
   digitalWrite(mcLCPin, HIGH);   //Drive the left wheel forward - setting pin C high and pin D low
   digitalWrite(mcLDPin, LOW);
   digitalWrite(mcRCPin, LOW);    //Stop the right wheel from turning - set pin C and pin D to the same value
   digitalWrite(mcRDPin, LOW);
  }
}


void driveStop(int vel)  //vel (velocity) is of the range 0-255 and is a measure of how quickly to stop
{
  analogWrite(mcLEnPin, vel);  //Turn on the enable pin for both motor controller sides at the given speed 
  analogWrite(mcREnPin, vel);
  
  digitalWrite(mcLCPin, LOW);   //Stopping  calls for setting the C and D pins to the same value
  digitalWrite(mcLDPin, LOW);
  digitalWrite(mcRCPin, LOW);
  digitalWrite(mcRDPin, LOW);
}
   
