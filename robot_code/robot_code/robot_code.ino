#include <Servo.h>
#define DEBUG

//This is the main code for BRUTUS the sumobot
//Written by Kayla Frost and Ben Straub

int motorPin = 3;     //Set motor pin - must be PWM
int buttonPin = 4;    //Set the start button pin - doesn't matter what kind of pin
int upPos = 0;        //Raised position for the plate
int downPos = 110;     //Lowered position for the plate

Servo motor;          //Instantiate the motor

void setup()
{
  #ifdef DEBUG               //opem serial connection if in debug mode
    Serial.begin(9600);
  #endif
  motor.attach(motorPin);    //assign the motor object to the defined motor pin
  pinMode(buttonPin, INPUT); //set the button pin as an input  
}

void loop()
{
  motor.write(upPos);  //Have the robot hold the plate up initially
  while(!digitalRead(buttonPin)) { } //Wait for the button to be pressed
  #ifdef DEBUG
    Serial.print("Button pressed!");
  #endif
  motor.write(downPos); //Drop the plate
  delay(10000);
}
