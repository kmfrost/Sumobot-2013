#include <NewPing.h>
#include <Servo.h>
#define DEBUG

//This is the main code for BRUTUS the sumobot
//Written by Kayla Frost and Ben Straub

int pingPin = A5;     //Set the ping sensor pin
int LRIRPin = A0;     //Set the long range IR (LRIR) pin
int SRIRPin = A1;     //Set the short range IR (SRIR) pin
int servoPin = 9;     //Set servo pin - must be PWM
int buttonPin = A4;   //Set the start button pin - doesn't matter what kind of pin
int mcLEnPin = 3;     //Left (labed #1 on board) motor controller enable pin - Must be PWM
int mcLCPin = 2;      //Left motor controller "C" pin
int mcLDPin = 4;      //Left motor controller "D" pin
int mcREnPin = 5;     //Right (labeled #2 on board) motor controller enable pin - Must be PWM
int mcRCPin = 7;      //Right motor controller "C" pin
int mcRDPin = 6;      //Right motor controller "D" pin 
int scanVel = 100;    //Ideal vel for scanning with sensors
int LEFT = 0;         //Define left and right as integers to minimize memory usage
int RIGHT = 1;        
int upPos = 0;        //Raised position for the plate
int downPos = 110;    //Lowered position for the plate
int maxPingDist = 70; //Maximum distance we want to ping for (in cm)

Servo motor;          //Instantiate the servo
NewPing sonar(pingPin, pingPin, maxPingDist); //NewPing setup of pin and maximum distance 


void setup()
{
  #ifdef DEBUG               //opem serial connection if in debug mode
    Serial.begin(9600);
  #endif
  motor.attach(servoPin);    //assign the motor object to the defined servo pin
  pinMode(buttonPin, INPUT); //set the button pin as an input  
  
  pinMode(mcLEnPin, OUTPUT);
  pinMode(mcLCPin, OUTPUT);
  pinMode(mcLDPin, OUTPUT);
  pinMode(mcREnPin, OUTPUT);
  pinMode(mcRCPin, OUTPUT);
  pinMode(mcRDPin, OUTPUT);
  pinMode(pingPin, INPUT);
  pinMode(LRIRPin, INPUT);
  pinMode(SRIRPin, INPUT);

  motor.write(upPos);  //Have the robot hold the plate up initially
  while(!digitalRead(buttonPin)) { } //Wait for the button to be pressed
  #ifdef DEBUG
    Serial.print("Button pressed!");
  #endif
  motor.write(downPos); //Drop the plate
  delay(5000);

}

void loop()
{
  //Somehow calibrate the reflective sensors  
  
  driveForward(100);
  delay(5000);
  driveBackward(100);
  delay(5000);
  driveTurn(100, LEFT);
  delay(5000);
  driveTurn(100, RIGHT);
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

void driveTurn(int vel, int direct) //vel (velocity) is of the range 0-255 and is a measure of how fast to turn the motors
{                              //direct  (direction) dictates which way to turn, LEFT or RIGHT
  analogWrite(mcLEnPin, vel);  //Turn on the enable pin for both motor controller sides at the given speed 
  analogWrite(mcREnPin, vel);     

  if (direct == LEFT)
  {
   digitalWrite(mcLCPin, LOW);    //Drive the left wheel backwards - setting the pin C low and pin D high
   digitalWrite(mcLDPin, HIGH);
   digitalWrite(mcRCPin, HIGH);   //Drive the right wheel forward - setting pin C high and pin D low
   digitalWrite(mcRDPin, LOW);
  }
  else if (direct == RIGHT)
  {
   digitalWrite(mcLCPin, HIGH);   //Drive the left wheel forward - setting pin C high and pin D low
   digitalWrite(mcLDPin, LOW);
   digitalWrite(mcRCPin, LOW);    //Drive the left wheel backwards - setting the pin C low and pin D high
   digitalWrite(mcRDPin, HIGH);
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

boolean searchForTarget()
//Search for target by swiveling left and right at intervals increasing by [addTime]
{
 int baseTime = 250;
 int addTime = 500;
 driveTurn(scanVel, LEFT);
 if (scan(baseTime))
   return true; 
 driveTurn(scanVel, RIGHT);
 if (scan(baseTime + addTime))
   return true;
 driveTurn(scanVel, LEFT);
 if (scan(baseTime + 2*addTime))
   return true;
 driveTurn(scanVel, RIGHT);
 if (scan(baseTime + 3*addTime))
   return true;
 driveStop(scanVel); 
 return false;
}

boolean scan(int interval)
//While searching for target, monitor the sonar and long range IR sensors for any objects
{
 int time = millis();
 int targetTime = time + interval;
 while(millis() < targetTime)
 {
   if(checkPing()){
     driveStop(scanVel);
     return true;}
   if(checkLRIR()){
     driveStop(scanVel);
     return true;}
 }
}

boolean checkPing()
{
  if (sonar.ping() > 0) //If the the sensor senses something
    return true;
  return false;
}

boolean checkLRIR()
{
 int sensorValue = analogRead(LRIRPin);       // reads the value of the long range IR sensor
 if (sensorValue > 100)
   return true;
 return false;
}

boolean checkSRIR()
{
return !digitalRead(SRIRPin);
}

