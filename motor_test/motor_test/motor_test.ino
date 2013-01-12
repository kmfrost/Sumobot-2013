#include <NewPing.h>
#include <Servo.h>

//This is the main code for BRUTUS the sumobot
//Written by Kayla Frost and Ben Straub

#define DEBUGPING_PIN
#define UP_POS = 0;        //Raised position for the plate
#define DOWN_POS = 110;    //Lowered position for the plate
#define LEFT = 0;         //Define left and right as integers to minimize memory usage
#define RIGHT = 1;        
#define MAX_PING_DIST = 70; //Maximum distance we want to ping for (in cm)
#define NO_TRIG = 0;       //Define strings to represent the different sensor triggers
#define PING_TRIG = 1;
#define LRIR_TRIG = 2;
#define SRIR_TRIG = 3;

#define PING_PIN = A5;     //Set the ping sensor pin
#define LRIR_PIN = A0;     //Set the long range IR (LRIR) pin
#define SRIR_PIN = A1;     //Set the short range IR (SRIR) pin
#define SERVO_PIN = 9;     //Set servo pin - must be PWM
#define BUTTON_PINMC_L_EN_PIN = A4;   //Set the start button pin - doesn't matter what kind of pin
#define MC_L_EN_PIN = 3;     //Left (labed #1 on board) motor controller enable pin - Must be PWM
#define MC_L_C_PIN = 2;      //Left motor controller "C" pin
#define MC_L_D_PIN = 4;      //Left motor controller "D" pin
#define MC_R_EN_PIN = 5;     //Right (labeled #2 on board) motor controller enable pin - Must be PWM
#define MC_R_C_PIN = 7;      //Right motor controller "C" pin
#define MC_R_D_PIN = 6;      //Right motor controller "D" pin

int scanVel = 100;    //Ideal vel for scanning with sensors     



Servo motor;          //Instantiate the servo
NewPing sonar(PING_PIN, PING_PIN, MAX_PING_DIST); //NewPing setup of pin and maximum distance 


void setup()
{
  #ifdef DEBUG               //opem serial connection if in debug mode
    Serial.begin(9600);
  #endif
  motor.attach(SERVO_PINBUTTON_PIN);    //assign the motor object to the defined servo pin
  pinMode(BUTTON_PIN, INPUT); //set the button pin as an input  
  
  pinMode(MC_L_EN_PIN, OUTPUT);
  pinMode(MC_L_C_PIN, OUTPUT);
  pinMode(MC_L_D_PIN, OUTPUT);
  pinMode(MC_R_EN_PIN, OUTPUT);
  pinMode(MC_R_C_PIN, OUTPUT);
  pinMode(MC_R_D_PIN, OUTPUT);
  pinMode(PING_PINLRIR_PIN, INPUT);
  pinMode(LRIR_PIN, INPUT);
  pinMode(SRIR_PIN, INPUT);

  motor.write(UP_POS);  //Have the robot hold the plate up initially
  while(!digitalRead(BUTTON_PIN)) { } //Wait for the button to be pressed
  #ifdef DEBUG
    Serial.print("Button pressed!");
  #endif
  motor.write(DOWN_POS); //Drop the plate
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
   analogWrite(MC_L_EN_PIN, vel);    //Turn on the enable pin for both motor controller sides at the given velocity
   analogWrite(MC_R_EN_PIN, vel);
   
   digitalWrite(MC_L_C_PIN, HIGH);   //Driving forward calls for setting pin C high and pin D low
   digitalWrite(MC_L_D_PIN, LOW);
   digitalWrite(MC_R_C_PIN, HIGH);
   digitalWrite(MC_R_D_PIN, LOW);
}

void driveBackward(int vel) //vel (velocity) is of the range 0-255 and is a measure of how fast to turn the motors
{
   analogWrite(MC_L_EN_PIN, vel);  //Turn on the enable pin for both motor controller sides at the given speed 
   analogWrite(MC_R_EN_PIN, vel);
   
   digitalWrite(MC_L_C_PIN, LOW);   //Driving backward calls for setting pin C low and pin D high
   digitalWrite(MC_L_D_PIN, HIGH);
   digitalWrite(MC_R_C_PIN, LOW);
   digitalWrite(MC_R_D_PIN, HIGH);
}

void driveTurn(int vel, int direct) //vel (velocity) is of the range 0-255 and is a measure of how fast to turn the motors
{                              //direct  (direction) dictates which way to turn, LEFT or RIGHT
  analogWrite(MC_L_EN_PIN, vel);  //Turn on the enable pin for both motor controller sides at the given speed 
  analogWrite(MC_R_EN_PIN, vel);     

  if (direct == LEFT)
  {
   digitalWrite(MC_L_C_PIN, LOW);    //Drive the left wheel backwards - setting the pin C low and pin D high
   digitalWrite(MC_L_D_PIN, HIGH);
   digitalWrite(MC_R_C_PIN, HIGH);   //Drive the right wheel forward - setting pin C high and pin D low
   digitalWrite(MC_R_D_PIN, LOW);
  }
  else if (direct == RIGHT)
  {
   digitalWrite(MC_L_C_PIN, HIGH);   //Drive the left wheel forward - setting pin C high and pin D low
   digitalWrite(MC_L_D_PIN, LOW);
   digitalWrite(MC_R_C_PIN, LOW);    //Drive the left wheel backwards - setting the pin C low and pin D high
   digitalWrite(MC_R_D_PIN, HIGH);
  }
}


void driveStop(int vel)  //vel (velocity) is of the range 0-255 and is a measure of how quickly to stop
{
  analogWrite(MC_L_EN_PIN, vel);  //Turn on the enable pin for both motor controller sides at the given speed 
  analogWrite(MC_R_EN_PIN, vel);
  
  digitalWrite(MC_L_C_PIN, LOW);   //Stopping  calls for setting the C and D pins to the same value
  digitalWrite(MC_L_D_PIN, LOW);
  digitalWrite(MC_R_C_PIN, LOW);
  digitalWrite(MC_R_D_PIN, LOW);
}

