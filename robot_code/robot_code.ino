#include <QTRSensors.h>
#include <NewPing.h>
#include <Servo.h>

//This is the main code for BRUTUS the sumobot
//Written by Kayla Frost and Ben Straub


#define DEBUG
#define UP_POS 0           //Raised position for the plate
#define DOWN_POS 110       //Lowered position for the plate
#define LEFT 0             //Define left and right as integers to minimize memory usage
#define RIGHT 1        
#define MAX_PING_DIST 40   //Maximum distance we want to ping for (in cm)
#define NO_TRIG 0          //Define strings to represent the different sensor triggers
#define PING_TRIG 1
#define LRIR_TRIG 2
#define SRIR_TRIG 3
#define WIDE_SEARCH_MODE 1 //Define constants for the different modes
#define FINE_SEARCH_MODE 2
#define CHARGE_MODE 3
#define MIN_PING_DELAY 75

#define NUM_LINE_SENSORS   2     // number of sensors used
#define LINE_SENSOR_TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define LINE_EMITTER_PIN   QTR_NO_EMITTER_PIN     // emitter is controlled by digital pin 2

#define R_LINE_PIN 12      //Set the right line sensor pin
#define L_LINE_PIN 13      //Set the left line sensor pin
#define PING_PIN A5        //Set the ping sensor pin
#define LRIR_PIN A0        //Set the long range IR (LRIR) pin
#define SRIR_PIN A1        //Set the short range IR (SRIR) pin
#define SERVO_PIN 9        //Set servo pin - must be PWM
#define BUTTON_PIN A4      //Set the start button pin - doesn't matter what kind of pin
#define MC_L_EN_PIN 3      //Left (labed #1 on board) motor controller enable pin - Must be PWM
#define MC_L_C_PIN 2       //Left motor controller "C" pin
#define MC_L_D_PIN 4       //Left motor controller "D" pin
#define MC_R_EN_PIN 5      //Right (labeled #2 on board) motor controller enable pin - Must be PWM
#define MC_R_C_PIN 7       //Right motor controller "C" pin
#define MC_R_D_PIN 6       //Right motor controller "D" pin

int timeLastPing = 0;        //global var of last ping
int scanVel = 50;           //Ideal velocity for scanning withsensors
int chargeVel = 75;         //Ideal velocity for charging!
int maxVel = 150;            //Maximum velocity (also for stopping quickly)
int fiveSeconds = 5000;      //Define 5 seconds in miliseconds
int mode = WIDE_SEARCH_MODE; //Define a global "mode" variable
boolean rightLineFlag = false; //Define a flag for seeing the line from the right line sensor
boolean leftLineFlag = false;  //Define a flag for seeing the line from the left line sensor

Servo motor;   //Instantiate the servo
NewPing sonar(PING_PIN, PING_PIN, MAX_PING_DIST);  //NewPing setup of pin and maximum distance
QTRSensorsRC qtrrc((unsigned char[]) {R_LINE_PIN, L_LINE_PIN},  //instantiate the left and right line sensors as an object
  NUM_LINE_SENSORS, LINE_SENSOR_TIMEOUT, LINE_EMITTER_PIN); 
unsigned int sensorValues[NUM_LINE_SENSORS];


void setup()
{
  #ifdef DEBUG               //opem serial connection if in debug mode
    Serial.begin(9600);
  #endif
  motor.attach(SERVO_PIN);    //assign the motor object to the defined servo pin
  pinMode(BUTTON_PIN, INPUT); //set the button pin as an input  
  pinMode(MC_L_EN_PIN, OUTPUT);
  pinMode(MC_L_C_PIN, OUTPUT);
  pinMode(MC_L_D_PIN, OUTPUT);
  pinMode(MC_R_EN_PIN, OUTPUT);
  pinMode(MC_R_C_PIN, OUTPUT);
  pinMode(MC_R_D_PIN, OUTPUT);
  pinMode(PING_PIN, INPUT);
  pinMode(LRIR_PIN, INPUT);
  pinMode(SRIR_PIN, INPUT);

  motor.write(UP_POS);  //Have the robot hold the plate up initially
  while(!digitalRead(BUTTON_PIN)) { } //Wait for the button to be pressed
  #ifdef DEBUG
    Serial.print("Button pressed!");
  #endif
  delay(fiveSeconds);
  motor.write(DOWN_POS); //Drop the plate


}

void loop()
{
  //Somehow calibrate the reflective sensors  
#ifdef DEBUG
  Serial.print("Mode:");
  Serial.println(mode);
#endif
if (mode == WIDE_SEARCH_MODE)
  wideSearch();
if (mode == FINE_SEARCH_MODE)  
  fineSearch();
if (mode == CHARGE_MODE)
  charge();
}

void driveForward(int vel) //vel (velocity) is of the range 0-255 and is a measure of how fast to turn the motors
{
   analogWrite(MC_L_EN_PIN, vel);    //Turn on the enable pin for both motor controller sides at the given velocity
   analogWrite(MC_R_EN_PIN, vel*1.2);
 
   digitalWrite(MC_L_C_PIN, HIGH);   //Driving forward calls for setting pin C high and pin D low
   digitalWrite(MC_L_D_PIN, LOW);
   digitalWrite(MC_R_C_PIN, HIGH);
   digitalWrite(MC_R_D_PIN, LOW);
}

void driveBackward(int vel) //vel (velocity) is of the range 0-255 and is a measure of how fast to turn the motors
{
  analogWrite(MC_L_EN_PIN, vel);  //Turn on the enable pin for both motor controller sides at the given speed 
  analogWrite(MC_R_EN_PIN, vel*1.2);
 
  digitalWrite(MC_L_C_PIN, LOW);   //Driving backward calls for setting pin C low and pin D high
  digitalWrite(MC_L_D_PIN, HIGH);
  digitalWrite(MC_R_C_PIN, LOW);
  digitalWrite(MC_R_D_PIN, HIGH);
}

void driveTurn(int vel, int direct) //vel (velocity) is of the range 0-255 and is a measure of how fast to turn the motors
{                                   //direct  (direction) dictates which way to turn, LEFT or RIGHT
  #ifdef DEBUG
    Serial.println(vel);
    Serial.println(direct);
  #endif
  analogWrite(MC_L_EN_PIN, vel);    //Turn on the enable pin for both motor controller sides at the given speed 
  analogWrite(MC_R_EN_PIN, vel*1.2);     

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
  analogWrite(MC_R_EN_PIN, vel*1.2);
  
  digitalWrite(MC_L_C_PIN, LOW);   //Stopping  calls for setting the C and D pins to the same value
  digitalWrite(MC_L_D_PIN, LOW);
  digitalWrite(MC_R_C_PIN, LOW);
  digitalWrite(MC_R_D_PIN, LOW);
}

void wideSearch()
//Search for target by swiveling left and right at intervals increasing by [addTime]
{
 int baseTime = 750;
 int addTime = 1500;
 #ifdef DEBUG
   Serial.println("About to turn left");
 #endif
 driveTurn(scanVel, LEFT);
 if (wideScan(baseTime))
   return; 
 driveTurn(scanVel, RIGHT);
 if (wideScan(baseTime + addTime))
   return;
 driveTurn(scanVel, LEFT);
 if (wideScan(baseTime + 2*addTime))
   return;
 driveTurn(scanVel, RIGHT);
 if (wideScan(baseTime + 3*addTime))
   return;
}

void fineSearch()
{
  driveTurn(scanVel, LEFT);
  while(checkPing() == PING_TRIG){
    if(checkSRIR() == SRIR_TRIG || checkLRIR() == LRIR_TRIG)
      {
        mode = CHARGE_MODE;
        return;
      }
  }
  int timeLeft = millis();
  driveTurn(scanVel, RIGHT);
  delay(50);   //Give the robot some time to get back into ping range
    while(checkPing() == PING_TRIG){
    if(checkSRIR() == SRIR_TRIG || checkLRIR() == LRIR_TRIG)
      {
        mode = CHARGE_MODE;
        return;
      }
    }
  int timeRight = millis();
  while(millis() < ((timeRight-timeLeft)/2))  //Use the average of the two times to return to approximately the middle/starting position
    driveTurn(scanVel, LEFT);
  driveForward(scanVel);
}

void charge()
{
  driveForward(chargeVel);
  while(true){
    if(checkLines()){
      mode = WIDE_SEARCH_MODE;
      getAwayFromEdge();
      break;
    }
    if(checkLRIR() != LRIR_TRIG && checkSRIR() != SRIR_TRIG){
      mode = FINE_SEARCH_MODE;
      break;
    }
    if(checkPing() != PING_TRIG){
      mode = WIDE_SEARCH_MODE;
      break;  
    }
  }
  driveStop(maxVel);
}

boolean wideScan(int interval)
//While searching for target, monitor the sonar and long range IR sensors for any objects
{
 int time = millis();
 int targetTime = time + interval;
 while(millis() < targetTime)
 {
   if(checkSRIR() == SRIR_TRIG){
     driveStop(scanVel);
     mode = CHARGE_MODE;
     #ifdef DEBUG
       Serial.println("SRIR Detected");
     #endif
     return true;}
   if(checkLRIR() == LRIR_TRIG){
     driveStop(scanVel);
     mode = CHARGE_MODE;
     #ifdef DEBUG
       Serial.println("LRIR Detected");
     #endif 
     return true;}
   if(checkPing() == PING_TRIG){
     driveStop(scanVel);
     mode = FINE_SEARCH_MODE;
     #ifdef DEBUG
       Serial.println("PING Detected");
     #endif     
     return true;}
   return false;
 }
}


int checkPing()
{
  int currentTime = millis();
  if(currentTime-timeLastPing < MIN_PING_DELAY)
    delay(MIN_PING_DELAY - currentTime + timeLastPing);
  timeLastPing = millis();
  if (sonar.ping() > 0)                      //Check to see if the ping sensor sees anything
    return PING_TRIG;
  return NO_TRIG;
}

int checkLRIR()
{
 int LRIRvalue = analogRead(LRIR_PIN);       // reads the value of the long range IR sensor
 if (LRIRvalue > 150)
   return LRIR_TRIG;
 return NO_TRIG;
}

int checkSRIR()
{
int SRIRvalue = digitalRead(SRIR_PIN);      //reads the value of the short range IR sensor
if (!SRIRvalue)
  return SRIR_TRIG;
return NO_TRIG;
}

boolean checkLines()
{
  qtrrc.read(sensorValues);
  #ifdef DEBUG
    Serial.println("Sensors");
    Serial.println(sensorValues[0]);
    Serial.println(sensorValues[1]);
  #endif
  if(sensorValues[0] < 1750){
    rightLineFlag = false;
    driveStop(maxVel);
    return true;
  }
  if(sensorValues[1] < 1750){
    leftLineFlag = true;
    driveStop(maxVel);
    return true;
  }
  return false;
}

void getAwayFromEdge()
{
  
  driveBackward(scanVel);
  delay(100);
  driveTurn(scanVel, LEFT);
  delay(100);
  #ifdef DEBUG
    Serial.println("I'm trying to get away from the edge");
  #endif
  driveStop(scanVel);
  rightLineFlag = false;
  leftLineFlag = false;
//  }
//  if(rightLineFlag == true){
//    driveTurn(scanVel, LEFT);
//    delay(750);
//    driveStop(scanVel);
//    rightLineFlag = false;   
//  }
//  if(leftLineFlag == true){
//    driveTurn(scanVel, RIGHT);
//    delay(750);
//    driveStop(scanVel);
//    leftLineFlag == false;
//  }
}
