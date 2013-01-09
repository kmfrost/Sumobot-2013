#include <Servo.h>

int motorPin = 3;
unsigned short pos = 0;
Servo motor;

void setup()
{
  Serial.begin(9600);
  motor.attach(motorPin);
//  pinMode(motorPin, OUTPUT);
//  analogWrite(motorPin, 0);
}
void loop()
{
  char temp;
  if (Serial.available())
    temp = Serial.read();
  if (temp >= 0x30 && temp <= 0x39)
  {
    pos = (pos*10)+(temp-0x30);
    Serial.print("temp: ");
    Serial.print(temp);
    Serial.print("\tpos: ");
    Serial.print(pos);
  }
  else if (temp == 'w')
  {
    motor.write((pos));
    Serial.println(pos);
    pos = 0;
  }
}
