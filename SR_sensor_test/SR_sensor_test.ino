int sensorPin = A5;
int sensorVal;

void setup()
{
  Serial.begin(9600);
}
void loop()
{
  sensorVal = digitalRead(sensorPin);
  Serial.print(sensorVal);
  if (sensorVal)
  {
    Serial.println("\t\tYES!");
  } else {
    Serial.println("\tNO!");
  }
  delay(200);
}
