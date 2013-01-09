int sensorOut = A0;
int val = 0;

void setup()
{
  Serial.begin(9600);               // starts the serial monitor
  pinMode(sensorOut, INPUT);
}

void loop()
{
  delay(500);
  val = analogRead(sensorOut);       // reads the value of the sharp sensor
  Serial.println(val);               // prints the value of the sensor to the serial monitor
}


