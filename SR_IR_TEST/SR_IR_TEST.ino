int sensorOut = 51;
int sensorVin = 53;
boolean val = 0;

void setup()
{
  Serial.begin(9600);               // starts the serial monitor
  
  pinMode(sensorOut, INPUT);
  pinMode(sensorVin, OUTPUT);

  digitalWrite(sensorVin, HIGH);
}

void loop()
{
  val = digitalRead(sensorOut);       // reads the value of the sharp sensor
  Serial.println(val);               // prints the value of the sensor to the serial monitor
  delay(500);
}


