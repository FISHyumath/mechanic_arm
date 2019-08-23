
const int trig = 42;
const int echo = 9;
void setup()
{
  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);
  Serial.begin(9600);
}

void loop()
{
  long IntervalTime = 0;
  while (1)
  {
    digitalWrite(trig, 1);
    delayMicroseconds(150);
    digitalWrite(trig, 0);
    delayMicroseconds(100);
    while(!digitalRead(echo) && IntervalTime<10000) IntervalTime++;
   // IntervalTime = pulseIn(echo, HIGH);
    float S = IntervalTime / 58.0;
    Serial.print(IntervalTime);
    Serial.print("\n");
    S = 0;
    IntervalTime = 0;
    delay(500);
  }
}
