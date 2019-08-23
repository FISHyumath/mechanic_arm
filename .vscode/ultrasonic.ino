#include <HardwareSerial.h>

const int trig = 8;
const int echo = 9;
void setup()
{
    pinMode(echo,INPUT);
    pinMode(trig,OUTPUT);
    Serial.begin(9600);
}

void loop()
{
    long IntervalTime = 0;
    while(1)
    {
        digitalWrite(trig,1);
        delayMicroseconds(15);
        digitalWrite(trig,0);
        IntervalTime = pulseIn(echo,HIGH);
        float S = IntervalTime/58.0;
        Serial.printIn(S);
        S = 0;
        IntervalTime = 0;
        delay(500);
    }
}