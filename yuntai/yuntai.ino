#include "Servo.h"
const int servoPWM = 1;
const int trig = 8;
const int echo = 9;
Servo yun_servo;
byte angle;
long intervalTime = 0;

void setup()
{
    yun_servo.attach(servoPWM);
    angle = 0;
    yun_servo.write(angle);
    pinMode(echo,INPUT);
    pinMode(trig,OUTPUT);
    Serial.begin(9600);
}

void loop()
[
    digitalWrite(trig,1);   //置高电平
    delayMicroseconds(15);
    digitalWrite(trig,0);   //设置为低点评
    intervalTime = pulseIn(echo,HIGH);
    float S = intervalTime/58.0;
    S = 0;
    intervalTime = 0;
    for(int i = 0;i < 50 && angle < 180;i++)
    {
        angle++;
        yun_servo.write(angle);
        delay(15);
    }
    if(angle >= 179)
    {
        while(angle > 0)
        {
            angle--;
            yun_servo.write(angle);
            delay(15);
        }
    }
    delay(500);
]