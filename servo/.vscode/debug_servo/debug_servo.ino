#include <Servo.h>
Servo ss;
byte angle;
bool rotation_way;
void setup()
{
    Serial.begin(9600);
    Serial.println("Begin your debug!!");
    angle = 0;
    ss.attach(3);
    ss.write(angle);
    delay(15);
}

void loop()
{
    if(rotation_way)
    {
        angle ++;
        ss.write(angle);
        delay(15);
    }
    else
    {
        angle--;
        ss.write(angle);
        delay(15);
    }

    if(angle >= 180)
    {
        angle = 180;
        ss.write(angle);
        delay(15);
        rotation_way = false;
    }
    if(angle <= 5)
    {
        angle = 0;
        ss.write(angle);
        delay(15);
        rotation_way = true;
    }
}
