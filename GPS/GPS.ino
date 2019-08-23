#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3);
void setup()
{

  mySerial.begin(9600);
  Serial.begin(9600);
}

void loop()
{
  if (mySerial.available())
  {
    Serial.print(char (mySerial.read())); 
  }
}
