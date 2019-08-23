int comdata;
String inString;

void setup()
{
// put your setup code here, to run once:
  Serial.begin(9600);
}
 
 
void loop() {
// put your main code here, to run repeatedly:
  while (Serial.available() > 0) 
  {
      int inChar = Serial.read();
      if ((char)inChar == '$') 
      {
        String GPS;
        do
        {
          inChar = Serial.read();
          GPS += (char)inChar;
        }
        while((char)inChar != ',');
        do 
         {
            inChar = Serial.read();
            inString += (char)inChar;
          }
        while (isDigit(inChar));
      comdata=inString.toInt();
      Serial.println(comdata);
      inString = "";
      }
  }
  inString = "";
  delay(200);
}
