int Button=2, Y=6, R=3, G=4, B=5;  

    
 void setup(void)  

{  

  Serial.begin(9600);           //Turn on the Serial Port at 9600 bps  

  pinMode(Button, INPUT_PULLUP);   

  pinMode(R, INPUT_PULLUP);        

   pinMode(G, INPUT_PULLUP);        

   pinMode(B, INPUT_PULLUP);        

  pinMode(Y, INPUT_PULLUP);   //Enable the pull-up resistor on  button   

    

 }  

    

 void loop(void)  

 {  

  Serial.print("X=");   

  Serial.print(analogRead(0));    //Read the position of the joysticks X axis and print it on the serial port.  

  Serial.print(",");  

  Serial.print("Y=");   

  Serial.print(analogRead(1));    //Read the position of the joysticks Y axis and print it on the serial port.  

   Serial.print(",");  

   

   Serial.print(digitalRead(Button));    

   Serial.print(digitalRead(R));     

  Serial.print(digitalRead(G));    

   Serial.print(digitalRead(B));   

   Serial.println(digitalRead(Y));   //Read the value of the select button and print it on the serial port.  

    

  delay(100); //Wait for 100 ms, then go back to the beginning of 'loop' and repeat.  

 } 

