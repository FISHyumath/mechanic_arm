#include <SoftwareSerial.h>

//发射端程序

#include <SPI.h>
#include <Mirf.h>
#include <MirfHardwareSpiDriver.h>
#include <MirfSpiDriver.h>
#include <nRF24L01.h>

char test[100];
int cnt;

int value;

SoftwareSerial mySerial(2, 3);

void setup()
{
  
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"ABCDE"); //设置自己的地址（发送端地址），使用5个字符
  Mirf.payload = sizeof(value);
  Mirf.channel = 50;              //设置所用信道
  Mirf.config();
  mySerial.begin(9600);
  Serial.begin(9600);
  Mirf.setTADDR((byte *)"FGHIJ");
}

void loop()
{
  if (mySerial.available())
  {
    Mirf.setTADDR((byte *)"FGHIJ");           //设置接收端地址                     //0-255的随机数
    Mirf.send(char (mySerial.read()));
    while(Mirf.isSending()) delay(1); //直到发送成功，退出循环
    //Serial.print(char (mySerial.read())); 
  }
  
  //Mirf.setTADDR((byte *)"FGHIJ");           //设置接收端地址
  //value = random(255);                      //0-255的随机数
  //for (int i=0;i<cnt;i++)
 // {
  //  Mirf.send(test[i]);
  //  while(Mirf.isSending()) delay(1); //直到发送成功，退出循环
  //}
  //delay(1000);
  
  

}
