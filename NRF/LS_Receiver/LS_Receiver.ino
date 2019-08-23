
//接收端程序

#include <SPI.h>
#include <Mirf.h>
#include <MirfHardwareSpiDriver.h>
#include <MirfSpiDriver.h>
#include <nRF24L01.h>


int value;

void setup()
{
  Serial.begin(9600);
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"FGHIJ"); //设置自己的地址（接收端地址），使用5个字符
  Mirf.payload = sizeof(value);   
  Mirf.channel = 50;   //设置使用的信道
  Mirf.config(); 
  Serial.println("Listening...");  //开始监听接收到的数据
}

void loop()
{
  if(Mirf.dataReady()) 
  {  //当接收到程序，便从串口输出接收到的数据
    Mirf.getData((byte *) &value);
    Serial.print("Got data: ");
    Serial.println(value);
  }
}
