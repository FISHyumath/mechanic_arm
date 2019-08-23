//*******预定义指令********
#define READ_REG 0x00
#define WRITE_REG 0x20
#define RD_RX_PLOAD 0x61
#define WR_TX_PLOAD 0xA0
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define REUSE_TX_PL 0xE3
#define NOP 0xFF
//*********中断标志位**********
#define RX_DR 0x40
#define TX_DS 0x20
#define MAX_RT 0x10
//########寄存器地址***********
#define CONFIG 0x00
#define EN_AA 0x01
#define EN_RXADDR 0x02
#define SETUP_AW 0x03
#define SETUP_RETR 0x04
#define RF_CH 0x05
#define RF_SETUP 0x06
#define STATUS 0x07
#define OBSERVE_TX 0x08
#define CD 0x09
#define RX_ADDR_P0 0x0A
#define RX_ADDR_P1 0x0B
#define RX_ADDR_P2 0x0C
#define RX_ADDR_P3 0x0D
#define RX_ADDR_P4 0x0E
#define RX_ADDR_P5 0x0F
#define TX_ADDR 0x10
#define RX_PW_P0 0x11
#define RX_PW_P1 0x12
#define RX_PW_P2 0x13
#define RX_PW_P3 0x14
#define RX_PW_P4 0x15
#define RX_PW_P5 0x16
#define FIFO_STATUS 0x17
//**************引脚定义************
#define IRQ 8
#define CE 9
#define CSN 10
//**************数据宽度定义*********
#define RX_ADR_WIDTH 5
#define RX_PLOAD_WIDTH 32

byte rx_buf[RX_PLOAD_WIDTH];

//*************数据包各位数据定义*****
#define X rx_buf[0] //水平移动信号x
#define Y rx_buf[1] //竖直移动信号y
#define A rx_buf[2] //A按键SW1
#define B rx_buf[3]  //B按键SW2
#define C rx_buf[4]  //C按键SW3
#define D rx_buf[5]  //D按键SW4
#define KEY rx_buf[6]  //按下操纵杆

#include "Servo.h"

byte RX_ADDRESS[RX_ADR_WIDTH] =
{
  0x34, 0x43, 0x10, 0x10, 0x01 //模块发送区地址定义
};
byte angle[6];
Servo servo[6];
void setup()
{
  pinMode(CE, OUTPUT);
  pinMode(CSN, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(IRQ, INPUT);
  pinMode(SCK, OUTPUT);
  //设置引脚
  servo[0].attach(2);
  servo[1].attach(3);
  servo[2].attach(4);
  servo[3].attach(5);
  servo[4].attach(6);
  servo[5].attach(7);

  angle[0] = 90; //底盘
  angle[1] = 50;  //腰部
  angle[2] = 100;//肩部
  angle[3] = 0; //大臂
  angle[4] = 0; //小臂
  angle[5] = 0; //手
  for (int i = 0; i < 6; i++)
  {
    servo[i].write(angle[i]);
  }
  Serial.begin(9600);
  init_nrf24l01();
  byte status = SPI_Read(STATUS);
  Serial.println("*************RX_Mode start***************");
  Serial.println(status, HEX);
  RX_Mode();
}

void loop()
{
  RX_DATA();  //接收数据包
  /*
    servoX_move();
    servoY_move();
    servo_hand_move();
  */
}

void servoX_move()
{
  /*
    if(X > 250 || X < 10)
    {
      Serial.print("X is: ");
      Serial.println(X);
    }
  */
  if (X > 138) //判断X轴是否向左（X轴的中位模拟量数值是128）
  {
    angle[0] += 2;
    if (angle[0] > 180) angle[0] = 180;
    servo[0].write(angle[0]);
    delay(15);
  }
  else if (X < 118) //判断X轴是否向左
  {
    angle[0] -= 2;//X轴摇杆向左则舵机角度减少2
    if (angle[0] < 2) angle[0] = 2; //左极限为2
    servo[0].write(angle[0]);
    delay(15);
  }
}

void servoY_move()
{
  /*
    if(Y > 250 || Y < 10)
    {
      Serial.print("X is: ");
      Serial.println(X);
    }
  */
  if (Y > 138)
  {
    angle[1] += 2;
    if (angle[1] > 180) angle[1] = 180;
    servo[1].write(angle[1]);
    delay(15);
  }
  if (Y < 118)
  {
    angle[1] -= 2;
    if (angle[1] < 2) angle[1] = 2;
    servo[1].write(angle[1]);
    delay(15);
  }
}


void servo_hand_move()
{
  if (A == 0xFF) //加紧
  {
    angle[5] += 2;
    if (angle[5] > 100) angle[2] = 100;
    servo[2].write(angle[2]);
    delay(15);
  }
  if (C == 0xFF) //张开
  {
    angle[5] -= 2;
    if (angle[5] < 10) angle[5] = 0;
    servo[5].write(angle[5]);
    delay(15);
  }
}

//*************模块引脚初始化************
void init_nrf24l01()
{
  digitalWrite(IRQ, 0);
  digitalWrite(CE, 0);
  digitalWrite(CSN, 1);
}

//*************SPI读写操作，单字节读取以及写入
byte SPI_RW(unsigned char Byte)
{
  byte i;
  for (int i = 0; i < 8; i++)
  {
    if (Byte & 0x80)
    {
      digitalWrite(MOSI, 1);
    }
    else
    {
      digitalWrite(MOSI, 0);
    }
    digitalWrite(SCK, 1);
    Byte <<= 1;
    if (digitalRead(MISO) == 1)
    {
      Byte |= 1;
    }
    digitalWrite(SCK, 0);
  }
  return (Byte);
}

//************SPI写寄存器函数，带寄存器地址参数和数据参数******
byte SPI_RW_Reg(byte reg, byte value)
{
  byte status;  //状态变量
  digitalWrite(CSN, 0); //拉低CSN，开始SPI通信
  status = SPI_RW(reg); //选取寄存器
  SPI_RW(value);  //向寄存器写入数据
  digitalWrite(CSN, 1);
  return (status);
}

//************SPI读寄存器操作***************
byte SPI_Read(byte reg)
{
  byte reg_val; //定义寄存器返回值形参
  digitalWrite(CSN, 0);
  SPI_RW(reg);
  reg_val = SPI_RW(0);  //给形参Byte赋初值
  digitalWrite(CSN, 1);
  return (reg_val);
}

//************SPI读数据包操作，带寄存器地址，存数据包数组名，需要读取的字节个数*****
byte SPI_Read_Buf(byte reg, byte *pBuf, byte bytes)
{
  byte status, i;
  digitalWrite(CSN, 0);
  status = SPI_RW(reg);
  for (int i = 0; i < bytes; i++)
  {
    pBuf[i] = SPI_RW(0);
  }
  digitalWrite(CSN, 1);
  return (status);
}

//***********SPI写数据包操作。带据存起地址，写入数据包数组名，需要写入的字节个数三个实参*****
byte SPI_Write_Buf(byte reg, byte *pBuf, byte bytes)
{
  byte status, i;
  digitalWrite(CSN, 0);
  status = SPI_RW(reg); //写寄存器地址，读寄存器状态值
  for (i = 0; i < bytes; i++)
  {
    SPI_RW(*pBuf++);  //数组内的竖直一次送入模块数据发送缓冲区待发
  }
  digitalWrite(CSN, 1);
  return (status);
}

void RX_DATA()
{
  byte status;
  status = SPI_Read(STATUS);  //读取状态寄存器STATUS的状态值
  if (status & RX_DR) //若接受到数据,RX_DR位置1，产生中断
  {
    SPI_Read_Buf(RD_RX_PLOAD, rx_buf, RX_PLOAD_WIDTH);  //读取数据包
    SPI_RW_Reg(FLUSH_RX, 0);   //清空RX_FIFO寄存器，为下一次接受数据包作准备
  }
  SPI_RW_Reg(WRITE_REG + STATUS, status);   //清除接受中断，原理同上
  for (int i = 0; i < 6; i++)
  {
    Serial.print(rx_buf[i]);
  }
  Serial.print("\n");
}

void RX_Mode()
{
  digitalWrite(CE, 0); //拉低CE线，准备写设置指令
  SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); //写数据通道0的接受地址
  SPI_RW_Reg(WRITE_REG + EN_AA, 0x01); //数据接受通道0开启自动应答信号
  SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01); //数据接受通道0开启
  SPI_RW_Reg(WRITE_REG + RF_CH, 40);  //设置工作通道频率为40
  SPI_RW_Reg(WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); //设置数据包有效位
  SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07); //设置发送速率2Mb/s，发射功率0dBm
  SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);   //设置为接受模式，模式商店，16位CRC校验
  digitalWrite(CE, 1); //拉高CE线产生上升沿没进入RX模式
}
