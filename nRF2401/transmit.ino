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
#define TX_ADR_WIDTH 5
#define TX_PLOAD_WIDTH 32
//*************扩展版数据读取定义*****
#define X analogRead(0)/4 //水平移动信号x
#define Y analogRead(1)/4 //竖直移动信号y
#define A digitalRead(2)  //A按键SW1
#define B digitalRead(3)  //B按键SW2
#define C digitalRead(4)  //C按键SW3
#define D digitalRead(5)  //D按键SW4
#define KEY digitalRead(6)  //按下操纵杆

byte TX_ADDRESS[TX_ADR_WIDTH] =
{
  0x34, 0x43, 0x10, 0x10, 0x01 //模块发送区地址定义
};
byte tx_buf[TX_PLOAD_WIDTH];  //定义存放发送的数据包数组，宽带为32个字节

void setup()
{
  // put your setup code here, to run once:
  pinMode(CE, OUTPUT);
  pinMode(CSN, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(IRQ, INPUT);
  pinMode(SCK, OUTPUT);
  Serial.begin(9600);
  init_nrf24l01();
  byte status = SPI_Read(STATUS);
  Serial.print("********TX_MODE_START************");
  Serial.print("status = ");
  Serial.println(status, HEX);
  TX_Mode();
}


void loop()
{
  // put your main code here, to run repeatedly:
  tx_buf[0] = X;
  tx_buf[1] = Y;
  if (A == 0)
  {
    tx_buf[2] = 0xFF; //判断按钮是否按下，按下发送0xFF,否则发送0
    Serial.print("A is push\n");
  }
  else tx_buf[2] = 0;
  if (B == 0)
  {
    tx_buf[3] = 0xFF;
    Serial.print("B is push\n");
  }
  else tx_buf[3] = 0;
  if (C == 0)
  {
    tx_buf[4] = 0xFF;
    Serial.print("C is push\n");
  }
  else tx_buf[4] = 0;
  if (D == 0)
  {
    tx_buf[5] = 0xFF;
    Serial.print("D is push\n");
  }
  else tx_buf[5] = 0;
  if (KEY == 0)
  {
    tx_buf[6] = 0xFF;
    Serial.print("KEY is push\n");
  }
  TX_DATA();  //发送数据包
  if (X > 250 || X < 10)
  {
    Serial.print("X is: ");
    Serial.println(X);
  }
  if (Y > 250 || Y < 10)
  {
    Serial.print("Y is: ");
    Serial.println(Y);
  }

}
//*************模块引脚初始化************
void init_nrf24l01()
{
  digitalWrite(IRQ, 0);
  digitalWrite(CE, 0);
  digitalWrite(CSN, 1);
  Serial.print("init\n");
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
  return Byte;
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
  return reg_val;
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
  return status;
}

//***********SPI写数据包操作。带据存起地址，写入数据包数组名，需要写入的字节个数三个实参*****
byte SPI_Write_Buf(byte reg, byte *pBuf, byte bytes)
{
  byte status, i;
  digitalWrite(CSN, 0);
  status = SPI_RW(reg); //写寄存器地址，读寄存器状态值
  for (i = 0; i < bytes; i++)
  {
    SPI_RW(*pBuf++);  //数组内的数值一次送入模块数据发送缓冲区待发
  }
  digitalWrite(CSN, 1);
  return status;
}

void TX_DATA()
{
  byte status;    //定义寄存器状态形参
  status = SPI_Read(STATUS);  //定义寄存器状态形参
  if (status & TX_DS)
  {
    SPI_RW_Reg(FLUSH_TX, 0); //  清空TX_FIFO寄存器，准备写下一个数据包
    SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); //写入代发数据包
  }
  if (status & MAX_RT)
  {
    SPI_RW_Reg(FLUSH_TX, 0);
    SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);
  }
  SPI_RW_Reg(WRITE_REG + STATUS, status);
  delay(20);
}

void TX_Mode()
{
  digitalWrite(CE, 0);
  SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);
  SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);
  SPI_RW_Reg(WRITE_REG + EN_AA, 0x01); //开启数据通道0自动应答
  SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01); //开启数据通道
  SPI_RW_Reg(WRITE_REG + CONFIG, 0x0E); //设置发射模式
  SPI_RW_Reg(WRITE_REG + RF_CH, 40); //设置发射频率
  SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07); //设置发送速率2Mb/s,发射功率0dbm
  SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x1A); //设置自动重发时间间隔500+86us
  SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);
  digitalWrite(CE, 1); //拉高CE线，产生上升沿，模块进入发射模式
}
