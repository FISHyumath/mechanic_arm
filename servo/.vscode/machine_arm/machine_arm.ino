#include <Servo.h>
#include <Arduino.h>
#include <HardwareSerial.h>

const double pi = 3.1415926;
// String buffer;

//直角坐标系
struct xyzPoint 
{
  double xLoc;
  double yLoc;
  double zLoc;
};

//柱坐标系
struct cylindCoord
{
  double zHeight;
  double lRadius;
  double thetaAngle;
};

class Angle //舵机角度控制类
{
public:
  Angle(byte a_m, byte a_n) : angMemory(a_m), angNow(a_n) {}
  Angle(void);
  void setServo(int Pin);
  void changeAngle(byte ang);
  void angleReach();
  byte angNow;

private:
  byte angMemory;
  Servo myServo;
};

//快速跳转脱机角度可能发生堵转，这个舵机需要缓慢调整才行
void Angle::angleReach()
{
  byte angleTmp = angMemory;
  bool turningUp = false;
  if (angNow > angMemory)
  {
    turningUp = true; //缓慢调整上加角度
  }
  else
  {
    turningUp = false; //缓慢下降角度
  }
  if (turningUp)
  {
    while (angleTmp < angNow)
    {
      angleTmp++;
      myServo.write(angleTmp);
      delay(15);
    }
  }
  else
  {
    while (angleTmp > angNow)
    {
      angleTmp--;
      myServo.write(angleTmp);
      delay(15);
    }
  }
}
Angle::Angle(void)
{
}
void Angle::changeAngle(byte ang)
{
  angMemory = angNow;
  angNow = ang;
}
void Angle::setServo(int Pin)
{
  myServo.attach(Pin, 500, 2500);
}
class machineArm
{
public:
  void moveFirst();      //预备运动到摄像头前（收缩，防止碰撞）
  void moveToCamera();   //运动到摄像头前
  void moveBack();       //回到初始位置
  void alphaAuto();      //自动判别俯仰角度，分区讨论，实际数据的精简
  void catchIt();        //机械臂实现自动抓取
  xyzPoint calcuLoc();   //计算当前机械手中心的位置
  void attitudeAdjust(); //自动调姿
  void sensorGetLoc();
  Angle angle[6];    //舵机角度，与舵机名称一一对应,角度群部位相对轴的角度
  Angle analogAngle; //控制超声波模拟舵机的角度

private:
  //摄像部分路径
  int move_photo[9] =
      {
          70, 50, 10,
          50, 70, 50,
          -15, 90, 70};
  //动作一 预备抓取
  int move_first[12] =
      {
          0, 0, 0,
          10, 20, 30,
          30, 35, 60,
          45, 45, 90};
  int angle_tmp[4] = {0};
  Servo servo[6];    //舵机名称，分别为0，1，2，3，4，5号，全代码统一
  Servo analogServo; //控制超声波的模拟舵机
};
machineArm MACHINE; //定义机械臂
//全局坐标
double X = 0, Y = 0, Z = 0, L = 0, Zp = 0; //以腰部舵机中心建立坐标系，直角坐标系与柱坐标系相互转换
//注意Zp是相对于一号舵机中心的
//X,Y,Z代表物体体心坐标，假设抓取足够准确，使体心坐标与机械手位于统一坐标平面之内
int angle_alpha = 0; //机械手相对竖直方向z轴的俯仰角，逆时针方向，0-180,根据物体状态判别输入
int theta = 0;       //底盘的极角，极轴以底盘舵机0˚方向为标准，theta为角度制，0-180˚
// String Z_store = "", L_store = "", theta_store = "", STORE = "", alpha_store = "";

/*失去历史意义,仅供数据参考
//抓取340mm
int moveCatch340[12] =
    {
        45, 45, 90,
        55, 42, 60,
        75, 38, 20,
        90, 35, -15};

//320mm抓取
int moveCatch320[12] =
    {
        45, 45, 90,
        60, 53, 60,
        70, 50, 30,
        80, 70, -45};
//抓取300mm处
int moveCatch300[12] =
    {
        45, 45, 90,
        55, 50, 40,
        75, 30, -10,
        85, 60, -60};
//抓取280mm
int moveCatch280[12] =
    {
        45, 45, 90,
        55, 60, 40,
        65, 75, -10,
        70, 90, -60};
//抓取260mm
int moveCatch260[12] =
    {
        45, 45, 90,
        47, 50, 60,
        52, 60, 35,
        55, 65, 15};
//抓取240mm
int moveCatch240[12] =
    {
        45, 45, 90,
        42, 55, 70,
        38, 63, 40,
        35, 70, 25};
//抓取220mm
int moveCatch220[12] =
    {
        45, 45, 90,
        45, 50, 75,
        45, 56, 60,
        45, 62, 45};
*/
/*用机械臂长度近似舵机中心之间的距离*/
double arm_l1 = 105.14;    // 机械臂腰部到肩部的距离，单位mm(杆轴中心点到中心点，精确，下同）
double arm_l2 = 89.90;     // 机械臂的大臂长度
double arm_l3 = 169.74;    //156 数据更改 103.74 + 65 = 169.74 //机械臂的小臂长度，杆轴中心点到4号舵机轴的中心点，近似
double up_to_land = 91.80; //1号舵机中心距离地面的高度,近似
double arm_ultra = 156;    //超声波舵机中心到机械臂中心的水平距离
double ultraSelf = 32;     //超声波舵机的中心到回声中心的水平距离
bool catch_valid = false;  //判断是否可以抓取，有输入才可以启动抓取(debug模式)
void setup()
{
  //角度初始化,已调试
  MACHINE.angle[0] = {90, 90}; //底盘，控制旋转，改变X,Y
  MACHINE.angle[1] = {90, 90}; //腰部舵机，改变Y,Z
  MACHINE.angle[2] = {90, 90}; //肩部舵机，改变Y,Z
  MACHINE.angle[3] = {90, 90}; //大臂关节舵机,改变Y,Z
  MACHINE.angle[4] = {90, 90}; //小臂关节舵机,不改变X,Y,Z，仅改变俯仰角
  MACHINE.angle[5] = {0, 0};   //手部关节舵机,小幅度内改变X,Y,Z
  MACHINE.analogAngle = {
      0,
      0,
  }; //超声波控制的舵机

  //mega2560开发板舵机接口
  for (int i = 0; i < 6; i++)
  {
    MACHINE.angle[i].setServo(i + 2);
  }
  MACHINE.analogAngle.setServo(12);
  //写入角度
  for (int i = 0; i < 6; i++)
  {
    MACHINE.angle[i].angleReach();
  }
  catch_valid = false;
  Serial.begin(9600);
  delay(500);
}

void loop()
{
  // inputScheme(); //输入调试时的参数，3个分别为：Z轴方向的坐标Z,极径长度L,极角theta

  if (catch_valid)
    MACHINE.catchIt(); //不断抓取物品
}

void machineArm::sensorGetLoc() //通过传感器来获得物体的位置坐标
{

}

void machineArm::attitudeAdjust() //负反馈调整机械臂的姿态
{

}

xyzPoint machineArm::calcuLoc() //计算当前舵机机械手的位置
{
  xyzPoint point = {0, 0, 0};
  double l_compute = 0;
  l_compute = sin(angle_tmp[1] * pi / 180) * arm_l1 + sin((angle_tmp[1] + angle_tmp[2]) * pi / 180) * arm_l2 + sin((angle_tmp[1] + angle_tmp[2] + angle_tmp[3]) * pi / 180) * arm_l3;
  point.zLoc = cos(angle_tmp[1] * pi / 180) * arm_l1 + cos((angle_tmp[1] + angle_tmp[2]) * pi / 180) * arm_l2 + cos((angle_tmp[1] + angle_tmp[2] + angle_tmp[3]) * pi / 180) * arm_l3 + up_to_land;
  point.xLoc = l_compute * cos(theta * pi / 180);
  point.yLoc = l_compute * sin(theta * pi / 180);

  Serial.print("X: ");
  Serial.println(point.xLoc);
  Serial.print("Y: ");
  Serial.println(point.yLoc);
  Serial.print("Z: ");
  Serial.println(point.zLoc);
  return point;
}

//逆运动学求解
void machineArm::catchIt() //控制机械手实现抓取
{
  angle[5].changeAngle(0); //张开舵机开始抓取
  angle[5].angleReach(servo[5]);
  Y = L * sin(theta * pi / 180); //直角坐标与柱坐标之间的换算
  X = L * cos(theta * pi / 180);
  Zp = Z - up_to_land; //坐标换算
  double m = 0, n = 0; //换元
  double arm1_square = arm_l1 * arm_l1;
  double arm2_square = arm_l2 * arm_l2;
  double arc_angle1, arc_angle2; //弧度值暂存

  //理论计算结果代码化
  m = Zp - arm_l3 * cos(angle_alpha * pi / 180); // 一般大于0
  /*
  if (m > arm_l2 + arm_l1)
  {
    Serial.println("input error");
    return;
  }
  */
  n = L - arm_l3 * sin(angle_alpha * pi / 180); // 一定大于0
  double m_square = m * m;
  //简化计算
  double n_square = n * n;
  arc_angle2 = acos((m_square + n_square - arm1_square - arm2_square) / (2 * arm_l1 * arm_l2));
  if (arc_angle2 > (0.5 * pi) && arc_angle2 < pi)
  {
    arc_angle2 = pi - arc_angle2;
  }
  angle_tmp[2] = (arc_angle2 / pi) * 180; //暂存解析法求解第二个主运动臂的角度
  double angle_auxiliary = asin((m_square + n_square + arm1_square - arm2_square) / (2 * arm_l1 * sqrt(m_square + n_square)));
  if (angle_auxiliary < 0 && angle_auxiliary > -pi / 2)
  {
    angle_auxiliary = -angle_auxiliary; // 反转
  }
  arc_angle1 = angle_auxiliary - atan(m / n);
  angle_tmp[1] = arc_angle1 * 180 / pi;                     //解析法求解第一个主运动臂的角度
  angle_tmp[3] = angle_alpha - angle_tmp[1] - angle_tmp[2]; //根据运动分析求解出最后的角度,暂存
  if (angle_tmp[3] > 90)
  {
    angle_tmp[3] = 90; //防止溢出
  }
  if (angle_tmp[1] = 90) //防止溢出
  {
    angle_tmp[1] = 90;
  }
  Serial.print("The angle 1 is: ");
  Serial.println(angle_tmp[1]);
  Serial.print("The angle 2 is: ");
  Serial.println(angle_tmp[2]);
  Serial.print("The angle 3 is: ");
  Serial.println(angle_tmp[3]);
  angle[0].changeAngle(theta);
  angle[1].changeAngle(angle_tmp[1] + 90);  //实际舵机的角度，以调试
  angle[2].changeAngle(-angle_tmp[2] + 90); //注意区别
  angle[3].changeAngle(angle_tmp[3] + 90);
  for (int i = 0; i < 6; i++)
  {
    angle[i].angleReach(servo[i]);
  }
  delay(50);
}

/*
void resetBuffer()
{
  buffer = "";
}

void input()
{
  while (Serial.available() > 0)
  {
    char c = Serial.read();
    buffer += c;
    delay(2);
  }
  if (buffer.length() > 0)
  {
    Serial.println("input : " + buffer);
  }
  //buffer = "";
}
//仅仅调试
void inputScheme() //调试时使用的向Serial中输入
{
  //Serial.println("=== begin to input ===");
  double stringToDouble(const String &);
  //char *data_store;
  int index[3] = {0, 0, 0};
  int j = 0;
  input();
  if (buffer.length() > 0) //如果已经读取，且不为空
  {
    //Serial.println(STORE);
    //data_length = buffer.length() + 1;
    //data_store = new char[data_length];
    //STORE.toCharArray(data_store, data_length);
    for (int i = 0; i < buffer.length(); i++)
    {
      if (buffer[i] == ',')
      {
        index[j] = i; //记录index的位置
        j++;
      }
    }
    Z_store = buffer.substring(0, index[0]); //读取子字符串
    L_store = buffer.substring(index[0] + 1, index[1]);
    theta_store = buffer.substring(index[1] + 1, index[2]);
    alpha_store = buffer.substring(index[2] + 1, buffer.length() - 1);

    Z = stringToDouble(Z_store);
    L = stringToDouble(L_store);
    theta = stringToDouble(theta_store);
    angle_alpha = stringToDouble(alpha_store);
    Serial.print("Z is: ");
    Serial.println(Z);
    Serial.print("L is: ");
    Serial.println(L);
    Serial.print("theta is: ");
    Serial.println(theta);
    Serial.print("alpha is: ");
    Serial.println(angle_alpha);
    catch_valid = true;
  }
  else
  {
    catch_valid = false;
  }
  resetBuffer();
  //STORE = ""; //置为空，等待下一次读取
  //delete data_store;
}
//仅仅调试
double stringToDouble(const String &str) //将string转换成为字符串，调试时使用
{
  double returnValue = 0;
  int index = 0;
  int dotIndex = (int)str.length();
  for (; index < str.length(); index++)
  {
    if (str[index] == '.')
    {
      dotIndex = index;
      index++;
      break;
    }
    if (str[index] < '0' || str[index] > '9')
    {
      return 0;
    }
    returnValue = 10 * returnValue + str[index] - '0';
  }
  for (; index < str.length(); index++)
  {
    if (str[index] < '0' || str[index] > '9')
    {
      return 0;
    }
    returnValue += double(str[index] - '0') / (double)pow(10, (index - dotIndex));
  }
  return returnValue;
}
*/
//在移动到摄像头之前，更标准运动至预备位置
void machineArm::moveFirst()
{
  int move_count1 = 0;
  for (int i = 0; i < 4; i++) //三个为一组，指导机械臂前进路径
  {
    angle[1].changeAngle(byte(90 + move_first[0 + 3 * i]));
    angle[2].changeAngle(byte(90 - move_first[1 + 3 * i]));
    angle[3].changeAngle(byte(90 + move_first[2 + 3 * i]));
  }
  for (int i = 1; i < 4; i++)
  {
    angle[1].angleReach(); //使舵机达到位置
  }
}

//移动到摄像头前，判断物体的颜色
void machineArm::moveToCamera()
{
  theta = 90;
  angle[0].changeAngle(theta);
  angle[0].angleReach();
  for (int i = 0; i < 3; i++) //三个为一组，指导机械臂前进路径
  {
    angle[1].changeAngle(byte(90 + move_first[0 + 3 * i]));
    angle[2].changeAngle(byte(90 - move_first[1 + 3 * i]));
    angle[3].changeAngle(byte(90 + move_first[2 + 3 * i]));
  }
  for (int i = 1; i < 3; i++)
  {
    angle[1].angleReach(); //使舵机达到位置
  }
}

//回到初始坐标，表示一套动作完成
void machineArm::moveBack()
{
  for (int i = 0; i < 3; i++)
  {
    angle[i + 1].changeAngle(90);
  }
  for (int i = 0; i < 3; i++)
  {
    angle[i + 1].angleReach();
  }
}

//自动判别俯仰角度，分区讨论，实际数据的精简
void machineArm::alphaAuto()
{
  if (L <= 140 && LED_BUILTIN >= 120)
  {
    angle_alpha = 190;
  }
  if (L > 140 && L <= 160)
  {
    angle_alpha = 180;
  }
  if (L > 160 && L <= 165)
  {
    angle_alpha = 170 + (165 - L) * 2;
  }
  if (L > 165 && L <= 180)
  {
    angle_alpha = 170;
  }
  if (L > 180 && L <= 200)
  {
    angle_alpha = 160 + int(0.4 * (200 - L));
  }
  if (L > 200 && L <= 260)
  {
    angle_alpha = 140 + int(0.3 * (260 - L));
  }
  if (L > 260 && L <= 275)
  {
    angle_alpha = 136;
  }
  if (L > 275 && L <= 280)
  {
    angle_alpha = 120 + (280 - L) * 3;
  }
  if (L > 280 && L <= 300)
  {
    angle_alpha = 90 + 1.5 * (300 - L);
  }
  if (L > 300)
  {
    angle_alpha = 90;
  }
}