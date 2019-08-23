#include <Servo.h>
const int pi = 3.1415926;

class Angle
{
public:
  Angle(byte a_m, byte a_n) : angMemory(a_m), angNow(a_n) {}
  Angle();
  void changeAngle(byte ang);
  void angleReach(Servo &s);
  byte angNow;

private:
  byte angMemory;
};
void Angle::angleReach(Servo &s) //快速跳转脱机角度可能发生堵转，这个舵机需要缓慢调整才行
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
      s.write(angleTmp);
      delay(15);
    }
  }
  else
  {
    while (angleTmp > angNow)
    {
      angleTmp--;
      s.write(angleTmp);
      delay(15);
    }
  }
}
Angle::Angle()
{
  
}
void Angle::changeAngle(byte ang)
{
  angMemory = angNow;
  angNow = ang;
}

Angle angle[6];    //舵机角度，与舵机名称一一对应,角度群部位相对轴的角度
Servo servo[6];    //舵机名称，分别为0，1，2，3，4，5号，全代码统一
double X = 0, Y = 0, Z = 0, L = 0; //以腰部舵机中心建立坐标系，直角坐标系与柱坐标系相互转换
double Xp = 0, Yp = 0, Zp = 0; //这是4号舵机转轴中心处的坐标值，运动学分析的核心数值
//X,Y,Z代表物体体心坐标，假设抓取足够准确，使体心坐标与机械手位于统一坐标平面之内
byte angle_alpha; //机械手相对竖直方向z轴的俯仰角，逆时针方向，0-180,根据物体状态判别输入
byte theta;       //底盘的极角，极轴以底盘舵机0˚方向为标准，theta为角度制，0-180˚
//X = L * cos(theta); Y = L * sin(theta)
String Z_store = "", L_store = "", theta_store = "", STORE = "";

/*用机械臂长度近似舵机中心之间的距离*/
double arm_l1 = 105.14;    // 机械臂腰部到肩部的距离，单位mm(杆轴中心点到中心点，精确，下同）
double arm_l2 = 89.90;     // 机械臂的大臂长度
double arm_l3 = 56.48;     //机械臂的小臂长度，杆轴中心点到4号舵机轴的中心点，近似
double hand_lenth = 80.50; //机械手的长度，4号舵机轴的额中心点到机械手伸直的中心抓取点，近似
double up_to_land = 91.80; //1号舵机中心距离地面的高度,近似
bool catch_valid = false;  //判断是否可以抓取，有输入才可以启动抓取（debug模式）

void setup()
{
  //mega2560开发板串口
  servo[0].attach(2,500,2500); //底盘，控制旋转，改变X,Y
  servo[1].attach(3,500,2500); //腰部舵机，改变Y,Z
  servo[2].attach(4,500,2500); //肩部舵机，改变Y,Z
  servo[3].attach(5,500,2500); //大臂关节舵机,改变Y,Z
  servo[4].attach(6,500,2500); //小臂关节舵机,不改变X,Y,Z，仅改变俯仰角
  servo[5].attach(7,500,2500); //手部关节舵机,小幅度内改变X,Y,Z
  //角度初始化,已调试
  angle[0] = {90, 90};
  angle[1] = {90, 90};
  angle[2] = {90, 90};
  angle[3] = {90, 90};
  angle[4] = {90, 90};
  angle[5] = {0, 0};
  //写入角度
  for (int i = 0; i < 6; i++)
  {
    angle[i].angleReach(servo[i]);
  }
  catch_valid = false;
  Serial.begin(9600);
  delay(500);
}

void loop()
{
  input_scheme(); //输入调试时的参数，3个分别为：Z轴方向的坐标Z,极径长度L,极角theta
  if (catch_valid)
    catchit(); //不断抓取物品
}

//逆运动学求解
void catchit() //控制机械手实现抓取
{
  Y = L * sin(theta * pi / 180); //直角坐标与柱坐标之间的换算
  X = L * cos(theta * pi / 180);
  Xp = X - hand_lenth * sin(angle_alpha * pi / 180);
  Yp = Y - hand_lenth * cos(angle_alpha * pi / 180);
  Zp = Z - up_to_land - hand_lenth * sin(angle_alpha * pi / 180); //减去相对高度差
  float m, n;                                                     //换元
  float m_square = m * m;                                         //简化计算
  float n_square = n * n;
  float arm1_square = arm_l1 * arm_l1;
  float arm2_square = arm_l2 * arm_l2;
  float arc_angle1, arc_angle2; //弧度值暂存

  //理论计算结果代码化
  m = Zp - arm_l3 * cos(angle_alpha); // 一般大于0
  n = Yp - arm_l3 * sin(angle_alpha); // 一定大于0
  arc_angle2 = acos(m_square + n_square - arm1_square - arm2_square) / (2 * arm_l1 * arm_l2);
  angle[2].changeAngle((arc_angle2 / pi) * 180); // 解析法求解第二个主运动臂的角度
  arc_angle1 = asin((m * m + n * n + arm2_square - arm1_square) / (2 * arm_l2)) - atan(m / n) - arc_angle2;
  angle[1].changeAngle(arc_angle1 * 180 / pi);                           //解析法求解第一个主运动臂的角度
  angle[3].changeAngle(angle_alpha - angle[1].angNow - angle[2].angNow); //根据运动分析求解出最后的角度
  Serial.print("The angle 1 is: ");
  Serial.println(angle[1].angNow);
  Serial.print("The angle 2 is: ");
  Serial.println(angle[2].angNow);
  Serial.print("The angle 3 is: ");
  Serial.println(angle[3].angNow);
  for (int i = 0; i < 6; i++)
  {
    angle[i].angleReach(servo[i]);
  }
  delay(50);
}

void input_scheme() //调试时使用的向Serial中输入
{
  double stringToDouble(const String &);
  char data; //字符型数据
  char *data_store;
  int data_length, index[2], j = 0;
  while (Serial.available() > 0) //读取字符串
  {
    data = Serial.read(); //读取数据
    STORE += data;        //拼接衬成为字符串
    delay(2);             //延时，等待传输
  }
  if (STORE.length() > 0) //如果已经读取，且不为空
  {
    Serial.println(STORE);
    data_length = STORE.length() + 1;
    data_store = new char[data_length];
    STORE.toCharArray(data_store, data_length);
    for (int i = 0; i < data_length; i++)
    {
      if (data_store[i] == ',')
      {
        index[j] = i; //记录index的位置
        j++;
      }
    }
    Z_store = STORE.substring(0, index[0]); //读取子字符串
    L_store = STORE.substring(index[0] + 1, index[1]);
    theta_store = STORE.substring(index[1] + 1, data_length - 2);

    Z = stringToDouble(Z_store);
    L = stringToDouble(L_store);
    theta = stringToDouble(theta_store);
    Serial.print("Z is: ");
    Serial.println(Z);
    Serial.print("L is: ");
    Serial.println(L);
    Serial.print("theta is: ");
    Serial.println(theta);
    catch_valid = true;
  }
  else
  {
    catch_valid = false;
  }
  STORE = ""; //置为空，等待下一次读取
  
}

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
