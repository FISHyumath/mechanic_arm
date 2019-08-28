bool catch_valid = false;
double X, Y, Z, L; //以腰部舵机中心建立坐标系，直角坐标系与柱坐标系相互转换
byte theta;     //底盘的极角，极轴以底盘舵机90˚方向，theta为弧度制
byte angle_alpha;
//X = L * cos(theta); Y = L * sin(theta);
String Z_store = "", L_store = "", theta_store = "", STORE = "", alpha_store = "";

void setup()
{
  Serial.begin(9600);
  Serial.println("********Serial Start*********");
}

void loop()
{
  input_scheme();
}

void input_scheme() //调试时使用的向Serial中输入
{
  double stringToDouble(const String &);
  char data; //字符型数据
  char *data_store;
  int data_length, index[3], j = 0;
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
    theta_store = STORE.substring(index[1] + 1, index[2]);
    alpha_store = STORE.substring(index[2] + 1, data_length - 2);

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
  STORE = ""; //置为空，等待下一次读取
  Z_store = "";
  L_store = "";
  theta_store = "";
  alpha_store = "";
  delete data_store;
}

double stringToDouble(const String &str)
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
        if(str[index]<'0'||str[index]>'9')
        {
            return 0;
        }
        returnValue = 10 * returnValue + str[index] - '0';
    }
    for (; index < str.length(); index++)
    {
        if(str[index]<'0'||str[index]>'9')
        {
            return 0;
        }
        returnValue += double(str[index] - '0') / (double)pow(10, (index - dotIndex));
    }
    return returnValue;
}
