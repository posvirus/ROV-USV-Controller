/*库声明段*/
#include <ArduinoQueue.h>
#include <Streaming.h>
#include <string.h>
#include <FlexiTimer2.h >
#include <math.h>
/*引脚声明段*/
#define l_f_m 3//左前推进器
#define r_f_m 4//右前推进器
#define l_b_m 5//左后推进器
#define r_b_m 6//右后推进器
/*重启函数声明*/
void(* resetFunc) (void) = 0;
/*函数声明段*/
void initialize();
void location_update();
void EVENT_motivate();
void feedback_init();
void clearqueue();
double ERR(char err[]);
/*系统参数声明段*/
#define INIT 187//初始化的PWM值
int PUSH = 200; //向前时艉部推进器的PWM值
int TURN_LBM = 160;
int TURN_RBM = 200;//转向时艉部推进器的PWM值
#define MAX_ANGLE_ERR 15//转向的最大误差角（度为单位）
#define REFRESH_TIME 5000
#define PAI 3.141592654
struct data//数据结构体
{
  char longitude[20];//经度
  char latitude[20];//纬度
  float anglez;//航向角，航向角的取值规则如下，以正北为0度
  void cleardata()//初始化函数
  {
    memset(longitude, '0', sizeof(longitude));
    memset(latitude, '0', sizeof(latitude));
    anglez = 0.0;
  }
  double X()//经度数值化函数
  {
    double temp;
    temp = (longitude[0] - '0') * 100.0 + (longitude[1] - '0') * 10.0 + (longitude[2] - '0') * 1.0 + ((longitude[3] - '0') * 10.0 + (longitude[4] - '0') * 1.0) / 60.0;
    temp += ((longitude[6] - '0') * 0.1 + (longitude[7] - '0') * 0.01 + (longitude[8] - '0') * 0.001 + (longitude[9] - '0') * 0.0001 + (longitude[10] - '0') * 1e-5) / 60.0;
    temp += ((longitude[11] - '0') * 1e-6 + (longitude[12] - '0') * 1e-7 + (longitude[13] - '0') * 1e-8 + (longitude[14] - '0') * 1e-9 + (longitude[15] - '0') * 1e-10) / 60.0;
    temp += ((longitude[16] - '0') * 1e-11 + (longitude[17] - '0') * 1e-12 + (longitude[18] - '0') * 1e-13 + (longitude[19] - '0') * 1e-14) / 60.0;
    return temp;
  }
  double Y()//纬度数值化函数
  {
    double temp;
    temp = (latitude[0] - '0') * 10.0 + (latitude[1] - '0') * 1.0 + ((latitude[2] - '0') * 10.0 + (latitude[3] - '0') * 1.0) / 60.0;
    temp += ((latitude[5] - '0') * 0.1 + (latitude[6] - '0') * 0.01 + (latitude[7] - '0') * 0.001 + (latitude[8] - '0') * 0.0001 + (latitude[9] - '0') * 1e-5) / 60.0;
    temp += ((latitude[10] - '0') * 1e-6 + (latitude[11] - '0') * 1e-7 + (latitude[12] - '0') * 1e-8 + (latitude[13] - '0') * 1e-9 + (latitude[14] - '0') * 1e-10) / 60.0;
    temp += ((latitude[15] - '0') * 1e-11 + (latitude[16] - '0') * 1e-12 + (latitude[17] - '0') * 1e-13 + (latitude[18] - '0') * 1e-14 + (latitude[19] - '0') * 1e-15) / 60.0;
    return temp;
  }
};
ArduinoQueue<data> orderqueue(20);//存储路径结点的队列，最多储存20个结点
data temp, present_target, present_location;//temp为中间变量，之后的2个变量分别存储当前位置、当前目标位置
char module;
int v_o_m[4];//4个推进器的PWM数值
int cnt1, cnt2, MAX_cnt;
int size_of_node;//总结点数
int result;//测试用变量
char feedback[81];//返回上位机数据
char err[5];
char depth[8];
boolean working_flag;
//char data_all[35];


void setup() {
  Serial1.begin(9600);//Serial1用于接收GPS的经纬度数据（这部分测试过，是正常的，不用管）
  Serial2.begin(9600);//Serial2用于接收姿态传感器的数据（这部分测试过，是正常的，不用管）
  Serial3.begin(9600);//Serial3用于与上位机通信
  Serial.begin(9600);//测试用，不用管
  pinMode (l_f_m, OUTPUT);
  pinMode (r_f_m, OUTPUT);
  pinMode (l_b_m, OUTPUT);
  pinMode (r_b_m, OUTPUT);//4个推进器，从上到下分别为左前、右前、左后、右后
  pinMode (8, INPUT);
  pinMode (9, INPUT);
  pinMode (10, INPUT);
  pinMode (11, INPUT);//用于接收遥控器的4路PWM信号
  initialize();

NODE0://这是goto语法的一个标志
  if (Serial3.available())//如果Serial3有可读取的数据
  {
    //Serial << "Processing...Please wait." << endl;
    while (1)//开始循环读取
    {
      if (Serial3.peek() == 'P')//如果读到数据头'P'（路径结点的数据头）
      {
        result = Serial3.read();//将'P'读掉
        result = Serial3.readBytes(temp.longitude, 20);
        result = Serial3.readBytes(temp.latitude, 20);//读取经纬度数据，存入temp中
        orderqueue.enqueue(temp);//将temp入队，作为一个路径结点
        //Serial3 << 'P' << temp.longitude << endl;
        //Serial3 << temp.latitude << endl;
        //Serial3 << "there are " << orderqueue.itemCount() << " node in this path" << endl;//向上位机返回总共的路径结点数
      }
      else if (Serial3.peek() == 'r')//如果读到数据头'r'（误差值的数据头）
      {
        result = Serial3.read();//将'r'读掉
        result = Serial3.readBytes(err, 5);//读取误差值
        //Serial3 << "rGET" << endl;
      }
      else if (Serial3.peek() == 'M')
      {
        char temp_data[9];
        memset(temp_data, '\0', sizeof(temp_data));
        result = Serial3.read();
        result = Serial3.readBytes(temp_data, 9);
        Serial3 << temp_data << endl;
        PUSH = (temp_data[0] - '0') * 100 + (temp_data[1] - '0') * 10 + (temp_data[2] - '0');
        TURN_LBM = (temp_data[3] - '0') * 100 + (temp_data[4] - '0') * 10 + (temp_data[5] - '0');
        TURN_RBM = (temp_data[6] - '0') * 100 + (temp_data[7] - '0') * 10 + (temp_data[8] - '0');
      }
      else if (Serial3.peek() == 'Q')
      {
        Serial3.read();
        goto NODE1;
      }
      else if (Serial3.available())
      {
        result = Serial3.read();//如果有其他可读取的数据，应该是不符规范的数据，直接读掉
      }
      delay(10);
    }
  }
  else
  {
    goto NODE0;//到NODE0再执行
  }

NODE1:
  Serial3 << 'M' << PUSH << ' ' << TURN_LBM << ' ' << TURN_RBM << endl;
  Serial3.println(ERR(err), 15);//向上位机返回数值化后的误差值，数值化后应该为角度值
  temp.cleardata();
  Serial3 << "there are " << orderqueue.itemCount() << " node in this path" << endl;//向上位机返回总共的路径结点数
  size_of_node = orderqueue.itemCount();//记录总结点数
NODEX:
  if (Serial3.available())//开始读取无人艇的模式
  {
    if (Serial3.peek() == 'R')//读到模式的数据头'R'
    {
      result = Serial3.read();//将'R'读掉
      //Serial<<'1'<<endl;
      delay(10);
      if (Serial3.peek() == '1' || Serial3.peek() == '3' || Serial3.peek() == '2')
      {
        module = Serial3.read();//读取模式并赋给module
      }
      else
      {
        goto NODEX;
      }
    }
    else
    {
      result = Serial3.read();//如果读到其他数据，不符规范，直接读掉
      goto NODEX;
    }
  }
  else
  {
    goto NODEX;
  }
}

void initialize()
{
  module = '0';

  for (int i = 2; i < 4; i++) v_o_m[i] = 190;
  v_o_m[0] = 187;
  v_o_m[1] = 190;//一些初始化，不用管

  feedback_init();

  temp.cleardata();
  present_target.cleardata();
  present_location.cleardata();

  clearqueue();

  cnt1 = 0;
  cnt2 = 0;
  MAX_cnt = 5000;
  working_flag = false;

  memset(depth, '0', sizeof(depth));
  analogWrite(l_f_m, v_o_m[0]);
  analogWrite(r_f_m, v_o_m[1]);
  analogWrite(l_b_m, v_o_m[2]);
  analogWrite(r_b_m, v_o_m[3]);

  while (Serial3.available()) {
    Serial3.read();
  }
  Serial3 << "Serial3 is clear." << endl;

}

void clearqueue()
{
  while (!orderqueue.isEmpty()) orderqueue.dequeue();
  return;
}

void feedback_init()
{
  feedback[0] = '$';
  feedback[1] = 'T';
  feedback[2] = '$';
  for (int i = 3; i <= 4; i++)feedback[i] = '0';
  feedback[5] = '.';
  feedback[6] = '0';
  feedback[7] = '0';
  feedback[8] = ';';
  for (int i = 9; i <= 16; i++)feedback[i] = '0';
  feedback[17] = ';';
  for (int i = 18; i <= 20; i++)feedback[i] = '0';
  feedback[21] = '.';
  feedback[22] = '0';
  feedback[23] = '0';
  feedback[24] = ';';
  for (int i = 25; i <= 32; i++)feedback[i] = '0';
  feedback[33] = ';';
  for (int i = 34; i <= 39; i++)feedback[i] = '0';
  feedback[40] = ';';
  for (int i = 41; i <= 45; i++)feedback[i] = '0';
  feedback[46] = ';';
  for (int i = 47; i <= 52; i++)feedback[i] = '0';
  feedback[53] = ';';
  for (int i = 54; i <= 59; i++)feedback[i] = '0';
  feedback[60] = ';';
  for (int i = 61; i <= 66; i++)feedback[i] = '0';
  feedback[67] = ';';
  for (int i = 68; i <= 72; i++)feedback[i] = '0';
  feedback[73] = ';';
  for (int i = 74; i <= 76; i++)feedback[i] = '0';
  feedback[77] = ';';
  feedback[78] = 'e';
  feedback[79] = 'n';
  feedback[80] = 'd';
}

double ERR(char err[])
{
  double temp;
  temp = ((err[0] - '0') * 10.0 + (err[1] - '0') * 1.0 + (err[3] - '0') * 0.1 + (err[4] - '0') * 0.01) / 111110.0;
  return temp;
}

void EVENT_motivate()
{
  cnt1++;
  if (Serial3.peek() == 'R')
  {
    Serial3.read();
    if (Serial3.peek() == '1' || Serial3.peek() == '2' || Serial3.peek() == '3')
    {
      module = Serial3.read();
    }
    else
    {
      Serial3.read();
    }
  }
  else
  {
    Serial3.read();
  }
  if (cnt1 >= MAX_cnt)
  {
    cnt1 = 0;
    feedback[3] = present_location.latitude[0];
    feedback[4] = present_location.latitude[1];
    for (int i = 9; i <= 16; i++)feedback[i] = present_location.latitude[i - 7];
    feedback[18] = present_location.longitude[0];
    feedback[19] = present_location.longitude[1];
    feedback[20] = present_location.longitude[2];
    for (int i = 25; i <= 32; i++)feedback[i] = present_location.longitude[i - 22];
    for (int i = 68; i <= 72; i++)feedback[i] = depth[i - 68];
    for (int i = 74; i <= 76; i++)feedback[i] = depth[i - 69];
    //Serial3.println(feedback);
    for (int i = 0; i <= 80; i++)Serial3 << feedback[i];
    Serial3 << endl;
    //Serial3 << v_o_m[0] << v_o_m[1] << v_o_m[2] << v_o_m[3] << endl;
  }
  if (module == '1')
  {
    MAX_cnt = 5000;
    if (!working_flag)//如果当前没有需要到达的结点
    {
      present_target = orderqueue.dequeue();//从队列中提取当前的目标结点
      //Serial3 << "No." << size_of_node - orderqueue.itemCount() << endl;//向上位机反馈这是第几个结点
      working_flag = true;//代表当前有需要到达的结点，处于循迹状态
      if (present_target.X() < present_location.X() && present_target.Y() > present_location.Y())
      {
        //present_target.anglez = 180.0 + atan((present_target.X() - present_location.X()) / (present_target.Y() - present_location.Y())) / PAI * 180.0;
        //present_target.anglez = -present_target.anglez;
        present_target.anglez = atan((present_location.X() - present_target.X()) / (present_target.Y() - present_location.Y())) / PAI * 180.0;
      }
      else if (present_target.X() < present_location.X() && present_target.Y() < present_location.Y())
      {
        present_target.anglez = 180.0 - atan((present_target.X() - present_location.X()) / (present_target.Y() - present_location.Y())) / PAI * 180.0;
        //present_target.anglez = -present_target.anglez;
      }
      else if (present_target.X() > present_location.X() && present_target.Y() > present_location.Y())
      {
        present_target.anglez = -atan((present_target.X() - present_location.X()) / (present_target.Y() - present_location.Y())) / PAI * 180.0;
        //present_target.anglez = -present_target.anglez;
      }
      else
      {
        present_target.anglez = -180.0 - atan((present_target.X() - present_location.X()) / (present_target.Y() - present_location.Y())) / PAI * 180.0;
      }
      //Serial3 << present_target.anglez << endl;//输出，方便校验
    }//角度换算
    cnt2++;//计数器
    if (cnt2 == REFRESH_TIME)//动态调整航行轨迹
    {
      cnt2 = 0;
      if (present_target.X() < present_location.X() && present_target.Y() > present_location.Y())
      {
        //present_target.anglez = 180.0 + atan((present_target.X() - present_location.X()) / (present_target.Y() - present_location.Y())) / PAI * 180.0;
        //present_target.anglez = -present_target.anglez;
        present_target.anglez = atan((present_location.X() - present_target.X()) / (present_target.Y() - present_location.Y())) / PAI * 180.0;
      }
      else if (present_target.X() < present_location.X() && present_target.Y() < present_location.Y())
      {
        present_target.anglez = 180.0 - atan((present_target.X() - present_location.X()) / (present_target.Y() - present_location.Y())) / PAI * 180.0;
        //present_target.anglez = -present_target.anglez;
      }
      else if (present_target.X() > present_location.X() && present_target.Y() > present_location.Y())
      {
        present_target.anglez = -atan((present_target.X() - present_location.X()) / (present_target.Y() - present_location.Y())) / PAI * 180.0;
        //present_target.anglez = -present_target.anglez;
      }
      else
      {
        present_target.anglez = -180.0 - atan((present_target.X() - present_location.X()) / (present_target.Y() - present_location.Y())) / PAI * 180.0;
      }
      //Serial3 << present_target.anglez << "  " << present_location.anglez << endl;
      if (v_o_m[2] == TURN_RBM && v_o_m[3] == TURN_LBM)
      {
        //Serial3 << "+" << endl;
      }
      else
      {
        //Serial3 << "-" << endl;
      }//输出当前是在顺时针转还是逆时针转，方便校验
      //Serial3 << "Current location: ";
      //Serial3.print(present_location.X(), 15);
      //Serial3.println(present_location.Y(), 15);
      //Serial3 << "Target location: ";
      //Serial3.print(present_target.X(), 15);
      //Serial3.println(present_target.Y(), 15);
    }//输出当前位置与当前目标位置，方便校验
    if (fabs(present_target.anglez - present_location.anglez) >= MAX_ANGLE_ERR && (360.0 - fabs(present_target.anglez - present_location.anglez)) >= MAX_ANGLE_ERR)
    { //如果当前航向角与目标航向角的差值大于MAX_ANGLE_ERR，先进行转向
      //Serial3<<"GET"<<endl;
      if (present_location.anglez > present_target.anglez && present_location.anglez - present_target.anglez < 180.0)//顺时针
      {
        v_o_m[2] = TURN_RBM;
        v_o_m[3] = TURN_LBM;
      }
      else if (present_location.anglez > present_target.anglez && present_location.anglez - present_target.anglez >= 180.0) //逆时针
      {
        v_o_m[2] = TURN_LBM;
        v_o_m[3] = TURN_RBM;
      }
      else if (present_location.anglez <= present_target.anglez && present_target.anglez - present_location.anglez >= 180.0) //顺时针
      {
        v_o_m[2] = TURN_RBM;
        v_o_m[3] = TURN_LBM;
      }
      else if (present_location.anglez <= present_target.anglez && present_target.anglez - present_location.anglez < 180.0) //逆时针
      {
        v_o_m[2] = TURN_LBM;
        v_o_m[3] = TURN_RBM;
      }
    }
    else if (fabs(present_target.X() - present_location.X()) >= ERR(err) || fabs(present_target.Y() - present_location.Y()) >= ERR(err))
    { //当转向完成，即航向角的误差落在MAX_ANGLE_ERR内，开始向前驱动
      //Serial3 << "GET" << endl;
      v_o_m[2] = PUSH;
      v_o_m[3] = PUSH;
    }
    else//直到到达目标结点
    {
      v_o_m[2] = INIT;
      v_o_m[3] = INIT;//停止驱动
      working_flag = false;//此时没有需要到达的结点
    }
  }
  if (module == '2')
  {
    //Serial3 << 1 << endl;
    MAX_cnt = 50;
    v_o_m[0] = map(pulseIn(11, HIGH), 1000, 1900, 130, 255);
    v_o_m[1] = map(pulseIn(10, HIGH), 1100, 2000, 130, 255);
    v_o_m[2] = map(pulseIn(9, HIGH), 1100, 2000, 130, 255);
    v_o_m[3] = map(pulseIn(8, HIGH), 1200, 2000, 130, 255);
  }
  if (module == '3')
  {
    MAX_cnt = 5000;
    for (int i = 0; i < 4; i++) v_o_m[i] = 187;
  }
  analogWrite(l_f_m, v_o_m[0]);
  analogWrite(r_f_m, v_o_m[1]);
  analogWrite(l_b_m, v_o_m[2]);
  analogWrite(r_b_m, v_o_m[3]);
}

void location_update()
{
  unsigned char Re_buf[11];
  memset(Re_buf, '\0', sizeof(Re_buf));
  /*
    if (cnt1 == 4900)
    {
     //Serial3 << present_location.longitude << endl;
     //Serial3 << present_location.latitude << endl;
     //Serial3 << depth << endl;
     //for (int i = 0; i < 8; i++)Serial3 << depth[i];
     //Serial3 << endl;
     //Serial3<<data_all<<endl;
    }
  */
  if (Serial1.peek() == 'X')
  {
    Serial1.read();
    //result = Serial1.readBytes(data_all, 35);
    result = Serial1.readBytes(present_location.longitude, 14);
    result = Serial1.readBytes(present_location.latitude, 13);
    result = Serial1.readBytes(depth, 8);
    //for (int i = 0; i < 35; i++) Serial3.print(data_all[i]);

    //Serial3<<present_location.longitude<<' '<<present_location.latitude<<endl;
  }
  else
  {
    result = Serial1.read();
  }
  if (Serial2.peek() == 0x55)
  {
    Serial2.readBytes(Re_buf, 11);
  }
  else
  {
    Serial2.read();
  }
  switch (Re_buf [1])
  {
    case 0x53:
      present_location.anglez = (short(Re_buf [7] << 8 | Re_buf [6])) / 32768.0 * 180;
      break;
    default:
      present_location.anglez = present_location.anglez;
      break;
  }
  return;
}

void loop() {
  location_update();
  EVENT_motivate();
  if (orderqueue.isEmpty() && !working_flag)//如果储存结点的队列空了，且当前也没有需要到达的结点，则说明本次循迹已完成
  {
    for (int i = 0; i < 4; i++) v_o_m[i] = 187;
    Serial3 << "Success!" << endl;
    Serial3.println("System Resetting...");
    delay(3000);
    resetFunc();//返回成功信号，并重启系统（无需关系重启系统的具体实现，经测试该函数确实能够重启系统）
  }
  if (module == '3')//如果切换为3模式（R3），直接停机重启
  {
    Serial3.println("System Resetting...");
    delay(3000);
    resetFunc();
  }
}
