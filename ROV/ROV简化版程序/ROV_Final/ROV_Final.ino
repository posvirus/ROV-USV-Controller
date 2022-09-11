#include <SPI.h>
#include <Ethernet.h>
#include <Streaming.h>

#define l_f_m 2//左前推进器
#define r_f_m 3//右前推进器
#define l_b_m 8//左后推进器
#define r_b_m 5//右后推进器
#define m_b_m 6//后部中央推进器
#define light 7//探照灯
#define arm 9//机械臂

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 177);
IPAddress myDns(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);

EthernetServer server(23);
EthernetClient client;
boolean alreadyConnected = false;//判断client是否初次进行连接

int v_o_m[7];//从v_o_m[0:6]分别为：l_f_m,r_f_m,l_b_m,r_b_m,m_b_m,light,arm

char message[42];
char MOD1_order[21];
boolean isStart;
int cnt;
int state[3];
unsigned char Re_buf[11];
float a[3], w[3], angle[3];
char GET[6];

void data_processor();
void F2C_a(char temp[], float x);
void F2C_w(char temp[], float x);
/*字符-整型转换函数*/
int char2int(char MSB, char MB, char LSB)
{
  return ((MSB - '0') * 100 + (MB - '0') * 10 + (LSB - '0'));
}

void setup() {
  Ethernet.begin(mac, ip, myDns, gateway, subnet);//配置Ethernet的MAC地址、IP地址、DNS地址、网关、子网掩码
  Serial.begin(9600);
  Serial1.begin(9600);//姿态传感器
  Serial2.begin(115200);//深度计
  pinMode (l_f_m, OUTPUT);
  pinMode (r_f_m, OUTPUT);
  pinMode (l_b_m, OUTPUT);
  pinMode (r_b_m, OUTPUT);
  pinMode (m_b_m, OUTPUT);
  pinMode (light, OUTPUT);
  pinMode (arm, OUTPUT);
  for (int i = 0; i < 7; i++)
  {
    v_o_m[i] = 187;
  }
  memset(Re_buf, 0, sizeof(Re_buf));
  memset(message, '0', sizeof(message));
  memset(MOD1_order, '0', sizeof(MOD1_order));
  message[0] = 'D';
  message[41] = 'E';
  state[0] = 0;
  state[1] = 0;
  state[2] = 0;
  cnt = 0;
  isStart = false;
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1);
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }
  server.begin();

  Serial.print("server address:");
  Serial.println(Ethernet.localIP());
  client = server.available();
  while (!client)
  {
    client = server.available();
  }
  analogWrite(l_f_m, v_o_m[0]);
  analogWrite(r_f_m, v_o_m[1]);
  analogWrite(l_b_m, v_o_m[2]);
  analogWrite(r_b_m, v_o_m[3]);
  analogWrite(m_b_m, v_o_m[4]);
  analogWrite(light, v_o_m[5]);
  analogWrite(arm, v_o_m[6]);
  Serial << "Client get!" << endl;
}

void data_processor()
{
  if (Serial1.peek() == 0x55)
  {
    Serial1.readBytes(Re_buf, 11);
  }
  else
  {
    Serial1.read();
  }
  switch (Re_buf [1])
  {
    case 0x51:
      a[0] = (short(Re_buf [3] << 8 | Re_buf [2])) / 32768.0 * 16;
      a[1] = (short(Re_buf [5] << 8 | Re_buf [4])) / 32768.0 * 16;
      a[2] = (short(Re_buf [7] << 8 | Re_buf [6])) / 32768.0 * 16;
      F2C_a(GET, a[0]);
      for (int i = 19; i <= 24; i++)message[i] = GET[i - 19];
      F2C_a(GET, a[1]);
      for (int i = 25; i <= 30; i++)message[i] = GET[i - 25];
      F2C_a(GET, a[2]);
      for (int i = 31; i <= 36; i++)message[i] = GET[i - 31];
      state[0] = 1;
      break;
    case 0x53:
      angle[0] = (short(Re_buf [3] << 8 | Re_buf [2])) / 32768.0 * 180;
      angle[1] = (short(Re_buf [5] << 8 | Re_buf [4])) / 32768.0 * 180;
      angle[2] = (short(Re_buf [7] << 8 | Re_buf [6])) / 32768.0 * 180;
      F2C_w(GET, angle[0]);
      for (int i = 1; i <= 6; i++)message[i] = GET[i - 1];
      F2C_w(GET, angle[1]);
      for (int i = 7; i <= 12; i++)message[i] = GET[i - 7];
      F2C_w(GET, angle[2]);
      for (int i = 13; i <= 18; i++)message[i] = GET[i - 13];
      state[1] = 1;
      break;
  }
  if (Serial2.peek() == 'D')
  {
    Serial2.read();
    Serial2.read();
    message[37] = Serial2.read();
    message[38] = Serial2.read();
    Serial2.read();
    message[39] = Serial2.read();
    message[40] = Serial2.read();
    state[2] = 1;
  }
  else
  {
    Serial2.read();
  }
  if (state[1] == 1 && state[0] == 1 && state[2] == 1)
  {
    state[0] = 0;
    state[1] = 0;
    state[2] = 0;
    server.println(message);
  }
}

/*浮点数转换函数*/
void F2C_w(char temp[], float x)
{
  temp[0] = (x >= 0) ? '+' : '-';
  x = fabs(x);
  temp[1] = (int)(x / 100) + 48;
  temp[2] = (int)((x - (temp[1] - '0') * 100) / 10) + 48;
  temp[3] = (int)(x - (temp[1] - '0') * 100 - (temp[2] - '0') * 10) + 48;
  temp[4] = ((int)(x * 10)) % 10 + 48;
  temp[5] = ((int)(x * 100)) % 10 + 48;
}

void F2C_a(char temp[], float x)
{
  temp[0] = (x >= 0) ? '+' : '-';
  x = fabs(x);
  temp[1] = ((int)(x)) % 10 + 48;
  temp[2] = ((int)(x * 10)) % 10 + 48;
  temp[3] = ((int)(x * 100)) % 10 + 48;
  temp[4] = ((int)(x * 1000)) % 10 + 48;
  temp[5] = ((int)(x * 10000)) % 10 + 48;
}

void loop() {
  if (client) //如果存在客户端对象
  {
    if (!alreadyConnected) //如果是初次接入
    {
      client.flush();
      client.println("Welcome!");
      alreadyConnected = true;
    }
    data_processor();
    if (client.available() > 0)
    {
      char temp;
      temp = client.read();
      if (temp == 'X')
      {
        isStart = true;
      }
      if (isStart && temp != 'X')
      {
        MOD1_order[cnt] = temp;
        cnt++;
      }
      if (cnt == 21)
      {
        cnt = 0;
        isStart = false;
        for (int i = 0; i < 21; i = i + 3)
        {
          v_o_m[i / 3] = char2int(MOD1_order[i], MOD1_order[i + 1], MOD1_order[i + 2]);
        }
        Serial << v_o_m[0] << ' ' << v_o_m[1] << ' ' << v_o_m[2] << ' ' << v_o_m[3] << ' ' << v_o_m[4] << ' ' << v_o_m[5] << ' ' << v_o_m[6] << endl;
      }
    }
  }
  analogWrite(l_f_m, v_o_m[0]);
  analogWrite(r_f_m, v_o_m[1]);
  analogWrite(l_b_m, v_o_m[2]);
  analogWrite(r_b_m, v_o_m[3]);
  analogWrite(m_b_m, v_o_m[4]);
  analogWrite(light, v_o_m[5]);
  analogWrite(arm, v_o_m[6]);
}
