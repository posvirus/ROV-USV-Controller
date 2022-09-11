#include "ping1d.h"
uint32_t temp1, temp2;
char temp5[5], temp3[3];
char sign[5];
char X[14];
char Y[13];
char* code;
Ping1D myPing {Serial1};
void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
  Serial1.begin(115200);
  Serial2.begin(460800);
  //code = "X12345.678910111234.5678910199999000";//第2象限
  //code = "X12545.678910113434.5678910199999000";//第3象限
  code = "X11945.678910112934.5678910199999000";//第1象限
  //code = "X11945.678910113534.5678910199999000";//第4象限
  //code = "X12126.264190143101.7168712899999000";
  memset(temp5, 0, sizeof(temp5));
  memset(temp3, 0, sizeof(temp3));

}

void loop() {
  // put your main code here, to run repeatedly:
  //code[35] = code[35] + 1 ;
  if (Serial2.read() == '$')
  {
    Serial2.readBytes(sign, 5);
    if (sign[2] == 'G' || sign[3] == 'G' || sign[4] == 'A')
    {
      for (int i = 0; i < 11; i++)Serial2.read();
      Serial2.readBytes(Y, 13);
      for (int i = 0; i < 3; i++)Serial2.read();
      Serial2.readBytes(X, 14);
      while (!myPing.update()) {
        Serial.println("Ping device update failed");
      }
      temp1 = myPing.distance();
      temp2 = myPing.confidence();
      temp5[4] = temp1 % 10 + '0';
      temp5[3] = (temp1 / 10) % 10 + '0';
      temp5[2] = (temp1 / 100) % 10 + '0';
      temp5[1] = (temp1 / 1000) % 10 + '0';
      temp5[0] = (temp1 / 10000) % 10 + '0';
      temp3[2] = temp2 % 10 + '0';
      temp3[1] = (temp2 / 10) % 10 + '0';
      temp3[0] = (temp2 / 100) % 10 + '0';
      //Serial.println(temp1);
      //Serial.println(temp2);
      strncpy(code + 1, X, 14);
      strncpy(code + 15, Y, 13);
      strncpy(code + 28, temp5, 5);
      strncpy(code + 33, temp3, 3);
      Serial.println(code);
      Serial3.println(code);
      //if (code[35] == '9')
      //code[35] = '0';
    }
  }
}
