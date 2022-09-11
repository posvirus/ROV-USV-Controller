#include <Streaming.h>
#include <string.h>
#include <FlexiTimer2.h >
#include "ping1d.h"
char trash[15];
char sign[5];
char X[14], Y[13], A[7], M[36], B[6];
int len_of_a;
char MES[500];
bool mes1, mes2;
Ping1D myPing {Serial1};
uint32_t temp1, temp2;
char temp5[5], temp3[3];
void setup() {
  Serial2.begin(460800);
  Serial.begin(9600);
  Serial1.begin(115200);
  Serial3.begin(9600);
  memset(sign, 0, sizeof(sign));
  memset(temp5, '0', sizeof(temp5));
  memset(temp3, '0', sizeof(temp3));
  memset(X, '0', sizeof(X));
  memset(Y, '0', sizeof(Y));
  memset(M, 0, sizeof(M));
  memset(MES, '\0', sizeof(MES));
  mes1 = false;
  mes2 = false;
}

void loop() {
  if (Serial2.read() == '$')
  {
    Serial2.readBytes(sign, 5);
    if (sign[2] == 'G' || sign[3] == 'G' || sign[4] == 'A')
      {
      Serial2.readBytes(trash, 11);
      Serial2.readBytes(Y, 13);
      Serial2.readBytes(trash, 3);
      Serial2.readBytes(X, 14);
      M[0] = 'X';
      //strncat(M, X, 14);
      //strncat(M, Y, 13);
      /*
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
      //strncat(M, temp5, 5);
      //strncat(M, temp3, 3);
      */
      mes1 = true;
      //Serial << M << endl;
      //Serial3.print(M);
      //memset(M, 0, sizeof(M));
      }
    if (sign[2] == 'S' || sign[3] == 'H' || sign[4] == 'R')
    {
      Serial2.readBytes(trash, 12);
      len_of_a = Serial2.readBytesUntil(',', A, 7);
      //Serial << A << endl;
      //Serial << len_of_a << endl;
      for (int i = 0; i <= 5 - len_of_a; i++)
      {
        B[i] = '0';
      }
      for (int i = 6 - len_of_a; i <= 5; i++)
      {
        B[i] = A[i - 6 + len_of_a];
      }
      //Serial << B << endl;
      mes2 = true;
    }
  }
  if (mes1 && mes2)
  {
    //mes1 = true;
    mes1 = false;
    mes2 = false;
    memset(M, 0, sizeof(M));
    M[0] = 'X';
    strncat(M, X, 14);
    strncat(M, Y, 13);
    strncat(M, temp5, 5);
    strncat(M, temp3, 3);
    strncat(M, A, 6);
    Serial << M << endl;
  }
  //MAC.println(1);

}
