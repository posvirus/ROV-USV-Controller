char temp[41];

void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);
  Serial3.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if (Serial1.peek() == 'X')
  {
    Serial1.read();
    Serial1.readBytes(temp, 41);
    Serial3.println(temp);
  }
  else
  {
    Serial1.read();
  }
}
