#include <SoftwareSerial.h>

SoftwareSerial mySerial(A0,A1);
int data;
int pres;
int prev;
int yaw_mpu,pitch_mpu,roll_mpu;
int yaw1,yaw2,pitch1,pitch2,roll1,roll2;
int check_pitch,check_yaw,check_roll;
int check_pitch1,check_yaw1,check_roll1;
void setup() {
  // put your setup code here, to run once:
  mySerial.begin(9600);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (mySerial.available())
  {
    data = mySerial.read();
  }
  pres = data;
    if(pres != prev && pres < 200)
    {
      switch(prev)
      {
        case 200:
                pitch1 = pres;
                check_pitch1 = 1;
        case 201:
                if(check_pitch1 == 1)
                {
                  pitch2 = pres;
                  check_pitch = 1;
                }
        case 202:
                roll1 = pres;
                check_roll1 = 1;
        case 203:
                if(check_roll1 == 1)
                {
                  roll2 = pres;
                  check_roll = 1;
                }
        case 204:
                yaw1 = pres;
                check_yaw1 = 1;
        case 205:
                if(check_yaw1 == 1) 
                {
                  yaw2 = pres;
                  check_yaw = 1;
                }
        
      }
  }
  if(check_pitch == 1 && check_yaw == 1 && check_roll == 1)
  {
    yaw_mpu = yaw1 + yaw2;
    pitch_mpu = pitch1 + pitch2;
    roll_mpu = roll1 + roll2;
    check_pitch = 0;
    check_yaw = 0;
    check_roll = 0;
    check_pitch1 = 0;
    check_yaw1 = 0;
    check_roll1 = 0;
    Serial.println(pitch_mpu);
    Serial.println(roll_mpu);
    Serial.println(yaw_mpu);
    Serial.println();
  }
  prev = pres;
}
