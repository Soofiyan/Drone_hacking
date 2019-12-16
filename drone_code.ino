#include <Servo.h>
#include <avr/interrupt.h> 
#include <avr/io.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(A0, A1); // RX, TX     
// ------------------------------------------
//     1            2
//      -          -
//        -      -
//          -  -
//           -              Drone motor numbering
//          -  -
//        -      -
//      -          -
//     3            4
// ------------------------------------------
Servo esc_signal0;
Servo esc_signal1;
Servo esc_signal2;
Servo esc_signal3;

uint16_t yaw_drone = 0;
uint16_t pitch_drone = 0;
uint16_t roll_drone = 0;

int x=0,y=0,z=0;
int RX_raw=-1, RX_ad=-1, RX_ad1 = -1, pwm_range=255, RX_range=200, prev_RX_raw;
int xj1=0,yj1=0,xj2=0,yj2=0;
int RX[16]={100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};
int butt[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int r_temp;
int data;
int pres, prev;
int yaw1,yaw2,pitch1,pitch2,roll1,roll2;
int check_pitch,check_yaw,check_roll;
int check_pitch1,check_yaw1,check_roll1;
int last_error_p, last_error_r, last_error_y;
unsigned long current_time_p, current_time_r, current_time_y;
unsigned long prev_time_p, prev_time_r, prev_time_y;
int base_speed = 90;
int yaw_mpu,pitch_mpu,roll_mpu;
float kp_p = 0.075,ki_p,kd_p,kp_r=0.075,ki_r,kd_r,kp_y = 0.5,ki_y,kd_y;
float error_pitch,error_roll,error_yaw;
float target_pitch,target_roll,target_yaw;
float P_p,I_p,D_p,P_r,I_r,D_r,P_y,I_y,D_y,pid_pitch,pid_roll,pid_yaw;
int stop_drone = 0, start_drone = 0;
//L1 - 231, L2 - 233, R1 - 232, R2 - 234, UP - 241, DOWN - 243, LEFT - 242,
//RIGHT - 244, TRIANGLE - 237, SQUARE - 238, CROSS - 239, CIRCLE - 240,
//SELECT - 246, START - 245, L3 - 235, R3 - 236
void motor_setup();
void callibration();
void mpu_values();
void receive();
void motors_run(int base_speed, int m1, int m2, int m3, int m4);
long map_value(long in_value, long in_min, long in_max, long out_min, long out_max);
int receive_mpu();
void pid_drone();
void pid_drone_pitch(int error_pitch);
void pid_drone_roll(int error_roll);
void pid_drone_yaw(int error_yaw);
void button_ps2();
void stop_drone_imm();

void setup()
{
  motor_setup();
  UBRR0 = 103; // for configuring baud rate of 9600bps
  UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00); 
  // Use 8-bit character sizes 
  UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);  
  // Turn on the transmission, reception, and Receive interrupt 
  sei();// enable global interrupt
  mySerial.begin(9600);
  callibration();   
  motors_run(70,0,0,0,0);
  base_speed = 70;
  mpu_values();
  delay(2000);
  mpu_values();
  target_pitch = pitch_mpu;
  target_roll = roll_mpu;
  target_yaw = yaw_mpu;
}

void loop()
{
    mpu_values();
//    if(prev_RX_raw == RX_raw)
//    {
//      stop_drone ++;
//      if(stop_drone >= 30000)
//      {
//        stop_drone_imm();
//      }
//    }
//    else
//    {
//      prev_RX_raw = RX_raw;
//      stop_drone = 0;
//    }
    button_ps2();
//    motors_run(base_speed,0,0,0,0);
    pid_drone();
    delay(1);
}

void stop_drone_imm()
{
//  for(int i = base_speed;i>=40;)
//  {
//    esc_signal0.write(i);
//    esc_signal1.write(i);
//    esc_signal2.write(i);
//    esc_signal3.write(i);
//    i = i - 10;
//    delay(500);
//  } 
//  motors_run(0,0,0,0,0);
  while(1){
    motors_run(40,0,0,0,0);
  }
}

void button_ps2()
{
  
  if (butt[0]==1)//L1
  {
    butt[0]=0;
  }
//  else if (butt[1]==1)
//  {
//    butt[1]=0;
//  }
//  else if (butt[2]==1)
//  {
//    butt[2]=0;
//  }
//  else if (butt[3]==1)
//  {
//    butt[3]=0;
//  }
//  else if (butt[4]==1)
//  {
//    butt[4]=0;
//  }
//  else if (butt[5]==1)//r3
//  {
//    butt[5]=0;
//  }
//  else if (butt[6]==1)//triangle
//  {
//    butt[6]=0;
//  }
//  else if (butt[7]==1)//square
//  {
//    butt[7]=0;
//  }
  else if (butt[8]==1)//cross
  {
    stop_drone_imm();
    butt[8] = 0;
  }
  else if (butt[9]==1)//circle
  {
    stop_drone_imm();
    butt[9]=0;
  }
  else if (butt[10]==1)//up
  {
    base_speed += 5;
    if(base_speed >= 140)
    {
      base_speed = 140;
    }
    butt[10]=0;
  }
  else if (butt[11]==1)//left
  {
    base_speed -= 5;
    if(base_speed <= 50)
    {
      base_speed = 50;
    }
    butt[11]=0;
  }
//  else if (butt[12]==1)//down
//  {
//    butt[12] = 0;
//  }
  else if (butt[13]==1)//right
  { 
    butt[13]=0;
  }
  else if (butt[14]==1)//start
  {
    start_drone = 1;
    butt[14]=0;
  }
  else if (butt[15]==1)//select
  {
    stop_drone_imm();
    butt[15] =0;
  }  
}

void mpu_values()
{
  pres = receive_mpu();
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
  }
  prev = pres;

//  
//  pres = receive_mpu();
//  if(pres != prev)
//  {
//    switch(prev)
//    {
//      case 200:
//              pitch1 = pres;
//      case 201:
//              pitch2 = pres;
//      case 202:
//              roll1 = pres;
//      case 203:
//              roll2 = pres;
//      case 204:
//              yaw1 = pres;
//      case 205:
//              yaw2 = pres;
//      
//    }
//  }
//  prev = pres;
//  yaw_mpu = yaw1 + yaw2;
//  pitch_mpu = pitch1 + pitch2;
//  roll_mpu = roll1 + roll2;
}

void pid_drone()
{
  error_pitch = pitch_mpu - target_pitch;
  if(error_pitch <= -200)
  {
    error_pitch -= pitch_mpu - target_pitch;
    error_pitch ++;
  }
  else if(error_pitch >= 200)
  {
    error_pitch -= pitch_mpu - target_pitch;
    error_pitch --;
  }
  error_roll = roll_mpu - target_roll;
  if(error_roll <= -200)
  {
    error_roll -= roll_mpu - target_roll;
    error_roll ++;
  }
  else if(error_roll >= 200)
  {
    error_roll -= roll_mpu - target_roll;
    error_roll --;
  }
  error_yaw = yaw_mpu - target_yaw;
  if(error_yaw <= -200)
  {
    error_yaw -= yaw_mpu - target_yaw;
    error_yaw ++;
  }
  else if(error_yaw >= 200)
  {
    error_yaw -= yaw_mpu - target_yaw;
    error_yaw --;
  }
  pid_drone_pitch(error_pitch);
  pid_drone_roll(error_roll);
  pid_drone_yaw(error_yaw);
  motors_run(base_speed, pid_pitch+pid_roll+pid_yaw, -pid_pitch+pid_roll-pid_yaw, pid_pitch-pid_roll-pid_yaw, -pid_pitch-pid_roll+pid_yaw);
}

void motors_run(int base_speed, int m1, int m2, int m3, int m4)
{
  esc_signal0.write(base_speed + m1);
  esc_signal1.write(base_speed + m2);
  esc_signal2.write(base_speed + m3);
  esc_signal3.write(base_speed + m4+5);
}

void pid_drone_pitch(int error_pitch)
{
  current_time_p = millis();
  P_p = error_pitch*kp_p;
  I_p += error_pitch*ki_p;
  if(current_time_p - prev_time_p != 0)
  {
    D_p = kd_p*(error_pitch - last_error_p)/(current_time_p - prev_time_p);
  }
  else 
  {
    D_p = 0;
  }
  pid_pitch = P_p + I_p + D_p;
  prev_time_p = current_time_p;
//  motors_run(base_speed,P_p,0,P_p,0);
}

void pid_drone_roll(int error_roll)
{
  P_r = error_roll*kp_r;
  I_r += error_roll*ki_r;
  pid_roll = P_r;
//  motors_run(base_speed,P_r,P_r,0,0);
}

void pid_drone_yaw(int error_yaw)
{
  P_y = error_yaw*kp_y;
  I_y += error_yaw*ki_y;
  pid_yaw = P_y;
//  motors_run(base_speed,P_y,0,0,P_y);
}

int receive_mpu()
{
  if (mySerial.available())
  {
    data = mySerial.read();
  }
  return data;
}



void receive()
{
  if ((RX_raw>200) && (RX_raw<255))         //201 to 216 for addresses of analog values, 231 to 246 for buttons;
  {
    RX_ad1=RX_raw;
    if ((RX_raw>230) && (RX_raw<247))
    {
      int r_temp0=(RX_raw-231);
      butt[r_temp0]=1;
    }
  }
  else if ((RX_raw>=0) && (RX_raw<201))
  {
    int r_temp1=(RX_ad1-201);
    if (r_temp1<16)
    {
      RX[r_temp1]=RX_raw;
    }
  }
  yj2=map_value(RX[0],0,RX_range,(-pwm_range),pwm_range);
  xj2=map_value(RX[1],0,RX_range,(-pwm_range),pwm_range);
  yj1=map_value(RX[2],0,RX_range,(-pwm_range),pwm_range);
  xj1=map_value(RX[3],0,RX_range,(-pwm_range),pwm_range);
}

ISR (USART_RX_vect)
{  
  RX_raw=UDR0;
  receive();
}

void callibration()
{
  for(int i = 130;i>=40;)
  {
    esc_signal0.write(i);
    esc_signal1.write(i);
    esc_signal2.write(i);
    esc_signal3.write(i);
    i = i - 10;
    delay(1000);
  } 
}

void motor_setup()
{
  esc_signal0.attach(7);
  esc_signal0.write(30);
  esc_signal1.attach(4);
  esc_signal1.write(30);
  esc_signal2.attach(2);
  esc_signal2.write(30);
  esc_signal3.attach(8);
  esc_signal3.write(30);
  delay(3000);
}

long map_value(long in_value, long in_min, long in_max, long out_min, long out_max)
{  
  return (in_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
