uint8_t i;

int error = 0;
byte type = 0;
byte vibrate = 0;

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h>

SoftwareSerial mySerial(A0, A1); // RX, TX

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t pitch1 = 0;
uint8_t roll1 = 0;
uint8_t pitch2 = 0;
uint8_t roll2 = 0;
int pitch_mpu = 0;
int roll_mpu = 0;
uint8_t yaw1 = 0;
uint8_t yaw2 = 0;
int yaw_mpu = 0;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() 
{
  mpuInterrupt = true;
}

void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  Serial.begin(9600);
  // initialize serial communication
//  mySerial.begin(115200);
  while (!Serial);
  mpu.initialize();

  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(65);//65
  mpu.setYGyroOffset(-16);//-16
  mpu.setZGyroOffset(-14);//-14
  mpu.setZAccelOffset(1368); // 1368 factory default for my test chip    // 1196

  if (devStatus == 0) 
  {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else 
  {
  }
}

void loop() 
{
  if (!dmpReady) return;
//  while (!mpuInterrupt && fifoCount < packetSize) 
//  { 
//  }
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
  {
      // reset so we can continue cleanly
      mpu.resetFIFO();
  } 
  else if (mpuIntStatus & 0x02) 
  {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    char x=209,y=210,z=211;
    int bot_yaw=map((ypr[0] * 180/M_PI),-180,180,0,360);
    int bot_pitch=map((ypr[1] * 180/M_PI),-90,90,0,360);
    int bot_roll=map((ypr[2] * 180/M_PI),-90,90,0,360);
    
    #endif  //commented 
    pitch_mpu = bot_pitch;
    roll_mpu = bot_roll;
    yaw_mpu = bot_yaw;
//    Serial.println(pitch_mpu);
//    Serial.println(roll_mpu);
//    Serial.println(yaw_mpu);
    if(pitch_mpu <= 180)
    {
      pitch1 = pitch_mpu;
      pitch2 = 0;
    }
    else
    {
      pitch1 = 180;
      pitch2 = pitch_mpu - 180;
    }
    if(roll_mpu <= 180)
    {
      roll1 = roll_mpu;
      roll2 = 0;
    }
    else
    {
      roll1 = 180;
      roll2 = roll_mpu - 180;
    }
    if(yaw_mpu <= 180)
    {
      yaw1 = yaw_mpu;
      yaw2 = 0;
    }
    else
    {
      yaw1 = 180;
      yaw2 = yaw_mpu - 180;
    }
//    Serial.println(200);
    Serial.write(200);
    delay(5);
    Serial.write(pitch1);
    delay(5);
    Serial.write(201);
    delay(5);
    Serial.write(pitch2);
    delay(5);
    Serial.write(202);
    delay(5);
    Serial.write(roll1);
    delay(5);
    Serial.write(203);
    delay(5);
    Serial.write(roll2);
    delay(5);
    Serial.write(204);
    delay(5);
    Serial.write(yaw1);
    delay(5);
    Serial.write(205);
    delay(5);
    Serial.write(yaw2);
    delay(5);
//    delay(500);
   }
 }   
