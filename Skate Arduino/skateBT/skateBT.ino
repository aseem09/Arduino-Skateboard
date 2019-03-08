#include "I2Cdev.h"
#include "SoftwareSerial.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

SoftwareSerial serial(10,11);
MPU6050 mpu;
const int brake=2;
int maxTilt=3.5;
bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;     
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 
bool b;

int temp=0;
Quaternion q;           
VectorInt16 aa;        
VectorInt16 aaReal;     
VectorInt16 aaWorld;    
VectorFloat gravity;    
float euler[3];         
float ypr[3];           
volatile bool mpuInterrupt = false;   

void setup() 
{
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; 
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    serial.begin(9600);  //For use with Arduino Uno
    
    serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    serial.println(F("Testing device connections..."));
    serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 
    if (devStatus == 0) {
        serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        serial.print(F("DMP Initialization failed (code "));
        serial.print(devStatus);
        serial.println(F(")"));
    }
   pinMode(brake, INPUT);

}

void sendData(int x, int y, int z, bool brakeTrue)
{
  
      if(z>18){
       serial.println("j");
     serial.println(z);
     serial.flush();
     }
     if(brakeTrue){
          serial.println("b");
          serial.flush();
         
        }
      else{
       
      serial.println(y);
      
     serial.flush();
      
      
      }
      delay(20);
}

void loop() 
{
  
    if (!dmpReady) return;
    
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        serial.println(F("FIFO Overflow"));
//b=true;
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) {
          fifoCount = mpu.getFIFOCount();
          
        }
        
        mpu.getFIFOBytes(fifoBuffer, packetSize);        
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
      if(digitalRead(brake)==HIGH){
          sendData(ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI, true);
      }else{
        sendData(ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI, false);
      }

        
    }
}

void dmpDataReady() 
{
    mpuInterrupt = true;
}
