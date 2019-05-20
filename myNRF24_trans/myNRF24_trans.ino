#include <SPI.h>
#include "RF24.h"
#include <printf.h>
#include "CommUAV.h"

//DMP
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins. CE is 8, CS is 10.
RF24 radio(8,10);
MPU6050 mpu;

byte add1[] = {0x34, 0xC3, 0x10, 0x10, 0x01};

uint8_t sendBuf[32];
uint8_t sendCnt=0;
uint8_t checksum=0;


enum{THROTTLE,YAW,PITCH,ROLL}; 
uint16_t rcData[4]={1500,1500,1500,1500}; 

int Throttle = 1001;
int Roll = 1499;
int Pitch = 1499;
int Yaw = 1499;

char cmd;

unsigned long counter = 0; //number that gets send

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  Serial.begin(115200);
  printf_begin();
  Serial.println("I'm alive");
  DMPinit();
  NRF24init();
}

void NRF24init() {
  radio.begin();
  radio.setPALevel(RF24_PA_LOW); //set to max to test drone
  radio.setChannel(40);
  radio.setCRCLength(RF24_CRC_16);
  radio.setDataRate(RF24_2MBPS);
  radio.setRetries(1,10);
  //radio.enableAckPayload();
  radio.enableDynamicAck();
  radio.openReadingPipe(0,add1);
  radio.openWritingPipe(add1);
  radio.closeReadingPipe(1); //was opened by radio.begin();
  radio.printDetails();
}

void DMPinit() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void NRF24Send() {
  Serial.print(F("Now sending "));
  Serial.println(counter);

  if (!radio.write( &counter, sizeof(unsigned long))){
    Serial.println(F("failed"));
  }
  counter++;
}

void NRF24SendBuffer(uint8_t *buf) {
  Serial.print(F("Now sending "));
  for(int i = 0; i < 32; i++)
  {
    Serial.print(buf[i], HEX);
    Serial.print('-');
  }
  Serial.println();

  if (!radio.write( buf, 32)) {
    Serial.println(F("failed"));
  }
}

static  void uart8chk(uint8_t _x) 
{
  sendBuf[sendCnt++]=_x;    
  checksum ^= _x; 
}

static void uart16chk(int16_t a)
{
    uart8chk((uint8_t)(a&0xff));
    uart8chk((uint8_t)(a>>8));    
}


void CommUAVUpload(uint8_t cmd)
{
//  uint8_t len;
    
    sendCnt=0;
    for(int i = 0; i++; i < 32) {
      sendBuf[i] = 0;
    }
    
    uart8chk('$');
    uart8chk('M');
    uart8chk('<');
    checksum = 0;   
    
    switch(cmd)
    {
      case MSP_SET_4CON:
        uart8chk(8);            //data payload len
        uart8chk(cmd);
        uart16chk(Throttle);
        uart16chk(Yaw);
        uart16chk(Pitch);
        uart16chk(Roll);
        break;
      case MSP_ARM_IT:
        uart8chk(0);
        uart8chk(cmd);
        break;
      case MSP_DISARM_IT:
        uart8chk(0);
        uart8chk(cmd);
        break;
      case MSP_HOLD_ALT:
        uart8chk(0);
        uart8chk(cmd);
        break;
      case MSP_STOP_HOLD_ALT:
        uart8chk(0);
        uart8chk(cmd);
        break;
      case MSP_HEAD_FREE:
        uart8chk(0);
        uart8chk(cmd);
        break;
      case MSP_STOP_HEAD_FREE:
        uart8chk(0);
        uart8chk(cmd); 
        break;
      case MSP_AUTO_LAND_DISARM:
        uart8chk(0);
        uart8chk(cmd); 
        break;
      case MSP_ACC_CALI:
        uart8chk(0);
        uart8chk(cmd); 
        break;
    }
    uart8chk(checksum);
    NRF24SendBuffer(sendBuf);
}

void processDMP() {
  if (mpuInterrupt && fifoCount < packetSize) {
        // try to get out of the infinite loop 
        fifoCount = mpu.getFIFOCount();
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  // wait for correct available data length, should be a VERY short wait
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);

  // track FIFO count here in case there is > 1 packet available
  // (this lets us immediately read more without waiting for an interrupt)
  fifoCount -= packetSize;

  //display YAW PITCH ROLL
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//  Serial.print(ypr[0]/M_PI);
//  Serial.print(",");
//  Serial.print(ypr[1]/M_PI);
//  Serial.print(",");
//  Serial.println(ypr[2]/M_PI);
}

void loop() {
  if (Serial.available())
  {
    int input = toupper(Serial.read());
    if (input != 0 && input != 13 && input != 10) {
      cmd = (char) input;
    }
  }
  
  if ( cmd == 'A') {
    CommUAVUpload(MSP_ARM_IT);
  }
  if( cmd=='D') {
    CommUAVUpload(MSP_DISARM_IT);
  }
  if(cmd == 'C') {
    CommUAVUpload(MSP_ACC_CALI);
  }
  if(cmd == 'T') {
    CommUAVUpload(MSP_SET_4CON);
  }
  processDMP();
  Pitch = map(ypr[1], -M_PI, M_PI, 1000, 2000);
  Roll = map(ypr[2], -M_PI, M_PI, 1000, 2000);
}
