#include <SPI.h>
#include "RF24.h"
#include <printf.h>
#include "CommUAV.h"


// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins. CE is 8, CS is 10.
RF24 radio(8,10);

byte add1[] = {0x34, 0xC3, 0x10, 0x10, 0x01};
byte add2[] = {0x01, 0x10, 0x10, 0xC3, 0x34}; //Not working for some weird reason.

uint8_t sendBuf[32];
uint8_t sendCnt=0;
uint8_t checksum=0;


enum{THROTTLE,YAW,PITCH,ROLL}; 
uint16_t rcData[4]={1500,1500,1500,1500}; 

int Throttle = 1001;
int Roll = 1499;
int Pitch = 1499;
int Yaw = 1499;

unsigned long counter = 0; //number that gets send

void setup() {
  Serial.begin(115200);
  printf_begin();
  Serial.println("I'm alive");

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

void loop() {
  if ( Serial.available() )
  {
    char c = toupper(Serial.read());
    if ( c == 'A') {
      CommUAVUpload(MSP_ARM_IT);
    }
    if( c=='D') {
      CommUAVUpload(MSP_DISARM_IT);
    }
    if(c == 'C') {
      CommUAVUpload(MSP_ACC_CALI);
    }
    if(c == 'T') {
      CommUAVUpload(MSP_SET_4CON);
    }
  delay(10);
  }
}
