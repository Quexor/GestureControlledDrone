#include <SPI.h>
#include "RF24.h"
#include <printf.h>

// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins. CE is 8, CS is 10.
RF24 radio(8,10);

byte add1[] = {0x34, 0xC3, 0x10, 0x10, 0x01};
byte add2[] = {0x01, 0x10, 0x10, 0xC3, 0x34};

void setup() {
  Serial.begin(115200);
  printf_begin();
  Serial.println("I'm alive");
  
  NRF24init();
}

void NRF24init() {
  radio.begin();
  radio.setPALevel(RF24_PA_MIN); //set to max to test drone
  radio.setChannel(40);
  radio.setCRCLength(RF24_CRC_16);
  radio.setDataRate(RF24_2MBPS);
  radio.setRetries(1,10);
  //radio.enableAckPayload();
  radio.enableDynamicAck();
  radio.closeReadingPipe(1); //was opened by radio.begin();
  radio.openReadingPipe(0, add2); //Open Pipe 0 as reading pipe
  radio.openWritingPipe(add2);
  radio.printDetails();
  radio.startListening();
}

void loop() {
  unsigned long received;
  if(radio.available()) {
    radio.read( &received, sizeof(unsigned long) );             // Get the payload
    //radio.writeAckPayload(0,&received, sizeof(unsigned long));
    Serial.println(received);
  } else {
    Serial.println("Nothing received.");
  }
  delay(500);
}
