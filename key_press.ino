
#define interruptPin1 9
#define PIO_mask9 (1 << 21)

#define interruptPin2 8
#define PIO_mask8 (1 << 22)

#define interruptPin3 7
#define PIO_mask7 (1 << 23)

#define interruptPin4 6
#define PIO_mask6 (1 << 24)

char btn1_flag=0;
char btn2_flag=0;
char btn3_flag=0;
char btn4_flag=0;

// connect the key to digital pin 9,8,7,6
void setup() {

  REG_PIOC_WPMR = 0x50494F00;
  REG_PIOC_PER |= PIO_mask9|PIO_mask8|PIO_mask7|PIO_mask6;
  REG_PIOC_MDDR |= PIO_mask9|PIO_mask8|PIO_mask7|PIO_mask6;	
  REG_PIOC_PUER | = PIO_mask9|PIO_mask8|PIO_mask7|PIO_mask6;
  REG_PIOC_DIFSR |= PIO_mask9|PIO_mask8|PIO_mask7|PIO_mask6;
  REG_PIOC_SCDR = 1;
  REG_PIOC_IFER |= PIO_mask9|PIO_mask8|PIO_mask7|PIO_mask6;
  
  attachInterrupt(digitalPinToInterrupt(interruptPin1), button1_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), button2_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(interruptPin3), button3_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(interruptPin4), button4_ISR, FALLING);
}
void button1_ISR() {
	btn1_flag=1;
}
void button2_ISR() {
	btn2_flag=1;
}
void button3_ISR() {
	btn3_flag=1;
}
void button4_ISR() {
	btn4_flag=1;
}
