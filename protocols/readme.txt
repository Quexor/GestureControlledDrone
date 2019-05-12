about sending data to the drone

including:
	1. protocol
	2. hardware calibration when initialization(setup)


1. three commands need sending to the drone. for the glove, may only need 1 and 2 below, considering there is not enough buttons
I may need to make the drone do calibration when power on.
-------------------------------------
check the CommUAV.h and CommUAV.c files to see the code

need to rewrite the function NRF24L01_TxPacket(sendBuf) to send by using Arduino Due
	#define NRFUpload() NRF24L01_TxPacket(sendBuf)

-------------------------------------

1.1. every 80 Hz or 12.5 ms, sending the attitude control data
	CommUAVUpload(MSP_SET_4CON);
	
1.2. to lock or unlock the drone, so it can fly or lock the propellers.
		//detect key pressed
		if(Locksta == 0xa5) 
		{
			for(i=0;i<5;i++)         
				CommUAVUpload(MSP_ARM_IT);   //unlock Crazepony
			Locksta = 0x5a;
			Lockflag = 0;
		}
			
		else if(Locksta == 0x5a )
		{
			for(i=0;i<5;i++)         
			CommUAVUpload(MSP_DISARM_IT);	//lock Crazepony
			Locksta = 0xa5;
			Lockflag = 0;
		}
1.3. check if button pressed for asking for the drone to calibrate
		/*IMUcalibrate  */
		void IMUcalibrate(void)
		{
			  LedSet(led4,IMUcalibratflag);
			  if(IMUcalibratflag) 
					{
						CommUAVUpload(MSP_ACC_CALI);
						IMUcalibratflag = 0;
					}
		}
		
2. value range 1000-2000 for Throttle, Pitch, Roll, Yaw
----------------------------
	check file in control.c. especially function: void LoadRCdata(void)
	some calculation are done because the hardware design, but the final values are in the rang 1000-2000
	* need to do calibration at the initialization. see function void controlClibra(void) in control.c