# Gesture Controlled Drone
This is the repository for specifically the glove-controller part of our gesture controlled drone. 
We used an Arduino Due for the microcontroller and attached all the sensors we used to it.

The peripherals that we attached to our Arduino Due are as follows:
- Three flex sensors used as voltage dividers down 3 of the fingers of the smart-glove.
- The MPU6050 as the inertial measurement unit, placed on the flat of the smart-glove.
- The nRF24L01+ is the 2.4Ghz radio module used to communicate to our drone, which also has a nRF24 radio module attached to its drone-embedded STM32F103 microcontroller.

Libraries used:
- https://github.com/ivanseidel/DueTimer
- https://github.com/JonHub/Filters
- https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
- https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev
- https://github.com/nRF24/RF24

Use the Arduino IDE with the Arduino Due extension to upload and compile ```controller_main/controller_main.ino``` onto an Arduino Due.
