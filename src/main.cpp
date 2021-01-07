/*
FIFO_SPI.ino
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on SPI bus 0 and chip select pin 10
MPU9250FIFO IMU(SPI,10);
int status;

// variables to hold FIFO data, these need to be large enough to hold the data
//float ax[100], ay[100], az[100]; 
float ax[1000], ay[1000], az[1000];
size_t fifoSize;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  // enabling the FIFO to record just the accelerometers
  IMU.enableAccelFifo();
  // Without delay 39 samples will be placed into FIFO when at 4kHz, FIFO can hold a max of 85 samples before overwriting
  delay(10); 
  // No addional samples will be placed into the FIFO
  IMU.haltSampleAccumulation();
  // read the fifo buffer from the IMU
  IMU.readFifo();

  // get the X, Y, and Z accelerometer data and their size
  IMU.getFifoAccelX_mss(&fifoSize,ax);
  IMU.getFifoAccelY_mss(&fifoSize,ay);
  IMU.getFifoAccelZ_mss(&fifoSize,az);
  
  // print the data
  Serial.print("The FIFO buffer is ");
  Serial.print(fifoSize);
  Serial.println(" samples long.");
  for (size_t i=0; i < fifoSize; i++) {
    Serial.print(ax[i],6);
    Serial.print("\t");
    Serial.print(ay[i],6);
    Serial.print("\t");
    Serial.println(az[i],6);
  }
}

void loop() {}

















// //an MPU9250 object with the MPU-9250 sensor on SPI bus 0 and chip select pin 10
// MPU9250 IMU(SPI,10);
// int status;

// void setup() {
//   // serial to display data
//   Serial.begin(115200);
//   while(!Serial) {}

//   // start communication with IMU 
//   status = IMU.begin();
//   if (status < 0) { 
//     Serial.println("IMU initialization unsuccessful");
//     Serial.println("Check IMU wiring or try cycling power");
//     while(1) {
//       Serial.print("Status: ");
//       Serial.println(status);
//     }
//   }
// }

// void loop() {
//   // read the sensor
//   IMU.readSensor();
//   // display the data
//   Serial.print(IMU.getAccelX_mss(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getAccelY_mss(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getAccelZ_mss(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getGyroX_rads(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getGyroY_rads(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getGyroZ_rads(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getMagX_uT(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getMagY_uT(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getMagZ_uT(),6);
//   Serial.print("\t");
//   Serial.println(IMU.getTemperature_C(),6);
//   delay(100);
// }









// #include "MPU9250.h"

// // an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
// MPU9250 IMU(Wire,0x68);
// int status;

// void setup() {
//   // serial to display data
//   Serial.begin(115200);
//   Serial.println("test");
//   while(!Serial) {}

//   // start communication with IMU 
//   status = IMU.begin();
//   if (status < 0) {
//     Serial.println("IMU initialization unsuccessful");
//     Serial.println("Check IMU wiring or try cycling power");
//     Serial.print("Status: ");
//     Serial.println(status);
//     while(1) {}
//   }
//   Serial.println("init ran properly");
// }

// void loop() {
//   Serial.println("this is running");
//   // read the sensor
//   IMU.readSensor();
//   // display the data
//   Serial.print(IMU.getAccelX_mss(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getAccelY_mss(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getAccelZ_mss(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getGyroX_rads(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getGyroY_rads(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getGyroZ_rads(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getMagX_uT(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getMagY_uT(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getMagZ_uT(),6);
//   Serial.print("\t");
//   Serial.println(IMU.getTemperature_C(),6);
//   delay(100);
// }

// #include "MPU9250.h"

// MPU9250 IMU(Wire,0x68);
// int status;

// void setup() {
//   // serial to display data
//   Serial.begin(115200);

//   while(!Serial) {}
//   Serial.println("infinite loop?");
//   // start communication with IMU 
//   status = IMU.begin();
//   if (status < 0) {
//     Serial.println("IMU initialization unsuccessful");
//     Serial.println("Check IMU wiring or try cycling power");
//     while(1) {
//     Serial.print("Status: ");
//     Serial.println(status);
//     }
//   }
// }
// void loop(){
//   Serial.println("turn on");
//   digitalWrite(LED_BUILTIN, HIGH);
//   delay(1000);
//   Serial.println("turn off");
//   digitalWrite(LED_BUILTIN, LOW);
//   delay(1000);
// }

