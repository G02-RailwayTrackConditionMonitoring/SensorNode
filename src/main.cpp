#include "Arduino.h"
#include "MPU9250.h"


#define BUTTON_PIN PIN_BUTTON1
// an MPU9250 object with the MPU-9250 sensor on SPI bus 0 and chip select pin 10
MPU9250FIFO IMU(SPI,7);
int status;


// variables to hold FIFO data, these need to be large enough to hold the data, maximum expected is 85 samples

uint8_t fifoSize2;
size_t fifoSize;
float ax[100], ay[100], az[100];
int16_t acc_x[85];
int16_t acc_y[85];
int16_t acc_z[85];
uint8_t buffer_index=0;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}


  IMU.init(); // start communication with IMU   

  IMU.enableAccelFifo(); // enabling the FIFO to record just the accelerometers
}

void loop() {

  fifoSize = IMU.getFifoNumBytes()/6;
  if(fifoSize<40){
    delay(5); // Wait for another 40 samples.
  }
  else if(fifoSize>=40 && fifoSize<80){
    delay((80-fifoSize)/8+1); // Delay proportional to number of samples needed.
  }
  else if(fifoSize>=80){
    
    // IMU.haltSampleAccumulation(); // No addional samples will be placed into the FIFO
    
    IMU.readFifoInt(acc_x,acc_y,acc_z,&fifoSize2,2); // read the fifo buffer from the IMU

    // // get the X, Y, and Z accelerometer data and their size
    // IMU.getFifoAccelX_mss(&fifoSize,ax);
    // IMU.getFifoAccelY_mss(&fifoSize,ay);
    // IMU.getFifoAccelZ_mss(&fifoSize,az);
    
    // print the data
    // Serial.print("The FIFO buffer is ");
    // Serial.print(fifoSize2);
    // Serial.println(" samples long.");
    // for (size_t i=0; i < fifoSize2; i++) {
    //     Serial.printf("X: %5d, Y: %5d, Z: %5d \n",acc_x[i],acc_y[i],acc_z[i]);
    // }
  }
}

















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

