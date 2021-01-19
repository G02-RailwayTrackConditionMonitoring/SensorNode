#include "Arduino.h"
#include "MPU9250.h"
#include "SdFat.h"
#include "sdios.h"
#include <nrf52840_peripherals.h>

#define BUTTON_PIN 9//PIN_BUTTON1

//Pin numbers are MOSI: D13, MISO: D12, SCK:D11, CS: D10
#define IMU_SPI_MOSI_PIN   13 
#define IMU_SPI_MISO_PIN   12 
#define IMU_SPI_SCK_PIN    11 
#define IMU_SPI_CS_PIN     7
//Setup a sepereate SPI bus for the IMU. 
SPIClass IMU_SPI(NRF_SPIM2,IMU_SPI_MISO_PIN,IMU_SPI_SCK_PIN,IMU_SPI_MOSI_PIN);

// an MPU9250 object with the MPU-9250 sensor on SPI bus 0 and chip select pin 10
MPU9250FIFO IMU(IMU_SPI,IMU_SPI_CS_PIN);
int status;

//Sd card object with FAT32 filesystem
SdFat32 sd;
File32 file;
#define SPI_SPEED SD_SCK_MHZ(50)

// variables to hold FIFO data, these need to be large enough to hold the data, maximum expected is 85 samples

uint8_t fifoSize2;
size_t fifoSize;
float ax[100], ay[100], az[100];
int16_t acc_x[85];
int16_t acc_y[85];
int16_t acc_z[85];
uint8_t buffer_index=0;//Keeps track of how many samples in the acc_x,acc_y,acc_z / sd buffer.
int16_t sdBuffer[512];

void setup() {
  
  
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}
  delay(5000);
  //Setup SD card with cs pin 2, max Freq 10MHz.
  if(!sd.begin(2,50)){
    Serial.println("Error initializing SD card...");
  }

  // create /test on sd if it does not exist
  if (!sd.exists("/test")) {
    if (!sd.mkdir("/test")) {
      Serial.println("SD card cannot find directory '/test', and cannot create it.");
    }
  }
  // Make /test the working directory on sd.
  if (!sd.chdir("test")) {
     Serial.println("failed changing directory to '/test'");
  }

  Serial.println(F("------sd-------"));
  sd.ls("/", LS_R | LS_SIZE);

Serial.printf("Card size: %f\n",sd.card()->sectorCount()*512E-9);


  if (!file.open("test.dat", O_WRONLY | O_CREAT | O_TRUNC)) {
    Serial.println("open failed");
  }
  
  IMU.init(); // start communication with IMU   
  IMU.enableAccelFifo(); // enabling the FIFO to record just the accelerometers
  Serial.println("Starting test");
  Serial.flush();
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
    Serial.println("reading imu");
    IMU.readFifoInt(&acc_x[buffer_index],&acc_y[buffer_index],&acc_z[buffer_index],&fifoSize2,2); // read the fifo buffer from the IMU
    buffer_index += fifoSize2;
    Serial.printf("buff idx: %d, fifoSize: %d\n",buffer_index,fifoSize);
    Serial.flush();

    //Pack the data into x,y,z for writing to sd card.
    for(int i=0; i< fifoSize; i++){
      sdBuffer[i*3] = acc_x[i];
      sdBuffer[(i*3)+1] = acc_y[i];
      sdBuffer[(i*3)+2] = acc_z[i];
    }


  }

  if(buffer_index>80){
    Serial.println("Writing to sd");
    Serial.flush();
    file.write(sdBuffer,buffer_index*6);
    //file.sync(); // Sounds like this is needed?
    buffer_index = 0;

  }

  if(digitalRead(BUTTON_PIN) == LOW ){

    Serial.println("End of test...");
    file.close();
    Serial.flush();
    while(1){};
  }

}

