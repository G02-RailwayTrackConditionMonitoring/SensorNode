#include "Arduino.h"
#include "MPU9250.h"
#include "SdFat.h"
#include "sdios.h"
#include <nrf52840_peripherals.h>
#include <arm_math.h>
#include "Resampler.h"
#include "filter.h"

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
#define SPI_CLOCK SD_SCK_MHZ(50)
#define SD_CS_PIN 2

// variables to hold FIFO data, these need to be large enough to hold the data, maximum expected is 85 samples

uint8_t fifoSize2;
size_t fifoSize;

float acc_x[85];
float acc_y[85];
float acc_z[85];

float acc_x_2khz[85];
float acc_y_2khz[85];
float acc_z_2khz[85];

int16_t acc_x_int[85];
int16_t acc_y_int[85];
int16_t acc_z_int[85];

uint8_t buffer_index=0;//Keeps track of how many samples in the acc_x,acc_y,acc_z / sd buffer.
int16_t sdBuffer[512];

//We need a downsampler for each signal.
Downsampler downsampler_x(2,anitaliasing_filter,FILTER_TAP_NUM,86);
Downsampler downsampler_y(2,anitaliasing_filter,FILTER_TAP_NUM,86);
Downsampler downsampler_z(2,anitaliasing_filter,FILTER_TAP_NUM,86);

void  convert_to_int(float* in, int16_t* out, int num_samples, float bias, float scale, float axis_scale);

void test_downsampling();

void setup() {
  
  
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}
  delay(5000);
  //Setup SD card with cs pin 2, max Freq 10MHz.
  if(!sd.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK))){
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
    IMU.readFifo(&acc_x[buffer_index],&acc_y[buffer_index],&acc_z[buffer_index],&fifoSize2); // read the fifo buffer from the IMU

    
    downsampler_x.downsample(acc_x,acc_x_2khz,fifoSize2);
    downsampler_y.downsample(acc_y,acc_y_2khz,fifoSize2);
    downsampler_z.downsample(acc_z,acc_z_2khz,fifoSize2);

    convert_to_int(acc_x_2khz,acc_x_int,fifoSize2/2,IMU.getAccelBiasX_mss(),IMU.getAccelScaleFactor(),IMU.getAccelScaleFactorX());
    convert_to_int(acc_y_2khz,acc_y_int,fifoSize2/2,IMU.getAccelBiasY_mss(),IMU.getAccelScaleFactor(),IMU.getAccelScaleFactorY());
    convert_to_int(acc_z_2khz,acc_z_int,fifoSize2/2,IMU.getAccelBiasZ_mss(),IMU.getAccelScaleFactor(),IMU.getAccelScaleFactorZ());

    buffer_index += fifoSize2;
    Serial.printf("buff idx: %d, fifoSize: %d\n",buffer_index,fifoSize2);
    Serial.flush();

    
    //Pack the data into x,y,z for writing to sd card.
    for(int i=0; i< fifoSize2; i++){
      sdBuffer[i*3] = acc_x_int[i];
      sdBuffer[(i*3)+1] = acc_y_int[i];
      sdBuffer[(i*3)+2] = acc_z_int[i];
    }

  }

  if(buffer_index>80){
    Serial.println("Writing to sd");
    Serial.flush();
    file.write(sdBuffer,buffer_index*6);
    //file.sync(); // Sounds like this is needed? NO!
    buffer_index = 0;

  }

  if(digitalRead(BUTTON_PIN) == LOW ){

    Serial.println("End of test...");
    file.close();
    Serial.flush();
    while(1){};
  }

}

void  convert_to_int(float* in, int16_t* out, int num_samples, float bias, float scale, float axis_scale){


    for(int i=0; i< num_samples;i++){

      //this is basically the reverse of the conversion code in MPU9250 readFifo function.
      out[i] = (int16_t)(((in[i]/axis_scale)+bias)/scale);
   
    }

}

void test_downsampling(){

  File32 inputFile;
  if(!inputFile.open("/test/test_data.dat",O_RDONLY)){
    Serial.println("test data open failed");
  }

  int file_size = inputFile.fileSize();
  Serial.printf("Input file is %d bytes long\n",file_size);

  if(file_size %4 != 0){
    Serial.println("Probelm with input data");
  }

  int samples_per_block = 84;
  int num_samples = file_size/4; //4 bytes to a float.
  int num_blocks = num_samples/samples_per_block; 
  int remainder = num_samples%samples_per_block;
  if(remainder) num_blocks = num_blocks+1;

  Downsampler downsampler(2,anitaliasing_filter,FILTER_TAP_NUM,86);

  for(int i = 0; i < num_blocks; i++){

    Serial.printf("Processing block %i\n",i);
    Serial.flush();

    if(remainder && (i == (num_blocks-1))){
        samples_per_block = remainder;
    }

    float32_t in_buff[samples_per_block];
    inputFile.readBytes((uint8_t*)in_buff,4*samples_per_block);
    
    float32_t out_buff[samples_per_block/2];
    long start = micros();
    downsampler.downsample(in_buff,out_buff,samples_per_block);
    long time = micros()-start;
    
    file.write((uint8_t*)out_buff,samples_per_block/2*4);

    Serial.printf("Time to downsample 1 block: %d\n", time);
    Serial.flush();
  }
  inputFile.close();
  file.close();
  Serial.flush();
}