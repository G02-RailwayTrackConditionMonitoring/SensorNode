#include "Arduino.h"

#include "NodeBLE.h"

#include "MPU9250.h"

#include "SdFat.h"
#include "sdios.h"
#include <nrf52840_peripherals.h>

#include <arm_math.h>
#include "Resampler.h"
#include "filter.h"

#include "DMA_SPI.h"


#ifdef NODE_1
#define BUTTON_PIN  9
#elif NODE_2
#define BUTTON_PIN  PIN_BUTTON1
#else
#define BUTTON_PIN PIN_BUTTON1
#endif

#define LOGGING     1

//Pin numbers are MOSI: D13, MISO: D12, SCK:D11, CS: D10
#define IMU_SPI_MOSI_PIN   13 
#define IMU_SPI_MISO_PIN   12 
#define IMU_SPI_SCK_PIN    11 
#define IMU_SPI_CS_PIN     7
//Setup a sepereate SPI bus for the IMU. 
DMA_SPI IMU_SPI(NRF_SPIM3,IMU_SPI_MISO_PIN,IMU_SPI_SCK_PIN,IMU_SPI_MOSI_PIN,IMU_SPI_CS_PIN);

// an MPU9250 object with the MPU-9250 sensor on SPI bus 0 and chip select pin 10
MPU9250FIFO IMU(IMU_SPI,IMU_SPI_CS_PIN);
int status;

//Sd card object with FAT32 filesystem
SdExFat sd;
ExFatFile file;
#define SPI_CLOCK SD_SCK_MHZ(50)
#define SD_CS_PIN 2

// variables to hold FIFO data, these need to be large enough to hold the data, maximum expected is 85 samples

uint8_t numSamples;
size_t fifoSize;

//These hold the 4kHz imu data streams. Fifo holds up to 85 samples, and we need one extra space for if we have odd number of samples.
uint8_t raw_data_offset = 0;
float acc_x[86];
float acc_y[86];
float acc_z[86];

//Holds the downsampled data streams, so we only need half the amount of space (43 samples).
float acc_x_2khz[43];
float acc_y_2khz[43];
float acc_z_2khz[43];

//Holds the downsampled data streams but in int form which is needed for transmitting and saving to SD card.
int16_t acc_x_int[43];
int16_t acc_y_int[43];
int16_t acc_z_int[43];

uint8_t buffer_index=0;//Keeps track of how many samples in the acc_x,acc_y,acc_z / sd buffer.
int16_t sdBuffer[512];


uint8_t mode = 0;
// //Double buffering for BLE transmit.
// int16_t bleBuffA[240]; // BLE can transmit 244 bytes, so 240/6 = 40 samples at a time.
// int16_t bleBuffB[240];
// uint8_t bleBuffIdxA = 0;
// uint8_t bleBuffIdxB =0;

//We need a downsampler for each signal.
Downsampler downsampler_x(2,anitaliasing_filter,FILTER_TAP_NUM,86);
Downsampler downsampler_y(2,anitaliasing_filter,FILTER_TAP_NUM,86);
Downsampler downsampler_z(2,anitaliasing_filter,FILTER_TAP_NUM,86);

void  convert_to_int(float* in, int16_t* out, int num_samples, float bias, float scale, float axis_scale);

void test_downsampling();

void setup() {

  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}
  delay(5000);
  
  //Pin for debuugging BLE. Goes high when in sendData function.
  pinMode(PIN_A0,OUTPUT);
  digitalWrite(PIN_A0,LOW);
  pinMode(PIN_A1,OUTPUT);
  digitalWrite(PIN_A1,LOW);

  // Serial.println("Starting BLE...");

  // #ifdef NODE_1
  // BLE_Stack.startBLE("G02_A");
  // #elif NODE_2
  // BLE_Stack.startBLE("G02_B");
  // #endif
  
  // while(!BLE_Stack.isConnected()){
  //   BLE_Stack.startAdvertising();
  //   Serial.println("Connecting to BLE.");
  //   delay(5000);
  // } // Wait to be connected.
  // delay(4000);//Wait for connectino to be good.
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
  Serial.flush();

  // IMU_SPI.begin();
  IMU.init(); // start communication with IMU   
  IMU.enableAccelFifo(); // enabling the FIFO to record just the accelerometers
  IMU_SPI.startReccuringTransfer(0,0,5);
  Serial.println("Starting test");
  Serial.flush();

  //mode = LOGGING;// Comment this out to empty the main loop.
}
  //Loop
void loop(){

if(mode == LOGGING){
  fifoSize = IMU.getFifoNumBytes()/6;
  if(fifoSize<40){
    delay(4); // Wait for another 20 samples.
    
  }
  else if(fifoSize>=40 && fifoSize<75){
    delay((74-fifoSize)/8+1); // Delay proportional to number of samples needed.
    
  }
  else if(fifoSize>=75){
    
    IMU.readFifo(&acc_x[raw_data_offset],&acc_y[raw_data_offset],&acc_z[raw_data_offset],&numSamples); // read the fifo buffer from the IMU
    Serial.printf("reading imu: %d samples.\n",numSamples);

    if(numSamples >80) numSamples = 80; //Just drop samples. If we don't then we just fall behind and drop more.

    //If we have a left over sample from last read, we need to increase num samples.
    if(raw_data_offset == 1){
      numSamples = numSamples+1; // Add from the last one.
      raw_data_offset =0; // reset
    }
    //We must process chunks that have length that is a multiple of the downsampling factor(2)
    //If the sample length is odd we just do the first N-1 samples, and add the last sample to the beginning of the next chunk.
    if(numSamples%2 != 0 ){
      //Serial.println("Odd numer of samples!");
      numSamples = numSamples-1; //Only process the first N-1 samples, which is an even chunk.
      raw_data_offset = 1;  // We want to copy the next chunk starting at index 1 so we dont overwrite the last sample.
    }
    
    //Downsample each signal. Input is the 4kHz data stream, output is half the number of samples, into the output buffer. 
    downsampler_x.downsample(acc_x,acc_x_2khz,numSamples);
    downsampler_y.downsample(acc_y,acc_y_2khz,numSamples);
    downsampler_z.downsample(acc_z,acc_z_2khz,numSamples);
    
    //Add one to num samples to get the original num samples.
    if((numSamples+1)%2 != 0 ){
      acc_x[0] = acc_x[numSamples]; //Copy the sample that was not processed to the begining of the buffer.
      acc_y[0] = acc_y[numSamples];
      acc_z[0] = acc_z[numSamples];  
    }

    numSamples = numSamples/2; // Now we're working with the downsampled data.
    //Serial.printf("acc_x_2k: %.15f ",acc_x_2khz[20]);

    convert_to_int(acc_x_2khz,acc_x_int,numSamples,IMU.getAccelBiasX_mss(),IMU.getAccelScaleFactor(),IMU.getAccelScaleFactorX());
    convert_to_int(acc_y_2khz,acc_y_int,numSamples,IMU.getAccelBiasY_mss(),IMU.getAccelScaleFactor(),IMU.getAccelScaleFactorY());
    convert_to_int(acc_z_2khz,acc_z_int,numSamples,IMU.getAccelBiasZ_mss(),IMU.getAccelScaleFactor(),IMU.getAccelScaleFactorZ());

    //Serial.printf("acc_x_int: %d",acc_x_int[20]);
    
    
    //Pack the data into x,y,z for writing to sd card.
    int count=0;
    for(int i=buffer_index; i< buffer_index+numSamples; i++){
      //Serial.printf("x[%d]: %d\n",i,acc_x_int[i]);

      sdBuffer[i*3] = acc_x_int[count];
      sdBuffer[(i*3)+1] = acc_y_int[count];
      sdBuffer[(i*3)+2] = acc_z_int[count];
      count++;
    }
    // long start = millis();
    // //Send the data to the Gateway. numSamples will always be between 40 and 42, inclusive. 
    // if(numSamples>40){
    //   Serial.printf("Sending %d samples.\n",numSamples);
    // long start = millis();
    // //BLE_Stack.sendData(sdBuffer,40*6);
    // Serial.printf("time ble: %d\n",millis()-start);
    //   BLE_Stack.sendData(&sdBuffer[buffer_index],40*6);//Send up to 40 samples in one packet.
    //   uint8_t overflow = numSamples-40; //Number of extra samples to deal with.
    //   BLE_Stack.sendData(&sdBuffer[buffer_index+(40*6)], overflow*6);
    // }
    // else{
    //   //There is exactly 40 samples, send in one go.
    //   BLE_Stack.sendData(&sdBuffer[buffer_index], 40*6);
    // }
    // Serial.printf("BT send time: %d",millis()-start);
    buffer_index += numSamples; 
    Serial.printf("buff idx: %d, fifoSize: %d, offset:%d \n",buffer_index,numSamples,raw_data_offset);
    Serial.flush();

    //Serial.flush();
  }

  if(buffer_index>73){


    //Serial.flush();
    

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
