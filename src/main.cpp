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
#define IMU_SPI_MOSI_PIN   7 
#define IMU_SPI_MISO_PIN   11 
#define IMU_SPI_SCK_PIN    9 
#define IMU_SPI_CS_PIN     10
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

uint16_t numSamples;
size_t fifoSize;

//Terminology: IMU Frame-> A chunk of samples read from the imu (one fifo's worth).
//             SPI_NUM_BLOCKS-> The number of samples in one frame

//These hold the 4kHz imu data streams. Fifo holds up to 85 samples, and we need one extra space for if we have odd number of samples.
uint8_t raw_data_offset = 0;
float acc_x[SPI_NUM_BLOCKS];
float acc_y[SPI_NUM_BLOCKS];
float acc_z[SPI_NUM_BLOCKS];

//Holds the downsampled data streams, so we only need half the amount of space (43 samples).
float acc_x_2khz[SPI_NUM_BLOCKS/2];
float acc_y_2khz[SPI_NUM_BLOCKS/2];
float acc_z_2khz[SPI_NUM_BLOCKS/2];

//Holds the downsampled data streams but in int form which is needed for transmitting and saving to SD card.
int16_t acc_x_int[SPI_NUM_BLOCKS/2];
int16_t acc_y_int[SPI_NUM_BLOCKS/2];
int16_t acc_z_int[SPI_NUM_BLOCKS/2];

//Holds the packed data (x,y,z).
uint8_t mergedData[SPI_NUM_BLOCKS/2*6]; 

typedef enum{
  BUFF_A,
  BUFF_B
} BufferSelection_t;
#define SD_BUFFER_SIZE  512
//Double buffer for sd card.
BufferSelection_t   sdBuff_selection = BUFF_A; 
uint16_t            sdBuff_idx=0;
uint8_t             sdBufferA[SD_BUFFER_SIZE] = {0};
uint8_t             sdBufferB[SD_BUFFER_SIZE] = {0};

uint8_t imu_num_frames=0;
uint8_t imu_buffer[(SPI_NUM_BLOCKS*SPI_NUM_FIFO*SPI_BYTES_PER_BLOCK)];

uint8_t mode =1;
// //Double buffering for BLE transmit.
// int16_t bleBuffA[240]; // BLE can transmit 244 bytes, so 240/6 = 40 samples at a time.
// int16_t bleBuffB[240];
// uint8_t bleBuffIdxA = 0;
// uint8_t bleBuffIdxB =0;

//We need a downsampler for each signal.
Downsampler downsampler_x(2,anitaliasing_filter,FILTER_TAP_NUM,80);
Downsampler downsampler_y(2,anitaliasing_filter,FILTER_TAP_NUM,80);
Downsampler downsampler_z(2,anitaliasing_filter,FILTER_TAP_NUM,80);

void  convert_to_int(float* in, int16_t* out, int num_samples, float bias, float scale, float axis_scale);
void mergeSampleStreams(uint8_t* outBuff, int16_t* in1, int16_t* in2, int16_t* in3, uint16_t numSamples);
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
  pinMode(LED_BLUE,OUTPUT);
  pinMode(BUTTON_PIN,INPUT_PULLUP);

  Serial.println("Starting BLE...");

  #ifdef NODE_1
  BLE_Stack.startBLE("G02_A");
  #elif NODE_2
  BLE_Stack.startBLE("G02_B");
  #endif
  
  while(!BLE_Stack.isConnected()){
    BLE_Stack.startAdvertising();
    Serial.println("Connecting to BLE.");
    delay(5000);
  } // Wait to be connected.
  delay(10000);//Wait for connectino to be good.
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
  
  delay(2000);
  // IMU_SPI.begin();
  IMU_SPI.setupReccuringTransfer();
  IMU.init(); // start communication with IMU   
  IMU.enableAccelFifo(); // enabling the FIFO to record just the accelerometers

  IMU_SPI.startRecuringTransfers();
  Serial.println("Starting test");
  Serial.flush();

  //mode = LOGGING;// Comment this out to empty the main loop.
}
  //Loop
void loop(){

if(mode == LOGGING){

    //Check if we have any frames available
    if((imu_num_frames = IMU_SPI.getRxBuffIndex())){

      // digitalWrite(PIN_A0,HIGH);
      Serial.printf("reading imu:");
      Serial.flush();
      
      //Copy the data from the SPI dma buffer, to our local buffer so we can reset the spi dma buffer.
      IMU_SPI.getRxData(imu_buffer,imu_num_frames,0);
      Serial.printf("%d frames. \n\r",imu_num_frames);
      
      for(int frameNum=0; frameNum<imu_num_frames; frameNum++){
        
        numSamples = 80;//We always read 80 samples.

        //Parse one frame into the sample values. This extracts the samples and converts to floating point.
        IMU.readFifo(&imu_buffer[frameNum*SPI_NUM_BLOCKS*SPI_BYTES_PER_BLOCK],&acc_x[0],&acc_y[0],&acc_z[0],numSamples);
        // Serial.printf("x0:%f y0:%f z0:%f\r\n",acc_x[0],acc_y[0],acc_z[0]);
        //Downsample each signal. Input is the 4kHz data stream, output is half the number of samples, into the output buffer. 
        downsampler_x.downsample(acc_x,acc_x_2khz,numSamples);
        downsampler_y.downsample(acc_y,acc_y_2khz,numSamples);
        downsampler_z.downsample(acc_z,acc_z_2khz,numSamples);

        numSamples = numSamples/2; // Now we're working with the downsampled data.
        // Serial.printf("down x0:%f y0:%f z0:%f\r\n",acc_x_2khz[0],acc_y_2khz[0],acc_z_2khz[0]);

        //Convert our samples back to integer values. This is done since int16 is half the space of float32.
        convert_to_int(acc_x_2khz,acc_x_int,numSamples,IMU.getAccelBiasX_mss(),IMU.getAccelScaleFactor(),IMU.getAccelScaleFactorX());
        convert_to_int(acc_y_2khz,acc_y_int,numSamples,IMU.getAccelBiasY_mss(),IMU.getAccelScaleFactor(),IMU.getAccelScaleFactorY());
        convert_to_int(acc_z_2khz,acc_z_int,numSamples,IMU.getAccelBiasZ_mss(),IMU.getAccelScaleFactor(),IMU.getAccelScaleFactorZ());
        // Serial.printf("int x0:%d y0:%d z0:%d\r\n",acc_x_int[0],acc_y_int[0],acc_z_int[0]);

        //Pack the data into x,y,z for writing to sd card.
        mergeSampleStreams(mergedData, acc_x_int, acc_y_int, acc_z_int,numSamples);
        // Serial.printf("merged %x %x %x %x %x %x\r\n",mergedData[0],mergedData[1],mergedData[2],mergedData[3],mergedData[4],mergedData[5]);

        //Now put  data into sd buffer and save to card if full.
        //We can fit all the samples in the current buffer.
        Serial.printf("sd indx:%d\r\n",sdBuff_idx);
        if(numSamples*6 + sdBuff_idx<SD_BUFFER_SIZE){
          // Serial.println("single buff");
          if(sdBuff_selection == BUFF_A){
            memcpy(&sdBufferA[sdBuff_idx],&mergedData[0],numSamples*6);
          }
          else if(sdBuff_selection == BUFF_B){
            memcpy(&sdBufferB[sdBuff_idx],&mergedData[0],numSamples*6);
          }
          // Serial.printf("A %x %x %x %x %x %x\r\n",sdBufferA[sdBuff_idx+0],sdBufferA[sdBuff_idx+1],sdBufferA[sdBuff_idx+2],sdBufferA[sdBuff_idx+3],sdBufferA[sdBuff_idx+4],sdBufferA[sdBuff_idx+5]);
          // Serial.printf("B %x %x %x %x %x %x\r\n",sdBufferB[sdBuff_idx+0],sdBufferB[sdBuff_idx+1],sdBufferB[sdBuff_idx+2],sdBufferB[sdBuff_idx+3],sdBufferB[sdBuff_idx+4],sdBufferB[sdBuff_idx+5]);
          sdBuff_idx += (numSamples*6);
        }
        else{
          // Serial.println("cross buff");
          //We split the 40 samples across the buffers.
          uint16_t bytesInPrevBuff = SD_BUFFER_SIZE - sdBuff_idx;
          uint16_t bytesLeft = (numSamples*6) - bytesInPrevBuff;

          //Copy as much data as fits, then switch buffers.
          //Write the full buffer to the card.
          if(sdBuff_selection == BUFF_A){
              memcpy(&sdBufferA[sdBuff_idx],mergedData,bytesInPrevBuff);
              sdBuff_selection = BUFF_B; 
              sdBuff_idx = 0;

              file.write(sdBufferA,SD_BUFFER_SIZE);
              memset(sdBufferA,0,SD_BUFFER_SIZE);
          }
          else if(sdBuff_selection == BUFF_B){
              memcpy(&sdBufferB[sdBuff_idx],mergedData,bytesInPrevBuff);
              sdBuff_selection = BUFF_A; 
              sdBuff_idx = 0;

              file.write(sdBufferB,SD_BUFFER_SIZE);
              memset(sdBufferB,0,SD_BUFFER_SIZE);
          }

          //Put rest of data into other buffer...
          if(sdBuff_selection == BUFF_A){
              memcpy(&sdBufferA[sdBuff_idx],&mergedData[bytesInPrevBuff],bytesLeft);
          }
          else if(sdBuff_selection == BUFF_B){
              memcpy(&sdBufferB[sdBuff_idx],&mergedData[bytesInPrevBuff],bytesLeft);
          }
          sdBuff_idx += bytesLeft;

        }
        
        //Send data over BLE.
        BLE_Stack.sendData(mergedData,numSamples*6);

        Serial.flush();
      }

      
      // if(numSamples >80) numSamples = 80; //Just drop samples. If we don't then we just fall behind and drop more.

      //If we have a left over sample from last read, we need to increase num samples.
      // if(raw_data_offset == 1){
      //   numSamples = numSamples+1; // Add from the last one.
      //   raw_data_offset =0; // reset
      // }
      // //We must process chunks that have length that is a multiple of the downsampling factor(2)
      // //If the sample length is odd we just do the first N-1 samples, and add the last sample to the beginning of the next chunk.
      // if(numSamples%2 != 0 ){
      //   //Serial.println("Odd numer of samples!");
      //   numSamples = numSamples-1; //Only process the first N-1 samples, which is an even chunk.
      //   raw_data_offset = 1;  // We want to copy the next chunk starting at index 1 so we dont overwrite the last sample.
      // }
      
        // //Downsample each signal. Input is the 4kHz data stream, output is half the number of samples, into the output buffer. 
        // downsampler_x.downsample(acc_x,acc_x_2khz,numSamples);
        // downsampler_y.downsample(acc_y,acc_y_2khz,numSamples);
        // downsampler_z.downsample(acc_z,acc_z_2khz,numSamples);
      
      //Add one to num samples to get the original num samples.
      // if((numSamples+1)%2 != 0 ){
      //   acc_x[0] = acc_x[numSamples]; //Copy the sample that was not processed to the begining of the buffer.
      //   acc_y[0] = acc_y[numSamples];
      //   acc_z[0] = acc_z[numSamples];  
      // }

      // numSamples = numSamples/2; // Now we're working with the downsampled data.
      // //Serial.printf("acc_x_2k: %.15f ",acc_x_2khz[20]);

      // convert_to_int(acc_x_2khz,acc_x_int,numSamples,IMU.getAccelBiasX_mss(),IMU.getAccelScaleFactor(),IMU.getAccelScaleFactorX());
      // convert_to_int(acc_y_2khz,acc_y_int,numSamples,IMU.getAccelBiasY_mss(),IMU.getAccelScaleFactor(),IMU.getAccelScaleFactorY());
      // convert_to_int(acc_z_2khz,acc_z_int,numSamples,IMU.getAccelBiasZ_mss(),IMU.getAccelScaleFactor(),IMU.getAccelScaleFactorZ());

      // //Serial.printf("acc_x_int: %d",acc_x_int[20]);
      
      
      //Pack the data into x,y,z for writing to sd card.
      // int count=0;
      // for(int i=sd_; i< sd_+numSamples; i++){
      //   //Serial.printf("x[%d]: %d\n",i,acc_x_int[i]);

      //   sdBuffer[i*3] = acc_x_int[count];
      //   sdBuffer[(i*3)+1] = acc_y_int[count];
      //   sdBuffer[(i*3)+2] = acc_z_int[count];
      //   count++;
      // }
      // long start = millis();
      // //Send the data to the Gateway. numSamples will always be between 40 and 42, inclusive. 
      // if(numSamples>40){
      //   Serial.printf("Sending %d samples.\n",numSamples);
      // long start = millis();
      // BLE_Stack.sendData(sdBuffer,40*6);
      // Serial.printf("time ble: %d\n",millis()-start);
      //   BLE_Stack.sendData(&sdBuffer[sd_],40*6);//Send up to 40 samples in one packet.
      //   uint8_t overflow = numSamples-40; //Number of extra samples to deal with.
      //   BLE_Stack.sendData(&sdBuffer[sd_+(40*6)], overflow*6);
      // }
      // else{
      //   //There is exactly 40 samples, send in one go.
      //   BLE_Stack.sendData(&sdBuffer[sd_], 40*6);
      // }
      // Serial.printf("BT send time: %d",millis()-start);
      // sd_ += numSamples; 
      // Serial.printf("buff idx: %d, fifoSize: %d, offset:%d \n\r",sd_,numSamples,raw_data_offset);
      // Serial.flush();
      // digitalWrite(PIN_A0,LOW);

      //Serial.flush();
    }

    // if(sd_>73){

    //   Serial.println("Writing to sd");
    //   Serial.flush();
    //   file.write(sdBuffer,sd_*6);
    //   //file.sync(); // Sounds like this is needed? NO!

    //   sd_ = 0;
    // }

    if(digitalRead(BUTTON_PIN) == LOW ){

      Serial.println("End of test...");
      file.close();
      IMU_SPI.pauseRecurringTransfers();
      IMU.haltSampleAccumulation();
      Serial.flush();
      while(1){};
    }
  }
  digitalToggle(LED_BLUE);
  Serial.println("Test");
  delay(5);
}

void  convert_to_int(float* in, int16_t* out, int num_samples, float bias, float scale, float axis_scale){


    for(int i=0; i< num_samples;i++){

      //this is basically the reverse of the conversion code in MPU9250 readFifo function.
      out[i] = (int16_t)(((in[i]/axis_scale)+bias)/scale);
   
    }

}

void mergeSampleStreams(uint8_t* outBuff, int16_t* in1, int16_t* in2, int16_t* in3, uint16_t numSamples){

    uint16_t count = 0;
    for(int i=0; i<numSamples; i++){
      
      //Get pointers to where each axis sample should go.
      int16_t* x_pos = (int16_t*)&outBuff[i*6];
      int16_t* y_pos = (int16_t*)&outBuff[i*6+2];
      int16_t* z_pos = (int16_t*)&outBuff[i*6+4];

      *x_pos = in1[count];
      *y_pos = in2[count];
      *z_pos = in3[count];

      count++;

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
