#include <Arduino.h>
#include "NodeBLE.h"

#define BUTTON_PIN  9

void setup(){

  delay(5000);

  Serial.begin(9600);
  Serial.println("BLE Throughput Test");

  pinMode(LED_BLUE,OUTPUT);
  pinMode(BUTTON_PIN,INPUT_PULLUP);

  Serial.println("Starting BLE...");
  
  BLE_Stack.startBLE("G02");
  

}
  //Loop
void loop(){

    Serial.println("Itsy Bitsy Test");
    digitalToggle(LED_BLUE);
  
    if(digitalRead(BUTTON_PIN) == LOW){

      if(!BLE_Stack.isConnected()){
        Serial.println("Starting Advertising");
        BLE_Stack.startAdvertising();
      }
      else{

        Serial.println("Starting Benchmark");
                
        uint16_t connHandle = Bluefruit.connHandle();
        BLEConnection* connection = Bluefruit.Connection(connHandle);

        Bluefruit.printInfo();

        //Try and set the Data length to be 236. This shoud go to 251 in theory.
        ble_gap_data_length_params_t dl_params;
        dl_params.max_rx_octets = 236;
        dl_params.max_tx_octets = 236;
        dl_params.max_rx_time_us = BLE_GAP_DATA_LENGTH_AUTO;  //Not sure what time this is representing exactly?
        dl_params.max_tx_time_us = BLE_GAP_DATA_LENGTH_AUTO;

        ble_gap_data_length_limitation_t limits;

        //This should set the data length parameters and return what the values are limited to. 
        bool res = connection->requestDataLengthUpdate(&dl_params,&limits);
        Serial.printf("DL exch, max_rx: %d\n",limits.rx_payload_limited_octets);
        Serial.printf("DL exch, max_tx: %d\n",limits.tx_payload_limited_octets); //This seem to be the payload is too large by this amount, not that you can set to this value.
        Serial.printf("DL exch, time: %d us\n",limits.tx_rx_time_limited_us);

        Serial.printf("DL exchange: %d\n",res); //Print the error if there was.
        
        //Try and set the MTU now to the value based on the limitations. Should be minus 3?
        res =  connection->requestMtuExchange(dl_params.max_rx_octets-limits.rx_payload_limited_octets);

        Serial.printf("MTU exchange: %d\n",res);//Print the error if there was.

        Bluefruit.printInfo();

        BLE_Stack.runBenchmark4(2000);
        delay(5000);
      }

    }
    delay(1500);
    // put your main code here, to run repeatedly:
  }
