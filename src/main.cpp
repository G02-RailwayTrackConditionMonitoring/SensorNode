#include <Arduino.h>
#include "NodeBLE.h"

//PIN_BUTTON1
#define BUTTON_PIN  PIN_BUTTON1

void setup(){

  delay(5000);

  Serial.begin(9600);
  Serial.println("BLE Throughput Test");

  pinMode(LED_BLUE,OUTPUT);
  pinMode(BUTTON_PIN,INPUT_PULLUP);

  Serial.println("Starting BLE...");
  
  BLE_Stack.startBLE("G02_B");
  

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

        Bluefruit.printInfo();

        BLE_Stack.runBenchmark4(18000);
        BLE_Stack.sendTelemetry("telem test",sizeof("telem_test"));

        delay(5000);

      }

    }
    delay(500);
    // put your main code here, to run repeatedly:
  }
