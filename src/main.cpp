#include <Arduino.h>
#include "NodeBLE.h"

#ifdef NODE_1
#define BUTTON_PIN  9
#elif NODE_2
#define BUTTON_PIN  PIN_BUTTON1
#else
#define BUTTON_PIN PIN_BUTTON1
#endif





void setup(){

  delay(5000);

  Serial.begin(9600);
  Serial.println("BLE Throughput Test");

  pinMode(LED_BLUE,OUTPUT);
  pinMode(BUTTON_PIN,INPUT_PULLUP);

  Serial.println("Starting BLE...");
  
  #ifdef NODE_1
  BLE_Stack.startBLE("G02_A");
  #elif NODE_2
  BLE_Stack.startBLE("G02_B");
  #endif

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
