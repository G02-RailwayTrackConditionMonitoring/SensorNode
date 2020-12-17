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
  
  BLE_Stack.startBLE("G02_A");
  

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

        BLE_Stack.runBenchmark4(8000);

        delay(5000);
      }

    }
    delay(1500);
    // put your main code here, to run repeatedly:
  }
