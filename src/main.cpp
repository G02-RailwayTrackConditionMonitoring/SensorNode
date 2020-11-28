#include <Arduino.h>
#include "NodeBLE.h"

  NodeBLE ble("G02");

void setup(){

  delay(5000);

  Serial.begin(9600);
  Serial.println("BLE Throughput Test");

  pinMode(LED_BLUE,OUTPUT);
  pinMode(PIN_BUTTON1,INPUT_PULLUP);

  Serial.println("Starting BLE...");
  ble.startBLE();
  

}
  //Loop
void loop(){

    Serial.println("Itsy Bitsy Test");
    digitalToggle(LED_BLUE);
  
    if(digitalRead(PIN_BUTTON1) == LOW){
      Serial.println("Starting Advertising");
      ble.startAdvertising();

    }
    delay(1500);
    // put your main code here, to run repeatedly:
  }
