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

    // put your main code here, to run repeatedly:
  }
