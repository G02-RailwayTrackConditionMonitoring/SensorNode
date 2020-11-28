#include <Arduino.h>
#include "NodeBLE.h"

  

void setup(){

  delay(5000);

  Serial.begin(9600);
  Serial.println("BLE Throughput Test");

  pinMode(LED_BLUE,OUTPUT);
  pinMode(PIN_BUTTON1,INPUT_PULLUP);

  Serial.println("Starting BLE...");
  
  BLE_Stack.startBLE("G02");
  

}
  //Loop
void loop(){

    Serial.println("Itsy Bitsy Test");
    digitalToggle(LED_BLUE);
  
    if(digitalRead(PIN_BUTTON1) == LOW){

      if(!BLE_Stack.isConnected()){
        Serial.println("Starting Advertising");
        BLE_Stack.startAdvertising();
      }
      else{
        
        int8_t rssi = BLE_Stack.getRSSI();
        uint8_t phy = BLE_Stack.getPHY();
        uint16_t mtu = BLE_Stack.getMTU();

        char message[60];
        snprintf(message,60,"Msg from sensor node: rssi:%d phy:%d mtu:%d\n",rssi,phy,mtu);
        
        Serial.printf("Sending the messge: %s",message);
        BLE_Stack.sendData(message,strlen(message));
      }


    }
    delay(1500);
    // put your main code here, to run repeatedly:
  }
