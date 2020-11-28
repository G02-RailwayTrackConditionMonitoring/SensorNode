#include <Arduino.h>



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BLUE,OUTPUT);

}

void loop() {
  Serial.println("Itsy Bitsy Test");
  digitalToggle(LED_BLUE);
  delay(1500);
  // put your main code here, to run repeatedly:
}