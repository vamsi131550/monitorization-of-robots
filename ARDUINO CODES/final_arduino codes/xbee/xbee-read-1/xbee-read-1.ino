#include <SoftwareSerial.h>
#include <XBee.h>
SoftwareSerial XBee(2, 3); // RX, TX
int data ;

void setup() {
  Serial.begin(9600);
  XBee.begin(9600);
  
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  while(XBee.available())
  data =XBee.read();
  Serial.println(data);

}
