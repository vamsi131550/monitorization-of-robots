#include<SoftwareSerial.h>
SoftwareSerial XBee (2,3);

int pushButton = 12;

void setup() {
  Serial.begin(9600);
  XBee.begin(9600);
  // put your setup code here, to run once:
pinMode(pushButton, INPUT);
}

void loop() {

 /* int buttonState = digitalRead(pushButton);
  // print out the state of the button:
 delay(1000);
  if (buttonState == 1)
  {
     String  ab = "amma ";
        
       // Serial.print(char(ab) );
         XBee.println((ab));
         Serial.println((ab) );
         
  }*/

if (Serial.available() > 0) {
                // read the incoming byte:
               int  aa = Serial.read(); 

                // say what you got:
               
                Serial.print(char(aa) );
                 XBee.write(char(aa));
        }
  if (XBee.available() > 0)
  {
    
    int data =XBee.read();
    Serial.println(char(data));
  }
  // put your main code here, to run repeatedly:

}
