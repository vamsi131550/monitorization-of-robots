
/*
 Stepper Motor Control - one revolution

 This program drives a unipolar or bipolar stepper motor.
 The motor is attached to digital pins 8 - 11 of the Arduino.

 The motor should revolve one revolution in one direction, then
 one revolution in the other direction.


 Created 11 Mar. 2007
 Modified 30 Nov. 2009
 by Tom Igoe

 */

#include <Stepper.h>
#include<SoftwareSerial.h>
SoftwareSerial XBee (2,3);


const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8,9,10,11);

float step_degree(float desired_degree)
{
  
    float degree = 360/desired_degree;
    
float jk = stepsPerRevolution/degree ;
    return (jk);
    
    }

void setup() {
  
  Serial.begin(9600);
  XBee.begin(9600);
  myStepper.setSpeed(30); // set the speed at 30 rpm:
 
  
}

void loop() {


   if (XBee.available() > 0)
  {

    int data =XBee.read();
    Serial.println(char(data));
    
    if(data == '1')
    {
        Serial.println(char(data));
        Serial.println("DATA RECEIVED");
        myStepper.step(step_degree(90));
        delay(200);
        }
}
}



