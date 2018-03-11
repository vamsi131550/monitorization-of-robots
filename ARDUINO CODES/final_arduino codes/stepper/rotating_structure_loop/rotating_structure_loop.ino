 
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

const int jk = 200;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(jk, 8,9,10,11);



void setup() {
  // set the speed at 60 rpm:
  myStepper.setSpeed(60);
  // initialize the serial port:
  Serial.begin(9600);

   Serial.println("clockwise");
  myStepper.step(step_degree(360));
   Serial.print(step_degree(360));
  
  delay(200);

  myStepper.step(step_degree(360));
  Serial.print(step_degree(360));
   delay(200);

    Serial.println("clockwise");
  myStepper.step(step_degree(360));
   Serial.print(step_degree(360));
  
  delay(200);
  

 

}


void loop() {
  
 

//int a =90;
  // step one revolution  in one direction:

   

  // step one revolution in the other direction:
 // Serial.println("counterclockwise");
  myStepper.step(-jk);
  delay(500);
}


int step_degree(float desired_degree){
    int degree = 360/desired_degree;
float stepsPerRevolution = jk/degree ;
    return (stepsPerRevolution);
    
    }

