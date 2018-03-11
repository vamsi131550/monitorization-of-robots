
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

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

int step_degree(float desired_degree)
{
  
    int degree = 360/desired_degree;
    
float jk = stepsPerRevolution/degree ;
    return (jk);
    
    }

void setup() {
  // set the speed at 60 rpm:
  myStepper.setSpeed(30);
  // initialize the serial port:
  Serial.begin(9600);
}

void loop() {
  // step one revolution  in one direction:
  Serial.println("clockwise");
  myStepper.step(step_degree(-360));
//  myStepper.step(jk);
  delay(200);

   Serial.println("clockwise");
  myStepper.step(step_degree(-360));
  delay(200);

  
   Serial.println("clockwise");
  myStepper.step(step_degree(-360));
  delay(200);
  
  
  

  // step one revolution in the other direction:
 // Serial.println("anticlockwise");
  //myStepper.step(stepsPerRevolution);
  //delay(500);
}



