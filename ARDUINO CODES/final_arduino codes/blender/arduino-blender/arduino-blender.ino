/** 
 * Photoresistor / Blender/Python

 
 * Reads values from a photoresistor connected to the 
 * analog input pin 0. The value read from the sensor is proportional
 * to the amount of light that hits the sensor surface.
 * The value read is printed through the serial to be monitored
 * in the console or received on Blender
 */
 String val ;
char char1;

void setup()
{
  Serial.begin(9600);      // sets the serial port to 9600
}

void loop() 
{
// read analog input pin 0

  val = " seenu";
   Serial.println(val);// prints the value read from the
                               // sensor in analog pin 0 divided by 4 (range 0-255)
  delay(200);                  // wait 200ms for next reading
  val = " bendra kappa lakshman";
   Serial.println(val);// prints the value read from the
                               // sensor in analog pin 0 divided by 4 (range 0-255)
  delay(300);                  // wait 200ms for next reading
 


;
  
  
}
