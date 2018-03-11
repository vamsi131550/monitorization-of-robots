/**
 * Simple Read/ Blender/Python
 * 
 * Read data from the serial port and turn ON the onboard LED if
 * the character received is an 'a'
 * For the Wiring boards v1 the on-board LED is on pin 48, 
 * on Wiring S the on-board LED is on pin 15.
 */

int data; // to read the char
int WLED =13;
void setup() {
  Serial.begin(9600);
  pinMode(WLED, OUTPUT);
}

void loop() {
  if(Serial.available()) {      // if data available
    data = Serial.read();       // read data
    if(data == 'H') {           // if value read is character 'a'
      digitalWrite(WLED, HIGH); // turn ON the onboard LED
    } else {
      digitalWrite(WLED, LOW);  // if not turn it OFF
    }
  }
  
}
 
