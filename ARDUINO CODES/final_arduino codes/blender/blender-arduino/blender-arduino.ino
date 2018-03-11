
char data; // to read the char

int led =13;

void setup() {
  Serial.begin(9600);
 pinMode(13,OUTPUT);
 
}

void loop() {

  if(Serial.available()>0) {      // if data available
    data = Serial.read();       // read data
    Serial.println(data);
    if(data == '1')
    {
      digitalWrite(13,HIGH);
    }
    else if(data == '0')
    {
        digitalWrite(13,LOW);
    }
  }
 // delay(100); // wait 100ms for next read
}
 
