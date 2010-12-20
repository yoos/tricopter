// zoomkat 10-4-10 serial servo test
// type servo position 0 to 180 in serial monitor
// for writeMicroseconds, use a value like 1500
// for 0019 and later

String readString;
#include <Servo.h>
Servo myservo;  // create servo object to control a servo

void setup() {
  Serial.begin(9600);
  myservo.attach(9);
}

void loop() {

  while (Serial.available()) {
    delay(10);  
    if (Serial.available() >0) {
	char c = Serial.read();  //gets one byte from serial buffer
	readString += c;  //makes the string readString
    }
  }

  if (readString.length() >0) {
    Serial.println(readString);
    int n;
    char carray[6];
    readString.toCharArray(carray, sizeof(carray));
    n = atoi(carray);
    myservo.writeMicroseconds(n);
    //myservo.write(n);
    readString="";
  }
}

