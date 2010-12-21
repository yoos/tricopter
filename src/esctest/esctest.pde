// zoomkat 10-4-10 serial servo test
// type servo position 0 to 180 in serial monitor
// for writeMicroseconds, use a value like 1500
// for 0019 and later

#include <Servo.cpp>

char input[1024];
Servo myServo;  // create servo object to control a servo

void setup() {
    Serial.begin(9600);
    myServo.attach(9);
}

void loop() {
    int i = 0;
    while (Serial.available()) {
        delay(10);
        if (Serial.available() > 0) {
    	    char c = Serial.read();  //gets one byte from serial buffer
    	    input[i] = c;  //makes the string input
            i++;
        }
    }

    if (i > 0) {
        Serial.println(input);
//      myServo.writeMicroseconds(atoi(input));
        myServo.write(atoi(input));
        for (int i; i<1024; i++) {
            input[i] = 0;
        }
    }
}

