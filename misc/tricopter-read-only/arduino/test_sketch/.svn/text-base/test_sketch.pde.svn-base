#include <WProgram.h>
#include <ServoTimer2.h>
#include <Messenger.h>


#define MINCOMMAND 1000
#define MAXCOMMAND 2000
 
ServoTimer2 myservo[2];  
int motor_command[2] = {MINCOMMAND, MINCOMMAND};    // variable to store the servo position 

int MOTOR_PINS[2] = {9, 8};

Messenger message = Messenger();

//void sweepMotor(int start, int stop, int interval) {
//  for (int i = start; i <= stop; i +=  
  
//}

void processInput() {
  int motorNum = message.readInt();
  int motorCmd = message.readInt();
  if (motorNum >= 0 && motorNum <= 1) {
    if (motorCmd == 0) {
      myservo[motorNum].attach(MOTOR_PINS[motorNum]);
      myservo[motorNum].write(MINCOMMAND);
      Serial.print("Attaching Motor ");
      Serial.print(motorNum);
      Serial.println();
    }
    else if (motorCmd == -1) {
      myservo[motorNum].detach();
      Serial.print("Detaching Motor ");
      Serial.print(motorNum);
      Serial.println();
    }
    else if (motorCmd >= MINCOMMAND && motorCmd <= MAXCOMMAND) {
      myservo[motorNum].write(motorCmd);
      Serial.print("Motor ");
      Serial.print(motorNum);
      Serial.print(": ");
      Serial.println(motorCmd);
    }
  }
} 
void setup() 
{ 
  
  Serial.begin(9600);
  
  delay(1000);
  Serial.println("Starting up");
  
} 

 
void loop() 
{ 
  if(Serial.available()) {
    //Serial.println("c");
//    message.process(Serial.read());
//    if (message.available()) {
      //Serial.println("msg");
//      processInput();
//    }
//  }
  
    char serial_in = Serial.read();

    switch (serial_in) {
      case 'q':
        myservo[0].attach(9);
        Serial.println("Attaching Motor 0");
        motor_command[0] = MINCOMMAND;
        break;
      
      case 'a':
        myservo[0].detach();
        Serial.println("Detaching Motor 0");
        break;
    
      case 'w':
        motor_command[0] += 1;
        break;
    
      case 's':
        motor_command[0] -= 1;
        break;
    
      case 'e':
        motor_command[0] += 10;
        break;
    
      case 'd':
        motor_command[0] -= 10;
        break;
     
      case 'r':
        motor_command[0] += 100;
        break;
    
      case 'f':
        motor_command[0] -= 100;
        break;
        
      case 'p':
        myservo[1].attach(8);
        Serial.println("Attaching Motor 1");
        motor_command[1] = MINCOMMAND;
        break;
      
      case 'l':
        myservo[1].detach();
        Serial.println("Detaching Motor 1");
        break;
    
      case 'o':
        motor_command[1] += 1;
        break;
    
      case 'k':
        motor_command[1] -= 1;
        break;
    
      case 'i':
        motor_command[1] += 10;
        break;
    
      case 'j':
        motor_command[1] -= 10;
        break;
     
      case 'u':
        motor_command[1] += 100;
        break;
    
      case 'h':
        motor_command[1] -= 100;
        break;
        
/*      case 'z':
        Serial.println("Starting Sweep");
        for(int i = 1000; i <= 1500; i+=10) {
          motor_command[0] = i;
          myservo.write(motor_command[0]);
          delay(400);
          if (i%100 == 0) {
            Serial.println(motor_command[0]);
          }
        }      
        Serial.println("Sweep Done");
        break;
*/
    }
    if (motor_command[0] != myservo[0].read()) {
        myservo[0].write(motor_command[0]);
        Serial.print("CMD 0: ");
        Serial.println(motor_command[0]);
      
    }
    if (motor_command[1] != myservo[1].read()) {
        myservo[1].write(motor_command[1]);
        Serial.print("CMD 1: ");
        Serial.println(motor_command[1]);
      
    }
  }
  
} 
