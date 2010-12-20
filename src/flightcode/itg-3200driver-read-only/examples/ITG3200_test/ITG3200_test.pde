// ITG-3200_test
// by Filipe Vieira - 2010
// Simple test of gyro sensors output using default settings.

#include <Wire.h>
#include "ITG3200.h"

ITG3200 gyro;
double  x,y,z,temperature;

void setup(void) {
  Wire.begin();
  Serial.begin(9600);

  gyro.init();   
 }

void loop(void) {
    while (gyro.isRawDataReady()) {
      gyro.readTemp(&temperature);
      gyro.readGyro(&x,&y,&z);
      Serial.print("X:");
      Serial.print(x);
      Serial.print("  Y:");
      Serial.print(y);
      Serial.print("  Z:");
      Serial.print(z);
      Serial.print("  T:");
      Serial.println(temperature);
    } 
}



