#include "itg3200.cpp"
#include "bma180.cpp"

char genStr[512];

//int main(void)
void setup() {
    Serial.begin(9600);
    Wire.begin();
    initGyro();
    BMA180 myAccel;
}

void loop() {
//    while (true) {
        readGyro();
//        myAccel.Poll();
//        sprintf(genStr, "AX: %d   AY: %d   AZ: %d", myAccel.Get(0), myAccel.Get(1), myAccel.Get(2));
//        Serial.println(genStr);
        delay(100);
//    }

//    return 0;
}

