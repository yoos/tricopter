#ifndef BMA180_H
#define BMA180_H

#include "i2c.h"

#define address 0x40

int x;

void readAccel()
{
  int temp, result;

  temp = 0;

  while(temp != 1)
  {
    Wire.beginTransmission(address);
    Wire.send(0x03);
    Wire.requestFrom(address, 1);
    while(Wire.available())
    {
	temp = Wire.receive() & 0x01;
    }
  }

  Wire.beginTransmission(address);
  Wire.send(0x02);
  Wire.requestFrom(address, 1);
  while(Wire.available())
  {
    temp |= Wire.receive();
    temp = temp >> 2;
  }
  Serial.print("X = ");
  Serial.println(temp);
  result = Wire.endTransmission();
}

void checkResult(int result)
{
  if(result >= 1)
  {
    Serial.print("PROBLEM..... Result code is ");
    Serial.println(result);
  }
  else
  {
    Serial.println("Read/Write success");
  }
}

void initBMA180()
{
  int temp, result, error;

  Wire.beginTransmission(address);
  Wire.send(0x00);
  Wire.requestFrom(address, 1);
  while(Wire.available())
  {
    temp = Wire.receive();
  }
  Serial.print("Id = ");
  Serial.println(temp);
  result = Wire.endTransmission();
  checkResult(result);
  if(result > 0)
  {
    error = 1;
  }
  delay(10);
  if(temp == 3)
  {
    // Connect to the ctrl_reg1 register and set the ee_w bit to enable writing.
    Wire.beginTransmission(address);
    Wire.send(0x0D);
    Wire.send(B0001);
    result = Wire.endTransmission();
    checkResult(result);
    if(result > 0)
    {
	error = 1;
    }
    delay(10);
    // Connect to the bw_tcs register and set the filtering level to 10hz.
    Wire.beginTransmission(address);
    Wire.send(0x20);
    Wire.send(B00001000);
    result = Wire.endTransmission();
    checkResult(result);
    if(result > 0)
    {
	error = 1;
    }
    delay(10);
    // Connect to the offset_lsb1 register and set the range to +- 2.
    Wire.beginTransmission(address);
    Wire.send(0x35);
    Wire.send(B0100);
    result = Wire.endTransmission();
    checkResult(result);
    if(result > 0)
    {
	error = 1;
    }
    delay(10);
  }

  if(error == 0)
  {
    Serial.print("BMA180 Init Successful");
  }
}

void readId()
{
  int temp, result;

  Wire.beginTransmission(address);
  Wire.send(0x00);
  Wire.requestFrom(address, 1);
  while(Wire.available())
  {
    temp = Wire.receive();
  }
  Serial.print("Id = ");
  Serial.println(temp);
  result = Wire.endTransmission();
  checkResult(result);
  delay(10);
}

#endif // BMA180_H

