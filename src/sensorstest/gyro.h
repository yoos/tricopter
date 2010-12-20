#include <Wire.cpp> // I2C library, gyroscope

#define GYRO_ADDR 0x69 // gyro address, binary = 11101001
#define SMPLRT_DIV 0x15
#define DLPF_FS 0x16
#define INT_CFG 0x17
#define PWR_MGM 0x3E

//initializes the gyroscope
void initGyro()
{
	/*****************************************
	*	ITG 3200
	*	power management set to:
	*	clock select = internal oscillator
	* 		no reset, no sleep mode
	*		no standby mode
	*	sample rate to = 3Hz
	*	parameter to +/- 2000 degrees/sec
	*	low pass filter = 5Hz
	*	no interrupt
	******************************************/
	Wire.beginTransmission(GYRO_ADDR);
	Wire.send(PWR_MGM);
	Wire.send(0x00);
	Wire.endTransmission();

	Wire.beginTransmission(GYRO_ADDR);
	Wire.send(SMPLRT_DIV);
	Wire.send(0xFF); // EB, 50, 80, 7F, DE, 23, 20, FF
	Wire.endTransmission();

	Wire.beginTransmission(GYRO_ADDR);
	Wire.send(DLPF_FS);
	Wire.send(0x1E); // +/- 2000 dgrs/sec, 1KHz, 1E, 19
	Wire.endTransmission();

	Wire.beginTransmission(GYRO_ADDR);
	Wire.send(INT_CFG);
	Wire.send(0x00);
	Wire.endTransmission();
}
void getGyroscopeData()
{
	int gyro = 0;
	/**************************************
		Gyro ITG-3200 I2C
		registers:
		x axis MSB = 1D, x axis LSB = 1E
		y axis MSB = 1F, y axis LSB = 20
		z axis MSB = 21, z axis LSB = 22
	**************************************/
	// Arduino Wire library (I2C)
	Wire.beginTransmission(GYRO_ADDR);
	Wire.send(0x1D); // MSB x axis
	Wire.endTransmission();
	Wire.requestFrom(GYRO_ADDR, 1); // one byte
	byte msb = 0;
	byte lsb = 0;
	while(!Wire.available())
	{
		// wait for data to be available
	};
		msb = Wire.receive();

	Wire.beginTransmission(GYRO_ADDR);
	Wire.send(0x1E); // LSB x axis
	Wire.endTransmission();
	Wire.requestFrom(GYRO_ADDR, 1); // one byte

	while(!Wire.available())
	{
		// wait for data to be available
	};
		 lsb = Wire.receive();

	// calculate total x axis
	gyro = (( msb << 8) | lsb);
	Serial.print(" gyroX= "); Serial.print(gyro);

	// clear variables
	msb = 0;
	lsb = 0;
	gyro = 0;

	Wire.beginTransmission(GYRO_ADDR);
	Wire.send(0x1F); // MSB y axis
	Wire.endTransmission();
	Wire.requestFrom(GYRO_ADDR, 1); // one byte

	while(!Wire.available())
	{
		// wait for data to be available
	};
		 msb = Wire.receive();

	Wire.beginTransmission(GYRO_ADDR);
	Wire.send(0x20); // LSB y axis
	Wire.endTransmission();
	Wire.requestFrom(GYRO_ADDR, 1); // one byte

	while(!Wire.available())
	{
		// wait!
	};
		 lsb = Wire.receive();

	// calculate total y axis
	gyro = (( msb << 8) | lsb);
	Serial.print(" gyroX= "); Serial.print(gyro);

	// clear variables
	msb = 0;
	lsb = 0;
	gyro = 0;

	Wire.beginTransmission(GYRO_ADDR);
	Wire.send(0x21); // MSB z axis
	Wire.endTransmission();
	Wire.requestFrom(GYRO_ADDR, 1); // one byte

	while(!Wire.available())
	{
		// wait...
	};
		 msb = Wire.receive();

	Wire.beginTransmission(GYRO_ADDR);
	Wire.send(0x22); // LSB z axis
	Wire.endTransmission();
	Wire.requestFrom(GYRO_ADDR, 1); // one byte

	while(!Wire.available())
	{
		// ...
	};
		 lsb = Wire.receive();

	// calculate z axis
	gyro = (( msb << 8) | lsb);
	Serial.print(" gyroZ= "); Serial.println(gyro);

}

