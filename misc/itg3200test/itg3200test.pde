/*
    ITG-3200 Test Code
	April 28, 2010
	by: Jim Lindblom
	License: Creative Commons Attribution Share-Alike 3.0
	http://creativecommons.org/licenses/by-sa/3.0
	
	ITG-3200 Triple-axis Digital Output Gyroscope test code. Streams the three axis output of the gyro.
	Provides options to average the readings, and modify the sample rate, DLPF, and power management registers.
	
	Tested on a 3.3V 8MHz Arduino Pro
	10kOhm pull-ups on I2C lines.
	VDD & VIO = 3.3V
	SDA -> A4 (PC4)
	SCL -> A5 (PC5)
	INT -> D2 (PB2) (or no connection, not used here)
	CLK -> GND
*/

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include "i2c.h"
#include "ITG-3200.h"

#define FOSC 8000000
#define ITG3200_R 0xD3	// ADD pin is pulled high
#define ITG3200_W 0xD2	// So address is 0x69

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

///============Function Prototypes=========/////////////////
char ITG3200Read(unsigned char address);
void ITG3200Write(unsigned char address, unsigned char data);
void ITG3200ViewRegisters(void);
int checkInterrupt(void);
void getITG3200(int average);
void ITG3200BlockRead(unsigned char address);

///============General Use Prototypes=====//////////////////
void ioinit(void);
void UART_Init(unsigned int ubrr);
static int uart_putchar(char c, FILE *stream);
void put_char(unsigned char byte);
uint8_t uart_getchar(void);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
void delay_ms(uint16_t x);

int main(void)
{
	char temp;
	unsigned int i = 0;
	int average = 1;
	unsigned short sample = 128;
	unsigned short dlpf = 0;
	unsigned char power = 0;
	long int count = 0;
	
	ioinit();
	i2cInit();
	delay_ms(100);
	
	ITG3200Write(PWR_M, 0x80);	// Reset to defaults
	ITG3200Write(SMPL, 0x00);	// SMLPRT_DIV = 0
	ITG3200Write(DLPF, 0x18);	// DLPF_CFG = 0, FS_SEL = 3
	ITG3200Write(INT_C, 0x05);	// Generate interrupt when device is ready or raw data ready
	ITG3200Write(PWR_M, 0x00);
	
	delay_ms(1000);
	
	while(1)
	{
		printf("\nITG-3200 Configuration Menu\n");
		printf("---------------------------\n");
		printf("1) Stream Values\n");
		printf("2) Change average value (currently %d)\n", average);
		printf("3) Change Sample Rate Divider (currently %d)\n", ITG3200Read(SMPL));
		printf("4) Adjust digital low pass filter (currently %d)\n", (ITG3200Read(DLPF)&0x07));
		printf("5) Adjust Power Management Register (currently 0x%x)\n", ITG3200Read(PWR_M));
		printf("6) View all ITG-3200 Registers\n");
		temp = uart_getchar();
		
		if (temp == '1')
		{
			count = 0;
			printf("\nStreaming, press any key to stop...\n\n");
			printf("Count	 x	 y	 z\n");
			printf("-----	---	---	---\n");
			while(!(UCSR0A & (1<<RXC0)))
			{
				printf("%ld:", count);
				getITG3200(average);
				count++;
			}
			temp = UDR0;
		}
		else if (temp == '2')
		{
			i = 0;
			average = 0;
			printf("\nType the new average value followed by the enter key...\n");
			while (1)
			{
				temp = uart_getchar();
				if (temp == 0x0D)
					break;
				else if ((temp>=0x30)&&(temp<=0x39))
				{
					temp -= 0x30;
					printf("%d", temp);
					average *= 10;
					average += temp;
					i++;
				}
			}
			printf("\n");
			if (average == 0) average = 1;
		}
		else if (temp == '3')
		{
			i = 0;
			sample = 0;
			printf("\nType the new sample rate divider (0-255) followed by the enter key...\n");
			printf("Sample Rate = (8KHz or 1KHz [determined by DLPF])/(divider +1)\n");
			while (1)
			{
				temp = uart_getchar();
				if (temp == 0x0D)
					break;
				else if ((temp>=0x30)&&(temp<=0x39))
				{
					temp -= 0x30;
					printf("%d", temp);
					sample *= 10;
					sample += temp;
					i++;
				}
			}
			ITG3200Write(SMPL, sample);
			printf("\n");
		}
		else if (temp == '4')
		{
			printf("\nType the new DLPF value (0-6)...\n");
			printf("DLPF	LPF-BW	Sample Rate\n");
			printf("0	256Hz	8kHz\n");
			printf("1	188Hz	1kHz\n");
			printf("2	98Hz	1kHz\n");
			printf("3	42Hz	1kHz\n");
			printf("4	20Hz	1kHz\n");
			printf("5	10Hz	1kHz\n");
			printf("6	5Hz	1kHz\n");
			while (1)
			{
				temp = uart_getchar();
				if ((temp>=0x30)&&(temp<=0x36))
				{
					dlpf = temp - 0x30;
					break;
				}
				else
					printf("Invalid\n");
			}
			temp = 0b00011000 | dlpf;
			ITG3200Write(DLPF, temp);
		}
		else if (temp == '5')
		{
			printf("\nEnter a 8-bit hex value...\n");
			printf("0x");
			
			temp = uart_getchar();
			if ((temp>=0x30)&&(temp<=0x39))
			{
				temp -= 0x30;
				printf("%x", temp);
			}
			else if ((temp>=0x41)&&(temp<=0x46))
			{
				temp -= 55;
				printf("%x", temp);
			}
			else if ((temp>=0x61)&&(temp<=0x66))
			{
				temp -= 87;
				printf("%x", temp);
			}
			else
			{
				printf("Invalid\n");
				temp = 0;
			}
			power = temp << 4;
			
			temp = uart_getchar();
			if ((temp>=0x30)&&(temp<=0x39))
			{
				temp -= 0x30;
				printf("%x", temp);
			}
			else if ((temp>=0x41)&&(temp<=0x46))
			{
				temp -= 55;
				printf("%x", temp);
			}
			else if ((temp>=0x61)&&(temp<=0x66))
			{
				temp -= 87;
				printf("%x", temp);
			}
			else
			{
				printf("Invalid\n");
				temp = 0;
			}
			temp &= 0x0F;
			power |= temp;
			
			printf("\n");
			ITG3200Write(PWR_M, power);
		}
		else if (temp == '6')
			ITG3200ViewRegisters();
	}
}

char ITG3200Read(unsigned char address)
{
	char data;
	
	cbi(TWCR, TWEN);	// Disable TWI
	sbi(TWCR, TWEN);	// Enable TWI
	
	i2cSendStart();
	i2cWaitForComplete();
	
	i2cSendByte(ITG3200_W);	// write 0xD2
	i2cWaitForComplete();
	
	i2cSendByte(address);	// write register address
	i2cWaitForComplete();
	
	i2cSendStart();
	
	i2cSendByte(ITG3200_R);	// write 0xD3
	i2cWaitForComplete();
	i2cReceiveByte(FALSE);
	i2cWaitForComplete();
	
	data = i2cGetReceivedByte();	// Get MSB result
	i2cWaitForComplete();
	i2cSendStop();
	
	cbi(TWCR, TWEN);	// Disable TWI
	sbi(TWCR, TWEN);	// Enable TWI
	
	return data;
}

void ITG3200Write(unsigned char address, unsigned char data)
{
	i2cSendStart();
	i2cWaitForComplete();
	
	i2cSendByte(ITG3200_W);	// write 0xB4
	i2cWaitForComplete();
	
	i2cSendByte(address);	// write register address
	i2cWaitForComplete();
	
	i2cSendByte(data);
	i2cWaitForComplete();
	
	i2cSendStop();
}

void ITG3200ViewRegisters(void)
{
	printf("\nWHO_AM_I (0x00): 0x%x\n", ITG3200Read(WHO));
	delay_ms(1000);
	printf("SMPLRT_DIV (0x15): 0x%x\n", ITG3200Read(SMPL));
	printf("DLPF_FS (0x16): 0x%x\n", ITG3200Read(DLPF));
	printf("INT_CFG (0x17): 0x%x\n", ITG3200Read(INT_C));
	printf("INT_STATUS (0x1A): 0x%x\n", ITG3200Read(INT_S));
	printf("TEMP_OUT_H (0x1B): 0x%x\n", ITG3200Read(TMP_H));
	printf("TEMP_OUT_L (0x1C): 0x%x\n", ITG3200Read(TMP_L));
	printf("GYRO_XOUT_H (0x1D): 0x%x\n", ITG3200Read(GX_H));
	printf("GYRO_XOUT_L (0x1E): 0x%x\n", ITG3200Read(GX_L));
	printf("GYRO_YOUT_H (0x1F): 0x%x\n", ITG3200Read(GY_H));
	printf("GYRO_YOUT_L (0x20): 0x%x\n", ITG3200Read(GY_L));
	printf("GYRO_ZOUT_H (0x21): 0x%x\n", ITG3200Read(GZ_H));
	printf("GYRO_ZOUT_L (0x22): 0x%x\n", ITG3200Read(GZ_L));
	printf("PWR_MGM (0x3E): 0x%x\n", ITG3200Read(PWR_M));
}

int checkInterrupt(void)
{
	if ((PIND & (1<<2)) == 0)
		return 0;
	else
		return 1;
}

void getITG3200(int average)
{
	char temp;
	signed int gx[average], gy[average], gz[average];
	signed int gyrox = 0;
	signed int gyroy = 0;
	signed int gyroz = 0;
	unsigned int i;
	
	for (i = 0; i<average; i++)
	{
		while (!(ITG3200Read(INT_S) & 0x01))
		;
		temp = 0;
		temp = ITG3200Read(GY_H);
		gy[i] = temp << 8;
		gy[i] |= ITG3200Read(GY_L);

		//while (!(ITG3200Read(INT_S) & 0x01))
		//;
		temp = 0;
		temp = ITG3200Read(GZ_H);
		gz[i] = temp << 8;
		gz[i] |= ITG3200Read(GZ_L);
		
		//while (!(ITG3200Read(INT_S) & 0x01))
		//;
		temp = 0;
		temp = ITG3200Read(GX_H);
		gx[i] = temp << 8;
		gx[i] |= ITG3200Read(GX_L);

		
		gyrox += gx[i];
		gyroy += gy[i];
		gyroz += gz[i];
	}
	
	gyrox = gyrox/average;
	gyroy = gyroy/average;
	gyroz = gyroz/average;
	
	printf("	%d	%d	%d\n", gyrox, gyroy, gyroz);
}

/*********************
 ****Initialize****
 *********************/
 
void ioinit (void)
{
    //1 = output, 0 = input
	DDRB = 0b01100000; //PORTB4, B5 output
    DDRC = 0b00010011; //PORTC4 (SDA), PORTC5 (SCL)
    DDRD = 0b11111010; //PORTD (RX on PD0)
	PORTC = 0b00110000; //pullups on the I2C bus
	//sbi(PORTD, 2);
	UART_Init(8);	// U2X0 is set! - 115200bps @ 8MHz
}

void UART_Init(unsigned int ubrr)
{
	// Set baud rate 
	UBRR0H = ubrr>>8;
	UBRR0L = ubrr;
	
	// Enable receiver and transmitter 
	UCSR0A = (1<<U2X0);
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	
	// Set frame format: 8 bit, no parity, 1 stop bit,   
	UCSR0C = (1<<UCSZ00)|(1<<UCSZ01);
	
	stdout = &mystdout; //Required for printf init
}

static int uart_putchar(char c, FILE *stream)
{
    if (c == '\n') uart_putchar('\r', stream);
  
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    
    return 0;
}

void put_char(unsigned char byte)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data */
	UDR0 = byte;
}

uint8_t uart_getchar(void)
{
    while( !(UCSR0A & (1<<RXC0)) );
    return(UDR0);
}
