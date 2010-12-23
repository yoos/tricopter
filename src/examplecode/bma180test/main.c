/*
	Created on: Feb 8, 2010
	Under project: BMA180
	By: Jim Lindblom & Viliam Klein
	
	Test code for the BMA180 - This code will verify SPI communication.
	It will also verify that the INT pin is correctly connected.
	INT test currenty uses the new_data_int functionality.
	After testing INT pin, we will output the acceleration values.
*/

//======================//
//======================//
#include <avr/io.h>
#include <stdio.h>
#include "bma180.h"
//======================//
//======================//

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

//======================//
//Arduino Pro Specific Defines
#define LED 5

//BMA180 Defines
#define CS 0 //pin for chip select
#define INT 1	// pin for interrupt

//======================//
//======================//

void ioinit(void);
void delay_ms(uint16_t x);
void delay_us(uint16_t x);

//======================//
//======================//

static int uart_putchar(char c, FILE *stream);
uint8_t uart_getchar(void);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

//======================//
//======================//

void init_SPI(void);
void txdata(char data);
char rxdata(void);
char read(uint8_t address);
void write(uint8_t address, char data);
int init_BMA180(uint8_t range, uint8_t bw);
int check_interrupt(void);
int test_interrupt(void);

//======================//
//Global Variables


int main(void)
{
	char temp;
	signed short temp2;
	char inkey;

	ioinit();
	init_SPI();
	sbi(PORTB,CS);
	printf("\f");
	printf("\n***************BMA180 Test****************\n\n");
	
	// Init BMA at +/-2g and 10Hz, return error if not connected
	while (init_BMA180(0x01, 0x00) != 0)
	{
		printf("Error connecting to BMA180\n");
		delay_ms(1000);
	}
	printf("Successfully connected to BMA180\n");
	
	while (test_interrupt() != 0)
	{
		printf("Interrupt test failed...\n");
		delay_ms(1000);
	}
	printf("Interrupt test passed\n");
	
	printf("\nPress any button to begin accel raw output...\n");
	inkey = uart_getchar();
	printf("\nACCx \tACCy \tACCZ\n\n");
	
	// main program loop
	while(1){
		temp = 0;
		
		while (temp != 1)
		{
			temp = read(ACCXLSB) & 0x01;
		}
			
		temp = read(ACCXMSB);
		temp2 = temp << 8;
		temp2 |= read(ACCXLSB);
		temp2 = temp2 >> 2;	// Get rid of two non-value bits in LSB
		printf("%d \t", temp2);
		
		while (temp != 1)
		{
			temp = read(ACCYLSB) & 0x01;
		}
		
		temp = read(ACCYMSB);
		temp2 = temp << 8;
		temp2 |= read(ACCYLSB);
		temp2 = temp2 >> 2;
		printf("%d \t", temp2);
		
		while (temp != 1)
		{
			temp = read(ACCZLSB) & 0x01;
		}
		
		temp = read(ACCZMSB);
		temp2 = temp << 8;
		temp2 |= read(ACCZLSB);
		temp2 = temp2 >> 2;
		printf("%d\n", temp2);
	}
}

// init_BMA180
// Input: range is a 3-bit value between 0x00 and 0x06 will set the range as described in the BMA180 datasheet (pg. 27)
// bw is a 4-bit value between 0x00 and 0x09.  Again described on pg. 27
// Output: -1 on error, 0 on success
int init_BMA180(uint8_t range, uint8_t bw)
{
	char temp, temp1;
	
	// if connected correctly, ID register should be 3
	if(read(ID) != 3)
		return -1;
		
	//-------------------------------------------------------------------------------------
	// Set ee_w bit
	temp = read(CTRLREG0);
	temp |= 0x10;
	write(CTRLREG0, temp);	// Have to set ee_w to write any other registers
	//-------------------------------------------------------------------------------------
	// Set BW
	temp = read(BWTCS);
	temp1 = bw;
	temp1 = temp1<<4;
	temp &= (~BWMASK);
	temp |= temp1;
	write(BWTCS, temp);		// Keep tcs<3:0> in BWTCS, but write new BW
	//-------------------------------------------------------------------------------------
	// Set Range
	temp = read(OLSB1);
	temp1 = range;
	temp1 = (temp1<<RANGESHIFT);
	temp &= (~RANGEMASK);
	temp |= temp1;
	write(OLSB1, temp); //Write new range data, keep other bits the same
	//-------------------------------------------------------------------------------------
	
	return 0;
}

// int check_interrupt()
// returns: 0 or 1 depending on status of PB1
int check_interrupt(void)
{
	if((PINB & (1<<1)) == 0)
		return 0;
	else
		return 1;
}

// Check interrupt functionality on PB1
// Output: 0 on success, -1 on error
int test_interrupt(void)
{
	while(check_interrupt() != 0)
		;
		
	write(CTRLREG3, 0x12);	// set new_data_int
	
	delay_ms(100);
	
	while(check_interrupt() != 1)
		;
		
	// write reset_int control bit to 1
	write(CTRLREG3, 0x10);
	write(CTRLREG0, 0x50);
	
	while(check_interrupt() != 0)
		;
		
	return 0;
}

void write(uint8_t address, char data)
{
	//write any data byte to any single address
	//adds a 0 to the MSB of the address byte (WRITE mode)

	address &= 0x7F;	

	//printf("\nWriting 0x%x to 0x%x\n", data, address);

	cbi(PORTB,CS);
	delay_ms(1);
	txdata(address);
	delay_ms(1);
	txdata(data);
	delay_ms(1);
	sbi(PORTB,CS);
}

char read(uint8_t address)
{
	//returns the contents of any 1 byte register from any address
	//sets the MSB for every address byte (READ mode)

	char byte;

	address |= 0x80;

	cbi(PORTB,CS);
	txdata(address);
	byte = rxdata();
	sbi(PORTB,CS);

	return byte;
}

char rxdata(void)
{
	//while((SPSR&0x80) == 0x80);
	SPDR = 0x55;
	while((SPSR&0x80) == 0x00);

	return SPDR;
}

void txdata(char data)
{
	//while((SPSR&0x80) == 0x80);
	SPDR = data;
	while((SPSR&0x80) == 0x00);
}

void init_SPI(void)
{
	sbi(SPCR,MSTR); //make SPI master
	sbi(SPCR,CPOL); sbi(SPCR,CPHA); //SCL idle high, sample data on rising edge
	cbi(SPCR,SPR1);cbi(SPCR,SPR0);cbi(SPSR,SPI2X); //Fosc/4 is SPI frequency
	sbi(SPCR,SPE); //enable SPI

}

static int uart_putchar(char c, FILE *stream)
{
    if (c == '\n') uart_putchar('\r', stream);

    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;

    return 0;
}

uint8_t uart_getchar(void)
{
    while( !(UCSR0A & (1<<RXC0)) );
    return(UDR0);
}


void ioinit (void)
{
	int MYUBRR = 103;	// Results in 9600bps@8MHz or 19200bps@16MHz
	
    //1 = output, 0 = input
    //DDRA = 0b00000000; //ADC inputs
    DDRB = 0b11101101; //MISO input, PB1 input
    DDRC = 0b11111111; //All outputs
    DDRD = 0b11111110; //PORTD (RX on PD0)
	stdout = &mystdout; //Required for printf init

	UBRR0H = (MYUBRR) >> 8;
	UBRR0L = MYUBRR;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (3<<UCSZ00);
	UCSR0A = (1<<U2X0);

	TCCR2B = (1<<CS21);
}

//General short delays
void delay_ms(uint16_t x)
{
    for (; x > 0 ; x--)
        delay_us(1000);
}

//General short delays
void delay_us(uint16_t x)
{
    while(x > 256)
    {
        TIFR2 = (1<<TOV2); //Clear any interrupt flags on Timer2
        TCNT2 = 0; //256 - 125 = 131 : Preload timer 2 for x clicks. Should be 1us per click
        while( (TIFR2 & (1<<TOV2)) == 0);

        x -= 256;
    }

    TIFR2 = (1<<TOV2); //Clear any interrupt flags on Timer2
    TCNT2= 256 - x; //256 - 125 = 131 : Preload timer 2 for x clicks. Should be 1us per click
    while( (TIFR2 & (1<<TOV2)) == 0);
}

