/*
 * pata_UART.h
 *
 * Created: 2018-08-30 오전 11:56:35
 *  Author: pata
 *
 *
 *
 *						Atmega8 pin map
 *
 *				 RESET  = PC6   PC5 =
 *					RX	= PD0   PC4 =
 *					TX	= PD1   PC3 =
 *						= PD2   PC2 =
 *						= PD3   PC1 =
 *						= PD4   PC0 =
 *				 VCC	= VCC   GND =
 *				 GND	= GND   AREF=
 *						= PB6   AVCC=
 *            			= PB7   PB5 =
 *			  			= PD5   PB4 =
 *						= PD6   PB3 =
 *						= PD7   PB2 =
 *						= PB0   PB1 =
 *
 *
 */ 


#ifndef PATA_UART_H_
#define PATA_UART_H_
#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#define TX		0x01
#define RX		0x02
#define TXRX	0x03
#define RXI		0x04
#define TXRXI	0x05

#define WRITE	0
#define READ	1

#define BAUD_9600	1
#define BAUD_14k	2
#define BAUD_19k	3
#define BAUD_38k	4


#include <avr/interrupt.h>
#include <stdint.h>
#include <string.h>


/*******************UART*********************/
// UART initialize
void UART_init(uint8_t baud, uint8_t mode);

// Standard TX/RX
void UART_tx(uint8_t data);
void UART_tx_m(uint8_t *data, uint8_t length);
uint8_t UART_rx();
void UART_rx_m(uint8_t *data, uint8_t length);
void UART_NWL();

/********************I2C*********************/
void i2c_init();
uint8_t i2c_start(uint8_t slave_address);
uint8_t i2c_rep_start(uint8_t slave_address);
void i2c_stop();
uint8_t i2c_write(uint8_t data);
uint8_t i2c_read_ACK();
uint8_t i2c_read_NAK();
void I2C_M_tx(uint8_t device_address, uint8_t data);
uint8_t I2C_M_rx(uint8_t device_address, uint8_t read_address);

volatile uint8_t data = 0;
volatile uint8_t data_m[10] = {0};
uint8_t ubrr = 0;



/********************************************/
/*					UART					*/
/*											*/
/********************************************/
ISR (USART_RXC_vect)
{
	data = UDR;
	
}

void UART_init(uint8_t baud, uint8_t mode)
{
	switch (baud)
	{
		case BAUD_9600:
			ubrr = 51;
			break;
		case BAUD_14k:
			ubrr = 34;
			break;
		case BAUD_19k:
			ubrr = 25;
			break;
		case BAUD_38k:
			ubrr = 12;
			break;
	}
	UBRRH = (ubrr >> 8);
	UBRRL = ubrr;
	
	if (mode & TX)
		UCSRB = (1 << TXEN);
	if ((mode & RX) || (mode & RXI))
		UCSRB |= (1 << RXEN);
	if (mode & RXI)
		UCSRB |= (1 << RXCIE);
	
	DDRD &= ~(1<<0);
	DDRD |= (1<<1);
}

void UART_tx(uint8_t data)
{
	while(!(UCSRA & (1<<UDRE)));
	UDR = data;
}

void UART_tx_m(uint8_t *data, uint8_t length)
{
	for (uint8_t i = 0; i < length; i++)
	{
		while(!(UCSRA & (1<<UDRE)));
		UDR = data[i];
	}
}

uint8_t UART_rx()
{
	while (!(UCSRA & (1<<RXC)));
	data = UDR;
	
	return data;
}

void UART_rx_m(uint8_t *data, uint8_t length)
{
	for (uint8_t i = 0; i < length; i++)
	{
		while(!(UCSRA & (1<<RXC)));
		UDR = data[i];
	}
}

void UART_NWL()
{
	while(!(UCSRA & (1<<UDRE)));
	UDR = 0x0A;
}


/********************************************/
/*					I2C						*/
/*											*/
/********************************************/
void i2c_init()
{
	TWSR = 0;                         // no prescaler
	TWBR = ((F_CPU/200000)-16)/2;  // must be > 10 for stable operation
	
}
uint8_t i2c_start(uint8_t slave_address)
{
	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// send device address
	TWDR = slave_address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR & (1<<TWINT)));
	
	return 0;

}
uint8_t i2c_rep_start(uint8_t address)
{
	return i2c_start(address);
	
}
void i2c_stop()
{
	 // send stop condition
	 TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	 
	 // wait until stop condition is executed and bus released
	 while(TWCR & (1<<TWSTO));
	 
}
uint8_t i2c_write(uint8_t data)
{
	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	return 0;
	
}
uint8_t i2c_read_ACK()
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));

	return TWDR;
	
}
uint8_t i2c_read_NAK()
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	
	return TWDR;
	
}
void I2C_M_tx(uint8_t slave_address , uint8_t data)
{
	i2c_start(slave_address<<1);
	i2c_write(data);
	i2c_stop();
	
}
uint8_t I2C_M_rx(uint8_t device_address, uint8_t read_address)
{
	unsigned char result = 0;
	
	i2c_start(device_address<<1);
	i2c_write(read_address);
	i2c_rep_start((device_address<<1) | 1);
	result = i2c_read_NAK();
	i2c_stop();
	
	return result;
	
}

#endif /* PATA_UART_H_ */