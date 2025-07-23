/*___________________________________________________________________________________________________

Title:
	uart.h v1.1

Description:
	Library for USART on AVR devices
	
	For complete details visit:
	https://www.programming-electronics-diy.xyz/2022/08/uart-library-for-avr-microcontrollers.html

Author:
 	Liviu Istrate
	istrateliviu24@yahoo.com
	www.programming-electronics-diy.xyz

Donate:
	Software development takes time and effort so if you find this useful consider a small donation at:
	paypal.me/alientransducer
_____________________________________________________________________________________________________*/


/* ----------------------------- LICENSE - GNU GPL v3 -----------------------------------------------

* This license must be included in any redistribution.

* Copyright (c) 2022 Liviu Istrate, www.programming-electronics-diy.xyz (istrateliviu24@yahoo.com)

* Project URL: https://www.programming-electronics-diy.xyz/2022/08/uart-library-for-avr-microcontrollers.html

* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.

* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.

* You should have received a copy of the GNU General Public License
* along with this program. If not, see <https://www.gnu.org/licenses/>.

--------------------------------- END OF LICENSE --------------------------------------------------*/


#ifndef UART_H_
#define UART_H_

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <avr/io.h>
#include <util/atomic.h>
#include <avr/interrupt.h>
#include "utils.h"

//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------
#define UART_NUMBER						0 // 0 or 1
#define UART_TX_BUFFER_SIZE				64 // bytes from 1 to 255
#define UART_RX_BUFFER_SIZE				64 // bytes from 1 to 255

#define UART_XCKn_DDR					DDRD
#define UART_XCKn_PIN					PD4

//typedef uint8_t	bool;
#define true	1
#define false	0

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------
// Operating Modes
#define UART_ASYNC_NORMAL				16	// Asynchronous Normal mode
#define UART_ASYNC_DOUBLE_SPEED			8	// Asynchronous Double Speed mode
#define UART_SYNC						2	// Synchronous Master/Slave mode

// Parity Mode
#define UART_NO_PARITY					0
#define UART_EVEN_PARITY				2
#define UART_ODD_PARITY					3

// Character Size
#define UART_5_BIT						0
#define UART_6_BIT						1
#define UART_7_BIT						2
#define UART_8_BIT						3
#define UART_9_BIT						7

// Used by some short delay loops
#define CYCLES_PER_US					(F_CPU / 1000000.0) // CPU cycles per microsecond
#define UART_TX_TIMEOUT					(CYCLES_PER_US * 50000); // ~1s timeout
#define UART_RX_TIMEOUT					(CYCLES_PER_US * 50000); // ~1s timeout

//-----------------------------------------------------------------------------
// Registers
//-----------------------------------------------------------------------------
#if UART_NUMBER == 0
	#define UART_UCSRA				UCSR0A
	#define UART_UCSRB				UCSR0B
	#define UART_UCSRC				UCSR0C
	#define UART_UBRR_LOW			UBRR0L
	#define UART_UBRR_HIGH			UBRR0H
	#define UART_UDR				UDR0

	#define UART_UCSRA_U2X			U2X0
	#define UART_UCSZ0				UCSZ00
	#define UART_UCSZ1				UCSZ01
	#define UART_UCSZ2				UCSZ02
	#define UART_UCSRB_RXEN			RXEN0
	#define UART_UCSRB_TXEN			TXEN0
	#define UART_UCSRB_UDRIE		UDRIE0
	#define UART_TXB8				TXB80
	#define UART_UDRE				UDRE0
	#define UART_UMSEL0				UMSEL00
	#define UART_UMSEL1				UMSEL01
	#define UART_UPM0				UPM00
	#define UART_UPM1				UPM01
	#define UART_RXCIE				RXCIE0
	#define UART_RXC				RXC0
	#define UART_FE					FE0
	#define UART_DOR				DOR0
	#define UART_UPE				UPE0
	#define UART_MPCM				MPCM0

	#define UART_USART_UDRE_vect	USART_UDRE_vect // USART Data Register Empty Interrupt
	#define UART_USART_RX_vect		USART_RX_vect // USART Receive Interrupt
	
#elif UART_NUMBER == 1
	#define UART_UCSRA				UCSR1A
	#define UART_UCSRB				UCSR1B
	#define UART_UCSRC				UCSR1C
	#define UART_UBRR_LOW			UBRR1L
	#define UART_UBRR_HIGH			UBRR1H
	#define UART_UDR				UDR1

	#define UART_UCSRA_U2X			U2X1
	#define UART_UCSZ0				UCSZ10
	#define UART_UCSZ1				UCSZ11
	#define UART_UCSZ2				UCSZ12
	#define UART_UCSRB_RXEN			RXEN1
	#define UART_UCSRB_TXEN			TXEN1
	#define UART_UCSRB_UDRIE		UDRIE1
	#define UART_TXB8				TXB81
	#define UART_UDRE				UDRE1
	#define UART_UMSEL0				UMSEL10
	#define UART_UMSEL1				UMSEL11
	#define UART_UPM0				UPM10
	#define UART_UPM1				UPM11
	#define UART_RXCIE				RXCIE1
	#define UART_RXC				RXC1
	#define UART_FE					FE1
	#define UART_DOR				DOR1
	#define UART_UPE				UPE1
	#define UART_MPCM				MPCM1

	#define UART_USART_UDRE_vect	USART1_UDRE_vect // USART Data Register Empty Interrupt
	#define UART_USART_RX_vect		USART1_RX_vect // USART Receive Interrupt
#endif

//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------
static uint8_t UART_TX_BUFFER[UART_TX_BUFFER_SIZE];
static uint8_t UART_RX_BUFFER[UART_RX_BUFFER_SIZE];

volatile uint8_t RX_ERROR_FLAGS;

static volatile uint8_t uartBytesToSend;
static volatile uint8_t uartTXWriteIdx;
static volatile uint8_t uartTXReadIdx;

static volatile uint8_t uartBytesToRead;
static volatile uint8_t uartRXWriteIdx;
static volatile uint8_t uartRXReadIdx;

typedef void (*voidFuncPtr)(uint8_t);
volatile static voidFuncPtr uartRXfunc;

//-----------------------------------------------------------------------------
// MACROS
//-----------------------------------------------------------------------------
#define UART_error()				(RX_ERROR_FLAGS & 0x1C) // Keep only error flag bits
#define UART_clearErrorFlag()		RX_ERROR_FLAGS = 0
#define UART_frameError()			(RX_ERROR_FLAGS & (1 << UART_FE))
#define UART_dataOverRunError()		(RX_ERROR_FLAGS & (1 << UART_DOR))
#define UART_parityError()			(RX_ERROR_FLAGS & (1 << UART_UPE))

//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------
void UART_begin(uint32_t baudrate, uint8_t mode, uint8_t parity, uint8_t bits);
uint8_t UART_send(uint8_t data);
void UART_sendString(char* s);
void UART_sendBytes(uint8_t* buff, uint8_t length);
void UART_send9bits(uint16_t data);
int16_t UART_read9bits(void);
void UART_mpcmEnable(void);
void UART_mpcmDisable(void);
void UART_mpcmSelectDevice(uint8_t address);

#ifdef UTILS_H_
void UART_sendInt(INT_SIZE number);
void UART_sendFloat(float number, uint8_t decimals);
#endif

void UART_sendHex8(uint8_t value);
void UART_sendHex16(uint16_t value);
void UART_sendHex32(uint32_t value);
bool UART_available(void);
uint8_t UART_read(void);
uint8_t UART_readBytes(uint8_t* buff, uint8_t length);
uint8_t UART_readBytesUntil(char character, uint8_t* buff, uint8_t length);
void USART_flush(void);
void UART_end(void);
void UART_setRXhandler(void (*rx_func)(uint8_t c));


//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

/*-----------------------------------------------------------------------------
	Initialization function sets the Baud rate, frame format, enables UART 
	interrupts and global interrupts. One STOP bit is used. The function
	can be used at any point to change baud rate or frame parameters.

	baudrate	bits per second
	
	mode		UART mode: Asynchronous Normal mode, Asynchronous Double Speed,
				Synchronous Master/Slave mode.

				Available constants: 
					UART_ASYNC_NORMAL
					UART_ASYNC_DOUBLE_SPEED
					UART_SYNC
					
	parity		UART_NO_PARITY, UART_EVEN_PARITY, UART_ODD_PARITY
	
	bits		number of bits in the frame, from 5 to 8.
				UART_5_BIT, UART_6_BIT, UART_7_BIT, UART_8_BIT.
				For 9 bits see UART_send9bits().
------------------------------------------------------------------------------*/
void UART_begin(uint32_t baudrate, uint8_t mode, uint8_t parity, uint8_t bits){
	// Initialize user receive handler
	uartRXfunc = 0;
	
	// For interrupt driven USART operation, the Global Interrupt Flag should
	// be cleared (and interrupts globally disabled) when doing the initialization.
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){		
		// Note for external clock: in Slave mode the fxck must be < than fosc / 4
		// Calculating Baud Rate Register Setting
		uint16_t ubrr = (F_CPU / (mode * baudrate)) - 1;
	
		// Set baudrate in UBRR
		UART_UBRR_HIGH = ubrr >> 8;
		UART_UBRR_LOW = ubrr;
		
		// Reset UART Registers
		UART_UCSRA = 0;
		UART_UCSRB = 0;
		UART_UCSRC = 0;
		
		// Mode Selection
		if(mode == UART_ASYNC_DOUBLE_SPEED){
			// Double the USART Transmission Speed
			UART_UCSRA = (1 << UART_UCSRA_U2X);
		}else if(mode == UART_SYNC){
			// Synchronous USART (clock polarity is UCPOL 0)
			UART_UCSRC = (1 << UART_UMSEL0);
		}
		
		// Parity, 1 Stop bit (default)
		UART_UCSRC |= (parity << UART_UPM0);
		
		// Number of bits
		if((bits == UART_6_BIT) || (bits == UART_8_BIT)){
			UART_UCSRC |= (1 << UART_UCSZ0);
		}
		
		if((bits == UART_7_BIT) || (bits == UART_8_BIT)){
			UART_UCSRC |= (1 << UART_UCSZ1);
		}
		
		// Enable the transmitter and receiver
		UART_UCSRB |= (1 << UART_UCSRB_RXEN) | (1 << UART_UCSRB_TXEN);
	}
	
	// RX Complete Interrupt Enable
	UART_UCSRB |= (1 << UART_RXCIE);
	
	// Enable global interrupts
	sei();
}



/*-----------------------------------------------------------------------------
	Put a byte in the transmit buffer and enables the USART Data Register 
	Empty Interrupt. If the UART is somehow disabled, a timeout will end the
	while loop after 2ms.
------------------------------------------------------------------------------*/
uint8_t UART_send(uint8_t data){
	uint32_t timeout = UART_TX_TIMEOUT;
	
	// Check for buffer overflow
	if(uartTXWriteIdx == UART_TX_BUFFER_SIZE){
		uartTXWriteIdx = 0;
		
		// Overflow condition means the write index is now behind the read index.
		// Wait until the next byte in the buffer is sent.
		// In case the UART is disabled the timeout will occur in 2ms
		while(uartBytesToSend){
			if(timeout) timeout--;
			else return 1;
		}
	}
	
	// Put data byte into the transmit buffer
	UART_TX_BUFFER[uartTXWriteIdx++] = data;
	
	uartBytesToSend++;
	
	// Enable USART Data Register Empty Interrupt
	UART_UCSRB |= (1 << UART_UCSRB_UDRIE);
	
	return 0;
}


/*-----------------------------------------------------------------------------
	Sends a null terminated string.
------------------------------------------------------------------------------*/
void UART_sendString(char* s){
	// Transmit a character until NULL is reached
	while(*s){
		if(UART_send(*s++)){
			// Exit if timeout
			break;
		}
	}
}


/*-----------------------------------------------------------------------------
	Send a series of bytes in a buffer.
------------------------------------------------------------------------------*/
void UART_sendBytes(uint8_t* buff, uint8_t length){
	for(uint8_t i = 0; i < length; i++){
		if(UART_send(buff[i])){
			// Exit if timeout
			break;
		}
	}
}


/*-----------------------------------------------------------------------------
	Sends 9 bits.
------------------------------------------------------------------------------*/
void UART_send9bits(uint16_t data){
	// Wait for empty transmit buffer
	while(!(UART_UCSRA & (1 << UART_UDRE)));
	
	// Copy 9th bit to TXB8
	if(data & 0x0100) UART_UCSRB |= (1 << UART_TXB8);
	else UART_UCSRB &= ~(1 << UART_TXB8);
	
	UART_UDR = data;
}


/*-----------------------------------------------------------------------------
	Read 9 bits.
------------------------------------------------------------------------------*/
int16_t UART_read9bits(void){
	uint8_t status, resh, resl;
	
	// Wait for data to be received
	while(!(UART_UCSRA & (1 << UART_RXC)));
	
	// Due to the buffering of the Error Flags, 
	// the UCSRnA must be read before the receive buffer (UDRn).
	// Get status and 9th bit, then data from buffer
	status = UART_UCSRA;
	resh = UART_UCSRB;
	resl = UART_UDR;
	
	// If error, return -1
	if(status & ((1 << UART_FE) | (1 << UART_DOR) | (1 << UART_UPE))) return -1;
	
	// Filter the 9th bit, then return
	resh = (resh >> 1) & 0x01;
	return ((resh << 8) | resl);
}


/*-----------------------------------------------------------------------------
	Enables the Multi-processor Communication mode (MPCM).
	After this function sets the MPCM bit, frames that do not contain an address
	will be ignored by the UART. Frames containing an address have the 
	9'th bit set to 1.
------------------------------------------------------------------------------*/
void UART_mpcmEnable(void){
	// Set number of bits to 9
	UART_UCSRB |= (1 << UART_UCSZ2);
	UART_UCSRC |= (1 << UART_UCSZ0) | (1 << UART_UCSZ1);
	
	// This bit enables the Multi-processor Communication mode. 
	// When the MPCM bit is written to one, all the incoming frames 
	// received by the USART Receiver that do not contain address 
	// information will be ignored. The Transmitter is unaffected 
	// by the MPCM setting.
	UART_UCSRA |= (1 << UART_MPCM);
}


/*-----------------------------------------------------------------------------
	Disables the Multi-processor Communication mode (MPCM).
	After this function sets the MPCM bit to 0, data frames can be received.
	This should be used after a valid address has been received after using
	UART_mpcmEnable().
------------------------------------------------------------------------------*/
void UART_mpcmDisable(void){	
	// This bit enables the Multi-processor Communication mode.
	// When the MPCM bit is written to one, all the incoming frames
	// received by the USART Receiver that do not contain address
	// information will be ignored. The Transmitter is unaffected
	// by the MPCM setting.
	UART_UCSRA &= ~(1 << UART_MPCM);
}


/*-----------------------------------------------------------------------------
	Select a device by sending an address frame over UART.
------------------------------------------------------------------------------*/
void UART_mpcmSelectDevice(uint8_t address){
	// Set 9'th bit to 1 indicating an address frame
	uint16_t data = address | 0x0100;
	UART_send9bits(data);
	
	// Wait for empty transmit buffer
	while(!(UART_UCSRA & (1 << UART_UDRE)));
	
	// Set 9'th bit in the register back to 0
	UART_UCSRB &= ~(1 << UART_TXB8);
}


/*-----------------------------------------------------------------------------
	Send an integer number.
------------------------------------------------------------------------------*/
#ifdef UTILS_H_
void UART_sendInt(INT_SIZE number){
	char string[MAX_NR_OF_DIGITS + 1] = {0};
	
	STRING_itoa(number, string, 0);
	UART_sendString(string);
}


/*-----------------------------------------------------------------------------
	Send an float number.
	
	decimals	number of digits after the dot
------------------------------------------------------------------------------*/
void UART_sendFloat(float number, uint8_t decimals){
	char string_integer[MAX_NR_OF_DIGITS + 1];
	char string_float[MAX_NR_OF_DIGITS + 1];
	
	STRING_ftoa(number, string_integer, string_float, 0, decimals);

	if(number < 0) UART_send('-');
	UART_sendString(string_integer);
	UART_send('.');
	UART_sendString(string_float);
}
#endif


/*-----------------------------------------------------------------------------
	Convert 1-byte integer number into hex and send over UART.
------------------------------------------------------------------------------*/
void UART_sendHex8(uint8_t value){
	// Extract upper and lower nibbles from input value
	uint8_t upperNibble = (value & 0xF0) >> 4;
	uint8_t lowerNibble = value & 0x0F;

	// Convert nibble to its ASCII hex equivalent
	upperNibble += upperNibble > 9 ? 'A' - 10 : '0';
	lowerNibble += lowerNibble > 9 ? 'A' - 10 : '0';

	// Print the characters
	UART_send(upperNibble);
	UART_send(lowerNibble);
}


/*-----------------------------------------------------------------------------
	Convert 2-byte integer number into hex and send over UART.
------------------------------------------------------------------------------*/
void UART_sendHex16(uint16_t value){
	// Transmit upper 8 bits
	UART_sendHex8(value >> 8);

	// transmit lower 8 bits
	UART_sendHex8((uint8_t) value);
}


/*-----------------------------------------------------------------------------
	Convert 4-byte integer number into hex and send over UART.
------------------------------------------------------------------------------*/
void UART_sendHex32(uint32_t value){
	// Transmit upper 16 bits
	UART_sendHex16((uint16_t) (value >> 16));

	// transmit lower 16 bits
	UART_sendHex16((uint16_t) value);
}


/*-----------------------------------------------------------------------------
	Returns true if new data in available.
------------------------------------------------------------------------------*/
bool UART_available(void){
	return uartBytesToRead;
}


/*-----------------------------------------------------------------------------
	Returns the next received byte or 0 if no new data is available.
	Should be used only if UART_available()	returns true.
------------------------------------------------------------------------------*/
uint8_t UART_read(void){
	uint8_t rx = 0;
	
	if(uartBytesToRead){
		// Check for buffer overflow
		if(uartRXReadIdx == UART_RX_BUFFER_SIZE){
			uartRXReadIdx = 0;
		}
		
		rx = UART_RX_BUFFER[uartRXReadIdx++];
		uartBytesToRead--;
	}
	
	return rx;
}


/*-----------------------------------------------------------------------------
	Reads received bytes into the provided buffer. The function terminates 
	if the determined length has been read, or it times out.
	
	Returns the number of characters read.
------------------------------------------------------------------------------*/
uint8_t UART_readBytes(uint8_t* buff, uint8_t length){
	uint32_t timeout = UART_RX_TIMEOUT;
	
	for(uint8_t i = 0; i < length; i++){
		while(UART_available() == false){
			if(timeout) timeout--;
			else return i;
		}
		
		buff[i] = UART_read();
	}
	
	return length;
}


/*-----------------------------------------------------------------------------
	Reads received bytes into the provided buffer. The function terminates 
	if the determined length has been read, if it times out or if the 
	terminator character is detected. The termination character is not 
	included into the buffer.
	
	Returns the number of characters read.
------------------------------------------------------------------------------*/
uint8_t UART_readBytesUntil(char character, uint8_t* buff, uint8_t length){
	uint32_t timeout = UART_RX_TIMEOUT;
	
	for(uint8_t i = 0; i < length; i++){
		while(UART_available() == false){
			if(timeout) timeout--;
			else return i;
		}
		
		buff[i] = UART_read();
		if(buff[i] == character){
			buff[i] = 0;
			return i;
		}
	}
	
	return length;
}


/*-----------------------------------------------------------------------------
	If the buffer has to be flushed during normal operation, due to 
	for instance an error condition, read the UDRn I/O location until 
	the RXCn Flag is cleared.
------------------------------------------------------------------------------*/
void USART_flush(void){
	volatile uint8_t dummy;
	
	while(UART_UCSRA & (1 << UART_RXC)){
		dummy = UART_UDR;
		(void)dummy;
	}
}


/*-----------------------------------------------------------------------------
	Disable UART.
------------------------------------------------------------------------------*/
void UART_end(void){
	// Disable the transmitter and receiver
	// Disable RX Complete Interrupt
	UART_UCSRB = 0;
}


/*-----------------------------------------------------------------------------
	Redirects received data to a user function.
------------------------------------------------------------------------------*/
void UART_setRXhandler(void (*rx_func)(uint8_t c)){
	// set the receive interrupt to run the supplied user function
	uartRXfunc = rx_func;
}



//-----------------------------------------------------------------------------
// ISR Handlers
//-----------------------------------------------------------------------------

// USART Data Register Empty Interrupt
ISR(UART_USART_UDRE_vect){
	// Check for buffer overflow
	if(uartTXReadIdx == UART_TX_BUFFER_SIZE){
		uartTXReadIdx = 0;
	}
	
	// Put data into transmit buffer
	UART_UDR = UART_TX_BUFFER[uartTXReadIdx++];
	
	// Disable USART Data Register Empty Interrupt
	if(--uartBytesToSend == 0){
		UART_UCSRB &= ~(1 << UART_UCSRB_UDRIE);
	}
	
}


// USART Receive Complete Interrupt
ISR(UART_USART_RX_vect){
	// Check for buffer overflow
	if(uartRXWriteIdx == UART_RX_BUFFER_SIZE){
		uartRXWriteIdx = 0;
	}
	
	// Due to the buffering of the Error Flags,
	// the UCSRnA must be read before the receive buffer (UDRn).
	RX_ERROR_FLAGS = UART_UCSRA;
	
	// Put data into receive buffer
	UART_RX_BUFFER[uartRXWriteIdx] = UART_UDR;
	
	// If there's a user function to handle this receive event
	if(uartRXfunc){
		// Call it and pass the received data
		uartRXfunc(UART_RX_BUFFER[uartRXWriteIdx]);
	}
	
	uartRXWriteIdx++;
	uartBytesToRead++;
}

#endif /* UART_H_ */