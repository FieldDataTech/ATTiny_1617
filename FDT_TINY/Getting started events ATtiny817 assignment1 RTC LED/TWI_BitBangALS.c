/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  Bit bang TWI master driver.
 *
 *      This file contains the function prototypes and enumerator definitions
 *      for various configuration parameters for the AVR TWI master driver.
 *
 *      The driver is not intended for size and/or speed critical code, since
 *      most functions are just a few lines of code, and the function call
 *      overhead would decrease code performance. The driver is intended for
 *      rapid prototyping and documentation purposes for getting started with
 *      the AVR TWI master.
 *
 *      For size and/or speed critical code, it is recommended to copy the
 *      function contents directly into your application instead of making
 *      a function call.
 *
 * 
 * $Date: 2012-06-01 13:03:43 $  \n
 *
 * Copyright (c) 2012, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: gary.grewal
 *****************************************************************************/
#include "TWI_BitBangALS.h"
#include "port.h"
#define NOP()	__asm__ __volatile__ ("nop")
/************************************************/
/************************************************/
/************************************************/
/************************************************/
void twi_init()
{
	write_sda(1);
	write_scl(1);
	PORTB_set_pin_dir(5,PORT_DIR_OUT);//SCL
} 
/******************************************/
void twi_disable()
{
	PORTB_set_pin_dir(4,PORT_DIR_IN);//SDA
	TPB0_set_dir(PORT_DIR_IN);//SCL
}
/******************************************/
char twi_start_cond(void)
{
    write_sda(0);
	myDelay_twi(TWI_DELAY);
	write_scl(0);	
	myDelay_twi(TWI_DELAY);
	return 1;
}
/******************************************/
void twi_stop_cond (void){
		write_sda(0);
		myDelay_twi(TWI_DELAY);
		PORTB_set_pin_level(5, 1);//SCL HIGH. 
		myDelay_twi(TWI_DELAY);
		write_sda(1);
		myDelay_twi(TWI_DELAY);
}
/******************************************/
char send_slave_address(unsigned char read)
{
 	return i2c_write_byte(SLAVE_ADDRESS | read );
} 
/******************************************/
char i2cAddressTestRd(char addrToTest)
{
	unsigned char index, ack = 0;

	char timeout, writeRet,readRet;
	char data[9];

	if(!twi_start_cond())
	return 0;
	if(!i2c_write_byte(addrToTest | WRITE )) {
		return 0;
		}else{
		return addrToTest;
	}

	myDelay_twi(250);
	myDelay_twi(250);
	myDelay_twi(250);
	myDelay_twi(250);
	myDelay_twi(250);
	myDelay_twi(250);
	myDelay_twi(250);
	myDelay_twi(250);
	myDelay_twi(250);

	if(!twi_start_cond())
	return 0;
	if(!i2c_write_byte(addrToTest | READ )) {
		return 0;
		}else{
		return addrToTest;
	}
}
/******************************************/
char i2cAddressTest(char addrToTest)
{
	unsigned char index, ack = 0;

	if(!twi_start_cond())
	return 0;
	if(!i2c_write_byte(addrToTest)) {
		return 0;
		}else{
		return addrToTest;
	}
}
/******************************************/
char i2c_write_data(unsigned char* indata, char bytes)
{
	unsigned char index, ack = 0;
	if(!twi_start_cond())
	return 0;
	if(!send_slave_address(WRITE))
	return 0;
	for(index = 0; index < bytes; index++)
	{
		ack = i2c_write_byte(indata[index]);
		if(!ack)
		break;
	}
	//	STOP
	PORTB_set_pin_level(5, 1);//SCL HIGH.
	myDelay_twi(TWI_DELAY);
	write_sda(1);
	return ack;
}
/******************************************/
char i2c_write_byte(unsigned char byte)
{
    char bit;
	char iters;
	for (bit = 0; bit < 8; bit++) 
	{
        write_sda((byte & 0x80) != 0);
		myDelay_twi(TWI_DELAY);
 		PORTB_set_pin_level(5, 1);//SCL HIGH. READS WHEN HIGH.
		myDelay_twi(TWI_DELAY);
  		PORTB_set_pin_level(5, 0);//SCL LOW. CHANGE WHEN LOW.
        byte <<= 1;
		myDelay_twi(TWI_DELAY);
		
		
		
		
		
    }
	SET_SDA_IN();//release SDA
	myDelay_twi(TWI_DELAY);
	
	SET_SCL_IN();
	myDelay_twi(TWI_DELAY);
	while(READ_SCL()==0){}
	
	SET_SCL_OUT();
 	PORTB_set_pin_level(5, 1);//SCL HIGH. READS ACK WHEN HIGH.
//	myDelay_twi(TWI_DELAY);//THIS DELAY MAKES TINY MISS THE ACK ON A VEML
	//CHECK FOR ACK
	if(READ_SDA())//NO ACK
	{
		return 0;			
	}
	//GOT ACK
	myDelay_twi(TWI_DELAY);
	PORTB_set_pin_level(5, 0);//SCL LOW
	SET_SDA_OUT();
	myDelay_twi(TWI_DELAY);
	return 1;
}	
/******************************************/
char i2c_read_data(unsigned char* data, char bytes)
{
	unsigned char index,success = 0;
	if(!twi_start_cond())
		return 0;
	if(!send_slave_address(READ))
		return 0;	
	for(index = 0; index < bytes; index++)
	{
		success = i2c_read_byte(data, bytes, index);//db: always returns 1?
		if(!success)
			break; 
	}
	//STOP   (SDA does Low-High while SCL is high)
	write_sda(0);//
	myDelay_twi(TWI_DELAY);
	PORTB_set_pin_level(5, 1);//SCL HIGH.
	myDelay_twi(TWI_DELAY);
	write_sda(1);
	return success;
}	
/******************************************/
char i2c_read_byte(unsigned char* rcvdata, unsigned char bytes, unsigned char index)
{
    unsigned char byte = 0;
	unsigned char bit = 0;
	SET_SDA_IN();
	for (bit = 0; bit < 8; bit++) 
	{
  		PORTB_set_pin_level(5, 1);//SCL
        if(READ_SDA())
        byte|= (1 << (7- bit));
		myDelay_twi(TWI_DELAY);
		PORTB_set_pin_level(5, 0);//SCL
		myDelay_twi(TWI_DELAY);
        }
	rcvdata[index] = byte;
	if(index < (bytes-1))//if last bit
	{
		write_sda(0);//BEGIN ACK
		PORTB_set_pin_level(5, 1);//SCL
		myDelay_twi(TWI_DELAY);
		PORTB_set_pin_level(5, 0);//SCL
		write_sda(1);//END ACK
		myDelay_twi(TWI_DELAY);
	}
	else //send NACK on the last byte
	{
		write_sda(1);//BEGIN NACK (on last byte to read)
 		myDelay_twi(TWI_DELAY);
 		PORTB_set_pin_level(5, 1);//SCL
 		myDelay_twi(TWI_DELAY);
		PORTB_set_pin_level(5, 0);//SCL
 		myDelay_twi(TWI_DELAY);
	}		
	SET_SDA_IN();
	return 1;
}	
/******************************************/
void write_scl (char x)
{
    if(x)
    {
	PORTB_set_pin_level(5, 1);//drive SCL high because not real I2C
    }
    else
    {
	PORTB_set_pin_level(5, 0);   //SCL         
    }
}
/******************************************/
void write_sda (char x)
{
	if(x)
	{
		PORTB_set_pin_dir(4,PORT_DIR_IN);//SDA
	}
	else
	{
		PORTB_set_pin_dir(4,PORT_DIR_OUT);//SDA
		PORTB_set_pin_level(4, 0);//SDA
	}
}
/*******************************************
*    HUNDRED uS DELAY
********************************************/
void myDelay_twi (char twiToDelay){
	uint32_t tickCtr,tusCtr;
	for(tusCtr=twiToDelay;tusCtr>0;tusCtr--){
		for(tickCtr=TICK_CONSTANT_TWI;tickCtr>0;tickCtr--){NOP();}
	}
}