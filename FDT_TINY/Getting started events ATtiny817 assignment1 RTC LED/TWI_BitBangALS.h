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

#ifndef TWI_MASTER_H_
#define TWI_MASTER_H_

//#include "ioport.h"
//#include "pio.h"
//#include "pio_handler.h"
//#include "delay.h"
//#include "Si114x_defs.h"
// #include "ioavr.h"
// #include "inavr.h"

/*! \brief Definition of pin used as SCL. */
//#define SCL SCL0	//PC1

/*! \brief Definition of pin used as SDA. */
//#define SDA SDA0	//PC4

/*! \brief Definition of PORT used as SCL. */
//#define PORT_SCL PORTC  //db: covered by ioport function
/*! \brief Definition of DDR used as SCL. */
//#define DDR_SCL	DDRC  //db: covered by ioport function
/*! \brief Definition of PIN used as SCL. */
//#define PIN_SCL 	//db: was PINC, which appears to be the register for PORTC_PIN
/*! \brief Definition of PORT used as SDA. */
//#define PORT_SDA PORTC //db: covered by ioport function
/*! \brief Definition of DDR used as SDA. */
//#define DDR_SDA	DDRC  //db: covered by ioport function
/*! \brief Definition of PIN used as SDA. */
//#define PIN_SDA 	//db: was PINC, which appears to be the register for PORTC_PIN

/*! \brief Slave 8 bit address (shifted). */
//for demo board? #define SLAVE_ADDRESS 0xB4//5A	//Sept 2 definitely works at B4 and not 5A like the datasheet error
//#define SLAVE_ADDRESS 0x20//A6 for SI1153 Apr 2018 custom board (datasheet address shifted one bit left)
#define SLAVE_ADDRESS 0x20//20 for VEML light sensor
//#define SLAVE_ADDRESS_READ 0x20//A6 for SI1153 Apr 2018 custom board (datasheet address shifted one bit left)

#define READ_SDA() PORTB_get_pin_level(4) //(PIN_SDA & (1 << SDA))
#define SET_SDA_OUT() PORTB_set_pin_dir(4,PORT_DIR_OUT)  //DDR_SDA |= (1 << SDA)
#define SET_SDA_IN() PORTB_set_pin_dir(4,PORT_DIR_IN)	//DDR_SDA &= ~(1 << SDA)
#define READ_SCL() PORTB_get_pin_level(5)  //(PIN_SCL & (1 << SCL))?1:0
#define SET_SCL_OUT() PORTB_set_pin_dir(5,PORT_DIR_OUT)  //
#define SET_SCL_IN() PORTB_set_pin_dir(5,PORT_DIR_IN)	//

#define WRITE 0x0
#define READ 0x1

/*! \brief Delay used to generate clock */
#define DELAY 1  //datasheet SI1153 up to 400KHz clock, doesn't work at 0, 1 does 230KHz
#define TWI_DELAY 0  //datasheet SI1153 up to 400KHz clock, doesn't work at 0, 1 does 230KHz
//nov 2017 works with DELAY=1 at 230KHz. DOesn't work at 0.  Going with 2 for margin
/*! \brief Delay used for STOP condition */
#define TICK_CONSTANT_TWI 5  //needs to be in same file with TWI functions or it takes too long// #include "ioavr.h"
#define SCL_SDA_DELAY 1

void twi_disable();
void twi_init();
void toggle_scl();
void write_scl(char x);
char twi_start_cond(void);
char i2c_write_data(unsigned char*, char);
char i2c_write_byte(unsigned char byte);
char i2c_read_byte(unsigned char*, unsigned char, unsigned char);
char i2c_read_data(unsigned char*, char);
void write_sda( char x);
void twi_stop_cond (void);

char i2cAddressTest(char);
char i2cAddressTestRd(char);
void myDelay_twi (char);

#endif /* TWI_MASTER_H_ */