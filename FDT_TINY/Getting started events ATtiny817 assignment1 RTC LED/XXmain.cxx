/***************************************************************
*
*
*  TO DO:   Switch to low powered 32KHz clock?  
*           or calibrate fast clock?
*			Accommodate re-start by searching for unused archive?
*
******************************************************************************/
#include <avr/io.h>
#include "atmel_start.h"
#include "tinyInclude.h"
#include "initTinyBoard.h"
#include "TWI_BitBang1153.h"
#include <avr/pgmspace.h>
#define DISABLE_INTERRUPTS()        __asm__ __volatile__ ( "cli" ::: "memory")
#define ENABLE_INTERRUPTS()         __asm__ __volatile__ ( "sei" ::: "memory")
#define NOP()	__asm__ __volatile__ ("nop")
#define SLEEP()	__asm__ __volatile__ ("sleep")
#define BTTIMEOUT 250	//only needs to count to about 7, with clocks per March 2018




int main(void)
{
	char readRet,writeRet = 0;
	char captureFlag=0;
	int archiveCtr=0;
	char iters=0;
	char LCDdata[16];
	char ibase=0;
	unsigned short crcrc;
	
	g_motionMinTotals=0;
	g_rxDataIndex=0;

	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	DISABLE_INTERRUPTS();
	ccp_write_io((void*)&(CLKCTRL.OSC32KCTRLA),1 << CLKCTRL_RUNSTDBY_bp /* Run standby: enabled */);
	ccp_write_io((void*)&(CLKCTRL.OSC20MCTRLA),1 << CLKCTRL_RUNSTDBY_bp /* Run standby: ENabled */);
	
	mState=BEGIN;
	for (iters=0;iters<SIZEOFMOTPARAMS;iters++)motParams[iters]=0;
/*	TPB0_set_dir(PORT_DIR_OUT);//force SI1153 all low to possibly reset, avoid getting stuck in non-responsive mode
	TPB1_set_dir(PORT_DIR_OUT);//force SI1153 all low to possibly reset, avoid getting stuck in non-responsive mode
	PORTB_set_pin_level(0, 0);//force SI1153 all low to possibly reset, avoid getting stuck in non-responsive mode
	PORTB_set_pin_level(1, 0);//force SI1153 all low to possibly reset, avoid getting stuck in non-responsive mode
	delay_ms(1000);//force SI1153 all low to possibly reset, avoid getting stuck in non-responsive mode*/


//	LED_set_dir(PORT_DIR_OUT);//C2 for custom board, different for dev kit
	LED_set_dir(PORT_DIR_IN);
	LED_set_pull_mode(PORT_PULL_UP);// Min current with either input PU or output

	TPC1_set_dir(PORT_DIR_IN);//C1=WAKEUP FROM BIG
	TPC1_set_pull_mode(PORT_PULL_OFF);//C1=WAKEUP FROM BIG
	TPC1_set_isc(PORT_ISC_LEVEL_gc);//C1=WAKEUP FROM BIG
	TPB6_set_dir(PORT_DIR_IN);//B6=Motion Switch
	TPB6_set_pull_mode(PORT_PULL_OFF); //B6=Motion Switch
	TPB6_set_isc(PORT_ISC_RISING_gc);
	TPA5_set_dir(PORT_DIR_IN);
	PORTC_set_pin_level(0, 0);	//WakeUp V71 Low
	TPC0_set_dir(PORT_DIR_OUT);//WakeUp V71
	PORTC_set_pin_level(0, 0);	//WakeUp V71 Low
//	TPB7_set_dir(PORT_DIR_IN);//B7=data to PIR
//	TPA2_set_dir(PORT_DIR_OUT);
	TPC3_set_dir(PORT_DIR_IN);
	TPC3_set_pull_mode(PORT_PULL_OFF);
//	TPC3_set_isc(PORT_ISC_RISING_gc);
//	TPB0_set_dir(PORT_DIR_OUT);
//	TPC0_set_dir(PORT_DIR_OUT);//XBee Pwr
//    TPB5_set_dir(PORT_DIR_OUT);//SCL	TEMPORARY
//    TPB4_set_dir(PORT_DIR_IN);//SI1153 Interrupt  (NOT USED??)
//	TPB4_set_pull_mode(PORT_PULL_UP);//SI1153 Interrupt
//	TPB4_set_isc(PORT_ISC_FALLING_gc);//SI1153 Interrupt
closeGPS();
	DISABLE_INTERRUPTS();

	//	TPB6_set_dir(PORT_DIR_IN);//B6=data from PIR
	//	TPB6_set_pull_mode(PORT_PULL_OFF); //B6=data from PIR
	LCDclear();
	usart_put_string(&("Startup A"),9);
//	delay_ms(100);
//	LCDbottomLine();
//	usart_put_string(&("Startup B"),9);
//	delay_ms(100);

//	PORTC_set_pin_level(0, 1); //XBee Pwr on
//    TPB7_set_dir(PORT_DIR_OUT);//SCL	TMEP
/*	for(;;){  //FOR DEV OF DATA HANDOFF
		delay_ms(250);//120 is too short with v71 at either 3 or 32 clock cycle trigger. Might need longer without
		PORTC_set_pin_level(0, 0);	//WakeUp V71 Low
		LCDclear();//might have to increase delay after LCD goes away
		LCDbottomLine();
		usart_put_string(&("Pin LOW "),8);
		LCDshort(crcrc,4);
		delay_ms(3000);
	}*/

	for(iters=0;iters<30;iters++){  //give time to recover before getting stuck in sleep
		delay_ms(5);
		redBlink(1);
	}

//	PORTA_pin_set_isc(0, PORT_ISC_INTDISABLE_gc);//reset and debug pin
	PORTA_pin_set_isc(1, PORT_ISC_INTDISABLE_gc);
	PORTA_pin_set_isc(2, PORT_ISC_INTDISABLE_gc);
	PORTA_pin_set_isc(3, PORT_ISC_INTDISABLE_gc);
	PORTA_pin_set_isc(4, PORT_ISC_INTDISABLE_gc);
	PORTA_pin_set_isc(5, PORT_ISC_INTDISABLE_gc);
	PORTA_pin_set_isc(6, PORT_ISC_INTDISABLE_gc);
	PORTA_pin_set_isc(7, PORT_ISC_INTDISABLE_gc);
//	PORTB_pin_set_isc(0, PORT_ISC_INTDISABLE_gc);
	PORTB_pin_set_isc(1, PORT_ISC_INTDISABLE_gc);
	PORTB_pin_set_isc(2, PORT_ISC_INTDISABLE_gc);
	PORTB_pin_set_isc(3, PORT_ISC_INTDISABLE_gc);
	PORTB_pin_set_isc(4, PORT_ISC_INTDISABLE_gc);
	PORTB_pin_set_isc(5, PORT_ISC_INTDISABLE_gc);
	PORTB_pin_set_isc(6, PORT_ISC_INTDISABLE_gc);
	PORTB_pin_set_isc(7, PORT_ISC_INTDISABLE_gc);
	PORTC_pin_set_isc(0, PORT_ISC_INTDISABLE_gc);
//	PORTC_pin_set_isc(1, PORT_ISC_INTDISABLE_gc);
NOP();
	PORTC_pin_set_isc(2, PORT_ISC_INTDISABLE_gc);
	PORTC_pin_set_isc(3, PORT_ISC_INTDISABLE_gc);
	PORTC_pin_set_isc(4, PORT_ISC_INTDISABLE_gc);
	PORTC_pin_set_isc(5, PORT_ISC_INTDISABLE_gc);
	VPORTA_INTFLAGS = 0xFF;
	VPORTB_INTFLAGS = 0xFF;
	VPORTC_INTFLAGS = 0x3F;
	TPB6_set_isc(PORT_ISC_RISING_gc);//Motion Switch
	TPC1_set_isc(PORT_ISC_LEVEL_gc);//C1=WAKEUP FROM BIG
    *((uint8_t*)0x0A4A)&=~0x31;//TCA0 Ints
    *((uint8_t*)0x0A4B)|=0x31;//TCA0 Flags
	  USART0.CTRLB = 0 << USART_MPCM_bp       /* Multi-processor Communication Mode: disabled */
	  | 0 << USART_ODME_bp     /* Open Drain Mode Enable: disabled */
	  | 0 << USART_RXEN_bp     /* Reciever enable: enabled */
	  | USART_RXMODE_NORMAL_gc /* Normal mode */
	  | 0 << USART_SFDEN_bp    /* Start Frame Detection Enable: disabled */
	  | 0 << USART_TXEN_bp;    /* Transmitter Enable: enabled */


	ENABLE_INTERRUPTS();
	SLPCTRL.CTRLA = 0x03;	//03=enable STANDBY SLEEP 0.0021 mA. 01=enable IDLE SLEEP  0.774 mA. 05=enable PWR DN SLEEP  0.0007 mA	but no RTC wakeup
	
	USART0.BAUD = (uint16_t)USART0_BAUD_RATE(9600); /* set baud rate register for LCD*/
	mState=0;
	
/**** DEV ****/	
/*for(crcrc=0;crcrc<40000;crcrc++){
	TPB0_set_dir(PORT_DIR_IN);
	TPB0_set_pull_mode(PORT_PULL_OFF);
	USART_0_initialization();
	LCDclear();
	usart_put_string(&("TEST "),5);
	LCDshort(crcrc,4);
	LCDbottomLine();
	LCDshort(rcvdCmd,2);
	usart_put_string(&(" "),1);
	LCDshort(rcvdPressureHigh,2);
	usart_put_string(&(" "),1);
	LCDshort(rcvdPressureLow,2);
	usart_put_string(&(" "),1);
	LCDshort(rcvdCrcHigh,2);
	usart_put_string(&(" "),1);
	LCDshort(rcvdCrcLow,2);

	delay_ms(1100);
}*/
/**** END DEV ****/


	LCDclear();
	usart_put_string(&("Startup C"),9);
	delay_ms(300);
	
for(;;){
	LCDclear();
	LCDshort(mState,2);
	LCDspace();	
	LCDshort(rcvdCmd,2);
	LCDspace();
	LCDshort(motParams[9],2);
	LCDshort(motParams[10],2);
	LCDspace();
	LCDshort(motParams[11],2);
	LCDshort(motParams[12],2);
	LCDbottomLine();
	LCDshort(motParams[13],2);
	LCDshort(motParams[14],2);
	LCDshort(motParams[15],2);
	LCDshort(motParams[16],2);
	LCDshort(motParams[17],2);
	LCDshort(motParams[18],2);
	LCDshort(motParams[19],2);
	LCDshort(motParams[20],2);

for(;;){
	redBlink(1);
	ENABLE_INTERRUPTS();//might not have to do this every time.
	NOP();
//	delay_ms(1000);
	SLEEP();
}
	switch(mState){
		case BEGIN://0
			if(chkMagnetSlow()){
				mState=FIRST_MAGNETON_CHECK;
				redBlinkLong(1);
				}
			else mState=FIRST_MAGNETOFF_CHECK;
		break;
		case FIRST_MAGNETON_CHECK://1
			if(chkMagnetSlow()){
				mState=SECOND_MAGNETON_CHECK;
				redBlinkLong(1);
			}else mState=FIRST_MAGNETOFF_CHECK;
		break;
		case SECOND_MAGNETON_CHECK://2
			if(chkMagnetSlow()){
				mState=MAGNET_IS_ON;
				redBlinkLong(3);
			}else{
				 mState=FIRST_MAGNETOFF_CHECK;
				 //ELSE SET SLEEP PIN TO V1
			}
		break;
		case MAGNET_IS_ON://3
			if(!chkMagnetFast()){
				mState=FIRST_MAGNETOFF_CHECK;
			}
		break;
		case FIRST_MAGNETOFF_CHECK://4
			if(!chkMagnetSlow()){
				redBlink(1);
				mState=SECOND_MAGNETOFF_CHECK;
			}else{
				mState=FIRST_MAGNETON_CHECK;
			}
		break;
		case SECOND_MAGNETOFF_CHECK://5
			if(!chkMagnetSlow()){
				redBlink(1);
				mState=MAGNET_WAS_RELEASED;
			}else{
				mState=FIRST_MAGNETON_CHECK;
			}
		break;
		case MAGNET_WAS_RELEASED://6
		//RELEASE SLEEP PIN TO V71
			mState=INITIAL_GPS;
		break;
		case INITIAL_GPS://7
			if(chkMagnetSlow()){
				mState=FIRST_MAGNETON_CHECK;
				redBlinkLong(1);
			}else{
				if(getGPS())mState=IDLE;
				else mState=NO_INITIAL_GPS_A;
			}
		break;
		case GET_GPS://8
			if(chkMagnetSlow()){
				mState=FIRST_MAGNETON_CHECK;
				redBlinkLong(1);
			}else{
				getGPS();
				mState=IDLE;
			}
		break;
		case NO_INITIAL_GPS_A://9
			if(chkMagnetSlow()){
				mState=FIRST_MAGNETON_CHECK;
				redBlinkLong(1);
			}else{
				if(!getGPS())mState=IDLE;
				else mState=NO_INITIAL_GPS_B;
			}
		break;
		case NO_INITIAL_GPS_B://A
			if(chkMagnetSlow()){
				mState=FIRST_MAGNETON_CHECK;
				redBlinkLong(1);
			}else{
				if(!getGPS())mState=IDLE;
				else mState=NO_INITIAL_GPS_C;
			}
		break;
		case NO_INITIAL_GPS_C://B
			if(chkMagnetSlow()){
				mState=FIRST_MAGNETON_CHECK;
				redBlinkLong(1);
			}else{
				if(!getGPS())mState=IDLE;
				else mState=NO_INITIAL_GPS_LONGTERM;
			}
		break;
		case NO_INITIAL_GPS_LONGTERM://0C
			if(chkMagnetSlow()){
				mState=FIRST_MAGNETON_CHECK;
				redBlinkLong(1);
			}else{
				if(!getGPS())mState=IDLE;
				else mState=NO_INITIAL_GPS_LONGTERM;
			}
		break;
		case IDLE://0D
		if((rcvdCmd&0x10)==0x10){
			rcvdCmd=0;
			mState=GET_GPS;
		}
		break;
		default: mState=IDLE;
		break;
	}


//	redBlink(1);
	ENABLE_INTERRUPTS();//might not have to do this every time.
	NOP();
//	delay_ms(1000);
	SLEEP();
}

	NOP();

	redBlink(10);

	g_archiveAddrHigh=0;
	g_archiveAddrLow=0;
	g_motionMask=0x80;
	for(iters=0;iters<8;iters++){g_motionData[iters]=0;}
	g_motionMinCtr=0;
	g_motionDataCtr=0;
	g_gotMotion=0;
	g_motion8bits=0;
	g_rxDataIndex=0;
	g_motion8ctr=0;

//    initSI1153();
//	ENABLE_INTERRUPTS();

	USART0.BAUD = (uint16_t)USART0_BAUD_RATE(9600); /* set baud rate register for LCD*/
//	USART0.BAUD = (uint16_t)USART0_BAUD_RATE(57600); /* set baud rate register for XBee*/


	char detectedMot=0;

	
	g_motionMinTotals=0;
	g_motionMask=0x80;
	for(iters=0;iters<8;iters++){g_motionData[iters]=0;}
	g_motionDataCtr=0;
//for(;;){	//interrupt driven mode
	
/*	if(PORTB_get_pin_level(6)){
		clearPIRinterrupt();
		redBlink(1);
		g_motion8bits|=g_motionMask;
		g_motionMinTotals++;
	}*/
	
	g_motionMask=g_motionMask>>1;
	g_motionDataCtr++;//counts up to 7, used to drive mask
	if(g_motionDataCtr>7){
		g_motionData[g_motion8ctr]=g_motion8bits;
		g_motion8ctr++;
		g_motion8bits=0;
		g_motionMask=0x80;
		g_motionDataCtr=0;
	}

	g_motionMinCtr++;

}
/*****************************************************************************************************
******************************************************************************************************
***********************  GPS  ************************************************************************
******************************************************************************************************/
/***********************************************************************/
/*  GET GPS
/***********************************************************************/
char getGPS(void){
	short crcrc;
	char iters;
	LCDclear();
	usart_put_string(&("GPS "),4);
	delay_ms(300);
	redBlink(3);
	openGPSuBlox();
	initGPSuBlox();
	timeToFix=0;
	gTryUblox();//does 3000 trials (50mins), then cycles pwr to GPS, runs again.
	if (uBloxStatus!='A') {
		if (chkMagnetSlow()){    // if GPS not good yet and magnet was installed
			mState=FIRST_MAGNETOFF_CHECK;
			closeGPS();
		}
		return 1;
		}else{        //if good GPS because 3x oneSecs with 'A' response
		GPSgetDate();//tries for 3 of 4 ZDA messages. Then grabs date for param[0] and param[1]  //Add loop test of return value
		GPSgetTime();
		closeGPS();
		send23BytesToBig(gpsParams);
		
		LCDclear();
		LCDshort(gpsParams[0], 2);
		LCDshort(gpsParams[1], 2);
		LCDshort(gpsParams[2], 2);
		LCDshort(gpsParams[3], 2);
		LCDshort(gpsParams[4], 2);
		LCDshort(gpsParams[5], 2);
		LCDshort(gpsParams[6], 2);
		LCDshort(gpsParams[7], 2);
		LCDbottomLine();
		LCDshort(gpsParams[8], 2);
		LCDshort(gpsParams[9], 2);
		LCDshort(gpsParams[10], 2);
		LCDshort(gpsParams[11], 2);
		LCDshort(gpsParams[12], 2);
		LCDshort(gpsParams[13], 2);
		LCDshort(gpsParams[14], 2);
		LCDshort(gpsParams[15], 2);
		delay_ms(2000);//120 is too short with v71 at either 3 or 32 clock cycle trigger. Might need longer without
		
		return 0;
		}
}
/***********************************************************************/
/*  GPS Search Receive Buff for two chars, return 66 if can't find
/***********************************************************************/
int searchRcvBuff (char a, char b){
	int j=666;
	GPSSearchPtr = p_rcvData;
	do{
		GPSSearchPtr--;
		if ((*GPSSearchPtr==a)&&(*(GPSSearchPtr+1)==b)){j=0;};
	}while((GPSSearchPtr >= g_uc_receive_buffer)&&(j==666));
	return j;
}
/***********************************************************************/
/*  GPS Search Receive Buff for two chars, return 66 if can't find
/***********************************************************************/
int searchRcvBuffuBlox (char a, char b, char c){
	int ret=666;
	GPSSearchPtr = g_uc_receive_buffer-1;
	do{
		GPSSearchPtr++;
		if ((*GPSSearchPtr==a)&&(*(GPSSearchPtr+1)==b)&&(*(GPSSearchPtr+2)==c)){ret=0;};
	}while((GPSSearchPtr < (p_rcvData-5))&&(ret==666));
	return ret;
}
/***********************************************************************/
/*  GPS Try 3 Req Date uBlox
/***********************************************************************/
char GPSgetDate (void){
	if((gpsReqDateUblox()==0)&&((*(GPSSearchPtr+10))!='X'))return 0;
	if((gpsReqDateUblox()==0)&&((*(GPSSearchPtr+10))!='X'))return 0;
	if((gpsReqDateUblox()==0)&&((*(GPSSearchPtr+10))!='X'))return 0;
	if((gpsReqDateUblox()==0)&&((*(GPSSearchPtr+10))!='X'))return 0;
	if((gpsReqDateUblox()==0)&&((*(GPSSearchPtr+10))!='X'))return 0;else return 666;
}
/***********************************************************************/
/*  GPS Req Date uBlox
/***********************************************************************/
char gpsReqDateUblox (void){
	char i=5;
	unsigned char validDate, numCommas=0;
	char stuffToSend[] = {'$','G','N','G','N','Q',',','Z','D','A','*',0x32,0x32,0x0D,0x0A};
	char validCtr=0;
	char monthHighTest[4];
	char monthLowTest[4];
	char dayLowTest[4];
	char dayHighTest[4];
	for(i=0;i<4;i++){//try it four times. This value must match the size of dateGoodTestX[4]
		usart_put_string(stuffToSend,15);
		clrRx();
		USART0.CTRLA |= USART_RXCIE_bm;
		ENABLE_INTERRUPTS();
		delay_ms(1100);
		timeToFix++;
		if((timeToFix>MAX_GPS_ACQ)&&(mState!=INITIAL_GPS)){
			uBloxStatus=66;
			return 66;
		}
	  	USART0.CTRLA &= ~USART_RXCIE_bm;//DISABLE Rx Interrupt
		if(searchRcvBuffuBlox('Z','D','A')==0){
			numCommas=0;
			while (GPSSearchPtr < &(g_uc_receive_buffer[BUFFER_SIZE])){
				GPSSearchPtr++;
				if(*GPSSearchPtr == ','){
					numCommas++;
					if(numCommas==2){
						if(((*(GPSSearchPtr+1))==',')||((*(GPSSearchPtr+5))=='X'))break;
						else{
							monthLowTest[validCtr]=(*(GPSSearchPtr+5));
							monthHighTest[validCtr]=(*(GPSSearchPtr+4));
							dayLowTest[validCtr]=(*(GPSSearchPtr+2));
							dayHighTest[validCtr]=(*(GPSSearchPtr+1));
							validCtr++;
							break;
		}
		}
		}
		}
	}
	}//end of if 3 of 4 are good, including the last one

validCtr=0;
if(monthLowTest[3]==monthLowTest[2])validCtr++;
if(monthLowTest[3]==monthLowTest[1])validCtr++;
if(monthLowTest[3]==monthLowTest[0])validCtr++;
if(validCtr<3)return 666;	
validCtr=0;
if(monthHighTest[3]==monthHighTest[2])validCtr++;
if(monthHighTest[3]==monthHighTest[1])validCtr++;
if(monthHighTest[3]==monthHighTest[0])validCtr++;
if(validCtr<3)return 666;
validCtr=0;
if(dayLowTest[3]==dayLowTest[2])validCtr++;
if(dayLowTest[3]==dayLowTest[1])validCtr++;
if(dayLowTest[3]==dayLowTest[0])validCtr++;
if(validCtr<3)return 666;
validCtr=0;
if(dayHighTest[3]==dayHighTest[2])validCtr++;
if(dayHighTest[3]==dayHighTest[1])validCtr++;
if(dayHighTest[3]==dayHighTest[0])validCtr++;
if(validCtr<3)return 666;
for(validCtr=0;validCtr<SIZEOFGPSPARAMS;validCtr++){
	gpsParams[validCtr]=0;
}
if((dayLowTest[3])!=','){
	gpsParams[0]=((char)(asciiToHex(0x30,*(GPSSearchPtr+10)))<<4)		   /*last digit of year*/
	+asciiToHex(monthHighTest[3],monthLowTest[3]);		/*month uBlox*/
	gpsParams[1]=(asciiToHex(dayHighTest[3],dayLowTest[3]))<<3; 	/*day uBlox*/
	return 0;
	}else{
	return 66;
}
}
/***********************************************************************/
/*  GPS Req Status
/***********************************************************************/
char gpsReqStatusUblox (void){
	int i=5;
	unsigned char validLoc, numCommas=0;
	validLoc=0;
	uBloxStatus=0;
	for(i=3;i>0;i--){//try it four times
		clrRx();
		USART0.CTRLA |= USART_RXCIE_bm;
		ENABLE_INTERRUPTS();
		delay_ms(1100);
		timeToFix++;
		if((timeToFix>MAX_GPS_ACQ)&&(mState!=INITIAL_GPS)){
			uBloxStatus=66;
			return 66;
		}
	  	USART0.CTRLA &= ~USART_RXCIE_bm;//DISABLE Rx Interrupt
		if(searchRcvBuffuBlox('R','M','C')==0){
			numCommas=0;
			while (GPSSearchPtr < &(g_uc_receive_buffer[BUFFER_SIZE])){
				GPSSearchPtr++;
				if(*GPSSearchPtr == ','){
					numCommas++;
					if(numCommas==2){
						if(((*(GPSSearchPtr+1))==',')||((*(GPSSearchPtr+1))=='X'))break;
						else{
							if(*(GPSSearchPtr+1)=='A')
								validLoc++;
								break;
							}
						}
					}
				}
			}
	}//end of if 3 of 4 are good, including the last one
	if((validLoc>=2)&&(*(GPSSearchPtr+1)=='A')){  //gets here pretty quick after status goes to 'A'
	uBloxStatus='A';
	return 0;
	}else{
	uBloxStatus=66;
	return 66;
	}
}
/***********************************************************************/
/*  GPS Try 5 Req Loc uBlox
/***********************************************************************/
char GPSgetTime(void){
	if((gpsReqLocUblox()==0)&&((*(GPSSearchPtr+43))!='X'))return 0;
	if((gpsReqLocUblox()==0)&&((*(GPSSearchPtr+43))!='X'))return 0;
	if((gpsReqLocUblox()==0)&&((*(GPSSearchPtr+43))!='X'))return 0;
	if((gpsReqLocUblox()==0)&&((*(GPSSearchPtr+43))!='X'))return 0;
	if((gpsReqLocUblox()==0)&&((*(GPSSearchPtr+43))!='X'))return 0;
	if((gpsReqLocUblox()==0)&&((*(GPSSearchPtr+43))!='X'))return 0;
	if((gpsReqLocUblox()==0)&&((*(GPSSearchPtr+43))!='X'))return 0;else return 666;
}
/***********************************************************************/
/*  GPS Req Number of Satellites
/***********************************************************************/
char gpsReqLocUblox (void){
	char i;
	unsigned char validDate, numCommas=0;
	char stuffToSend[] = {'$','G','N','G','N','Q',',','G','N','S','*',0x32,0x37,0x0D,0x0A};
	char validCtr=0;
	char hourHighTest[4];
	char hourLowTest[4];
	char minLowTest[4];
	char minHighTest[4];

	for(i=4;i>0;i--){//try it four times.This value must match the size of monthHourLowTest[4]
		usart_put_string(stuffToSend,15);
		clrRx();
		USART0.CTRLA |= USART_RXCIE_bm;
		ENABLE_INTERRUPTS();
		delay_ms(1100);
		if((timeToFix>MAX_GPS_ACQ)&&(mState!=INITIAL_GPS)){
			uBloxStatus=66;
			return 66;
		}
		timeToFix++;
	  	USART0.CTRLA &= ~USART_RXCIE_bm;//DISABLE Rx Interrupt
		if(searchRcvBuffuBlox('G','N','S')==0){
			numCommas=0;
			while (GPSSearchPtr < &(g_uc_receive_buffer[BUFFER_SIZE])){
			GPSSearchPtr++;
				if(*GPSSearchPtr == ','){
					numCommas++;
					if(numCommas==2){
						if(((*(GPSSearchPtr+1))==',')||((*(GPSSearchPtr+1))=='X'))break;
						else{
							hourLowTest[validCtr]=(*(GPSSearchPtr-8));
							hourHighTest[validCtr]=(*(GPSSearchPtr-9));
							minLowTest[validCtr]=(*(GPSSearchPtr-6));
							minHighTest[validCtr]=(*(GPSSearchPtr-7));
							validCtr++;
							break;
							}
						}
				}
			}
		}
	}//end of if 3 of 4 are good, including the last one
	validCtr=0;
	if(hourLowTest[3]==hourLowTest[2])validCtr++;
	if(hourLowTest[3]==hourLowTest[1])validCtr++;
	if(hourLowTest[3]==hourLowTest[0])validCtr++;
	if(validCtr<3)return 666;	
	validCtr=0;
	if(hourHighTest[3]==hourHighTest[2])validCtr++;
	if(hourHighTest[3]==hourHighTest[1])validCtr++;
	if(hourHighTest[3]==hourHighTest[0])validCtr++;
	if(validCtr<3)return 666;
	validCtr=0;
	if(minHighTest[3]==minHighTest[2])validCtr++;
	if(minHighTest[3]==minHighTest[1])validCtr++;
	if(minHighTest[3]==minHighTest[0])validCtr++;
	if(validCtr<3)return 666;
	validCtr=0;
	if(minLowTest[3]==minLowTest[2])validCtr++;
	if(minLowTest[3]==minLowTest[1])validCtr++;
	if(minLowTest[3]==minLowTest[0])validCtr++;
	if(validCtr<3)return 666;
	if((minLowTest[3])!=','){
	char gotHour=(asciiToHex(hourHighTest[3],hourLowTest[3]));
	char gotSecs=(asciiToHex((*(GPSSearchPtr+31)),(*(GPSSearchPtr+32))));  ///uBLOX sats
	gpsParams[1]+=(gotHour>>2);
	gpsParams[2]=gotHour<<6;
	gpsParams[2]+=asciiToHex(minHighTest[3],minLowTest[3]);
	gpsParams[3]+=((char)(asciiToHex((*(GPSSearchPtr-5)),(*(GPSSearchPtr-4)))));	/*Seconds*/
	if (((char)(*(GPSSearchPtr+12)))=='N')gpsParams[4]|=0x80;//LAT SIGN. gpsParams were initialized to zero in getDate
	
	gpsParams[4]+=((char)(asciiToHex((*(GPSSearchPtr+1)),(*(GPSSearchPtr+2)))));	/*LAT WHOLE uBlox*/
	
	gpsParams[5]=asciiToHex(((*(GPSSearchPtr+3))),((unsigned int)(*(GPSSearchPtr+4))));//LAT FRAC
	gpsParams[6]=asciiToHex(((*(GPSSearchPtr+6))),((unsigned int)(*(GPSSearchPtr+7))));
	gpsParams[7]=asciiToHex(((*(GPSSearchPtr+8))),((unsigned int)(*(GPSSearchPtr+9))));
  
	if (((char)(*(GPSSearchPtr+26)))=='E')gpsParams[8]|=0x40;//LONG SIGN. gpsParams were initialized to zero in getDate
	if (((char)(*(GPSSearchPtr+14)))=='1')gpsParams[9]=100; /*high digit of Long whole uBlox*/
	gpsParams[9]+=((char)(asciiToHex((*(GPSSearchPtr+15)),(*(GPSSearchPtr+16)))));	/*LONG WHOLE uBlox*/
  
	gpsParams[10]=asciiToHex(((*(GPSSearchPtr+17))),((unsigned int)(*(GPSSearchPtr+18))));//LONG FRAC
	gpsParams[11]=asciiToHex(((*(GPSSearchPtr+20))),((unsigned int)(*(GPSSearchPtr+21))));
	gpsParams[12]=asciiToHex(((*(GPSSearchPtr+22))),((unsigned int)(*(GPSSearchPtr+23))));

	gpsParams[13]=timeToFix;
	gpsParams[14]=(char)(asciiToHex((*(GPSSearchPtr+31)),(*(GPSSearchPtr+32))));  ///uBLOX sats
	
  
	return 0;
}else return 66;
}
/***********************************************************************/
/*  Try uBlox GPS
/*********gpsReqNumSatsUblox**************************************************************/
char gTryUblox (void) {
	char i,j,k,probablyOK=0;
	i=8;
	do{
		j=255;
		do{
			k=3;
			do{
				delay_ms(250);
				clrRx();
				USART0.CTRLA |= USART_RXCIE_bm;
				ENABLE_INTERRUPTS();
				probablyOK = gpsReqStatusUblox();//This takes a little more than one sec
				k--;
			}while((k>0)&&(probablyOK == 66)&&(chkMagnetSlow()==0));
			/*end try*/
			j--;
		}while ((j>0)&&(probablyOK == 66)&&(chkMagnetSlow()==0));  //end 4000x middle loop

		if(probablyOK==0x66){
		closeGPS();
		delay_ms(1000);
		openGPSuBlox();
		initGPSuBlox();
		};
		i--;
	}while((i>0)&&(probablyOK == 66)&&(chkMagnetSlow()==0)); //end 8x outer loop

	return probablyOK; /*todooooooooooooooooooooo what should this be?*/
}/***********************************************************************/
/*  GPS Init  uBlox
	 TPA5_set_dir(PORT_DIR_IN);		//Sense
	 TPA4_set_dir(PORT_DIR_OUT);	//Pwr
	 PORTA_set_pin_level(4, 1);		//Pwr

/***********************************************************************/
void openGPSuBlox (void) {
	  TPB4_set_dir(PORT_DIR_OUT);	//GPS Reset
	  TPB3_set_dir(PORT_DIR_OUT);	//From GPS
	  TPB2_set_dir(PORT_DIR_OUT);	//To GPS
	  PORTB_set_pin_level(2, 1);	//To GPS
	  PORTB_set_pin_level(3, 1);	//From GPS
	  PORTB_set_pin_level(4, 1);	//GPS Reset
	  TPA2_set_dir(PORT_DIR_OUT);	//GPS PWRC
	  PORTA_set_pin_level(2, 1);	//GPS PWRC
	  TPB3_set_dir(PORT_DIR_IN);	//From GPS
	  USART0.BAUD = (uint16_t)USART0_BAUD_RATE(9600); /* set baud rate register for LCD*/
	  USART0.CTRLB = 0 << USART_MPCM_bp       /* Multi-processor Communication Mode: disabled */
	  | 0 << USART_ODME_bp     /* Open Drain Mode Enable: disabled */
	  | 1 << USART_RXEN_bp     /* Reciever enable: enabled */
	  | USART_RXMODE_NORMAL_gc /* Normal mode */
	  | 0 << USART_SFDEN_bp    /* Start Frame Detection Enable: disabled */
	  | 1 << USART_TXEN_bp;    /* Transmitter Enable: enabled */
	  p_rcvData = g_uc_receive_buffer;
}
/***********************************************************************/
/*  Close GPS
/***********************************************************************/
void closeGPS (void) {
	  USART0.CTRLB = 0 << USART_MPCM_bp       /* Multi-processor Communication Mode: disabled */
	  | 0 << USART_ODME_bp     /* Open Drain Mode Enable: disabled */
	  | 0 << USART_RXEN_bp     /* Reciever enable: enabled */
	  | USART_RXMODE_NORMAL_gc /* Normal mode */
	  | 0 << USART_SFDEN_bp    /* Start Frame Detection Enable: disabled */
	  | 0 << USART_TXEN_bp;    /* Transmitter Enable: enabled */
	  TPA2_set_dir(PORT_DIR_OUT);	//GPS PWRC
	  PORTA_set_pin_level(2, 0);	//GPS PWRC
	  TPB4_set_dir(PORT_DIR_OUT);	//GPS Reset
	  TPB3_set_dir(PORT_DIR_OUT);	//From GPS
///	  TPB2_set_dir(PORT_DIR_OUT);	//To GPS
///	  PORTB_set_pin_level(2, 0);	//To GPS
	  PORTB_set_pin_level(3, 0);	//From GPS
//	  PORTB_set_pin_level(4, 0);	//GPS Reset. Leaving this high shortens acq time, but likrly draws current
}
/***********************************************************************/
/*  GPS Init for uBlox
 *	uBlox: B5 62 class(1) ID(1) LenPayload(2) payload CRC(2)
 */
/***********************************************************************/
void initGPSuBlox (void) {
  delay_ms(1000);    //Aug 2011, Trimble would fail at halfSec. Didn't use for HCS12
  char stuffToSend[] = {'$','P','U','B','X',',','4','0',',','G','L','L',',','1',',','0',',','1',',','1',',','1',',','0','*',0x35,0x43,0x0D,0x0A};
  usart_put_string(stuffToSend, 29);
  char stuffToSend2[] = {'$','P','U','B','X',',','4','0',',','G','S','A',',','1',',','0',',','1',',','1',',','1',',','0','*',0x34,0x45,0x0D,0x0A};
  usart_put_string(stuffToSend2, 29);
  char stuffToSend3[] = {'$','P','U','B','X',',','4','0',',','G','S','V',',','1',',','0',',','1',',','1',',','1',',','0','*',0x35,0x39,0x0D,0x0A};
  usart_put_string(stuffToSend3, 29);
  char stuffToSend4[] = {'$','P','U','B','X',',','4','0',',','G','G','A',',','1',',','0',',','1',',','1',',','1',',','0','*',0x35,0x41,0x0D,0x0A};
  usart_put_string(stuffToSend4, 29);
  char stuffToSend5[] = {'$','P','U','B','X',',','4','0',',','V','T','G',',','1',',','0',',','1',',','1',',','1',',','0','*',0x35,0x45,0x0D,0x0A};
  usart_put_string(stuffToSend5, 29);
  }
/***********************************************************************/
/*  Ascii to Hex.  Converts two ascii DECIMAL digits to hex integer.
/***********************************************************************/
int asciiToHex(char hiDig, char loDig){
	int hexTotal=0;
	switch (loDig){
	case 0x30: hexTotal=0;break;
	case 0x31: hexTotal=1;break;
	case 0x32: hexTotal=2;break;
	case 0x33: hexTotal=3;break;
	case 0x34: hexTotal=4;break;
	case 0x35: hexTotal=5;break;
	case 0x36: hexTotal=6;break;
	case 0x37: hexTotal=7;break;
	case 0x38: hexTotal=8;break;
	case 0x39: hexTotal=9;break;
	default: hexTotal=0;break;
	};
	switch (hiDig){
	case 0x30: hexTotal+=0;break;
	case 0x31: hexTotal+=10;break;
	case 0x32: hexTotal+=20;break;
	case 0x33: hexTotal+=30;break;
	case 0x34: hexTotal+=40;break;
	case 0x35: hexTotal+=50;break;
	case 0x36: hexTotal+=60;break;
	case 0x37: hexTotal+=70;break;
	case 0x38: hexTotal+=80;break;
	case 0x39: hexTotal+=90;break;
	default: hexTotal=+0;break;
	};
return hexTotal;
}/*******************************************
*    RED BLINK
********************************************/
void redBlink (char numBlinks){
	char ctr;
	LED_set_dir(PORT_DIR_OUT);//no measured diff for IN or OUT
	for(ctr=numBlinks;ctr>0;ctr--){
		PORTC_set_pin_level(2, 0);//my LED. 1=ON.(opposite for dev kit)
		delay_ms(10);
		PORTC_set_pin_level(2, 1);//my LED. 0=OFF.
		delay_ms(50);
	}
	LED_set_dir(PORT_DIR_IN);
	LED_set_pull_mode(PORT_PULL_UP);// Min current with either input PU or output
}
/*******************************************
*    RED BLINK
********************************************/
void redBlinkLong (char numBlinks){
	char ctr;
	LED_set_dir(PORT_DIR_OUT);//no measured diff for IN or OUT
	for(ctr=numBlinks;ctr>0;ctr--){
		PORTC_set_pin_level(2, 0);//my LED. 1=ON.(opposite for dev kit)
		delay_ms(300);
		PORTC_set_pin_level(2, 1);//my LED. 0=OFF.
		delay_ms(50);
	}
	LED_set_dir(PORT_DIR_IN);
	LED_set_pull_mode(PORT_PULL_UP);// Min current with either input PU or output
}
/*******************************************
*    MS DELAY
********************************************/
void delay_ms (uint32_t msToDelay){
	uint32_t tickCtr,msCtr;
	for(msCtr=msToDelay;msCtr>0;msCtr--){
		for(tickCtr=TICK_CONSTANT_MS;tickCtr>0;tickCtr--){NOP();}
	}
}
/*******************************************
*    HUNDRED uS DELAY
********************************************/
void myDelay_hus (uint32_t husToDelay){
	uint32_t tickCtr,tusCtr;
	for(tusCtr=husToDelay;tusCtr>0;tusCtr--){
		for(tickCtr=TICK_CONSTANT_HUS;tickCtr>0;tickCtr--){NOP();}
	}
}
/*******************************************
*    PIR SETUP DELAY
********************************************/
void pirSUdelay (void){
	uint32_t tickCtr;
	for(tickCtr=PIR_SU_DELAY;tickCtr>0;tickCtr--){NOP();}
}
/*******************************************
*    PIR RX DELAY
********************************************/
void pirCLKdelay (void){
	uint32_t tickCtr;
	for(tickCtr=PIR_CLK_DELAY;tickCtr>0;tickCtr--){NOP();}
}
/*******************************************
*    SEND SPI BUFF
********************************************/
void sendSPIbuff(char* sendBuff,char len){
	char lenCtr,garbage;
	PORTA_set_pin_level(4, 0);//SPI SS/CS
	for(lenCtr=0;lenCtr<len;lenCtr++){
			while(!(SPI0.INTFLAGS & SPI_DREIE_bm)){NOP();}
			SPI0.DATA = sendBuff[lenCtr];
			garbage=SPI0.DATA;
	}
			garbage=SPI0.DATA;
}
/*******************************************
*    SEND SPI BYTE
********************************************/
void sendSPIbyte(char byteToSend){
	while(!(SPI0.INTFLAGS & SPI_DREIE_bm)){NOP();}
	SPI0.DATA = byteToSend;
}
/*******************************************
*    RECEIVE SPI BUFF
********************************************/
void rcvSPIbuff(char* rcvBuffer){
	while(!(SPI0.INTFLAGS & SPI_RXCIE_bm)){NOP();}
		*rcvBuffer = SPI0.DATA;
}
/*******************************************
 * \brief Write one character on USART_0
 *******************************************/
int8_t USART_0_putc(const uint8_t data)
{
	while( !(USART0.STATUS & USART_DREIF_bm) );
	USART0.TXDATAL = data;
	return 0;
}

// USART Functions
void usart_put_string(const char str[], const uint8_t STR_LEN)
{
	for (int i = 0; i < STR_LEN; i++) {
		while( !(USART0.STATUS & USART_DREIF_bm) );
		USART0.TXDATAL = str[i];
	}
}
/***********************************************************************/
 /*  send LCD int
 /***********************************************************************/
 void LCDshort(short a, char d){
    short z,zz;
    z=hex2ToAscii((a>>8)&0x000000FF);
    zz=hex2ToAscii(a&0x000000FF);
    if(d>=4){
      char sendchars2[4]={
    	(char)(z>>8),(char)(z),(char)(zz>>8),(char)(zz)};
      usart_put_string(sendchars2,4);
    }
    if(d==3){
      char sendchars2[3]={
     (char)(z),(char)(zz>>8),(char)(zz)};
      usart_put_string(sendchars2,3);
    }
    if(d==2){
      char sendchars2[2]={
      (char)(zz>>8),(char)(zz)};
      usart_put_string(sendchars2,2);
    }
    if(d==1){
      char sendchars2[1]={(char)(zz)};
      usart_put_string(sendchars2,1);
    }
 }
   /***********************************************************************/
 /*  hex2ToAscii 4 bytes hex to 8 bytes Ascii
 /***********************************************************************/
 unsigned int hex2ToAscii(int hexx){
   int a,b,c,d;
   hexx &= 0x0000FFFF;
   a = (hex1ToAscii(hexx));
   hexx >>= 4;
   b = (hex1ToAscii(hexx));
   b <<= 8;
   hexx >>= 4;
   c = (hex1ToAscii(hexx));
   c <<= 16;
   hexx >>= 4;
   d = (hex1ToAscii(hexx));
   d <<= 24;
   return a + b + c + d;
 }
/***********************************************************************/
 /*  hex1ToAscii nibble to 2-byte Ascii
 /***********************************************************************/
 char hex1ToAscii(char hex){
   int a;
   hex &= 0x000F;
     switch (hex) {
       case 0: a = 0x30;
       break;
       case 1: a = 0x31;
       break;
       case 2: a = 0x32;
       break;
       case 3: a = 0x33;
       break;
       case 4: a = 0x34;
       break;
       case 5: a = 0x35;
       break;
       case 6: a = 0x36;
       break;
       case 7: a = 0x37;
       break;
       case 8: a = 0x38;
       break;
       case 9: a = 0x39;
       break;
       case 10: a = 0x41;
       break;
       case 11: a = 0x42;
       break;
       case 12: a = 0x43;
       break;
       case 13: a = 0x44;
       break;
       case 14: a = 0x45;
       break;
       case 15: a = 0x46;
       break;
     };
    return a;
 }

 /***********************************************************************
 *    LCD Functions
 ***********************************************************************/
void LCDclear (void){
	USART_0_initialization();
	char sndStr2[8] = {0xFE,0x46,0xFE,0x48,0xFE,0x4C,0xFE,0x51};//clear display, cursor home
	usart_put_string(sndStr2,8);
	delay_ms(10);
}
void LCDbottomLine (void){
	char sndStr3[3] = {0xFE,0x45,0x40};//clear display, cursor home
	usart_put_string(sndStr3,3);
	delay_ms(10);
}
void LCDspace (void){
	usart_put_string(&(" "),1);
	delay_ms(10);
}

/***********************************************************************
*  XBee Clear Rx Buff and initialize the pointer
***********************************************************************/
void clrRx (void){
  int i;
	p_rcvData = g_uc_receive_buffer;
   for(i=0;i<BUFFER_SIZE;i++){
     p_rcvData[i]='X';
     }
	p_rcvData = g_uc_receive_buffer;
 }
/**********************************************************************
***********************************************************************
*********************** BIG-SMALL COMMS *******************************
***********************************************************************
***********************************************************************
/*******************************************
*    GET CMD FROM BIG
********************************************/
/*char getCmdFromBig (void){    //Alternate version (different pin) in driver_isr.c
	DISABLE_INTERRUPTS();
	TPC1_set_dir(PORT_DIR_IN);
	TPC1_set_pull_mode(PORT_PULL_OFF);

	char bigTinyTimeoutCtr;
	char ret=0;

	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==0)break;
	if(PORTC_get_pin_level(1)==1)  ret |= 0x08;

	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==1)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==0)break;
	if(PORTC_get_pin_level(1)==1)  ret |= 0x04;

	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==1)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==0)break;
	if(PORTC_get_pin_level(1)==1)  ret |= 0x02;

	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==1)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==0)break;
	if(PORTC_get_pin_level(1))  ret |= 0x01;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==1)break;

	PORTC.INTFLAGS |= (1 << 3);
	ENABLE_INTERRUPTS();
	return ret;
}*/
/*******************************************
*    SEND BYTE TO BIG
********************************************/
/*void sendByteToBig (char byteToSendToBig){  //Alternate version (different pin) in driver_isr.c
	char bigTinyTimeoutCtr;
	TPC1_set_dir(PORT_DIR_OUT);

	if((byteToSendToBig & 0x80)==0)PORTC_set_pin_level(1, 0);
	else PORTC_set_pin_level(1, 1);

	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==0)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==1)break;

	if((byteToSendToBig & 0x40)==0)PORTC_set_pin_level(1, 0);
	else PORTC_set_pin_level(1, 1);
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==0)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==1)break;

	if((byteToSendToBig & 0x20)==0)PORTC_set_pin_level(1, 0);
	else PORTC_set_pin_level(1, 1);
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==0)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==1)break;

	if((byteToSendToBig & 0x10)==0)PORTC_set_pin_level(1, 0);
	else PORTC_set_pin_level(1, 1);
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==0)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==1)break;

	if((byteToSendToBig & 0x08)==0)PORTC_set_pin_level(1, 0);
	else PORTC_set_pin_level(1, 1);
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==0)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==1)break;

	if((byteToSendToBig & 0x04)==0)PORTC_set_pin_level(1, 0);
	else PORTC_set_pin_level(1, 1);
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==0)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==1)break;

	if((byteToSendToBig & 0x02)==0)PORTC_set_pin_level(1, 0);
	else PORTC_set_pin_level(1, 1);
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==0)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==1)break;

	if((byteToSendToBig & 0x01)==0)PORTC_set_pin_level(1, 0);
	else PORTC_set_pin_level(1, 1);
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==0)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==1)break;

	TPC1_set_dir(PORT_DIR_IN);

}*/
/***********************************************************************/
 /*  DL05-6 Check Magnet  0=magnet off, 1=magnet on  MCFV1AC256 Fast version if already in mState=0;
 /***********************************************************************/
 char chkMagnetFast (void){
	 TPA5_set_dir(PORT_DIR_IN);		//Sense
	 TPA4_set_dir(PORT_DIR_OUT);	//Pwr
	 PORTA_set_pin_level(4, 1);		//Pwr
	 if(PORTA_get_pin_level(5)==0){	//Sense
		 PORTA_set_pin_level(4, 0);	//Pwr
		 TPA5_set_pull_mode(PORT_PULL_OFF); //Sense. remove pullup when sw is closed
		 TPA4_set_dir(PORT_DIR_IN); //Pwr
		 return 1;
	 }else{
		 TPA5_set_pull_mode(PORT_PULL_UP); //Sense. use pullup when sw is open
		 TPA4_set_dir(PORT_DIR_IN); //Pwr
		 return 0;
		 }
}
/***********************************************************************/
 /*  DL05-6 Check Magnet  0=magnet off, 1=magnet on  MCFV1AC256 Slow version confirms magnet really on instead of
 /*  just bumped the reed switch. Created June 2012 v1.05
 /***********************************************************************/
 char chkMagnetSlow (void){
	 TPA5_set_dir(PORT_DIR_IN);		//Sense
	 TPA4_set_dir(PORT_DIR_OUT);	//Pwr
	 PORTA_set_pin_level(4, 1);		//Pwr
	 if(PORTA_get_pin_level(5)==0){	//Sense
		 PORTA_set_pin_level(4, 0);	//Pwr
		 delay_ms(150);
		 if(PORTA_get_pin_level(5)==0){	//Sense
			 PORTA_set_pin_level(4, 0);	//Pwr
			 TPA5_set_pull_mode(PORT_PULL_OFF); //Sense. remove pullup when sw is closed
			 TPA4_set_dir(PORT_DIR_IN); //Pwr
			 return 1;
		 }else{
			 TPA5_set_pull_mode(PORT_PULL_UP); //Sense. use pullup when sw is open
			 TPA4_set_dir(PORT_DIR_IN); //Pwr
			 return 0;
			}
		}else{
		TPA5_set_pull_mode(PORT_PULL_UP); //Sense. use pullup when sw is open
		TPA4_set_dir(PORT_DIR_IN); //Pwr
		return 0;
		}
 }
/***********************************************************************/
/*  CRC   tag IDs took 550usec at intern ref clk,0div
/***********************************************************************/
unsigned short calcCRC(char cbuff[], char LEN) {
	int i,j;
	unsigned short X = 0xFFFF;
	unsigned short Y = 0x0080;
	unsigned short Z;
	for (i=0;i<LEN;i++){       //for each element
	Y = 0x0080;
	for (j=0;j<8;j++){
	Z = X;
	X <<= 1;
	if((Y & cbuff[i]) != 0){ X++;};
	Y >>= 1;
	if ((Z & 0x8000) != 0) {X ^= 0x1021; };
	};   //end 8x
	//    __RESET_WATCHDOG();	/*needed Jan 2014*/
	};    // end for each element
	for (i=0;i<16;i++){
	if ((X & 0x8000) != 0) { X<<=1; X ^= 0x1021; } else X <<= 1;
	};     //end 16x
	return X;
}
/*******************************************
*    GET CMD PACKET FROM BIG
********************************************/
char getPacketFromBig (void){
	unsigned short crcrc;
	char rcdArray[5];
	char iters;
	char minuteCycle;
	DISABLE_INTERRUPTS();
	TPB0_set_dir(PORT_DIR_IN);
	TPB0_set_pull_mode(PORT_PULL_OFF);

	rcdArray[0]=getByteFromBig();
	rcdArray[1]=getByteFromBig();
	rcdArray[2]=getByteFromBig();
	rcdArray[3]=getByteFromBig();
	rcdArray[4]=getByteFromBig();
	
	crcrc=calcCRC(rcdArray,3);
	if(crcrc==(rcdArray[3]<<8)+rcdArray[4]){
		rcvdCmd=rcdArray[0];
		minuteCycle=rcdArray[0]&0x07;
		switch (minuteCycle){
			case 0:
			motParams[9]=rcdArray[1];
			motParams[10]=rcdArray[2];
			break;
			case 1:
			motParams[11]=rcdArray[1];
			motParams[12]=rcdArray[2];
			break;
			case 2:
			motParams[13]=rcdArray[1];
			motParams[14]=rcdArray[2];
			break;
			case 3:
			motParams[15]=rcdArray[1];
			motParams[16]=rcdArray[2];
			break;
			case 4:
			motParams[17]=rcdArray[1];
			motParams[18]=rcdArray[2];
			break;
			default:
			motParams[19]=rcdArray[1];
			motParams[20]=rcdArray[2];
			send23BytesToBig(motParams);
			for (iters=0;iters<SIZEOFMOTPARAMS;iters++)motParams[iters]=0;
			break;
		}
		
	}else rcvdCmd = 0x66;

	mState=IDLE;
	PORTC.INTFLAGS |= (1 << 1);
	
	LCDclear();
	LCDshort(crcrc,4);
	LCDspace();
	LCDshort(rcdArray[3],2);
	LCDshort(rcdArray[4],2);
	delay_ms(2000);
	
	ENABLE_INTERRUPTS();
	return rcdArray[0];
}
/*******************************************
*    GET Byte FROM
********************************************/
char getByteFromBig (void){
	DISABLE_INTERRUPTS();
	TPB0_set_dir(PORT_DIR_IN);
	TPB0_set_pull_mode(PORT_PULL_OFF);
	TPC3_set_dir(PORT_DIR_IN);
	TPC3_set_pull_mode(PORT_PULL_OFF);

	char bigTinyTimeoutCtr;
	char ret=0;

	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==0)break;
	if(PORTB_get_pin_level(0)==1)  ret |= 0x80;

	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)if(PORTC_get_pin_level(3)==1)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)if(PORTC_get_pin_level(3)==0)break;
	if(PORTB_get_pin_level(0)==1)  ret |= 0x40;

	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)if(PORTC_get_pin_level(3)==1)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)if(PORTC_get_pin_level(3)==0)break;
	if(PORTB_get_pin_level(0)==1)  ret |= 0x20;

	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)if(PORTC_get_pin_level(3)==1)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)if(PORTC_get_pin_level(3)==0)break;
	if(PORTB_get_pin_level(0)==1)  ret |= 0x10;

	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)if(PORTC_get_pin_level(3)==1)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)if(PORTC_get_pin_level(3)==0)break;
	if(PORTB_get_pin_level(0)==1)  ret |= 0x08;

	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)if(PORTC_get_pin_level(3)==1)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)if(PORTC_get_pin_level(3)==0)break;
	if(PORTB_get_pin_level(0)==1)  ret |= 0x04;

	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)if(PORTC_get_pin_level(3)==1)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)if(PORTC_get_pin_level(3)==0)break;
	if(PORTB_get_pin_level(0)==1)  ret |= 0x02;

	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)if(PORTC_get_pin_level(3)==1)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)if(PORTC_get_pin_level(3)==0)break;
	if(PORTB_get_pin_level(0))  ret |= 0x01;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)if(PORTC_get_pin_level(3)==1)break;
	
	return ret;
}
/*******************************************
*    SEND BYTE TO BIG
********************************************/
void sendByteToBig (char byteToSendToBig){
	short bigTinyTimeoutCtr;
	TPB0_set_dir(PORT_DIR_OUT);
	TPC3_set_dir(PORT_DIR_IN);
	short iters;
	

	if((byteToSendToBig & 0x80)==0)PORTB_set_pin_level(0, 0);
	else PORTB_set_pin_level(0, 1);
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==0)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==1)break;

	if((byteToSendToBig & 0x40)==0)PORTB_set_pin_level(0, 0);
	else PORTB_set_pin_level(0, 1);
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==0)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==1)break;

	if((byteToSendToBig & 0x20)==0)PORTB_set_pin_level(0, 0);
	else PORTB_set_pin_level(0, 1);
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==0)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==1)break;

	if((byteToSendToBig & 0x10)==0)PORTB_set_pin_level(0, 0);
	else PORTB_set_pin_level(0, 1);
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==0)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==1)break;

	if((byteToSendToBig & 0x08)==0)PORTB_set_pin_level(0, 0);
	else PORTB_set_pin_level(0, 1);
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==0)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==1)break;

	if((byteToSendToBig & 0x04)==0)PORTB_set_pin_level(0, 0);
	else PORTB_set_pin_level(0, 1);
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==0)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==1)break;

	if((byteToSendToBig & 0x02)==0)PORTB_set_pin_level(0, 0);
	else PORTB_set_pin_level(0, 1);
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==0)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==1)break;

	if((byteToSendToBig & 0x01)==0)PORTB_set_pin_level(0, 0);
	else PORTB_set_pin_level(0, 1);
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==0)break;
	for(bigTinyTimeoutCtr=BTTIMEOUT;bigTinyTimeoutCtr>0;bigTinyTimeoutCtr--)
	if(PORTC_get_pin_level(3)==1)break;

	TPB0_set_dir(PORT_DIR_IN);
}
/*******************************************
*    SEND 23 BYTEs TO BIG
********************************************/
void send23BytesToBig(char* packetForBig){
	unsigned short crcrc;
	char iters;
		crcrc=calcCRC(packetForBig,21);
		packetForBig[21]=crcrc>>8;
		packetForBig[22]=crcrc&0x00FF;
		PORTC_set_pin_level(0, 1);	//WakeUp V71
		
		for(crcrc=30000;crcrc>0;crcrc--){
			PORTB_set_pin_level(0,0);
			myDelay_hus(1);
			if(PORTC_get_pin_level(3)==0)break;
		}
		
		short bigTinyStartTimeoutCtr;
		for(bigTinyStartTimeoutCtr=30000;bigTinyStartTimeoutCtr>0;bigTinyStartTimeoutCtr--){
			if(PORTC_get_pin_level(3)==0)break;	//wait for V71 clk to go low.
		}
		for(bigTinyStartTimeoutCtr=30000;bigTinyStartTimeoutCtr>0;bigTinyStartTimeoutCtr--){
			if(PORTC_get_pin_level(3)==1)break;	//wait for V71 clk to go high.
		}
		
		
		for(iters=0;iters<23;iters++){
			sendByteToBig(packetForBig[iters]);
		}

		PORTC_set_pin_level(0, 0);	//End WakeUp V71 Low
}
/***********************************************************************
***********************************************************************
**************************** ISRs *************************************
***********************************************************************
***********************************************************************/
/***********************************************************************
*  ISR WAKEUP FROM BIG
***********************************************************************/
ISR(PORTC_PORT_vect){
	DISABLE_INTERRUPTS();
	TPC1_set_isc(PORT_ISC_INTDISABLE_gc);//C1=WAKEUP FROM BIG
	PORTC.INTFLAGS |= (1 << 1);
	char gotCmd;
	char iters;
	gotCmd=getPacketFromBig();
//	sendByteToBig(g_motionMinTotals);
/*	if(gotCmd==REQ_ALL_MOTION_SECONDS){
		sendByteToBig(g_motionData[0]);
		sendByteToBig(g_motionData[1]);
		sendByteToBig(g_motionData[2]);
		sendByteToBig(g_motionData[3]);
		sendByteToBig(g_motionData[4]);
		sendByteToBig(g_motionData[5]);
		sendByteToBig(g_motionData[6]);
		sendByteToBig(g_motionData[7]);
		for(iters=0;iters<8;iters++){g_motionData[iters]=0;}
		g_motionMinCtr=0;
		g_motionMask=0x80;
		g_motionDataCtr=0;
		g_motion8ctr=0;
		g_motionMinTotals=0;*/
//}
delay_ms(2);
	PORTC.INTFLAGS |= (1 << 1);

	TPC1_set_isc(PORT_ISC_LEVEL_gc);//C1=WAKEUP FROM BIG
	ENABLE_INTERRUPTS();
	char ctr;
	LED_set_dir(PORT_DIR_OUT);//no measured diff for IN or OUT
	PORTC_set_pin_level(2, 0);//my LED. 1=ON.(opposite for dev kit)
	delay_ms(300);
	PORTC_set_pin_level(2, 1);//my LED. 0=OFF.
	LED_set_dir(PORT_DIR_IN);
	LED_set_pull_mode(PORT_PULL_UP);// Min current with either input PU or output*/
}
/***********************************************************************
*  ISR MOTION
***********************************************************************/
ISR(PORTB_PORT_vect){
	// B6 is Motion
	//	redBlink(3);
	g_gotMotion=1;
	PORTB.INTFLAGS |= (1 << 6);
	redBlink(10);
}
/***********************************************************************
*  ISR USART RX
***********************************************************************/
ISR(USART0_RXC_vect){
	if(p_rcvData>=(g_uc_receive_buffer+BUFFER_SIZE-2))p_rcvData=g_uc_receive_buffer+BUFFER_SIZE-2;
	*p_rcvData=USART0.RXDATAL;
	p_rcvData++;
}
/***********************************************************************
*  ISR RTC (which stores motion)
***********************************************************************/
ISR(RTC_CNT_vect)
{
	char iters,writeRet,ambLo,ambHi,irLo,irHi;
	char sndStr[8];
	char data[32];
	uint32_t irNow;
	uint32_t irThresh,irTemp;

	//redBlink(1);


	if(irNow>irThresh){	//this determines how hair-triggered it is to an increase in reflected IR
		//		redBlink(5);
		g_motion8bits|=g_motionMask;
		g_motionMinTotals++;
	}
	
	g_motionMask=g_motionMask>>1;
	g_motionDataCtr++;//counts up to 7, used to drive mask
	if(g_motionDataCtr>7){
		g_motionData[g_motion8ctr]=g_motion8bits;
		g_motion8ctr++;
		g_motion8bits=0;
		g_motionMask=0x80;
		g_motionDataCtr=0;
	}

	g_motionMinCtr++;
	if(g_motionMinCtr>=64){

		//			for(iters=0;iters<8;iters++){g_motionData[iters]=0;}
		g_motionMinCtr=0;
		g_motionMask=0x80;
		g_motionDataCtr=0;
		g_motion8ctr=0;

	}
	g_gotMotion=0;
	/* Overflow interrupt flag has to be cleared manually */
	RTC.INTFLAGS = RTC_OVF_bm; 
}
/***********************************************************************
*  Unhandled ISRs
***********************************************************************/
/*void showISR (char isrNum){
	USART_0_initialization();
		LCDclear();
		usart_put_string(&("ISR Trap: "),10);
		LCDshort(isrNum,3);
		redBlink(50);
}
ISR(CRCSCAN_NMI_vect)
{
	showISR(1);
}
ISR(BOD_VLM_vect)
{
	showISR(2);
}
ISR(PORTA_PORT_vect)
{
	showISR(3);
}
ISR(RTC_PIT_vect)
{
	RTC.PITINTCTRL = 0 << RTC_PI_bp; 
		showISR(7);
}
ISR(TCA0_LUNF_vect)
{
    *((uint8_t*)0x0A4A)&=~0x31;//TCA0 Ints
    *((uint8_t*)0x0A4B)|=0x31;//TCA0 Flags
	showISR(8);
}
ISR(TCA0_HUNF_vect)
{
	showISR(9);
}
ISR(TCA0_LCMP0_vect)
{
	showISR(10);
}
ISR(TCA0_CMP1_vect)
{
	showISR(11);
}
ISR(TCA0_LCMP2_vect)
{
	showISR(12);
}
ISR(TCB0_INT_vect)
{
	showISR(13);
}
ISR(TCD0_OVF_vect)
{
	showISR(14);
}
ISR(TCD0_TRIG_vect)
{
	showISR(15);
}
ISR(AC0_AC_vect)
{
	showISR(16);
}
ISR(TCA0HUNF_vect)
{
	showISR(13);
}
ISR(SPI0_INT_vect)
{
	showISR(14);
}
ISR(TWI0_TWIM_vect)
{
	showISR(15);
}
ISR(TWI0_TWIS_vect)
{
	showISR(16);
}
ISR(ADC0_RESRDY_vect)
{
	showISR(17);
}
ISR(ADC0_WCOMP_vect)
{
	showISR(18);
}
ISR(USART0_DRE_vect)
{
	showISR(23);
}
ISR(USART0_TXC_vect)
{
	showISR(24);
}
ISR(NVMCTRL_EE_vect)
{
	showISR(25);
}*/
