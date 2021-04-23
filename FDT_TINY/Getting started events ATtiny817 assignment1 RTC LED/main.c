/***************************************************************
*
*
*  TO DO:   ACTUAL YEAR INSTEAD OF HARD-CODED 8 IN TINY
*
******************************************************************************/
#include <avr/io.h>
#include "atmel_start.h"
#include "tinyInclude.h"
#include "initTinyBoard.h"
#include "TWI_BitBangALS.h"
#include <avr/pgmspace.h>
#define DISABLE_INTERRUPTS()        __asm__ __volatile__ ( "cli" ::: "memory")
#define ENABLE_INTERRUPTS()         __asm__ __volatile__ ( "sei" ::: "memory")
#define NOP()	__asm__ __volatile__ ("nop")
#define SLEEP()	__asm__ __volatile__ ("sleep")
#define BTTIMEOUT 250	//only needs to count to about 7, with clocks per March 2018
#define MY_WDT 0x0B	//0x0B = 8 secs
/**************************************************
	CRITICAL CONFIG DEFINES
**************************************************/

#define PIR_SENSITIVITY 0x0050;	//Bit 0080 gets shifted out. Use up to 007F, then jump to 0100.
								//0040 (shifted to 0080) had false triggers April 13.

//#define DO_DIAGS
#define HAS_GPS
#define TAKE_PWRUP_GPS
#define NUMMOTSECS_BETWEEN_TRIPS 5
#define FOR_PVC2020
//#define DISPLAY_PIR_CONFIG
//#define DO_ENDtoENDnoGPS_TEST
//#define KILL_CURR_TEST
//#define DO_HEARTBEAT
//#define DO_SOLAR_BATT_CHECK
//#define SOLAR_BATT_WAIT 200
//#define SOLAR_BATT_THRESHOLD 0xC0
//#define HAS_SEC_PIR

int main(void)
{
	unsigned char goodBattCtr;
	char readRet,writeRet = 0;
	char captureFlag=0;
	int archiveCtr=0;
	char iters=0;
	char ibase=0;
	unsigned short crcrc;
	unsigned int pirA=0;
	unsigned int pirB=0;
	unsigned int pirC=0;
	unsigned int pirD=0;
	unsigned int pirE=0;
	short sensitivtyAll = PIR_SENSITIVITY;
	char returnStr[]={0x0A,0x0D};
	int countGPStest = 0;
	int delay_ctr;
	short gotALS,gotProx;

	g_motionMinTotalDN=0;
	g_motionMinTotalENT=0;
	g_rxDataIndex=0;
	g_durCtr=DURCTR_MAX;
//	g_numTripZeroCtrDN=0;
	g_numTripZeroCtrENT=0;
	g_numTrips=0;
	g_durIndexENT=DUR_INDEX;


	for (iters=0;iters<SIZEOFGPSPARAMS;iters++)gpsParams[iters]=0;

	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	CCP = 0xD8;//enable writes to WDT
	WDT.STATUS = 0x00;//
	CCP = 0xD8;//enable writes to WDT

	DISABLE_INTERRUPTS();
	ccp_write_io((void*)&(CLKCTRL.OSC32KCTRLA),1 << CLKCTRL_RUNSTDBY_bp /* Run standby: enabled */);
#ifdef DO_DIAGS
	redBlink(11);
	userBlink(10);
#endif
	mState=BEGIN;
	for (iters=0;iters<SIZEOFMOTPARAMS;iters++)motParams[iters]=0x00;//
	for (iters=0;iters<10;iters++)partialParams[iters]=0;//Initialize place holders for partial GPS loc params.
	sleepPins(1);
#ifdef KILL_CURR_TEST
	USART_0_initialization();
	USART_0_initialization();
	redBlink(10);
	usart_put_string("\r\nKILL CURRENT TEST\r\n",21);
	sleepPins(0);//0=no wakeup from Big
	delay_ms(3500);
	DISABLE_INTERRUPTS();
	SLPCTRL.CTRLA = 0x03;	//03=enable STANDBY SLEEP 0.0021 mA. 01=enable IDLE SLEEP  0.774 mA. 05=enable PWR DN SLEEP  0.0007 mA	but no RTC wakeup
	NOP();
	sleep_enable();
	sleep_cpu();
#endif
	WDT.CTRLA = MY_WDT;//MY_WDT;//0=disable watchdog. 0x0B=8 seconds
#ifdef DO_DIAGS
	USART_0_initialization();
	USART_0_initialization();
#endif
	NOP();
	WDT_RST();
#ifdef DO_HEARTBEAT
	DISABLE_INTERRUPTS();
  for(;;){
	redBlink(1);
	WDT_RST();
	delay_ms(2000);
	WDT_RST();
	delay_ms(2000);
#ifdef DO_DIAGS
//	usart_put_string(&("HeartBeat "),10);
#endif
  }
#endif
	DISABLE_INTERRUPTS();

#ifdef DO_DIAGS
	short battVal;
   USART_0_initialization();
	redBlink(3);
	usart_put_string(&("\r\nBOOT TINY\r\n"),13);
	WDT_RST();
	delay_ms(1000);
#endif
/**** LOOP ****/
/* GPS TEST */

/***** THIS IS THE DEV LOOP FOR SPRING 2020  **********/
//     USART_0_initialization();
//     USART_0_initialization();
// 	short testALS;
//  for(;;){
//  	WDT_RST();
// 	 testALS=getALSfromVEML();
// 	 LCDshort(testALS,5);
// 	 ret
// 	usart_put_string(&("TEST LOOP\r\n"),11);
// 	TPC0_set_dir(PORT_DIR_OUT);			//WU Big
// 	PORTC_set_pin_level(0, 0);			//WU Big
// 	send25BytesToBig(gpsParams);
//  	WDT_RST();
//  }
//  	WDT_RST();
// 	WDT.CTRLA = 0;//disable watchdog
// 	closeGPS();
//  	clearPIRinterrupt();
// // 	if((rcvdCmd&0x10)==0x10){
// // 		rcvdCmd=0;
// // 		mState=GET_GPS;
// // 	}else usart_put_string(". ",2);
// 	delay_ms(15);
// 	sleepPins(1);
// 	ENABLE_INTERRUPTS();//might not have to do this every time.
// 	TPC1_set_isc(PORT_ISC_LEVEL_gc);//C1=WAKEUP FROM BIG
// 	TPB6_set_isc(PORT_ISC_RISING_gc);	//Motion Direct Line
// 	SLPCTRL.CTRLA = 0x03;	//03=enable STANDBY SLEEP 0.0021 mA. 01=enable IDLE SLEEP  0.774 mA. 05=enable PWR DN SLEEP  0.0007 mA	but no RTC wakeup
//  	NOP();
// 	SLEEP();
// }

/**** END DEV ****/

#ifdef DO_SOLAR_BATT_CHECK
// 	char battery;
// 	goodBattCtr=0;
// 	while(goodBattCtr<SOLAR_BATT_WAIT){//Wait here, possibly forever, until batt has been above threshhold for a while
// 		TPA6_set_dir(PORT_DIR_IN);
// 		TPA6_set_pull_mode(PORT_PULL_OFF);
// 		WDT_RST();
// 		mState=SOLAR_WAIT;
// 		TPC1_set_isc(PORT_ISC_INTDISABLE_gc);//C1=WAKEUP FROM BIG
// 		PORTC_set_pin_level(4, 0);	//Sleep V71 Low
// 		USART0_ON(0);
// 		usart_put_string("S",1);
// 		if ((goodBattCtr==(SOLAR_BATT_WAIT-10))||(goodBattCtr==2)){
// 			TPB5_set_dir(PORT_DIR_OUT);	//reworked to reset big
// 			PORTB_set_pin_level(5, 0);	//reworked to reset big
// 			delay_ms(2000);
// 		}
// 		TPB5_set_dir(PORT_DIR_IN);			//reworked to reset big
// 		TPB5_set_pull_mode(PORT_PULL_UP);	//reworked to reset big
//
// 		delay_ms(100);
// 		battery=getBatt();The VREF for getBatt() can mess up kill current after a handoff to Big
// 		if(battery>=SOLAR_BATT_THRESHOLD)goodBattCtr++;else goodBattCtr=0;
// 		LCDspace();
// 		LCDshort(battery,2);
// 		LCDspace();
// 		delay_ms(200);
// 		sleepPins(0);
// 		//		delay_ms(1050);
// 		ENABLE_INTERRUPTS();//might not have to do this every time.
// 		TPC1_set_isc(PORT_ISC_LEVEL_gc);//C1=WAKEUP FROM BIG
// 		SLPCTRL.CTRLA = 0x03;	//03=enable STANDBY SLEEP 0.0021 mA. 01=enable IDLE SLEEP  0.774 mA. 05=enable PWR DN SLEEP  0.0007 mA	but no RTC wakeup
// 		SLEEP();
// 	}
// 	WDT_RST();
// 	delay_ms(1000);
#endif
//	usart_put_string("PAST SOLAR\r\n",12);
	DISABLE_INTERRUPTS();
	for (iters=0;iters<SIZEOFMOTPARAMS;iters++)motParams[iters]=0x00;//
	for (iters=0;iters<10;iters++)partialParams[iters]=iters+0xF0;//Initialize place holders for partial GPS loc params.
	sleepPins(1);
	NOP();
	WDT_RST();
	DISABLE_INTERRUPTS();

	g_sensitivityHiBit = sensitivtyAll>>8;
	g_sensitivityLoByte = (char)((sensitivtyAll&0x00FF)<<1);
// 	TPB7_set_dir(PORT_DIR_OUT);

 int PIRa;
 short PIRb,PIRc;
 char* lastByte;

	VPORTA_INTFLAGS = 0xFF;
	VPORTB_INTFLAGS = 0xFF;

 /** CONFIG PRIMARY PIR **/
	TPB7_set_dir(PORT_DIR_OUT);
	setPIRconfigDN(g_sensitivityHiBit,g_sensitivityLoByte,0x01,0x31);//sensitivity 01,FE,00,00. Lower number is more sensitive. 0,08
	delay_ms(500);
	setPIRconfigDN(g_sensitivityHiBit,g_sensitivityLoByte,0x01,0x31);//sensitivity 01,FE,00,00. Lower number is more sensitive. 0,08
	TPB6_set_isc(PORT_ISC_RISING_gc);//Motion Switch

#ifdef HAS_SEC_PIR
 /** CONFIG SECONDARY PIR **/
	 TPA2_set_dir(PORT_DIR_OUT);
	 setPIRconfigENT(g_sensitivityHiBit,g_sensitivityLoByte,0x01,0x31);//sensitivity 01,FE,00,00. Lower number is more sensitive. 0,08
	 delay_ms(500);
	 setPIRconfigENT(g_sensitivityHiBit,g_sensitivityLoByte,0x01,0x31);//sensitivity 01,FE,00,00. Lower number is more sensitive. 0,08
	 TPA1_set_isc(PORT_ISC_RISING_gc);//Motion Switch
#endif

mState=IDLE;
#ifdef HAS_GPS
#ifdef TAKE_PWRUP_GPS
	mState=BEGIN;
#endif
#endif

for(;;){
	WDT_RST();
	clearBothPIRinterrupts();
	switch(mState){
		case BEGIN://0
			mState=INITIAL_GPS;
			break;
		case INITIAL_GPS://
			if(!getGPS(1))mState=IDLE; //returns 0 if uBloxStatus = 41 and then got good date and time and sent to BIG
			else mState=NO_INITIAL_GPS_A;
			#ifdef DO_DIAGS
				USART_0_initialization();
				LCDclear();
				usart_put_string("Got Initial GPS",15);
			#endif
			break;
		case GET_GPS://2
			getGPS(0);
			closeGPS();
			#ifdef DO_DIAGS
				USART_0_initialization();
				LCDclear ();
				usart_put_string("GotGPS",6);
			#endif
			mState=IDLE;
			break;
		case NO_INITIAL_GPS_A://3
			if(!getGPS(1))mState=IDLE; //returns 0 if uBloxStatus = 41 and then got good date and time and sent to BIG
			else mState=NO_INITIAL_GPS_B;
			break;
		case NO_INITIAL_GPS_B://4
			if(!getGPS(0))mState=IDLE; //returns 0 if uBloxStatus = 41 and then got good date and time and sent to BIG
			else mState=NO_INITIAL_GPS_C;
			break;
		case NO_INITIAL_GPS_C://5
			if(!getGPS(0))mState=IDLE; //returns 0 if uBloxStatus = 41 and then got good date and time and sent to BIG
			else mState=NO_INITIAL_GPS_LONGTERM;
			break;
		case NO_INITIAL_GPS_LONGTERM://6
			if(!getGPS(0))mState=IDLE; //returns 0 if uBloxStatus = 41 and then got good date and time and sent to BIG
			else mState=NO_INITIAL_GPS_LONGTERM;
			break;
		case IDLE://7
 			WDT_RST();
			WDT.CTRLA = MY_WDT;//watchdog
			closeGPS();
 			WDT_RST();
			clearBothPIRinterrupts();
#ifdef HAS_GPS
			if((g_rcvdCmd&0x10)==0x10){
				g_rcvdCmd=0;
				mState=GET_GPS;
				}
#endif
			#ifdef DO_DIAGS
				USART_0_initialization();
//				char battv = getBatt();The VREF for getBatt() can mess up kill current after a handoff to Big
//				usart_put_string("IDLE ",5);
// 				LCDshort(battv,2);
// 				LCDspace();

				delay_ms(15);
			#endif
			break;
		default: mState=IDLE;
 			WDT_RST();
			break;
	}
	sleepPins(1);
	ENABLE_INTERRUPTS();//might not have to do this every time.
	TPC1_set_isc(PORT_ISC_LEVEL_gc);//C1=WAKEUP FROM BIG
	TPB6_set_isc(PORT_ISC_RISING_gc);	//Motion Direct Line
#ifdef HAS_SEC_PIR	
	TPA1_set_isc(PORT_ISC_RISING_gc);	//Motion Direct Line
#endif	
	SLPCTRL.CTRLA = 0x03;	//03=enable STANDBY SLEEP 0.0021 mA. 01=enable IDLE SLEEP  0.774 mA. 05=enable PWR DN SLEEP  0.0007 mA	but no RTC wakeup
	NOP();
	SLEEP();
}

}

/***********************************************************************/
/*  GET PROX from PROX SI1120
/*  for prox use PRX50 mode
/***********************************************************************/
// short getMotion(void){
// 	short res16;
// 	short iterInner=0;
// 	 EVSYS_ASYNCUSER0 = 0x05;//Directs AsyncChannel2 to TimerCounterB
// 	 EVSYS_ASYNCCH2 = 0x0E;//Directs PortCPin4 as source for AsynchChannel2.
// 	 TCB0_CTRLA = 0x01;//
// 	 TCB0_CTRLB = 0x04;//
// 	 TCB0_EVCTRL = 0x10;//Leave disabled for now
// 	 WDT_RST();
// 	TPB5_set_dir(PORT_DIR_OUT);//*** SC  *** (changed to B5 for PVC)
// 	TPB1_set_dir(PORT_DIR_OUT);//*** STX *** (changed to B1 for PVC)
// 	TPA2_set_dir(PORT_DIR_OUT);//*** MD  ***
// 	TPC4_set_dir(PORT_DIR_IN); //*** PRX ***  From PROX PWM output
// 	TPB5_set_level(1);//         *** SC  High ***
// 	TPB1_set_level(0);//         *** STX Low  *** for Prox
// 	TPA2_set_level(1);//         *** MD  High *** for Prox50
// 	NOP();NOP();NOP();NOP();
// 	TPB5_set_level(0);//         *** SC Low to Latch STX and MD ***  transition low to latch STX and MD, and brings sensor out of sleep. 500us latency before valid output.
// 	 NOP();NOP();NOP();NOP();
// 	 TPA2_set_level(0);//         *** MD  static low  *** for PRX50
// 	 delay_ms(1);//Datasheet says 500us latency after SC goes low
// 	 TPB1_set_level(0);//         *** STX Pulse low to high *** to output pulse on PRX
// 	 TCB0_INTCTRL = 0x01;//Enable capture interrupt flag
// 	 TCB0_EVCTRL = 0x11;//Enable Capture. Starts counting here. Needs to be before first negative going pulse.
// 	 NOP();NOP();NOP();NOP();
// 	 TPB1_set_level(1);//         *** STX Pulse low to high *** to output pulse on PRX
//
// 	 TPB1_set_level(1);//STX pulse low to high to output pulse on PRX
// 	 res16=TCB0_CCMP;//Read stale value
// 	 WDT_RST();
// 	 iterInner=0;
// 	 while((TCB0_INTFLAGS == 0)&&(iterInner<9000)){iterInner++;}
// 	 res16=TCB0_CCMP;
// 	 TPB5_set_level(1);//SC High, Prox goes to sleep
// 	 TCB0_INTCTRL = 0x00;//Disable capture interrupt flag
// 	 TCB0_EVCTRL = 0x10;//Disable Capture
//	 TCB0_INTCTRL = 0x00;//Disable capture interrupt flag
// 	 return res16;
// }
/***********************************************************************/
/*  GET BATT ADC
/***********************************************************************/
char getBatt(void){
	short battVal;
	TPA6_set_dir(PORT_DIR_IN);			//
	TPA6_set_pull_mode(PORT_PULL_OFF);	//
	VREF_CTRLA = 0x20;//20=2.5v, 30=4.3V VREF
	VREF_CTRLB = 0x02;//Enable VREF
	ADC0_CTRLA |= 0x04;//5=8bit, enabled
	ADC0_CTRLB |= 0x06;//6=accumulate 64 results
	ADC0_CTRLC = 0x27;//internal ref, divide by 3.
	ADC0_CTRLD = 0xD8;//
	ADC0_CTRLE = 0;//
	ADC0_EVCTRL = 0;//
	ADC0_INTCTRL = 0;
	ADC0_SAMPCTRL = 0x06;
	ADC0_MUXPOS = 0x06;//pin 6 to be wired with 1.5M 2.2M from batt
	ADC0_INTFLAGS |= 0x01; //clear flag
	ADC0_CTRLA |= 0x01;//5=8bit, enabled
	ADC0_COMMAND = 0x01; //start conversion.
	while((ADC0_INTFLAGS & 0x01) == 0){}
	battVal = (ADC0_RESH<<8) + ADC0_RESL;//first one is crap, throw it away
	ADC0_COMMAND = 0x01; //start conversion.
	while((ADC0_INTFLAGS & 0x01) == 0){}
	battVal = (ADC0_RESH<<8) + ADC0_RESL;
	battVal >>= 6;
	VREF_CTRLB = 0;//Disable VREF
	ADC0_CTRLA = 0;
	return (char)battVal;
}
/***********************************************************************/
/*  GET LIGHT ADC
/***********************************************************************/
char getLight(void){
// 	short lightVal;
// 	TPA6_set_dir(PORT_DIR_IN);			//Batt sense
// 	TPA6_set_pull_mode(PORT_PULL_OFF);	//Batt sense
// 	TPA5_set_dir(PORT_DIR_OUT);			//power ALS
// 	TPA5_set_level(1);
// 	NOP();
// 	VREF_CTRLA = 0x30;//20=2.5v, 30=4.3V VREF
// 	VREF_CTRLB = 0x02;//Enable VREF
// 	ADC0_CTRLA |= 0x05;//5=8bit, enabled
// 	ADC0_CTRLB |= 0x06;//6=accumulate 64 results
// 	ADC0_CTRLC = 0x27;//internal ref, divide by 3.
// 	ADC0_CTRLD = 0xD8;//
// 	ADC0_CTRLE = 0;//
// 	ADC0_EVCTRL = 0;//
// 	ADC0_INTCTRL = 0;
// 	ADC0_SAMPCTRL = 0x06;
// 	ADC0_MUXPOS = 0x07;//pin 7
// 	ADC0_INTFLAGS |= 0x01; //clear flag
// 	ADC0_CTRLA |= 0x05;//5=8bit, enabled
// 	ADC0_COMMAND = 0x01; //start conversion.
// 	while((ADC0_INTFLAGS & 0x01) == 0){}
// 	lightVal = (ADC0_RESH<<8) + ADC0_RESL;//first one is crap, throw it away
// 	ADC0_COMMAND = 0x01; //start conversion.
// 	while((ADC0_INTFLAGS & 0x01) == 0){}
// 	lightVal = (ADC0_RESH<<8) + ADC0_RESL;
// 	lightVal >>= 6;
// 	VREF_CTRLB = 0;//Disable VREF
// 	ADC0_CTRLA = 0;
// 	return (char)lightVal;
// 	return 1;
}
/*****************************************************************************************************
******************************************************************************************************
***********************  GPS  ************************************************************************
******************************************************************************************************/
/*******************************************
*    SEND 25 BYTEs TO BIG
********************************************/
void send25BytesToBig(char* packetForBig){
	unsigned short crcrc,wctr;
	char iters;
	TPB0_set_dir(PORT_DIR_OUT);
	TPC3_set_dir(PORT_DIR_IN);
	crcrc=calcCRC(packetForBig,23);
	packetForBig[23]=crcrc>>8;
	packetForBig[24]=crcrc&0x00FF;

	for(wctr=30000;wctr>0;wctr--){
		PORTB_set_pin_level(0,0);//Drive DAT low
		myDelay_hus(1);
		if(PORTC_get_pin_level(3)==0)break;//wait for CLK_frm_Big to be low
	}

	short bigTinyStartTimeoutCtr;
	for(bigTinyStartTimeoutCtr=30000;bigTinyStartTimeoutCtr>0;bigTinyStartTimeoutCtr--){
		if(PORTC_get_pin_level(3)==0)break;	//wait for CLK_frm_Big to go low.
	}
	for(bigTinyStartTimeoutCtr=30000;bigTinyStartTimeoutCtr>0;bigTinyStartTimeoutCtr--){
		if(PORTC_get_pin_level(3)==1)break;	//wait for CLK_frm_Big to go high.
	}

	TPC0_set_dir(PORT_DIR_OUT);			//WU Big
	PORTC_set_pin_level(0, 1);			//End WakeUp

	for(iters=0;iters<25;iters++){
		sendByteToBig(packetForBig[iters]);
	}
}
/***********************************************************************/
/*  GET GPS  returns 0 if uBloxStatus = 41 and then got good date and time and sent to BIG
/***********************************************************************/
short getGPS(char userLED){
 	char iters,ackFromBig;
#ifdef HAS_GPS
	openGPSuBlox();
	initGPSuBlox();
	if(userLED)userBlink(1);
	initGPSuBlox();
	if(userLED)userBlink(1);
	initGPSuBlox();
	if(userLED)userBlink(1);
	timeToFix=0;
	gpsParams[15]=0;//getBatt();The VREF for getBatt() can mess up kill current after a handoff to Big
	if (gTryUblox(userLED)!=0) {
		closeGPS();
		return 666;
		}else{        //if gTryUblox==0
			iters = 15;
			while((iters>0)&&(GPSgetDate(userLED)!=0))iters--;
			while((iters>0)&&(GPSgetTime(userLED)!=0))iters--;
			usart_put_string("GPS GNS \r\n",10);

/*might want to recreate this to set Big RTC. Or maybe fold that into the normal tiny-big hand-off*/
			DISABLE_INTERRUPTS();
			ackFromBig=0;
			iters=0;

			while((ackFromBig!='G')&&(iters<30)){
				WDT_RST();
				iters++;
				TPC0_set_dir(PORT_DIR_OUT);			//WU Big
				PORTC_set_pin_level(0, 0);			//WU Big
				send25BytesToBig(gpsParams);
				ackFromBig=getByteFromBig();
			    usart_put_string("TINY-BIG CLOCK SET ATTEMPT \r\n",10);
				LCDshort(ackFromBig,2);
				delay_ms(iters>>6);
			}

			ENABLE_INTERRUPTS();
			WDT_RST();
			if (userLED){
				TPB1_set_dir(PORT_DIR_OUT);//no measured diff for IN or OUT
				PORTB_set_pin_level(1, 0);//my LED. 1=ON.(opposite for dev kit)
			}
			delay_ms(2000);
			WDT_RST();
			delay_ms(2000);
			WDT_RST();
			USART0_OFF();
			TPB1_set_dir(PORT_DIR_IN);//user LED off
			TPB1_set_pull_mode(PORT_PULL_UP);// user LED off
			return 0;
			}
			closeGPS();
#endif
#ifndef HAS_GPS
	return 0;
#endif
			sleepPins(1);//
			return 666;
}
/***********************************************************************/
/*  GPS Search Receive Buff for two chars, return 666 if can't find
/***********************************************************************/
short searchRcvBuff (char a, char b){
	short j=666;
	GPSSearchPtr = p_GPSrcvData;
	do{
		GPSSearchPtr--;
		if ((*GPSSearchPtr==a)&&(*(GPSSearchPtr+1)==b)){j=0;};
	}while((GPSSearchPtr >= g_GPSrcv_buffer)&&(j==666));
	return j;
}
/***********************************************************************/
/*  GPS Search Receive Buff for two chars, return 666 if can't find
/***********************************************************************/
short searchRcvBuffuBlox (char a, char b, char c){
	short ret=666;
	GPSSearchPtr = g_GPSrcv_buffer-1;
	do{
		GPSSearchPtr++;
		if ((*GPSSearchPtr==a)&&(*(GPSSearchPtr+1)==b)&&(*(GPSSearchPtr+2)==c)){ret=0;};
	}while((GPSSearchPtr < (p_GPSrcvData-5))&&(ret==666));
	return ret;
}
/***********************************************************************/
/*  GPS Try 3 Req Date uBlox
/***********************************************************************/
short GPSgetDate (char userLED){
	if((gpsReqDateUblox(userLED)==0)&&((*(GPSSearchPtr+10))!='X'))return 0;
	if((gpsReqDateUblox(userLED)==0)&&((*(GPSSearchPtr+10))!='X'))return 0;//might not need these repeats now that checksum has been added
	if((gpsReqDateUblox(userLED)==0)&&((*(GPSSearchPtr+10))!='X'))return 0;
	if((gpsReqDateUblox(userLED)==0)&&((*(GPSSearchPtr+10))!='X'))return 0;else return 666;
}
/***********************************************************************/
/*  GPS Req Date uBlox
/***********************************************************************/
short gpsReqDateUblox (char userLED){
	char i=5;
	char uartIters;
	char saveBatt;
	unsigned char validDate, numCommas=0;
	char stuffToSend[] = {'$','G','N','G','N','Q',',','Z','D','A','*',0x32,0x32,0x0D,0x0A};
	char validCtr=0;
	char monthHighTest[4];
	char monthLowTest[4];
	char dayLowTest[4];
	char dayHighTest[4];
	char *checksumPtr;
	char gpsChecksum;

	for(i=0;i<4;i++){//try it four times. This value must match the size of dateGoodTestX[4]
		WDT_RST();
		usart_put_string(stuffToSend,15);
		clrRx();
		USART0.CTRLA |= USART_RXCIE_bm;
		endOfLineFlag=0;
		ENABLE_INTERRUPTS();
		uartIters=0;
		while((endOfLineFlag==0)&&(uartIters<15)){
			delay_ms(100);
			uartIters++;
		}
		timeToFix++;
		if(((timeToFix>MAX_GPS_ACQ)&&(mState!=INITIAL_GPS))||(timeToFix>MAX_INITIAL_GPS_ACQ)){
			uBloxStatus=666;
			return 666;
		}
	  	USART0.CTRLA &= ~USART_RXCIE_bm;//DISABLE Rx Interrupt
		if(searchRcvBuffuBlox('Z','D','A')==0){
			checksumPtr=GPSSearchPtr-2;
			gpsChecksum=0;
			while((checksumPtr<=g_GPSrcv_buffer+GPS_BUFFER_SIZE)&&(*checksumPtr!='*')){
				gpsChecksum^=*checksumPtr++;
			}
			if(gpsChecksum==asciiHexToHex(*(checksumPtr+1),*(checksumPtr+2))){
				numCommas=0;
				while (GPSSearchPtr < &(g_GPSrcv_buffer[GPS_BUFFER_SIZE])){
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
		}//end of good checksum
	}//end of 'ZDA'
	if (userLED)userBlink(2);
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
saveBatt=gpsParams[15];
for(validCtr=0;validCtr<SIZEOFGPSPARAMS;validCtr++){
	gpsParams[validCtr]=0;
}
gpsParams[15]=saveBatt;
if((dayLowTest[3])!=','){
	gpsParams[9]=(char)(asciiToHex(*(GPSSearchPtr+9),*(GPSSearchPtr+10)));		   /*last digit of year*/
	gpsParams[10]=asciiToHex(monthHighTest[3],monthLowTest[3]);		/*month uBlox*/
	gpsParams[11]=asciiToHex(dayHighTest[3],dayLowTest[3]); 	/*day uBlox*/
	return 0;
	}else{
	return 666;
}
}
/***********************************************************************/
/*  GPS Req Status
/***********************************************************************/
short gpsReqStatusUblox (char userGPS){
	char i=5;
	unsigned char validLoc, numCommas=0;
	validLoc=0;
	uBloxStatus=0;
	char *checksumPtr;
	char gpsChecksum;
	char uartIters;

	for(i=4;i>0;i--){//try it four times
		WDT_RST();
		clrRx();
		USART0.CTRLA |= USART_RXCIE_bm;
		endOfLineFlag=0;
		ENABLE_INTERRUPTS();
		uartIters=0;
		while((endOfLineFlag==0)&&(uartIters<15)){
			delay_ms(100);
			uartIters++;
		}
		timeToFix++;
		if(((timeToFix>MAX_GPS_ACQ)&&(mState!=INITIAL_GPS))||(timeToFix>MAX_INITIAL_GPS_ACQ)){
			uBloxStatus=666;
			return 666;
		}
	  	USART0.CTRLA &= ~USART_RXCIE_bm;//DISABLE Rx Interrupt
		if(searchRcvBuffuBlox('R','M','C')==0){
			checksumPtr=GPSSearchPtr-2;
			gpsChecksum=0;
			while((checksumPtr<=g_GPSrcv_buffer+GPS_BUFFER_SIZE)&&(*checksumPtr!='*')){
				gpsChecksum^=*checksumPtr++;
			}
			if(gpsChecksum==asciiHexToHex(*(checksumPtr+1),*(checksumPtr+2))){
				numCommas=0;
				while (GPSSearchPtr < &(g_GPSrcv_buffer[GPS_BUFFER_SIZE])){
					GPSSearchPtr++;
					if(*GPSSearchPtr == ','){
						numCommas++;
						if(numCommas==2){
							if(((*(GPSSearchPtr+1))==',')||((*(GPSSearchPtr+1))=='X'))break;
							else{
								if(*(GPSSearchPtr+1)=='A')
									validLoc++;
									break;
								}//found status=A
							}//end of if found second comma
						}// end of if found comma
					}//end of WHILE LOOP, searching buffer
				}//end of if good checksum from uBlox
			}//end of found RMC
		if(userGPS)userBlink(1);
	}//end of FOR LOOP, if 3 of 4 are good, including the last one

	if((validLoc>=2)&&(*(GPSSearchPtr+1)=='A')){  //gets here pretty quick after status goes to 'A'
		uBloxStatus=41;
		return 0;
	}else{
		uBloxStatus=666;
		return 666;
	}
}
/***********************************************************************/
/*  GPS Try 5 Req Loc uBlox
/***********************************************************************/
short GPSgetTime(char userLED){
	if((gpsReqLocUblox(userLED)==0)&&((*(GPSSearchPtr+43))!='X')&&(gpsParams[16]>3))return 0;
	if((gpsReqLocUblox(userLED)==0)&&((*(GPSSearchPtr+43))!='X')&&(gpsParams[16]>3))return 0;//might not need these repeats now that checksum has been added
	if((gpsReqLocUblox(userLED)==0)&&((*(GPSSearchPtr+43))!='X')&&(gpsParams[16]>3))return 0;
	if((gpsReqLocUblox(userLED)==0)&&((*(GPSSearchPtr+43))!='X')&&(gpsParams[16]>3))return 0;
	if((gpsReqLocUblox(userLED)==0)&&((*(GPSSearchPtr+43))!='X')&&(gpsParams[16]>3))return 0;else return 666;
}
/***********************************************************************/
/*  GPS Req Number of Satellites
/***********************************************************************/
short gpsReqLocUblox (char userLED){
	char i;
	unsigned char validDate, numCommas=0;
	char stuffToSend[] = {'$','G','N','G','N','Q',',','G','N','S','*',0x32,0x37,0x0D,0x0A};
	char validCtr=0;
	char hourHighTest[8];
	char hourLowTest[8];
	char minLowTest[8];
	char minHighTest[8];
	char *checksumPtr;
	char gpsChecksum;
	char uartIters;

	hourLowTest[7]=1;hourLowTest[2]=2;hourLowTest[1]=3;//force fail if stuck
	hourHighTest[7]=1;hourHighTest[2]=2;hourHighTest[1]=3;//force fail if stuck
	minLowTest[7]=1;minLowTest[2]=2;minLowTest[1]=3;//force fail if stuck
	for(i=8;i>0;i--){//try it four times.This value must match the size of monthHourLowTest[4]
		WDT_RST();
		usart_put_string(stuffToSend,15);
		clrRx();
		USART0.CTRLA |= USART_RXCIE_bm;
		endOfLineFlag=0;
		ENABLE_INTERRUPTS();
		uartIters=0;
		while((endOfLineFlag==0)&&(uartIters<15)){
			delay_ms(100);
			uartIters++;
		}
		if(((timeToFix>MAX_GPS_ACQ)&&(mState!=INITIAL_GPS))||(timeToFix>MAX_INITIAL_GPS_ACQ)){
			uBloxStatus=666;
			return 666;
		}
		timeToFix++;
	  	USART0.CTRLA &= ~USART_RXCIE_bm;//DISABLE Rx Interrupt
		if(userLED)	userBlink(3);
		if(searchRcvBuffuBlox('G','N','S')==0){
			checksumPtr=GPSSearchPtr-2;
			gpsChecksum=0;
			while((checksumPtr<=g_GPSrcv_buffer+GPS_BUFFER_SIZE)&&(*checksumPtr!='*')){
				gpsChecksum^=*checksumPtr++;
			}

			if(gpsChecksum==asciiHexToHex(*(checksumPtr+1),*(checksumPtr+2))){
				numCommas=0;
				while (GPSSearchPtr < &(g_GPSrcv_buffer[GPS_BUFFER_SIZE])){
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
		  }//end of good checksum
		}//end of 'GNS'
	}//end of if 7 of 8 are good, including the last one
	validCtr=0;
	if(hourLowTest[7]==hourLowTest[6])validCtr++;
	if(hourLowTest[7]==hourLowTest[5])validCtr++;
	if(hourLowTest[7]==hourLowTest[4])validCtr++;
	if(hourLowTest[7]==hourLowTest[3])validCtr++;
	if(hourLowTest[7]==hourLowTest[2])validCtr++;
	if(hourLowTest[7]==hourLowTest[1])validCtr++;
	if(hourLowTest[7]==hourLowTest[0])validCtr++;
	if(validCtr<6)return 666;
	validCtr=0;
	if(hourHighTest[7]==hourHighTest[6])validCtr++;
	if(hourHighTest[7]==hourHighTest[5])validCtr++;
	if(hourHighTest[7]==hourHighTest[4])validCtr++;
	if(hourHighTest[7]==hourHighTest[3])validCtr++;
	if(hourHighTest[7]==hourHighTest[2])validCtr++;
	if(hourHighTest[7]==hourHighTest[1])validCtr++;
	if(hourHighTest[7]==hourHighTest[0])validCtr++;
	if(validCtr<6)return 666;
	validCtr=0;
	if(minHighTest[7]==minHighTest[6])validCtr++;
	if(minHighTest[7]==minHighTest[5])validCtr++;
	if(minHighTest[7]==minHighTest[4])validCtr++;
	if(minHighTest[7]==minHighTest[3])validCtr++;
	if(minHighTest[7]==minHighTest[2])validCtr++;
	if(minHighTest[7]==minHighTest[1])validCtr++;
	if(minHighTest[7]==minHighTest[0])validCtr++;
	if(validCtr<6)return 666;
	validCtr=0;
	if(minLowTest[7]==minLowTest[6])validCtr++;
	if(minLowTest[7]==minLowTest[5])validCtr++;
	if(minLowTest[7]==minLowTest[4])validCtr++;
	if(minLowTest[7]==minLowTest[3])validCtr++;
	if(minLowTest[7]==minLowTest[2])validCtr++;
	if(minLowTest[7]==minLowTest[1])validCtr++;
	if(minLowTest[7]==minLowTest[0])validCtr++;
	if(validCtr<6)return 666;

	if((minLowTest[7])!=','){

	gpsParams[12]=(asciiToHex(hourHighTest[7],hourLowTest[7]));//Hour
	gpsParams[13]=asciiToHex(minHighTest[7],minLowTest[7]);//Min
	gpsParams[14]=((char)(asciiToHex((*(GPSSearchPtr-5)),(*(GPSSearchPtr-4)))));	/*Seconds*/
	gpsParams[16]=(char)(asciiToHex((*(GPSSearchPtr+31)),(*(GPSSearchPtr+32))));  ///uBLOX sats

	if (((char)(*(GPSSearchPtr+12)))=='N')gpsParams[0]=0x80;else gpsParams[0]=0;//LAT SIGN. gpsParams were initialized to zero in getDate
	gpsParams[0]+=((char)(asciiToHex((*(GPSSearchPtr+1)),(*(GPSSearchPtr+2)))));	/*LAT WHOLE uBlox*/

	gpsParams[1]=asciiToHex(((*(GPSSearchPtr+3))),((unsigned int)(*(GPSSearchPtr+4))));//LAT FRAC
	gpsParams[2]=asciiToHex(((*(GPSSearchPtr+6))),((unsigned int)(*(GPSSearchPtr+7))));
	gpsParams[3]=asciiToHex(((*(GPSSearchPtr+8))),((unsigned int)(*(GPSSearchPtr+9))));

	if (((char)(*(GPSSearchPtr+26)))=='E')gpsParams[4]='-';else gpsParams[4]='+';//LONG SIGN. gpsParams were initialized to zero in getDate

	if (((char)(*(GPSSearchPtr+14)))=='1')gpsParams[5]=100; /*high digit of Long whole uBlox*/
	gpsParams[5]+=((char)(asciiToHex((*(GPSSearchPtr+15)),(*(GPSSearchPtr+16)))));	/*LONG WHOLE uBlox*/

	gpsParams[6]=asciiToHex(((*(GPSSearchPtr+17))),((unsigned int)(*(GPSSearchPtr+18))));//LONG FRAC
	gpsParams[7]=asciiToHex(((*(GPSSearchPtr+20))),((unsigned int)(*(GPSSearchPtr+21))));
	gpsParams[8]=asciiToHex(((*(GPSSearchPtr+22))),((unsigned int)(*(GPSSearchPtr+23))));

	gpsParams[17]=timeToFix>>8;
	gpsParams[18]=timeToFix&0x00FF;
	gpsParams[16]=(char)(asciiToHex((*(GPSSearchPtr+31)),(*(GPSSearchPtr+32))));  ///uBLOX sats


	return 0;
}else return 666;
}
/***********************************************************************/
/*  Try uBlox GPS.
/*********gpsReqNumSatsUblox**************************************************************/
short gTryUblox (char userLED) {
	char i,j,k;
	short probablyOK=0;
	i=8;
	do{
		j=250;
		do{
			k=3;
			do{
				WDT_RST();
				delay_ms(250);
				clrRx();
				USART0.CTRLA |= USART_RXCIE_bm;
				ENABLE_INTERRUPTS();
				probablyOK = gpsReqStatusUblox(userLED);//This takes a little more than one sec
				k--;
				TPB6_set_isc(PORT_ISC_RISING_gc);	//Motion Direct Line
#ifdef HAS_SEC_PIR
				TPA1_set_isc(PORT_ISC_RISING_gc);	//Motion Direct Line
#endif				
				if(userLED)userBlink(1);
			}while((k>0)&&(probablyOK == 666));
			/*end try*/

			j--;
			if(userLED)userBlink(1);
		}while ((j>1)&&(probablyOK == 666));  //end 4000x middle loop

		if(j<=1){
		closeGPS();
		delay_ms(1000);
		openGPSuBlox();
		initGPSuBlox();
		};
		i--;
	}while((i>0)&&(probablyOK == 666)); //end 8x outer loop

	return probablyOK; /*returns value from gpsReqStatusUblox()*/
}
/***********************************************************************/
/*  GPS Init  uBlox
	 TPA5_set_dir(PORT_DIR_IN);		//Sense
	 TPA4_set_dir(PORT_DIR_OUT);	//Pwr
	 PORTA_set_pin_level(4, 1);		//Pwr

/***********************************************************************/
void openGPSuBlox (void) {
	  TPA4_set_dir(PORT_DIR_IN);	//GPS Reset  this was not connected in 2017 ARG22 board
	  TPA4_set_pull_mode(PORT_PULL_UP);	//GPS Reset
	  TPC5_set_dir(PORT_DIR_OUT);	//GPS PWRC
	  PORTC_set_pin_level(5, 1);	//GPS PWRC
	  p_GPSrcvData = g_GPSrcv_buffer;
	  USART0_ON(1);
}
/***********************************************************************/
/*  Close GPS
/***********************************************************************/
void closeGPS (void) {
	  TPC5_set_dir(PORT_DIR_OUT);	//GPS PWRC
	  PORTC_set_pin_level(5, 0);	//GPS PWRC
	  TPA4_set_pull_mode(PORT_PULL_OFF);	//GPS Reset
	  TPA4_set_dir(PORT_DIR_OUT);	//GPS Reset
	  PORTA_set_pin_level(4, 0);	//GPS Reset
	  USART0_OFF();
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
}
/***********************************************************************/
/*  Ascii to Hex.  Converts two ascii HEX digits to hex integer.
/***********************************************************************/
int asciiHexToHex(char hiDig, char loDig){
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
case 0x41: hexTotal=10;break;
case 0x42: hexTotal=11;break;
case 0x43: hexTotal=12;break;
case 0x44: hexTotal=13;break;
case 0x45: hexTotal=14;break;
case 0x46: hexTotal=15;break;
default: hexTotal=0;break;
};
switch (hiDig){
case 0x30: hexTotal+=0;break;
case 0x31: hexTotal+=16;break;
case 0x32: hexTotal+=32;break;
case 0x33: hexTotal+=48;break;
case 0x34: hexTotal+=64;break;
case 0x35: hexTotal+=80;break;
case 0x36: hexTotal+=96;break;
case 0x37: hexTotal+=112;break;
case 0x38: hexTotal+=128;break;
case 0x39: hexTotal+=144;break;
case 0x41: hexTotal=160;break;
case 0x42: hexTotal=176;break;
case 0x43: hexTotal=192;break;
case 0x44: hexTotal=208;break;
case 0x45: hexTotal=224;break;
case 0x46: hexTotal=240;break;
default: hexTotal=+0;break;
};
return hexTotal;
}
/*******************************************
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
*    USER LED BLINK
********************************************/
void userBlink (char numBlinks){
	char ctr;
	TPB1_set_dir(PORT_DIR_OUT);//no measured diff for IN or OUT
	for(ctr=numBlinks;ctr>0;ctr--){
		PORTB_set_pin_level(1, 0);//my LED. 1=ON.(opposite for dev kit)
		delay_ms(50);
		PORTB_set_pin_level(1, 1);//my LED. 0=OFF.
		delay_ms(80);
	}
	TPB1_set_dir(PORT_DIR_IN);
	TPB1_set_pull_mode(PORT_PULL_UP);// Min current with either input PU or output
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
 #ifdef DO_DIAGS
	char sendchars[4];
	short z,zz;
	z=hex2ToAscii((a>>8)&0x00FF);
	zz=hex2ToAscii(a&0x00FF);
	sendchars[0]=(char)(z>>8);
	sendchars[1]=(char)z;
	sendchars[2]=(char)(zz>>8);
	sendchars[3]=(char)zz;
	switch (d) {
		case 1: usart_put_string(&sendchars[3],1);
		break;
		case 2: usart_put_string(&sendchars[2],2);
		break;
		case 3: usart_put_string(&sendchars[1],3);
		break;
		default: usart_put_string(&sendchars[0],4);
		break;
	}
 #endif
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
#ifdef DO_DIAGS
	USART_0_initialization();
	char sndStr2[8] = {0xFE,0x46,0xFE,0x48,0xFE,0x4C,0xFE,0x51};//clear display, cursor home
	usart_put_string(sndStr2,8);
	delay_ms(10);
#endif
}
void LCDbottomLine (void){
	char sndStr3[3] = {0xFE,0x45,0x40};//clear display, cursor home
#ifdef DO_DIAGS
	usart_put_string(sndStr3,3);
	delay_ms(10);
#endif
}
void LCDspace (void){
#ifdef DO_DIAGS
usart_put_string(&(" "),1);
	delay_ms(10);
#endif
}

/***********************************************************************
* Clear Rx Buff and initialize the pointer
***********************************************************************/
void clrRx (void){
  int i;
	p_GPSrcvData = g_GPSrcv_buffer;
   for(i=0;i<GPS_BUFFER_SIZE;i++){
     p_GPSrcvData[i]='X';
     }
	p_GPSrcvData = g_GPSrcv_buffer;
 }
/**********************************************************************
***********************************************************************
*********************** BIG-SMALL COMMS *******************************
***********************************************************************
***********************************************************************

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
*    SLEEP PINS
********************************************/
void sleepPins(char bigWakeup){
	TPA0_set_dir(PORT_DIR_IN);			//Reset from Big
	TPA0_set_pull_mode(PORT_PULL_UP);	//Reset from Big
	TPA1_set_dir(PORT_DIR_IN);			//Prox or Secondary PIR Direct Line
#ifdef FOR_PVC2020
	TPA1_set_pull_mode(PORT_PULL_UP);	//Prox
#else	
	TPA1_set_pull_mode(PORT_PULL_UP);	//Secondary PIR Direct Line
#endif	
#ifdef HAS_SEC_PIR
	TPA1_set_pull_mode(PORT_PULL_OFF);	//Secondary PIR Direct Line
#endif
	TPA2_set_dir(PORT_DIR_IN);			//Secondary PIR Serial In or NC
	TPA2_set_pull_mode(PORT_PULL_UP);	//Secondary PIR Serial In or NC
	TPA3_set_dir(PORT_DIR_IN);			//Reset Big
	TPA3_set_pull_mode(PORT_PULL_UP);	//Reset Big
	TPA4_set_pull_mode(PORT_PULL_OFF);	//GPS Reset (rev 1 mistakingly tied to VCC)
	TPA4_set_dir(PORT_DIR_OUT);			//GPS Reset (rev 1 mistakingly tied to VCC)
	PORTA_set_pin_level(4,0);			//GPS Reset
	TPA5_set_dir(PORT_DIR_IN);			//USER PUSHBUTTON
	TPA5_set_pull_mode(PORT_PULL_UP);	//USER PUSHBUTTON
	TPA5_set_isc(PORT_ISC_LEVEL_gc);	//A5=WAKEUP FROM  USER PUSHBUTTON
	TPA6_set_dir(PORT_DIR_IN);			//Analog Input
	TPA6_set_pull_mode(PORT_PULL_UP);	//Analog Input
	TPA7_set_dir(PORT_DIR_IN);			//Analog Input
	TPA7_set_pull_mode(PORT_PULL_UP);	//Analog Input
	TPB0_set_dir(PORT_DIR_IN);			//DAT to Big
	TPB0_set_pull_mode(PORT_PULL_OFF);	//DAT to Big
	TPB1_set_dir(PORT_DIR_IN);			//Microphone Analog Input
	TPB1_set_pull_mode(PORT_PULL_UP);	//Microphone Analog Input
//	TPB2_set_dir(PORT_DIR_IN);			//To GPS  Output Low in closeGPS() which calls USART_OFF()
//	TPB2_set_pull_mode(PORT_PULL_OFF);	//To GPS  Output Low in closeGPS() which calls USART_OFF()
//	TPB3_set_dir(PORT_DIR_IN);			//From GPS  Output Low in closeGPS() which calls USART_OFF()
//	TPB3_set_pull_mode(PORT_PULL_OFF);	//From GPS  Output Low in closeGPS() which calls USART_OFF()
	TPB4_set_dir(PORT_DIR_IN);			//Analog Input
	TPB4_set_pull_mode(PORT_PULL_UP);	//Analog Input
	TPB5_set_dir(PORT_DIR_IN);			//Microphone
	TPB5_set_pull_mode(PORT_PULL_UP);	//Microphone
	TPB6_set_dir(PORT_DIR_IN);			//Motion DN Direct Line
	TPB6_set_pull_mode(PORT_PULL_OFF);  //Motion DN Direct Line
	TPB7_set_dir(PORT_DIR_IN);			//Motion DN Serial In
	TPB7_set_pull_mode(PORT_PULL_UP);	//Motion DN Serial In
	TPC0_set_dir(PORT_DIR_IN);			//WU Big
	TPC0_set_pull_mode(PORT_PULL_UP);	//WU Big
	TPC1_set_dir(PORT_DIR_IN);			//C1=WAKEUP FROM BIG
	TPC1_set_pull_mode(PORT_PULL_UP);	//C1=WAKEUP FROM BIG
	if(bigWakeup){
		TPC1_set_isc(PORT_ISC_LEVEL_gc);	//C1=WAKEUP FROM BIG
		}else{
		TPC1_set_isc(PORT_ISC_INTDISABLE_gc);//C1=WAKEUP FROM BIG
	}
	TPC2_set_dir(PORT_DIR_IN);			//C2=LED
	TPC2_set_pull_mode(PORT_PULL_UP);	// Min current with either input PU or output
	TPC3_set_dir(PORT_DIR_IN);			//CLK from Big
	TPC3_set_pull_mode(PORT_PULL_OFF);	//CLK from Big
	TPC4_set_dir(PORT_DIR_IN);			//C4 = Interrupt from Big's RTC
#ifdef FOR_PVC2020
	TPC4_set_pull_mode(PORT_PULL_UP);	//Prox
#else
	TPC4_set_pull_mode(PORT_PULL_OFF);	//RTC Interrupt
#endif
	TPC5_set_pull_mode(PORT_PULL_OFF);  //GPS PWRC OUT LOW in closeGPS()
//	TPC5_set_dir(PORT_DIR_OUT);			//GPS PWRC OUT LOW in closeGPS()
//	PORTC_set_pin_level(5,0);			//GPS PWRC OUT LOW in closeGPS()
	VREF_CTRLB = 0x00;//Disable VREF
	ADC0_CTRLA = 0x00;//ADC off
	closeGPS();//A2,B2,B3,B4,C5  Calls USART_OFF()
}
/*******************************************
*    GET CMD PACKET FROM BIG
*	Table of byte positions in motParams[] that will be sent to Big
*	Motion:	0  1  2  3  4  5
*	Alt:	6-7    8-9    10-11  12-13  14-15  16-17
*	Temp:	18-19  20-21  22-23  24-25  26-27  28-29
*	Humid:	30-31  32-33  34-35  36-37  38-39  40-41
*	Light:	42-43  44-45  46-47  48-49  50-51  52-53
*
*	APRIL 2, 2020: THERE CAN BE A 30ms DELAY RESPONDING TO THE INTERRUPT FROM BIG
********************************************/
char getPacketFromBig (void){
	unsigned short crcrc;
	char rcdArray[9];
	char iters,ackIters;
//	char minuteCycle;
	char lightHolder;

	DISABLE_INTERRUPTS();
//Don't put any delays here
	rcdArray[0]=getByteFromBig();//GPS CMD (should take a new GPS? which byte of last good GPS or Batt should be sent?)
	rcdArray[1]=getByteFromBig();//for partial param index
	rcdArray[2]=getByteFromBig();//crc high byte
	rcdArray[3]=getByteFromBig();//crc low byte

	crcrc=calcCRC(rcdArray,2);
	TPB0_set_dir(PORT_DIR_OUT);
	if(crcrc==(rcdArray[2]<<8)+rcdArray[3]){
		g_rcvdCmd=rcdArray[0];
		motParams[MOTION_TOTAL_INDEX]=g_motionMinTotalDN;
		motParams[PARTIAL_LOC_INDEX]=gpsParams[rcdArray[1]];//Big determines which partial location param to send.

		TPC3_set_dir(PORT_DIR_IN);
		crcrc=calcCRC(motParams,26);//PVC needs only 25? use 28?
		motParams[26]=crcrc>>8;
		motParams[27]=crcrc&0x00FF;
		if((motParams[0] & 0x80)==0)PORTB_set_pin_level(0, 0);
		else PORTB_set_pin_level(0, 1);		//likely don't need this prelim setting of DAT, it sets it again in sendByteToBig()

		for(crcrc=30000;crcrc>0;crcrc--){
			if(PORTC_get_pin_level(3)==0)break;	//wait for V71 clk to go high.
		}

		for(iters=0;iters<28;iters++){
			sendByteToBig(motParams[iters]);
		}
		TPB0_set_dir(PORT_DIR_IN);//Read this as an ack from Big so know if okay to clear motParams
		ackIters = 200;
		while((ackIters>0)){
			if(PORTB_get_pin_level(0)==0)break;
			NOP();NOP();
			NOP();NOP();
			ackIters--;
		}
#ifdef DO_DIAGS
	USART_0_initialization();
 	USART_0_initialization();
 	char returnStr[]={0x0A,0x0D};
 	usart_put_string(returnStr,2);
 	usart_put_string(returnStr,2);
 	usart_put_string(returnStr,2);
//  	LCDshort(motParams[1],2);
 	LCDshort(motParams[2],2);
 	LCDshort(motParams[3],2);
 	LCDshort(motParams[4],2);
 	LCDshort(motParams[5],2);
 	LCDshort(motParams[6],2);
 	LCDshort(motParams[7],2);
 	LCDshort(motParams[8],2);
 	LCDspace();LCDshort(motParams[9],2);
 	LCDspace();LCDshort(motParams[10],2);
	LCDspace();LCDshort(motParams[11],2);
	LCDspace();LCDshort(motParams[12],2);
	LCDspace();LCDshort(motParams[13],2);
	LCDspace();LCDshort(motParams[14],2);
	LCDspace();LCDshort(motParams[15],2);
	LCDspace();LCDshort(motParams[16],2);
	LCDspace();LCDshort(motParams[17],2);
	LCDspace();LCDshort(motParams[18],2);
	LCDspace();LCDshort(motParams[19],2);
	LCDspace();LCDshort(motParams[20],2);
	LCDspace();LCDshort(motParams[21],2);
 	LCDspace();LCDshort(motParams[22],2);
// 	LCDspace();LCDspace();
// 	LCDspace();LCDshort((((motParams[0]>>1))&0x03),2);LCDshort(motParams[ALS_INDEX],2);//ALS
// 	LCDspace();LCDspace();
// 	LCDspace();LCDshort(motParams[PARTIAL_LOC_INDEX],2);//Loc, Batt, Misc bytes
	usart_put_string(returnStr,2);
	WDT_RST();
	delay_ms(50);
#endif
	if(ackIters>0){//erase only if received ack from BIG
		g_durIndexENT=DUR_INDEX;
		g_motionPatternIndexENT=MOTION_INDEX;
		g_motSecCtrENT=0;
		g_noMotionFlagENT=1;
		g_motionMinTotalENT=0;
		g_motionMinTotalDN=0;
		for (iters=0;iters<NUM_TRIPS_INDEX;iters++)motParams[iters]=0x00;//This will hold GPS params
		motParams[NUM_TRIPS_INDEX]=0;
		motParams[ALS_INDEX]=0;
 	}
	}else{//end good crc  [THESE 'BAD' ONES HIT WHEN TINY IS BUSY WITH RTC-PIT INTERRUPT WHEN INTERRUPT FROM BIG HITS. RTC-PIT INTERRUPT ROUTINE TAKES ABOUT 30ms.]
	if(ackIters>0){//erase only if received ack from BIG
		g_rcvdCmd = 0x08;//else if bad CRC
		g_durIndexENT=DUR_INDEX;
		g_motionPatternIndexENT=MOTION_INDEX;
		g_motSecCtrENT=0;
		g_noMotionFlagENT=1;
		g_motionMinTotalENT=0;
		g_motionMinTotalDN=0;
		for (iters=0;iters<NUM_TRIPS_INDEX;iters++)motParams[iters]=0x00;//This will hold GPS params
		motParams[NUM_TRIPS_INDEX]=0;
		motParams[ALS_INDEX]=0;
	}
#ifdef DO_DIAGS
	char returnStr[]={0x0A,0x0D};
	USART_0_initialization();
	USART_0_initialization();
	usart_put_string(returnStr,2);
	usart_put_string(&("BAD"),3);
	usart_put_string(returnStr,2);
	usart_put_string(&("        "),8);
	WDT_RST();
	delay_ms(50);
#endif
	}//end bad crc

//	g_light=getLight();
	PORTC.INTFLAGS |= (1 << 1);

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
	short bigTinyTimeoutCtrShort;
	char ret=0;

	for(bigTinyTimeoutCtrShort=5000;bigTinyTimeoutCtrShort>0;bigTinyTimeoutCtrShort--)
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
	short iters;

	TPB0_set_dir(PORT_DIR_OUT);			//DAT to Big

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

}
/***********************************************************************
*  UART_ON  _OFF
***********************************************************************/
void USART0_ON(char rcvOn){
	char dummyRead;
	TPB3_set_dir(PORT_DIR_IN);
	TPB3_set_pull_mode(PORT_PULL_OFF);
	TPB2_set_level(1);
	TPB2_set_dir(PORT_DIR_OUT);
	USART0.BAUD = (uint16_t)USART0_BAUD_RATE(9600); /*USART sometimes fails to xmt if don't set the baud everytime*/
	if(rcvOn){
		USART0.CTRLB = 0 << USART_MPCM_bp       /* Multi-processor Communication Mode: disabled */
		| 0 << USART_ODME_bp     /* Open Drain Mode Enable: disabled */
		| 1 << USART_RXEN_bp     /* Reciever enable: enabled */
		| USART_RXMODE_NORMAL_gc /* Normal mode */
		| 0 << USART_SFDEN_bp    /* Start Frame Detection Enable: disabled */
		| 1 << USART_TXEN_bp;    /* Transmitter Enable: enabled */
		}else{
		TPB3_set_dir(PORT_DIR_IN);
		TPB3_set_pull_mode(PORT_PULL_OFF);
		TPB2_set_level(1);
		TPB2_set_dir(PORT_DIR_OUT);
		USART0.BAUD = (uint16_t)USART0_BAUD_RATE(9600); /*USART sometimes fails to xmt if don't set the baud everytime*/
		USART0.CTRLB = 0 << USART_MPCM_bp       /* Multi-processor Communication Mode: disabled */
		| 0 << USART_ODME_bp     /* Open Drain Mode Enable: disabled */
		| 0 << USART_RXEN_bp     /* Reciever enable: enabled */
		| USART_RXMODE_NORMAL_gc /* Normal mode */
		| 0 << USART_SFDEN_bp    /* Start Frame Detection Enable: disabled */
		| 1 << USART_TXEN_bp;    /* Transmitter Enable: enabled */
	}
	dummyRead = USART0.RXDATAH;//likely not needed
	dummyRead = USART0.RXDATAL;//likely not needed
}
void USART0_OFF(void){
	USART0.CTRLB = 0 << USART_MPCM_bp       /* Multi-processor Communication Mode: disabled */
	| 0 << USART_ODME_bp     /* Open Drain Mode Enable: disabled */
	| 0 << USART_RXEN_bp     /* Reciever enable: enabled */
	| USART_RXMODE_NORMAL_gc /* Normal mode */
	| 0 << USART_SFDEN_bp    /* Start Frame Detection Enable: disabled */
	| 0 << USART_TXEN_bp;    /* Transmitter Enable: enabled */
	TPB3_set_dir(PORT_DIR_OUT);	//From GPS
	TPB2_set_dir(PORT_DIR_OUT);	//To GPS
	PORTB_set_pin_level(2, 0);	//To GPS
	PORTB_set_pin_level(3, 0);	//From GPS
	TPB2_set_pull_mode(PORT_PULL_OFF);
	TPB3_set_pull_mode(PORT_PULL_OFF);
}
/*****************************************************************************************************
******************************************************************************************************
******************************************************************************************************
******************************************************************************************************/
/*******************************************
*    PIR DOWN Set Byte
********************************************/
void setPIRconfigDN(char pirrA,char pirrB,char pirrC,char pirrD){
	char iters;
	PORTB_set_pin_level(7, 0);
	TPB7_set_dir(PORT_DIR_OUT);
	pirSUdelay();
	PORTB_set_pin_level(7, 1);
	//	pirSUdelay();
	if((pirrA & 0x01)==0)PORTB_set_pin_level(7, 0); else PORTB_set_pin_level(7, 1);
	pirSUdelay();
	setPIRbitDN(pirrB);
	setPIRbitDN(pirrC);
	setPIRbitDN(pirrD);
}
/*******************************************
*    PIR ENTRANCE Set Byte
********************************************/
void setPIRconfigENT(char pirrA,char pirrB,char pirrC,char pirrD){
	char iters;
	PORTA_set_pin_level(2, 0);
	TPA2_set_dir(PORT_DIR_OUT);
	pirSUdelay();
	PORTA_set_pin_level(2, 1);
	//	pirSUdelay();
	if((pirrA & 0x01)==0)PORTA_set_pin_level(2, 0); else PORTA_set_pin_level(2, 1);
	pirSUdelay();
	setPIRbitENT(pirrB);
	setPIRbitENT(pirrC);
	setPIRbitENT(pirrD);
}
/*******************************************
*    PIR Set Bit
********************************************/
void setPIRbitDN(char byteToSet){
	char iters;
	for(iters=8;iters>0;iters--){
		PORTB_set_pin_level(7, 0);
		NOP();
		NOP();
		PORTB_set_pin_level(7, 1);
		NOP();
		NOP();
		if((byteToSet & (1<<(iters-1)))==0)PORTB_set_pin_level(7, 0); else PORTB_set_pin_level(7, 1);
		pirSUdelay();
	}
}
/*******************************************
*    PIR Set Bit
********************************************/
void setPIRbitENT(char byteToSet){
	char iters;
	for(iters=8;iters>0;iters--){
		PORTA_set_pin_level(2, 0);
		NOP();
		NOP();
		PORTA_set_pin_level(2, 1);
		NOP();
		NOP();
		if((byteToSet & (1<<(iters-1)))==0)PORTA_set_pin_level(2, 0); else PORTA_set_pin_level(2, 1);
		pirSUdelay();
	}
}
/*******************************************
*    PIR Get Byte
********************************************/
char getPIRbyte(void){
	char iters;
	char ret=0;
	for(iters=7;iters>0;iters--){
		if(getPIRbit()==1)ret+=(1<<iters);
	}
	if(getPIRbit()==1)ret+=1;
	return ret;
}
/*******************************************
*    PIR Get All 40 Bits from PIR
********************************************/
//void getPIR(void){

/*g_motionValueDN=0;
g_motionValidDN=0;
char vala=0;
char valb=0;
char valc=0;
char vald=0;

	PORTB_set_pin_level(6, 1);
	TPB6_set_dir(PORT_DIR_OUT);
	PORTB_set_pin_level(6, 1);
	pirSUdelay();

 	PORTB_set_pin_level(6, 0);//This bit indicates PIR ADC overflow (1=normal, 0=overflow)
 	TPB6_set_dir(PORT_DIR_OUT);
 	PORTB_set_pin_level(6, 1);
 	TPB6_set_dir(PORT_DIR_IN);
 	NOP();
 	NOP();
 	if(PORTB_get_pin_level(6))g_motionValidDN=1;

  	PORTB_set_pin_level(6, 0);//The next eight bits are the highest 8 of 14 PIR ADC bits.
	TPB6_set_dir(PORT_DIR_OUT);
 	PORTB_set_pin_level(6, 1);
 	TPB6_set_dir(PORT_DIR_IN);
 	NOP();
  	NOP();
 	if(PORTB_get_pin_level(6))g_motionValueDN|=0x80;
 	PORTB_set_pin_level(6, 0);
 	TPB6_set_dir(PORT_DIR_OUT);
 	PORTB_set_pin_level(6, 1);
 	TPB6_set_dir(PORT_DIR_IN);
 	NOP();
  	NOP();
 	if(PORTB_get_pin_level(6))g_motionValueDN|=0x40;
 	PORTB_set_pin_level(6, 0);
 	TPB6_set_dir(PORT_DIR_OUT);
 	PORTB_set_pin_level(6, 1);
 	TPB6_set_dir(PORT_DIR_IN);
 	NOP();
 	NOP();
 	if(PORTB_get_pin_level(6))g_motionValueDN|=0x20;
 	PORTB_set_pin_level(6, 0);
 	TPB6_set_dir(PORT_DIR_OUT);
 	PORTB_set_pin_level(6, 1);
 	TPB6_set_dir(PORT_DIR_IN);
 	NOP();
  	NOP();
 	if(PORTB_get_pin_level(6))g_motionValueDN|=0x10;
 	PORTB_set_pin_level(6, 0);
 	TPB6_set_dir(PORT_DIR_OUT);
 	PORTB_set_pin_level(6, 1);
 	TPB6_set_dir(PORT_DIR_IN);
 	NOP();
  	NOP();
 	if(PORTB_get_pin_level(6))g_motionValueDN|=0x08;
 	PORTB_set_pin_level(6, 0);
 	TPB6_set_dir(PORT_DIR_OUT);
 	PORTB_set_pin_level(6, 1);
 	TPB6_set_dir(PORT_DIR_IN);
 	NOP();
  	NOP();
 	if(PORTB_get_pin_level(6))g_motionValueDN|=0x04;
 	PORTB_set_pin_level(6, 0);
 	TPB6_set_dir(PORT_DIR_OUT);
 	PORTB_set_pin_level(6, 1);
 	TPB6_set_dir(PORT_DIR_IN);
 	NOP();
  	NOP();
 	if(PORTB_get_pin_level(6))g_motionValueDN|=0x02;
 	PORTB_set_pin_level(6, 0);
 	TPB6_set_dir(PORT_DIR_OUT);
 	PORTB_set_pin_level(6, 1);
 	TPB6_set_dir(PORT_DIR_IN);
 	NOP();
  	NOP();
	if(PORTB_get_pin_level(6))g_motionValueDN|=0x01;

#ifdef DISPLAY_PIR_CONFIG
  	PORTB_set_pin_level(6, 0);
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))vala|=0x40;
  	PORTB_set_pin_level(6, 0);
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))vala|=0x20;
  	PORTB_set_pin_level(6, 0);
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))vala|=0x10;
  	PORTB_set_pin_level(6, 0);
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))vala|=0x08;
  	PORTB_set_pin_level(6, 0);
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))vala|=0x04;
  	PORTB_set_pin_level(6, 0);
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))vala|=0x02;
  	PORTB_set_pin_level(6, 0);
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))vala|=0x01;

  	PORTB_set_pin_level(6, 0);//The next eight bits are the highest 8 of 14 PIR ADC bits.
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))valb|=0x80;
  	PORTB_set_pin_level(6, 0);
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))valb|=0x40;
  	PORTB_set_pin_level(6, 0);
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))valb|=0x20;
  	PORTB_set_pin_level(6, 0);
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))valb|=0x10;
  	PORTB_set_pin_level(6, 0);
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))valb|=0x08;
  	PORTB_set_pin_level(6, 0);
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))valb|=0x04;
  	PORTB_set_pin_level(6, 0);
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))valb|=0x02;
  	PORTB_set_pin_level(6, 0);
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))valb|=0x01;

  	PORTB_set_pin_level(6, 0);//The next eight bits are the highest 8 of 14 PIR ADC bits.
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))valc|=0x80;
  	PORTB_set_pin_level(6, 0);
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))valc|=0x40;
  	PORTB_set_pin_level(6, 0);
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))valc|=0x20;
  	PORTB_set_pin_level(6, 0);
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))valc|=0x10;
  	PORTB_set_pin_level(6, 0);
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))valc|=0x08;
  	PORTB_set_pin_level(6, 0);
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))valc|=0x04;
  	PORTB_set_pin_level(6, 0);
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))valc|=0x02;
  	PORTB_set_pin_level(6, 0);
  	TPB6_set_dir(PORT_DIR_OUT);
  	PORTB_set_pin_level(6, 1);
  	TPB6_set_dir(PORT_DIR_IN);
  	NOP();
  	NOP();
  	if(PORTB_get_pin_level(6))valc|=0x01;


	PORTB_set_pin_level(6, 0);//The next eight bits are the highest 8 of 14 PIR ADC bits.
	TPB6_set_dir(PORT_DIR_OUT);
	PORTB_set_pin_level(6, 1);
	TPB6_set_dir(PORT_DIR_IN);
	NOP();
	NOP();
	if(PORTB_get_pin_level(6))vald|=0x80;
	PORTB_set_pin_level(6, 0);
	TPB6_set_dir(PORT_DIR_OUT);
	PORTB_set_pin_level(6, 1);
	TPB6_set_dir(PORT_DIR_IN);
	NOP();
	NOP();
	if(PORTB_get_pin_level(6))vald|=0x40;
	PORTB_set_pin_level(6, 0);
	TPB6_set_dir(PORT_DIR_OUT);
	PORTB_set_pin_level(6, 1);
	TPB6_set_dir(PORT_DIR_IN);
	NOP();
	NOP();
	if(PORTB_get_pin_level(6))vald|=0x20;
	PORTB_set_pin_level(6, 0);
	TPB6_set_dir(PORT_DIR_OUT);
	PORTB_set_pin_level(6, 1);
	TPB6_set_dir(PORT_DIR_IN);
	NOP();
	NOP();
	if(PORTB_get_pin_level(6))vald|=0x10;
	PORTB_set_pin_level(6, 0);
	TPB6_set_dir(PORT_DIR_OUT);
	PORTB_set_pin_level(6, 1);
	TPB6_set_dir(PORT_DIR_IN);
	NOP();
	NOP();
	if(PORTB_get_pin_level(6))vald|=0x08;
	PORTB_set_pin_level(6, 0);
	TPB6_set_dir(PORT_DIR_OUT);
	PORTB_set_pin_level(6, 1);
	TPB6_set_dir(PORT_DIR_IN);
	NOP();
	NOP();
	if(PORTB_get_pin_level(6))vald|=0x04;
	PORTB_set_pin_level(6, 0);
	TPB6_set_dir(PORT_DIR_OUT);
	PORTB_set_pin_level(6, 1);
	TPB6_set_dir(PORT_DIR_IN);
	NOP();
	NOP();
	if(PORTB_get_pin_level(6))vald|=0x02;
	PORTB_set_pin_level(6, 0);
	TPB6_set_dir(PORT_DIR_OUT);
	PORTB_set_pin_level(6, 1);
	TPB6_set_dir(PORT_DIR_IN);
	NOP();
	NOP();
	if(PORTB_get_pin_level(6))vald|=0x01;

	USART_0_initialization();
	USART_0_initialization();
	LCDspace();
	LCDshort(vala,2);
	LCDspace();
	LCDshort(valb,2);
	LCDspace();
	LCDshort(valc,2);
	LCDspace();
	LCDshort(vald,2);
	LCDspace();
#endif
 	PORTB_set_pin_level(6, 0);
 	TPB6_set_dir(PORT_DIR_OUT);
}*/
/*******************************************
*    PIR Get Bit
********************************************/
char getPIRbit(void){
	pirCLKdelay();
// 	PORTB_set_pin_level(6, 0);
 	TPB6_set_dir(PORT_DIR_OUT);
	PORTB_set_pin_level(6, 0);
	NOP();//pirCLKdelay();
//	pirCLKdelay();
	PORTB_set_pin_level(6, 1);
//	NOP();//pirCLKdelay();
//	pirCLKdelay();
	TPB6_set_dir(PORT_DIR_IN);
	NOP();
  	NOP();
//  	NOP();//Doesn't work with two NOPs
//  	NOP();//Works with three NOPs
	return PORTB_get_pin_level(6);
	pirENDdelay();
	TPB6_set_dir(PORT_DIR_IN);
}
/*******************************************
*    PIR Clear Interrupt
********************************************/
void clearBothPIRinterrupts(){
	PORTB_set_pin_level(6, 0);
	TPB6_set_dir(PORT_DIR_OUT);
#ifdef HAS_SEC_PIR	
	PORTA_set_pin_level(1, 0);
	TPA1_set_dir(PORT_DIR_OUT);
#endif	
	pirENDdelay();
	TPB6_set_dir(PORT_DIR_IN);
#ifdef HAS_SEC_PIR
	TPA1_set_dir(PORT_DIR_IN);
#endif
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
void pirENDdelay (void){
	uint32_t tickCtr;
	for(tickCtr=PIR_END_DELAY;tickCtr>0;tickCtr--){NOP();}
}
/***********************************************************************
***********************************************************************
**************************** ISRs *************************************
***********************************************************************
***********************************************************************/
/***********************************************************************
*  ISR TCB0 Timer for Prox PWM capture
***********************************************************************/
ISR(TCB0_INT_vect){
	short dumb;
		dumb=TCB0_CCMP;
// 		TCB0_INTCTRL = 0x01;//Enable capture interrupt flag
// 		TCB0_EVCTRL = 0x11;//Enable Captu
}
/***********************************************************************
*  ISR WAKEUP FROM BIG
***********************************************************************/
//	APRIL 2, 2020: THERE CAN BE A 30ms DELAY RESPONDING TO THE INTERRUPT FROM BIG BECAUSE TINY IS BUSY WITH RTC-PIT INTERRUPT
ISR(PORTC_PORT_vect){
	DISABLE_INTERRUPTS();
	TPC1_set_isc(PORT_ISC_INTDISABLE_gc);//C1=WAKEUP FROM BIG
	PORTC.INTFLAGS |= (1 << 1);
	char gotCmd;
	char iters;

	if(mState!=SOLAR_WAIT)gotCmd=getPacketFromBig();
	PORTC.INTFLAGS |= (1 << 1);
	if(mState==IDLE){
		sleepPins(1);//
		sleepPins(1);//
	}
	WDT_RST();//
	TPC1_set_isc(PORT_ISC_LEVEL_gc);//C1=WAKEUP FROM BIG
	ENABLE_INTERRUPTS();
}
/***********************************************************************
*  ISR USART RX
***********************************************************************/
ISR(USART0_RXC_vect){
	if(p_GPSrcvData>=(g_GPSrcv_buffer+GPS_BUFFER_SIZE))p_GPSrcvData=g_GPSrcv_buffer+GPS_BUFFER_SIZE-1;
	if((p_GPSrcvData>=g_GPSrcv_buffer)&&(p_GPSrcvData<g_GPSrcv_buffer+GPS_BUFFER_SIZE))
		*p_GPSrcvData=USART0.RXDATAL;
		if(USART0.RXDATAL==0x0A)endOfLineFlag=1;
	p_GPSrcvData++;
}
/***********************************************************************
*  ISR PRIMARY MOTION
***********************************************************************/
ISR(PORTB_PORT_vect){
	// B6 is Motion
	WDT_RST();//
	if(!g_tripFlagDN){
		g_tripFlagDN=1;
		motParams[NUM_TRIPS_INDEX]++;
		g_durCtr=0;
	}
	g_numTripZeroCtrDN=0;
	TPB6_set_dir(PORT_DIR_IN);
	g_gotMotionDN=1;
	PORTB.INTFLAGS |= (1 << 6);
	TPB6_set_isc(PORT_ISC_INTDISABLE_gc);
}
/***********************************************************************
*  ISR USER PUSHBUTTON AND SECONDARY MOTION
***********************************************************************/
ISR(PORTA_PORT_vect){
	DISABLE_INTERRUPTS();
	if((PORTA.INTFLAGS & 0x20)==0x20){//THIS LOOKS TO SEE IF IT WAS A5 (Pushbutton) or secondary PIR
		PORTA.INTFLAGS |= (1 << 5);
		WDT_RST();//
		userBlink(1);
		#ifdef HAS_GPS
			mState=BEGIN;
		#endif
		TPA5_set_isc(PORT_ISC_LEVEL_gc);//C1=WAKEUP FROM  USER PUSHBUTTON
		PORTA.INTFLAGS |= (1 << 5);
		ENABLE_INTERRUPTS();
		return;
	}else{// It wasn't A5 for User Pushbutton so assume it was A1 for secondary PIR
	// A1 is Motion
#ifdef HAS_SEC_PIR
	WDT_RST();//
	if(!g_tripFlagENT){
		g_tripFlagENT=1;
		motParams[NUM_TRIPS_INDEX]++;
		g_durCtr=0;
	}
	g_numTripZeroCtrENT=0;
	TPA1_set_dir(PORT_DIR_IN);
	g_gotMotionENT=1;
	PORTA.INTFLAGS |= (1 << 1);
	TPA1_set_isc(PORT_ISC_INTDISABLE_gc);
#endif	
	PORTA.INTFLAGS |= (1 << 1);//for safety, in case get an interrupt from secondary PIR, which we shouldn't. But if do the ISR will get stuck here.
	}
}

/***********************************************************************
*  ISR PIT (RTC)
*  BAD COMMS FROM BIG HIT WHEN TINY IS BUSY WITH THIS RTC-PIT INTERRUPT WHEN INTERRUPT FROM BIG HITS. RTC-PIT INTERRUPT ROUTINE TAKES ABOUT 30ms.
***********************************************************************/
ISR(RTC_PIT_vect)
{
	char iters;

	WDT_RST();//seems to require this one. Not sure why the main loop doesn't take care of it.
	if((g_tripFlagENT)&&(g_durCtr!=DURCTR_MAX))g_durCtr++;
#ifdef DO_DIAGS
	USART_0_initialization();
	USART_0_initialization();
	if(g_gotMotionDN) usart_put_string("1",1);
	else usart_put_string("0",1);
#ifdef HAS_SEC_PIR	
	if(g_gotMotionENT) usart_put_string("2",1);
	else usart_put_string("0",1);
#endif	
	delay_ms(30);
#endif
	if(g_motSecCtrDN>59)g_holdForBigFlag--;
	if((++g_motSecCtrDN>59)&&(g_holdForBigFlag<5)){
		g_holdForBigFlag=58;//reset for one minute of holding data before giving up on Big and restarting anyway.
		g_noMotionFlagDN=1;
		g_motSecCtrDN=0;
		g_durIndexDN=DUR_INDEX;
		g_motionPatternIndexDN=MOTION_INDEX;//1 because the raw motion patterns go into motParams[1..8]
		g_motionPatternMaskDN=0x80;
		}else{
		g_motionPatternMaskDN>>=1;
		if(g_motionPatternMaskDN==0){
			g_motionPatternMaskDN=0x80;
			if(++g_motionPatternIndexDN>8)g_motionPatternIndexDN=1;
		}
	}//end of <59 or >59

	if(g_gotMotionDN){
		g_noMotionFlagDN=0;
		g_motionMinTotalDN++;
		if(g_motSecCtrDN<60)motParams[g_motionPatternIndexDN]|=g_motionPatternMaskDN;

	}else{
		g_numTripZeroCtrDN++;
		if(g_numTripZeroCtrDN>100)g_numTripZeroCtrDN=100;
		if(g_numTripZeroCtrDN==NUMMOTSECS_BETWEEN_TRIPS){
			motParams[g_durIndexDN]=g_durCtr-NUMMOTSECS_BETWEEN_TRIPS;
			g_tripFlagDN=0;
			g_durCtr=DURCTR_MAX;
			g_durIndexDN++;
		}
	}

// 	if(g_gotMotionENT){
// 		g_noMotionFlagENT=0;
// 		g_motionMinTotalENT++;
// 		if(g_motSecCtrENT<60)motParams[g_motionPatternIndexENT]|=g_motionPatternMaskENT;
// 
// 		}else{
// 		g_numTripZeroCtrENT++;
// 		if(g_numTripZeroCtrENT>100)g_numTripZeroCtrENT=100;
// 		if(g_numTripZeroCtrENT==NUMMOTSECS_BETWEEN_TRIPS){
// 			motParams[g_durIndexENT]=g_durCtr-NUMMOTSECS_BETWEEN_TRIPS;
// 			g_tripFlagENT=0;
// 			g_durCtr=DURCTR_MAX;
// 			g_durIndexENT++;
// 		}
// 	}
	clearBothPIRinterrupts();//PIR DL line (interrupts) can get stuck high if not acknowledged by a pull low.
	/* Overflow interrupt flag has to be cleared manually */
	g_gotMotionDN=0;
	g_gotMotionENT=0;
	TPB6_set_isc(PORT_ISC_RISING_gc);
#ifdef HAS_SEC_PIR	
	TPA1_set_isc(PORT_ISC_RISING_gc);
#endif	
	RTC.PITINTFLAGS = 1;
}


