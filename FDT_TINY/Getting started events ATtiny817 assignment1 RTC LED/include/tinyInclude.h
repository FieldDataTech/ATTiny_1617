/*
 * tinyInclude.h
 *
 * Created: 2/17/2018 8:01:08 PM
 *  Author: doug
 */ 


#ifndef TINYINCLUDE_H_
#define TINYINCLUDE_H_

#define TICK_CONSTANT_MS 410
//#define TICK_CONSTANT_HUS 46
#define TICK_CONSTANT_HUS 46
//unsigned int g_motionCtr=0;
#define PIR_SU_DELAY 40 //40 for Interrupt Mode.  Needs to be high for at least 75 uSeconds, less than 150 us after sensor drives it high.
#define PIR_END_DELAY 60 //was 1  Needs to be at least 145 us.
#define REQ_ONE_MINUTE_TOTALS (0)
#define REQ_ALL_MOTION_SECONDS (1)
#define USART0_BAUD_RATE(BAUD_RATE) ((float)3333333.33333 * 64 / (16 * (float)BAUD_RATE))
#define BTTIMEOUT 200	//only needs to count to about 7, with clocks per March 2018
#define DISABLE_INTERRUPTS()        __asm__ __volatile__ ( "cli" ::: "memory")
#define ENABLE_INTERRUPTS()         __asm__ __volatile__ ( "sei" ::: "memory")
#define NOP()			__asm__ __volatile__ ("nop")
#define WDT_RST()		__asm__ __volatile__ ("wdr")
char g_rcvdCmd=0;
enum {
	BEGIN,//0
	INITIAL_GPS,//1
	GET_GPS,//2
	NO_INITIAL_GPS_A,//3
	NO_INITIAL_GPS_B,//4
	NO_INITIAL_GPS_C,//5
	NO_INITIAL_GPS_LONGTERM,//6
	IDLE,//7
	SOLAR_WAIT//8
	} mState;
	
#define MAX_GPS_ACQ 720
#define MAX_INITIAL_GPS_ACQ 940
#define DUR_INDEX 10
#define VERS_HBITS_INDEX 0
#define MOTION_INDEX 1
#define MOTION_TOTAL_INDEX 9
#define NUM_TRIPS_INDEX 22
#define ALS_INDEX 23
#define PARTIAL_LOC_INDEX 24  //CRC are placed at motParams[26..27]
static char endOfLineFlag=0;
static short timeToFix;
static char g_motionCtr;
char *GPSSearchPtr;
char uBloxStatus;
unsigned int g_ul_tick_count;
#define SIZEOFGPSPARAMS 25 //
#define SIZEOFMOTPARAMS 28  //28 normal plus up to 16 more for partial params
#define SIZEOFPARTIALPARAMS 10  //limited by need to do it with only one decimal char? something like that, I don't remember exactly
static char partialParams[SIZEOFPARTIALPARAMS];
static char gpsParams[SIZEOFGPSPARAMS];
static char motParams[SIZEOFMOTPARAMS];
#define GPS_BUFFER_SIZE 182	//182
static char g_GPSrcv_buffer[GPS_BUFFER_SIZE];//total RAM 512 bytes
char* p_GPSrcvData;
#define BIG_BUFFER_SIZE 30
static char g_BIGrcv_buffer[BIG_BUFFER_SIZE];//total RAM 512 bytes
char* p_BIGrcvData;
static char g_rxData[10];
static char g_rxDataIndex=0;

static char g_gotMotionDN;
static char g_numTripZeroCtrDN;
static char g_tripFlagDN;
static char g_motionValueDN;
static char g_motionValidDN;
static char g_motSecCtrDN;//
static char g_durIndexDN;//
static char g_motionMinTotalDN=0;
static char g_motionPatternIndexDN=0;//
static char g_motionPatternMaskDN;//
static char g_noMotionFlagDN;//

static char g_gotMotionENT;
static char g_numTripZeroCtrENT;
static char g_tripFlagENT;
static char g_motionValueENT;
static char g_motionValidENT;
static char g_motSecCtrENT;
static char g_durIndexENT;
static char g_motionMinTotalENT=0;
static char g_motionPatternIndexENT=0;
static char g_motionPatternMaskENT;
static char g_noMotionFlagENT;

static char g_holdForBigFlag;
static short g_light;
// static char g_ADvalsB;
// static char g_ADvalsC;
// static char g_ADvalsD;
// static char g_ADvalsE;
// static char g_ADvalsF;
// static char g_ADvalsG;
// static char g_ADvalsH;
// static char g_ADvalsI;
// static char g_ADvalsJ;
static char g_numTrips;
static char g_durCtr;
#define DURCTR_MAX 254

short getALSfromVEML(void);
short getMotion(void);



unsigned short calcCRC(char*, char);
void clearMotionData8(void);

void delay_ms(uint32_t);
void myDelay_hus(uint32_t);
void redBlink(char);
void userBlink(char);
void redBlinkLong(char);

void send25BytesToBig(char*);
void sendSPIbyte(char);
void sendSPIbuff(char*,char);
void rcvSPIbuff(char*);
void sendByteToBig (char);
char getPacketFromBig(void);
char getByteFromBig(void);

int8_t USART_0_putc(const uint8_t);
int8_t USART_0_tx_empty(void);
void usart_put_string(const char*, const uint8_t);
char hex1ToAscii(char);
int asciiHexToHex(char, char);

void LCDclear (void);
void LCDbottomLine(void);
void LCDspace(void);
void LCDshort(short, char);
unsigned int hex2ToAscii(int);

void initGPSuBlox (void);
void closeGPS (void);
short GPSgetDate(char);
short GPSgetTime(char);
void GPSgetLoc(void);
short gpsReqDateUblox (char);
short gTryUblox (char);
short gpsReqLocUblox (char);
short getGPS(char);

int asciiToHex(char,char);
void showISR(char);
void sleepPins(char);
void USART0_ON(char);
void USART0_OFF(void);

char getBatt(void);
char getLight(void);

void pirSUdelay (void);
void pirENDdelay (void);
char getPIRbit(void);
char getPIRbyte(void);
void getPIR();
char g_sensitivityHiBit;
char g_sensitivityLoByte;
void setPIRconfigDN(char,char,char,char);
void setPIRconfigENT(char,char,char,char);
void setPIRbitDN(char);
void setPIRbitENT(char);
void configXBee(void);
void clearBothPIRinterrupts(void);

#endif /* TINYINCLUDE_H_ */