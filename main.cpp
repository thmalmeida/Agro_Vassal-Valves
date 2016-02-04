/*
Used ports

PG5 --> k1
PE5 --> k2
PE4	--> k3

PC2 --> s11
PC1 --> s10
PC0 --> s09
PD7 --> s08
PG2 --> s07
PG1 --> s06
PG0 --> s05
PL7 --> s04
PL6 --> s03
PL5 --> s02
PL4 --> s01
PB4 --> f01
PB5 --> f02

PH3 --> k1_read
PE3 --> k3_read
PH4 --> Th_read

PF0 --> (A0) Damage!!!
PF1 --> (A1) ADC Irms
PF2	-->	(A2) ADC Pressure Sensor
PF3 -->	(A3) ADC Reservoir Level (useful)
PD1 --> SDA DS1307
PD0 --> SDL DS1307

PD3 --> TX1 BT
PD2 --> RX1 BT

Errors:
0x00: No error detected;
0x01: AND OP: no valves opened!
0x02: Thermal safe!
0x03: Im sensor down!
0x04: PRessure DOWN!
0x05: PRessure HIGH!
0x06:
0x07: Time after green flag
0x08: Bluetooth serial first error
0x09: Bluetooth serial second error
0x10: Mission Acomplished!
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "Comm/Wire.h"

#include <Arduino.h>

#define s11_on()	PORTB |=  (1<<4);	// S11
#define s11_off()	PORTB &= ~(1<<4);	// S11
#define	s10_on()	PORTB |=  (1<<1);	// S10
#define	s10_off()	PORTB &= ~(1<<1);	// S10
#define s09_on()	PORTB |=  (1<<3);	// S09
#define s09_off()	PORTB &= ~(1<<3);	// S09
#define s08_on()	PORTB |=  (1<<2);	// S08
#define s08_off()	PORTB &= ~(1<<2);	// S08
#define s07_on()	PORTB |=  (1<<0);	// S07
#define s07_off()	PORTB &= ~(1<<0);	// S07
#define s06_on()	PORTD |=  (1<<7);	// S06
#define s06_off()	PORTD &= ~(1<<7);	// S06
#define s05_on()	PORTD |=  (1<<6);	// S05
#define s05_off()	PORTD &= ~(1<<6);	// S05
#define s04_on()	PORTD |=  (1<<5);	// S04
#define s04_off()	PORTD &= ~(1<<5);	// S04
#define s03_on()	PORTD |=  (1<<4);	// S03
#define s03_off()	PORTD &= ~(1<<4);	// S03
#define s02_on()	PORTD |=  (1<<3);	// S02
#define s02_off()	PORTD &= ~(1<<3);	// S02
#define s01_on()	PORTD |=  (1<<2);	// S01
#define s01_off()	PORTD &= ~(1<<2);	// S01

#define s11_readPin	bit_is_set(PINB, 4)
#define s10_readPin	bit_is_set(PINB, 1)
#define s09_readPin	bit_is_set(PINB, 3)
#define s08_readPin	bit_is_set(PINB, 2)
#define s07_readPin	bit_is_set(PINB, 0)
#define s06_readPin	bit_is_set(PIND, 7)
#define s05_readPin	bit_is_set(PIND, 6)
#define s04_readPin	bit_is_set(PIND, 5)
#define s03_readPin	bit_is_set(PIND, 4)
#define s02_readPin	bit_is_set(PIND, 3)
#define s01_readPin	bit_is_set(PIND, 2)


// ---------- Global ----------

// EEPROM memory Addresses
const uint8_t addr_maxIs		= 6;		// 2  bytes
uint16_t maxIs = 0;

//// Logs
//const int nLog = 12;
//uint8_t loadLog[nLog];
//uint8_t loadStatus[nLog];
//uint8_t hourLog[nLog];
//uint8_t minuteLog[nLog];

// Wake up interrupts
uint8_t flag_WDRF = 0;			// Watchdog System Reset Flag
uint8_t flag_BORF = 0;			// Brown-out Reset Flag
uint8_t flag_EXTRF = 0;			// External Reset Flag
uint8_t flag_PORF = 0;			// Power-on Reset Flag

//uint8_t motorStatus = 0;
//uint8_t periodState = 0;
uint8_t stateMode = 0;
//
//uint8_t flag_iValveCheck = 1;
//uint8_t flag_waitPowerOn = 0;	// Minutes before start motor after power line ocasionally down
//uint8_t powerOn_min_Standy = 0;
//uint8_t powerOn_min = 0;
//uint8_t powerOn_sec = 0;
//
//volatile uint16_t timeSector = 0;
//uint8_t timeSectorVectorMin[11];
//uint8_t flag_1s = 0;
//uint8_t lastError = 0;
//
//uint8_t valveSequence[11] 	= {0,0,0,0,0,0,0,0,0,0,0};
uint8_t valveStatus[11] 	= {0,0,0,0,0,0,0,0,0,0,0};
uint8_t sectorCurrently 	= 0;
//uint8_t sectorChanged 		= 0;

volatile uint8_t frame[10];

uint8_t flag_frameStartBT = 0;

// Communicaton variables
char inChar, aux[3], aux2[5], sInstr[15];
uint8_t rLength;
char sInstrBluetooth[15];
uint8_t j2 = 0;
uint8_t enableTranslate_Bluetooth = 0;
uint8_t enableDecode = 0;

uint16_t valveIs = 0;
uint8_t hbyte_valveSensor = 0;
uint8_t lbyte_valveSensor = 0;

//struct Valves
//{
//	uint8_t sectorStatus[11];
//	uint8_t sector;
//	uint8_t status;
//};

void init_valves()
{
	DDRB |= (1<<4);	// S11
	DDRB |= (1<<3);	// S10
	DDRB |= (1<<2); // S09
	DDRB |= (1<<1); // S08
	DDRB |= (1<<0); // S07
	DDRD |= (1<<7);	// S06
	DDRD |= (1<<6);	// S05
	DDRD |= (1<<5);	// S04
	DDRD |= (1<<4); // S03
	DDRD |= (1<<3); // S02
	DDRD |= (1<<2); // S01
}
void init_ADC()
{
//	ADCSRA ==> ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);	// Set 128 division clock
	ADCSRA |= (1<<ADEN); 				// Enable module

//	ADCSRB ==>	–	ACME	–	–	MUX5	ADTS2	ADTS1	ADTS0
	ADCSRB &= ~(1<<ADTS2);				// Free running mode.
	ADCSRB &= ~(1<<ADTS1);
	ADCSRB &= ~(1<<ADTS0);

//	ADCSRB &= ~(1<<MUX5);				// To select ADC0;

//	ADMUX ==> REFS1 REFS0 ADLAR MUX4 MUX3 MUX2 MUX1 MUX0
//	ADMUX &=  ~(1<<REFS1);				// AREF, Internal Vref turned off
//	ADMUX &=  ~(1<<REFS0);

	ADMUX &=  ~(1<<REFS1);				// AVCC with external capacitor at AREF pin
	ADMUX |=   (1<<REFS0);

//	ADMUX |=   (1<<REFS1);				// Reserved
//	ADMUX &=  ~(1<<REFS0);

//	ADMUX |=  (1<<REFS0);				// Internal 1.1V Voltage Reference with external capacitor at AREF pin
//	ADMUX |=  (1<<REFS1);


//	ADMUX |=  (1<<ADLAR);				// Left Adjustment. To ADCH register.
//										// Using 8 bits. Get ADCH only.
	ADMUX &= ~(1<<ADLAR);				// Right Adjustment. To ADCL register.
										// Using 10 bits
//	ADMUX &= ~(1<<MUX4);
	ADMUX &= ~(1<<MUX3);
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX1);
	ADMUX &= ~(1<<MUX0);				// Select ADC0
}
void init_WDT()
{
	// Configuring to enable only Reset System if occurs 4 s timeout
//	WDTCSR <== WDIF WDIE WDP3 WDCE WDE WDP2 WDP1 WDP0
//	WDTCSR |=  (1<<WDCE) | (1<<WDE);	// Enable Watchdog Timer
//	WDTCSR &= ~(1<<WDIE);				// Disable interrupt
//
//	WDTCSR |=  (1<<WDP3);				// 512k (524288) Cycles, 4.0s
//	WDTCSR &= ~(1<<WDP2);
//	WDTCSR &= ~(1<<WDP1);
//	WDTCSR &= ~(1<<WDP0);

//	WDTCSR |=  (1<<WDCE);
//	WDTCSR = 0b00111000;

//	wdt_enable(WDTO_8S);
	// WDT enable

	wdt_enable(WDTO_8S);
}
void stop_WDT()
{
	cli();
//	__watchdog_reset();
	/* Clear WDRF in MCUSR */
	MCUSR &= ~(1<<WDRF);
	/* Write logical one to WDCE and WDE */
	/* Keep old prescaler setting to prevent unintentional time-out */
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	/* Turn off WDT */
	WDTCSR = 0x00;
	sei();
}

float calcIrms()//uint8_t channel)//, uint8_t numberOfCycles)
{
	int i, j=0;
	uint8_t high, low;
	int divScale_count = 1;									// Não mexer!

	ADMUX  &= ~(1<<MUX3);
	ADMUX  &= ~(1<<MUX2);
	ADMUX  &= ~(1<<MUX1);
	ADMUX  &= ~(1<<MUX0);									// Select ADC0

	// Parameters for ADC converter - Variables
	const float f = 60.0;									// Hertz;
	const int numberOfCycles = 2;							// Number of cycles;
	const int divScale = 16;									// Prescale for real sample rate Fs;

	const float Fs = 16000000/128/13;									// Sample rate of signal processed;
	const int nPointsPerCycle = (int) Fs/f;								// Number of points per cycle;
	const int nPoints = (int) nPointsPerCycle*numberOfCycles; 			// Number of signal points.

	const float Fs_div = 16000000/128/13/divScale;						// Sample rate of signal processed;
	const int nPointsPerCycle_div = (int) Fs_div/f;						// Number of points per cycle;
	const int nPoints_div = (int) nPointsPerCycle_div*numberOfCycles;	// Number of signal points.


//	sprintf(buffer,"---- Signal Captured ----");
//	Serial.println(buffer);
//	Serial.println("");
//
//	Serial.print("Fs:");
//	Serial.println(Fs);
//	Serial.println("");
//
//	sprintf(buffer,", nPointsPerCycle:%d", nPointsPerCycle);
//	Serial.println(buffer);
//	Serial.println("");
//
//	sprintf(buffer,"nPoints:%d", nPoints);
//	Serial.println(buffer);
//	Serial.println("");
//
//
//
//	sprintf(buffer,"---- Signal Processed ----");
//	Serial.println(buffer);
//	Serial.println("");
//
//	Serial.print("Fs:");
//	Serial.println(Fs_div);
//	Serial.println("");
//
//	sprintf(buffer,", nPointsPerCycle:%d", nPointsPerCycle_div);
//	Serial.println(buffer);
//	Serial.println("");
//
//	sprintf(buffer,"nPoints:%d", nPoints_div);
//	Serial.println(buffer);
//	Serial.println("");

//	char buffer[10];
//	sprintf(buffer,"A%d ",freeMemory());
//	Serial.println(buffer);

	int *adcSamples = NULL;
	adcSamples = (int*)malloc(nPoints_div * sizeof(int));

//	sprintf(buffer,"B% d",freeMemory());
//	Serial.println(buffer);

	// 160.2564 = 16000000/128/13/60.0;
	for(i=0;i<nPoints;i++)
	{
		ADCSRA |= (1<<ADSC);				// Start conversion;
		while (bit_is_set(ADCSRA, ADSC));	// wait until conversion done;

//		Serial.println((ADCH << 8) | ADCL);

		if(divScale_count == 1)
		{
			low  = ADCL;
			high = ADCH;

			j = (int) i/divScale;
			adcSamples[j] = (high << 8) | low;
			divScale_count = divScale;
		}
		else
		{
			divScale_count--;
		}
	}
//	sprintf(buffer,"C%d",freeMemory());
//	Serial.println(buffer);

//	Serial.println("ENTROU!");
//	for(i=0;i<nPoints_div;i++)
//	{
//		Serial.println(adcSamples[i]);
//	}
//	Serial.println("SAIU!");

	float *vs = NULL;
	vs = (float*)malloc(nPoints_div*sizeof(float));

	for(i=0;i<nPoints_div;i++)
	{
		vs[i] = (adcSamples[i]*5.0)/1024.0;
	}

//	sprintf(buffer,"D%d",freeMemory());
//	Serial.println(buffer);

	free(adcSamples);
//	sprintf(buffer,"E%d",freeMemory());
//	Serial.println(buffer);

	// Offset remove.
	float Vmean = 0.0;
	for(i=0;i<nPoints_div;i++)
		Vmean += vs[i];

	Vmean = Vmean/nPoints_div;

	for(i=0;i<nPoints_div;i++)
		vs[i] = vs[i] - Vmean;

//	sprintf(buffer,"F %d",freeMemory());
//	Serial.println(buffer);

	float *vs2 = NULL;
	vs2 = (float*)malloc(nPoints_div*sizeof(float));
	// Power signal
	for(i=0;i<nPoints_div;i++)
		vs2[i] = vs[i]*vs[i];

//	sprintf(buffer,"G %d",freeMemory());
//	Serial.println(buffer);
	free(vs);

//	sprintf(buffer,"H %d",freeMemory());
//	Serial.println(buffer);

	float sum=0;
	float V2mean;

	// mean finder
	for(i=0;i<nPoints_div;i++)
		sum += vs2[i];
	V2mean = sum/nPoints_div;

	free(vs2);

//	sprintf(buffer,"I%d",freeMemory());
//	Serial.println(buffer);

	float I = 0.0;
	float k = 2000.0;		// Não lembro o porquê desta constante. Talvez numero de voltas;
	float R = 3900.0;

	// RMS equation
	I = (k*sqrt(V2mean))/R;

	return I;
}

uint8_t valveRead(uint8_t sectorPrivate)
{
	uint8_t status = 0;

	switch (sectorPrivate)
	{
		case 1:
			if(s01_readPin)
				status = 1;
			else
				status = 0;
		break;

		case 2:
			if(s02_readPin)
				status = 1;
			else
				status = 0;
			break;

		case 3:
			if(s03_readPin)
				status = 1;
			else
				status = 0;
			break;

		case 4:
			if(s04_readPin)
				status = 1;
			else
				status = 0;
			break;

		case 5:
			if(s05_readPin)
				status = 1;
			else
				status = 0;
			break;

		case 6:
			if(s06_readPin)
				status = 1;
			else
				status = 0;
			break;

		case 7:
			if(s07_readPin)
				status = 1;
			else
				status = 0;
			break;

		case 8:
			if(s08_readPin)
				status = 1;
			else
				status = 0;
			break;

		case 9:
			if(s09_readPin)
				status = 1;
			else
				status = 0;
			break;

		case 10:
			if(s10_readPin)
				status = 1;
			else
				status = 0;
			break;

		case 11:
			if(s11_readPin)
				status = 1;
			else
				status = 0;
			break;

		default:
			break;

	}

	return status;
}
void valveSet(uint8_t sectorPrivate, uint8_t status)
{
	switch (sectorPrivate)
	{
		case 1:
			if(status)
			{
				s01_on();
				valveStatus[0] = 1;
			}
			else
			{
				s01_off();
				valveStatus[0] = 0;
			}
			break;

		case 2:
			if(status)
			{
				s02_on();
				valveStatus[1] = 1;
			}
			else
			{
				s02_off();
				valveStatus[1] = 0;
			}
			break;

		case 3:
			if(status)
			{
				s03_on();
				valveStatus[2] = 1;
			}
			else
			{
				s03_off();
				valveStatus[2] = 0;
			}
			break;

		case 4:
			if(status)
			{
				s04_on();
				valveStatus[3] = 1;
			}
			else
			{
				s04_off();
				valveStatus[3] = 0;
			}
			break;

		case 5:
			if(status)
			{
				s05_on();
				valveStatus[4] = 1;
			}
			else
			{
				s05_off();
				valveStatus[4] = 0;
			}
			break;

		case 6:
			if(status)
			{
				s06_on();
				valveStatus[5] = 1;
			}
			else
			{
				s06_off();
				valveStatus[5] = 0;
			}
			break;

		case 7:
			if(status)
			{
				s07_on();
				valveStatus[6] = 1;
			}
			else
			{
				s07_off();
				valveStatus[6] = 0;
			}
			break;

		case 8:
			if(status)
			{
				s08_on();
				valveStatus[7] = 1;
			}
			else
			{
				s08_off();
				valveStatus[7] = 0;
			}
			break;

		case 9:
			if(status)
			{
				s09_on();
				valveStatus[8] = 1;
			}
			else
			{
				s09_off();
				valveStatus[8] = 0;
			}
			break;

		case 10:
			if(status)
			{
				s10_on();
				valveStatus[9] = 1;
			}
			else
			{
				s10_off();
				valveStatus[9] = 0;
			}
			break;

		case 11:
			if(status)
			{
				s11_on();
				valveStatus[10] = 1;
			}
			else
			{
				s11_off();
				valveStatus[10] = 0;
			}
			break;

		default:
			break;
	}
}

int get_ADC0()
{
	uint8_t low, high;
	int adcValue;

	ADMUX &= ~(1<<MUX3);
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX1);
	ADMUX &= ~(1<<MUX0);				// Select ADC0

	ADCSRA |= (1<<ADSC);				// Start conversion;
	while (bit_is_set(ADCSRA, ADSC));	// wait until conversion done;

//		Serial.println((ADCH << 8) | ADCL);
	low  = ADCL;
	high = ADCH;

	adcValue = (high << 8) | low;

	return adcValue;
}

void refreshStoredData()
{
	uint8_t lbyte, hbyte;

	lbyte = eeprom_read_byte((uint8_t *)(addr_maxIs));
	hbyte = eeprom_read_byte((uint8_t *)(addr_maxIs+1));

	maxIs = ((hbyte << 8) | lbyte);
}

//void process_valveTest()
//{
//	wdt_reset();
//	valveInstr(sectorCurrently, 1, 0);
//	_delay_ms(4000);
//	wdt_reset();
//	valveInstr(sectorCurrently, 0, 0);
//	_delay_ms(4000);
////	Serial.println(sectorCurrently);
////	{
////		stateMode = 1;
////
////		tm.Hour = 21;
////		tm.Minute = 29;
////		tm.Second = 57;
////		RTC.write(tm);
////
////	}
//}
//void process_Mode()
//{
//	switch(stateMode)
//	{
//		case 0:	// Manual Process
////			process_Manual();
//			break;
//
//		case 1: // nightMode
////			process_Programmed();
//			break;
//
//		case 9:
////			process_valveTest();
//			break;
//
//		default:
//			stateMode = 0;
//			break;
//
////		case 2:
////			process_Working();
////			break;
//
////		case 3: //onlyOneSector. Turn on at night only one sector
////			process_Programmed();
////			break;
//
////		case 4: //valveTesting
//			// Automatic Process
////			process_valveTest();
////			break;
//	}
//}
//
//void refreshVariables()
//{
//
//}
//
//void comm_Bluetooth()
//{
//	// Rx - Always listening
////	uint8_t j2 =0;
//	while((Serial.available()>0))	// Reading from serial
//	{
//		inChar = Serial.read();
//
//		if(inChar=='$')
//		{
//			j2 = 0;
//			flag_frameStartBT = 1;
////			Serial.println("Frame Start!");
//		}
//
//		if(flag_frameStartBT)
//			sInstrBluetooth[j2] = inChar;
//
////		sprintf(buffer,"J= %d",j2);
////		Serial.println(buffer);
//
//		j2++;
//
//		if(j2>=sizeof(sInstrBluetooth))
//		{
//			memset(sInstrBluetooth,0,sizeof(sInstrBluetooth));
//			j2=0;
////			summary_Print(28);
//		}
//
//		if(inChar==';')
//		{
////			Serial.println("Encontrou ; !");
//			if(flag_frameStartBT)
//			{
////				Serial.println("Frame Stop!");
//				flag_frameStartBT = 0;
//				rLength = j2;
//				j2 = 0;
//				enableTranslate_Bluetooth = 1;
//			}
//		}
//	}
////	flag_frameStart = 0;
//
//	if(enableTranslate_Bluetooth)
//	{
////		Serial.println("enableTranslate_Bluetooth");
//		enableTranslate_Bluetooth = 0;
//
//		char *pi0, *pf0;
//		pi0 = strchr(sInstrBluetooth,'$');
//		pf0 = strchr(sInstrBluetooth,';');
//
//		if(pi0!=NULL)
//		{
//			uint8_t l0=0;
//			l0 = pf0 - pi0;
//
//			int i;
//			for(i=1;i<=l0;i++)
//			{
//				sInstr[i-1] = pi0[i];
////				Serial.write(sInstr[i-1]);
//			}
////			memset(sInstrBluetooth,0,sizeof(sInstrBluetooth));
//	//		Serial.println(sInstr);
//
//			enableDecode = 1;
//		}
//		else
//		{
////			summary_Print(29);
//			Serial.write(pi0[0]);
//			Serial.write(pf0[0]);
//		}
//		memset(sInstrBluetooth,0,sizeof(sInstrBluetooth));
//	}
//}
//
//void handleMessage()
//{
//	// read I2C buffer
//
//	// Take the code
//
//	// Execute what is requested
//
//	// Send answer back;
//}

//Master change the maxIs
//aux2[0] = '0';
//aux2[1] = sInstr[5];
//aux2[2] = sInstr[6];
//aux2[3] = sInstr[7];
//aux2[4] = '\0';
//
//maxIv = (uint16_t) atoi(aux2);
//
//uint8_t lbyteRef = 0, hbyteRef = 0;
//lbyteRef = maxIv;
//hbyteRef = (maxIv >> 8);
//
//eeprom_write_byte((uint8_t *)(addr_maxIv+1), hbyteRef);
//eeprom_write_byte((uint8_t *)(addr_maxIv), lbyteRef);


//	 ____________________________
//	|start|type|size|payload|stop|
//
// 	start 	-> 1 byte;
// 	type	-> 1 byte;
// 	size	-> 1 byte (values 0 to 16);
// 	payload -> 16 bytes; (fixed)
// 	stop	-> 1 byte;

// Types:
// Type 1:
//	- set one valve;
//	- size: 11

uint8_t flag_frameProssesing = 0;

void frameProcessing()
{
	Serial.println("");
	Serial.print("Frame[0]");
	Serial.println(frame[0]);
	Serial.print("Frame[1]");
	Serial.println(frame[1]);
	Serial.print("Frame[2]");
	Serial.println(frame[2]);
	Serial.print("Frame[3]");
	Serial.println(frame[3]);

	// Frame processing
	uint8_t type = frame[0];
	uint8_t sectorPrivate = 0;
	uint8_t sectorCommand = 0;
	int i;

	switch (type)
	{
		case 1:	// Valve set ou clear
			sectorPrivate = frame[2];
			sectorCommand = frame[3];

			valveSet(sectorPrivate, sectorCommand);	// Command set

			for(i=0; i<11; i++)
			{
				valveStatus[i] = valveRead(i+1);	// refresh valveStatus vector reading from pin
			}
			break;

		case 2: // maxIs set;
			maxIs = ((frame[3] << 8) | frame[2]);

			eeprom_write_byte((uint8_t *)(addr_maxIs), frame[2]);
			eeprom_write_byte((uint8_t *)(addr_maxIs+1), frame[3]);

//			Serial.print("maxIs: ");
//			Serial.println(maxIs);
			break;

		case 3:
			for(i=0; i<11; i++)
			{
				valveSet(i+1, 0);
				valveStatus[i] = valveRead(i+1);	// refresh valveStatus vector reading from pin
			}
			break;

		default:
			break;
	}
}
void receiveEvent(int howMany)	// Rx - when some master write we get information and execute something
{
	int k=0;

	while (1 < Wire.available())	// loop through all but the last
	{
		frame[k] = Wire.read();		// receive byte as a character
		Serial.print(frame[k]);		// print the character
		k++;
	}
	frame[k] = Wire.read();   		// receive byte as an integer
	Serial.println(frame[k]);		// print the integer

	flag_frameProssesing = 1;
}
void requestEvent()	// Tx when some master request for information we write down on buffer
{
//	Wire.write("Fuck you too!");
	int k;
	uint8_t *vector = NULL;
	vector = (uint8_t*)malloc(15 * sizeof(uint8_t));

	for(k=0; k<11; k++)
	{
		vector[k] = valveStatus[k];
	}

	vector[11] = valveIs;
	vector[12] = (valveIs >> 8);

	vector[13] =  maxIs;
	vector[14] = (maxIs >> 8);

	Wire.write(vector, 15);
	free(vector);
}

int main()
{
	cli();								// Clear all interruptions
	flag_WDRF 	= ((1<<WDRF)  & MCUSR);	// PowerOFF / Reset verification
	flag_BORF 	= ((1<<BORF)  & MCUSR);
	flag_EXTRF 	= ((1<<EXTRF) & MCUSR);
	flag_PORF 	= ((1<<PORF)  & MCUSR);
	MCUSR = 0x00;
	sei();								// System enable interruptions

	init();								// Initialize arduino hardware requirements.
//	init_WDT();
//	init_ADC();
	init_valves();
	init_ADC();
//	init_Timer1_1Hz();
	Wire.begin(8);
	Wire.onRequest(requestEvent);
	Wire.onReceive(receiveEvent);

	Serial.begin(9600);					// Debug
	Serial.println("_V_");				// Welcome!

	refreshStoredData();

	while (1)
	{
		if(flag_frameProssesing)
		{
			flag_frameProssesing = 0;
			frameProcessing();
		}

		valveIs = (uint16_t) (1000*calcIrms());

		if(valveIs > maxIs)
		{
			int k=0;
			uint8_t sumValves = 0;
			for(k=0; k<11; k++)
			{
				sumValves += valveStatus[k];
			}

			if(sumValves < 2)
			{
				Serial.print("valveIs: ");
				Serial.println(valveIs);

				Serial.println("");
				Serial.print("maxIs: ");
				Serial.println(maxIs);

				valveSet(sectorCurrently, 0);
				for(k=0; k<11; k++)
				{
					valveSet(k+1, 0);
				}
			}
		}

//		Debug ------
//		valveIs = (hbyte_valveSensor << 8) | lbyte_valveSensor;
//		Serial.println(valor);
//		Serial.println(valveIs);
//		_delay_ms(500);
//		------------
	}
}
