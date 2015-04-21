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
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include <string.h>
#include <stdlib.h>

#include <Arduino.h>

#include "DS1307RTC.h"
#include "Time.h"

//#include "MemoryFree.h"

tmElements_t tm;

#define k3_on()		PORTD |=  (1<<5);	// Triac 3 is enabled
#define k3_off()	PORTD &= ~(1<<5);	// Triac 3 is disabled
#define k2_on()		PORTD |=  (1<<6);	// Triac 2 is enabled
#define k2_off()	PORTD &= ~(1<<6);	// Triac 2 is disabled
#define k1_on()		PORTD |=  (1<<7);	// Triac 1 is enabled
#define k1_off()	PORTD &= ~(1<<7);	// Triac 1 is disabled

#define s11_on()	PORTB |=  (1<<4);	// S11
#define s11_off()	PORTB &= ~(1<<4);	// S11
#define	s10_on()	PORTB |=  (1<<3);	// S10
#define	s10_off()	PORTB &= ~(1<<3);	// S10
#define s09_on()	PORTB |=  (1<<2);	// S09
#define s09_off()	PORTB &= ~(1<<2);	// S09
#define s08_on()	PORTB |=  (1<<1);	// S08
#define s08_off()	PORTB &= ~(1<<1);	// S08
#define s07_on()	PORTB |=  (1<<0);	// S07
#define s07_off()	PORTB &= ~(1<<0);	// S07

#define s06_on()	PORTB |=  (1<<5);	// S06
#define s06_off()	PORTB &= ~(1<<5);	// S06
#define s05_on()	PORTB |=  (1<<4);	// S05
#define s05_off()	PORTB &= ~(1<<4);	// S05
#define s04_on()	PORTB |=  (1<<3);	// S04
#define s04_off()	PORTB &= ~(1<<3);	// S04
#define s03_on()	PORTB |=  (1<<2);	// S03
#define s03_off()	PORTB &= ~(1<<2);	// S03
#define s02_on()	PORTB |=  (1<<1);	// S02
#define s02_off()	PORTB &= ~(1<<1);	// S02
#define s01_on()	PORTB |=  (1<<0);	// S01
#define s01_off()	PORTB &= ~(1<<0);	// S01

// Contactors input
#define k1_readPin	bit_is_clear(PIND, 4)
#define k3_readPin 	bit_is_clear(PINC, 2)
#define Th_readPin	bit_is_clear(PINC, 3)

// Time decision variables
#define HourOn	21
#define MinOn 	30
#define HourOff	6
#define MinOff	0
//const uint8_t HourOn  = 21;
//const uint8_t MinOn   = 30;
//const uint8_t HourOff = 6;
//const uint8_t MinOff  = 0;

// PRessure
#define minPRess	49
#define Imin 		80
#define Iconst		1.30

// Global
const uint8_t sectorMax = 11;
const uint8_t timePipeB = 185;					// Time to turn down while valve does not open
volatile uint8_t flag_timePipeB = 0;
volatile uint8_t count_timePipeB = timePipeB;

// Logs
const int nLog = 12;
uint8_t loadLog[nLog];
uint8_t loadStatus[nLog];
uint8_t hourLog[nLog];
uint8_t minuteLog[nLog];

// Wake up interrupts
uint8_t flag_WDRF = 0;			// Watchdog System Reset Flag
uint8_t flag_BORF = 0;			// Brown-out Reset Flag
uint8_t flag_EXTRF = 0;			// External Reset Flag
uint8_t flag_PORF = 0;			// Power-on Reset Flag

uint8_t motorStatus = 0;
uint8_t periodState = 0;

// EEPROM memory Addresses
const uint8_t addr_stateMode    	= 0;		// 1  byte  allocated
const uint8_t addr_standBy_min  	= 1;		// 1  byte  allocated
const uint8_t addr_lastError		= 2;		// 1  byte  allocated
const uint8_t addr_timeSector   	= 10;		// 11 bytes allocated
const uint8_t addr_valveSequence	= 21;		// 11 bytes allocated

uint8_t stateMode = 0;

uint8_t flag_waitPowerOn = 0;	// Minutes before start motor after power line ocasionally down
uint8_t powerOn_min_Standy = 0;
uint8_t powerOn_min = 0;
uint8_t powerOn_sec = 0;

volatile uint16_t timeSector = 0;
uint8_t timeSectorVectorMin[11];
uint8_t flag_1s = 0;
uint8_t lastError = 0;

uint8_t valveSequence[11] = {0,0,0,0,0,0,0,0,0,0,0};
uint8_t valveStatus[11] = {0,0,0,0,0,0,0,0,0,0,0};
uint8_t sectorCurrently = 0;
uint8_t sectorChanged = 0;

uint8_t flag_BrokenPipeVerify = 0;
uint8_t flag_timeOVF = 0;
uint8_t flag_timeMatch = 0;

uint8_t flag01 = 0;
uint8_t flag02 = 0;
uint8_t flag03 = 0;
uint8_t flag04 = 0;
uint8_t flag05 = 0;
uint8_t flag_Th = 0;
uint8_t flag_frameStartBT = 0;

// Communicaton variables
char inChar, aux[3], aux2[5], sInstr[15];
uint8_t rLength;
char sInstrBluetooth[15];
uint8_t j2 = 0;
uint8_t enableTranslate_Bluetooth = 0;
uint8_t enableDecode = 0;

int PRess;
int Pdig=0;

//void init_SIM900()
//{
//	DDRH |= (1<<PH5);	// Reset pin
//	DDRH |= (1<<PH6);	// Power pin
//
//	PORTH &= ~(1<<PH5);
//	PORTH &= ~(1<<PH6);
//}
void init_valves()
{
	DDRB |= (1<<5);	// S06
	DDRB |= (1<<4);	// S05
	DDRB |= (1<<3);	// S04
	DDRB |= (1<<2); // S03
	DDRB |= (1<<1); // S02
	DDRB |= (1<<0); // S01
}
void init_contactors()
{
	DDRD |=  (1<<PD5);	// k1
	DDRD |=  (1<<PD6);	// k2
	DDRD |=  (1<<PD7);	// k3

	DDRD &= ~(1<<PD4);	// K1 NO input
	DDRC &= ~(1<<PC2);	// K3 NO input
	DDRC &= ~(1<<PC3);	// Thermal device protection
}

void init_Timer1_1Hz()
{
	// Timer 1 with 16 bit time counter. On a Fast PWM
	// TCCR1A <==	COM1A1	COM1A0	COM1B1	COM1B0	COM1C1	COM1C0	WGM11	WGM10
	TCCR1A = 0b00000010;

	// TCCR1B <==	ICNC1	ICES1	–		WGM13	WGM12	CS12	CS11	CS10
	TCCR1B = 0b00011101;	// Start timer at Fcpu/1024

	// TIMSK1 <==	–		–		ICIE1	–		OCIE1C	OCIE1B	OCIE1A	TOIE1
//	TIMSK1 |= (1 << OCIE1A);
	TIMSK1 = 0b00000010;

	ICR1 = 15624;	// To obtain 1Hz clock.
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
//	ADMUX &= ~(1<<REFS1);				// AREF, Internal Vref turned off
//	ADMUX &= ~(1<<REFS0);
	ADMUX &= ~(1<<REFS1);				// AVCC with external capacitor at AREF pin
	ADMUX |=  (1<<REFS0);
//	ADMUX |=  (1<<REFS1);				// Internal 1.1V Voltage Reference with external capacitor at AREF pin
//	ADMUX &= ~(1<<REFS0);
//	ADMUX |=  (1<<REFS0);				// Internal 2.56V reference
//	ADMUX |=  (1<<REFS1);

//	ADMUX |=  (1<<ADLAR);				// Left Adjustment. To ADCH register.
//										// Using 8 bits. Get ADCH only.
	ADMUX &= ~(1<<ADLAR);				// Right Adjustment. To ADCL register.
										// Using 10 bits
//	ADMUX &= ~(1<<MUX4);				// Select ADC1
	ADMUX &= ~(1<<MUX3);
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX1);
	ADMUX |=  (1<<MUX0);
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

double get_Pressure()
{
	/*
	Sensor details

    Thread size : G 1/4" (BSP)
    Sensor material:  Carbon steel alloy
    Working voltage: 5 VDC
    Output voltage: 0.5 to 4.5 VDC
    Working Current: <= 10 mA
    Working pressure range: 0 to  1.2 MPa
    Maxi pressure: 2.4 MPa
    Working temperature range: 0 to 100 graus C
    Accuracy: ± 1.0%
    Response time: <= 2.0 ms
    Package include: 1 pc pressure sensor
    Wires : Red---Power (+5V)  Black---Power (0V) - blue ---Pulse singal output


    4.5 V___	   922___	1.2 MPa___	 12 Bar___	 120 m.c.a.___
	  	  |				|			|			|				|
	 	  |				|			|			|				|
	 	  |				|			|			|				|
	  out_|			Pd__|		  __|			|			Pa__|
	 	  |				|			|			|				|
	 	  |				|			|			|				|
	 	  |				|			|			|				|
		 _|_		   _|_		   _|_		   _|_			   _|_
	0.5 V			103			0 MPa		0 Bar		0 m.c.a.

	(out-0.5)/(4.5-0.5) = 1024

	(out-0.0)/(5-0) = (x-0)/(1024-0)

	(Pd - 103)/(922-103) = (Pa - 0)/(120 - 0)
	Pa = 120.0*Pd/(1024.0);

	(xs - 0) = temp - (0)
	(255 - 0)  +50 - (0)

	Direct Conversion
	xs = 255*(temp+0)/51
	tempNow_XS = (uint8_t) 255.0*(tempNow+0.0)/51.0;

	Inverse Conversion
	temp = (TempMax*xs/255) - TempMin
	tempNow = (uint8_t) ((sTempMax*tempNow_XS)/255.0 - sTempMin);
    */

	uint8_t low, high;
	int Pd;
	double Pa;

	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX1);				// Select ADC0
	ADMUX &= ~(1<<MUX0);

	ADCSRA |= (1<<ADSC);				// Start conversion;
	while (bit_is_set(ADCSRA, ADSC));	// wait until conversion done;

//		Serial.println((ADCH << 8) | ADCL);
	low  = ADCL;
	high = ADCH;

	Pd = (high << 8) | low;

	Pdig = Pd;
	Pa = (120.0)*(Pd-102.4)/(921.6-102.4);
//	(Pd - 103)/(922-103) = (Pa - 0)/(120 - 0);
	return Pa;
}

float calcIrms()//uint8_t channel)//, uint8_t numberOfCycles)
{
	int i, j=0;
	uint8_t high, low;
	int divScale_count = 1;									// Não mexer!

//	ADCSRB &= ~(1<<MUX5);
//	ADMUX  &= ~(1<<MUX4);
//	ADMUX  &= ~(1<<MUX3);
	ADMUX  &= ~(1<<MUX2);
	ADMUX  &= ~(1<<MUX1);
	ADMUX  |=  (1<<MUX0);									// Select ADC1

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
	float k = 2020.0;
	float R = 310.0;

	// RMS equation
	I = (k*sqrt(V2mean))/R;

	return I;
}
//float getAirTemperature()
//{
////	int *adcSamples = NULL;
////	adcSamples = (int*)malloc(nPoints_div * sizeof(int));
//
//	byte i;
//	byte present = 0;
//	byte type_s = 0;
//	byte data[12];
//	byte addr[8];
//	float celsius;//, fahrenheit;
//
//	if ( !ds.search(addr))
//	{
////		Serial.println("No more addresses.");
////		Serial.println();
//		ds.reset_search();
//		delay(250);
//	}
//
////	Serial.print("ROM =");
////	for( i = 0; i < 8; i++)
////	{
////		Serial.write(' ');
////		Serial.print(addr[i], HEX);
////	}
//
//	if (OneWire::crc8(addr, 7) != addr[7])
//	{
////		Serial.println("CRC is not valid!");
//	}
////	Serial.println();
//
//	// the first ROM byte indicates which chip
//	switch (addr[0])
//	{
//		case 0x10:
////			Serial.println("  Chip = DS18S20");  // or old DS1820
//			type_s = 1;
//			break;
//
//		case 0x28:
////			Serial.println("  Chip = DS18B20");
//			type_s = 0;
//			break;
//
//		case 0x22:
////			Serial.println("  Chip = DS1822");
//			type_s = 0;
//			break;
//
//		default:
////			Serial.println("Device is not a DS18x20 family device.");
//			break;
//	}
//
//	ds.reset();
//	ds.select(addr);
//	ds.write(0x44, 1);        // start conversion, with parasite power on at the end
//
//	delay(1000);     // maybe 750ms is enough, maybe not
//	// we might do a ds.depower() here, but the reset will take care of it.
//
//	present = ds.reset();
//	ds.select(addr);
//	ds.write(0xBE);         // Read Scratchpad
//
////	Serial.print("  Data = ");
////	Serial.print(present, HEX);
////	Serial.print(" ");
//
//	for ( i = 0; i < 9; i++) {           // we need 9 bytes
//		data[i] = ds.read();
////		Serial.print(data[i], HEX);
////		Serial.print(" ");
//	}
//
////	Serial.print(" CRC=");
////	Serial.print(OneWire::crc8(data, 8), HEX);
////	Serial.println();
//
//	// Convert the data to actual temperature
//	// because the result is a 16 bit signed integer, it should
//	// be stored to an "int16_t" type, which is always 16 bits
//	// even when compiled on a 32 bit processor.
//	int16_t raw = (data[1] << 8) | data[0];
//
//	if (type_s)
//	{
//		raw = raw << 3; // 9 bit resolution default
//		if (data[7] == 0x10)
//		{
//			// "count remain" gives full 12 bit resolution
//			raw = (raw & 0xFFF0) + 12 - data[6];
//		}
//	}
//	else
//	{
//		byte cfg = (data[4] & 0x60);
//		// at lower res, the low bits are undefined, so let's zero them
//		if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
//		else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
//		else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
//		//// default is 12 bit resolution, 750 ms conversion time
//	}
//
//	celsius = (float)raw / 16.0;
////	fahrenheit = celsius * 1.8 + 32.0;
////	Serial.print("  Temperature = ");
////	Serial.print(celsius);
////	Serial.println(" Celsius, ");
////	Serial.print(fahrenheit);
////	Serial.println(" Fahrenheit");
//
//	return celsius;
//}
//void temperatureAnalysis()
//{
//	/*
//	The temperature interval is from 0 C to +50 C represented by one
//	8-bit variable (0-255). Using deltaT = 51 C with 255/51 = 0.2 C step.
//	It need some simple equation to make the conversion.
//
//	+50 C___		255___
//	  	  |				|
//	 	  |				|
//	 	  |				|
//	temp__|			xs__|
//	 	  |				|
//	 	  |				|
//	 	  |				|
//		 _|_		   _|_
//	0 C			 0
//
//	(xs - 0) = temp - (0)
//	(255 - 0)  +50 - (0)
//
//	Direct Conversion
//	xs = 255*(temp+0)/51
//	tempNow_XS = (uint8_t) 255.0*(tempNow+0.0)/51.0;
//
//	Inverse Conversion
//	temp = (TempMax*xs/255) - TempMin
//	tempNow = (uint8_t) ((sTempMax*tempNow_XS)/255.0 - sTempMin);
//
//	 tempRead
//	 */
//
//	const float sTempMax = 50.0;
//	const float sTempMin = 0.0;
//
//	uint8_t flag_dayRefresh = 0;
//
//	uint8_t temperature_Day[nTempDay];
//	uint8_t temperature_Max[nTempMonth];
//	uint8_t temperature_Avg[nTempMonth];
//	uint8_t temperature_Min[nTempMonth];
//
////	float tempMax, tempMin;
//	float tempMean_XS, tempMax_XS, tempMin_XS;
//	uint8_t tempNow_XS;
//
//	// Reset! (for the first time)---------------------------
//	tempNow = getAirTemperature();
//	tempNow_XS = (uint8_t) 255.0*(tempNow+sTempMin)/sTempMax;
//
//	int i;
//	for(i=0;i<nTempDay;i++)
//		temperature_Day[i] = 25;
//
//	for(i=0;i<nTempMonth;i++)
//	{
//		temperature_Max[i] = tempNow_XS;
//		temperature_Min[i] = tempNow_XS;
//	}
//
//	tempMean_XS = tempNow_XS;
//	tempMax_XS = tempNow_XS;
//	tempMin_XS = tempNow_XS;
//	// -------------------------------------------------------
//
//
//
//	// Refresh every 5 minutes;
//	float tempMean = tempNow;
//	if(flag_5min)
//	{
//		flag_5min = 0;
//
//		tempNow = getAirTemperature();
//		tempNow_XS = (uint8_t) 255.0*(tempNow+sTempMin)/sTempMax;
//
//		// in 5 min interval -------------------------------------
//		for(i=(nTempDay-1);i>0;i--)
//		{
//			temperature_Day[i] = temperature_Day[i-1];
//		}
//		temperature_Day[0] = tempNow_XS;
//
//		// mean
//		tempMean = (tempMean+tempNow)/2.0;
//	}
//
//
//
//	// Refresh once by day
//	if(flag_dayRefresh)
//	{
//		flag_dayRefresh = 0;
//
//		// at the and of day write the max and min values of day ---
//		for(i=0;i<nTempDay;i++)
//		{
//			if(tempMax_XS<temperature_Day[i])
//			{
//				tempMax_XS = temperature_Day[i];
//			}
//
//			if(tempMin_XS>temperature_Day[i])
//			{
//				tempMin_XS = temperature_Day[i];
//			}
//		}
//
//		uint8_t tempAux;
//		for(i=(nTempMonth-1);i>0;i--)
//		{
//			temperature_Max[i] = temperature_Max[i-1];
//			tempAux = eeprom_read_byte((uint8_t *)(i-1+addr_tempMax));
//			eeprom_write_byte(( uint8_t *)(i+addr_tempMin), tempAux);
//
//			temperature_Min[i] = temperature_Min[i-1];
//			tempAux = eeprom_read_byte((uint8_t *)(i-1+addr_tempMin));
//			eeprom_write_byte(( uint8_t *)(i+addr_tempMin), tempAux);
//
//			temperature_Avg[i] = temperature_Avg[i-1];
//			tempAux = eeprom_read_byte((uint8_t *)(i-1+addr_tempMean));
//			eeprom_write_byte(( uint8_t *)(i+addr_tempMean), tempAux);
//		}
//		temperature_Max[0] = tempMax_XS;
//		eeprom_write_byte(( uint8_t *)(addr_tempMax), tempMax_XS);
//
//		temperature_Min[0] = tempMin_XS;
//		eeprom_write_byte(( uint8_t *)(addr_tempMin), tempMin_XS);
//
//		temperature_Avg[0] = tempMean_XS;
//		eeprom_write_byte(( uint8_t *)(addr_tempMean), tempMean_XS);
//	}
//
//	// Read Temperature
//
//}
//
//int soilSensorRead()
//{
//	uint8_t high, low;
//
//	ADMUX |=  (1<<MUX1);							// Select ADC2
//	ADMUX &= ~(1<<MUX0);
//
//	ADCSRA |= (1<<ADSC);				// Start conversion;
//	while (bit_is_set(ADCSRA, ADSC));	// wait until conversion done;
//
//	low  = ADCL;
//	high = ADCH;
//
//	soilHumidity = (high << 8) | low;
//
//	return soilHumidity;
//}

void logMessage(uint8_t loadType, uint8_t newStatus)
{
	int i;

	for(i=(nLog-1);i>0;i--)
	{
		hourLog[i] = hourLog[i-1];
		minuteLog[i] = minuteLog[i-1];

		loadLog[i] = loadLog[i-1];
		loadStatus[i] = loadStatus[i-1];
	}
	loadLog[0] = loadType;
	loadStatus[0] = newStatus;

	hourLog[0] = tm.Hour;
	minuteLog[0] = tm.Minute;
}

void motor_start()
{
	k1_on();
	k3_on();
	wdt_reset();
	_delay_ms(5000);
	wdt_reset();

	k3_off();
	uint32_t count = 0;
	while(k3_readPin)
	{
		count++;
		if(count>=50000)
		{
			k1_off();
			k2_off();
			k3_off();
			return;
		}
	}
	_delay_ms(100);
	k2_on();

	// carga, estado;
	logMessage(12, 1);
//	Serial.print("Count = ");
//	Serial.println(count);
}
void motor_stop()
{
	k1_off();
	k2_off();
	logMessage(12, 0);
//	motorStatus = 0;
}
void valveInstr(uint8_t sectorPrivate, uint8_t status, uint8_t makeLog)
{
	// These flags comes here because when you change sector the pressure go down.
	// With this, you disable the pressure turn system down verify.
	flag_BrokenPipeVerify = 0;
	flag_timePipeB = 0;
	count_timePipeB = timePipeB;

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

//		case 12:
//			if(status)
//			{
//				f01_on();
//				valveStatus[11] = 1;
//			}
//			else
//			{
//				f01_off();
//				valveStatus[11] = 0;
//			}
//			break;
//
//		case 13:
//			if(status)
//			{
//				f02_on();
//				valveStatus[12] = 1;
//			}
//			else
//			{
//				f02_off();
//				valveStatus[12] = 1;
//			}
//			break;
	}

	if(makeLog)
	{
		logMessage(sectorPrivate, status);
	}
}
void turnAll_OFF()
{
	motor_stop();
	int i;
	for(i=1;i<12;i++)
	{
		valveInstr(i, 0, 0);
	}

	flag_timeMatch = 0;
	stateMode = 0;	// Manual
	eeprom_write_byte(( uint8_t *)(addr_stateMode), stateMode);

	timeSector = 0;
	sectorCurrently = 0;
}

void summary_Print(uint8_t opt)
{
	char buffer[50];
	char buffer2[8];
	int i;

	switch (opt)
	{
		case 0:
			sprintf(buffer,"%.2d:%.2d:%.2d", tm.Hour, tm.Minute, tm.Second);
			Serial.println(buffer);
			sprintf(buffer," %.2d/%.2d/%d ", tm.Day, tm.Month, tmYearToCalendar(tm.Year));
			Serial.println(buffer);
			sprintf(buffer,"m:%d p:%d tm:%d", stateMode, periodState, flag_timeMatch);
			Serial.println(buffer);

			sprintf(buffer," d:%d", (day()-1));
			Serial.println(buffer);
			sprintf(buffer,"%.2d:%.2d:%.2d", hour(), minute(), second());
			Serial.println(buffer);

//			Serial.print("Uptime: ");
//			sprintf(buffer,"%.2d:%.2d:%.2d", hour(), minute(), second());
//			Serial.println(buffer);
//			sprintf(buffer,"d:%d, m:%d, y:%d", (day()-1), (month()-1), (year()-1970));
//			Serial.println(buffer);
			break;

		case 1:
			for(i=0;i<11;i++)
			{
				sprintf(buffer2,"t%d:%d, ",i+1, timeSectorVectorMin[i]);
				Serial.print(buffer2);
			}
			Serial.println("");
//			sprintf(buffer,"t1:%d, t2:%d, t3:%d", timeSectorVectorMin[0], timeSectorVectorMin[1], timeSectorVectorMin[2]);
//			Serial.println(buffer);
//			sprintf(buffer,"t4:%d, t5:%d, t6:%d", timeSectorVectorMin[3], timeSectorVectorMin[4], timeSectorVectorMin[5]);
//			Serial.println(buffer);
//			sprintf(buffer,"t7:%d, t8:%d, t9:%d", timeSectorVectorMin[6], timeSectorVectorMin[7], timeSectorVectorMin[8]);
//			Serial.println(buffer);
//			sprintf(buffer,"t10:%d, t11:%d", timeSectorVectorMin[9],timeSectorVectorMin[10]);
//			Serial.println(buffer);
			break;

		case 2:
			for(i=0;i<11;i++)
			{
				sprintf(buffer2,"s%d:%d, ",i+1, valveSequence[i]);
				Serial.print(buffer2);
			}
			Serial.println("");

//			Serial.println(buffer);
//			sprintf(buffer,"s5:%d, s6:%d, s7:%d, s8:%d", valveStatus[4], valveStatus[5], valveStatus[6], valveStatus[7]);
//			Serial.println(buffer);
//			sprintf(buffer,"s9:%d, s10:%d, s11:%d", valveStatus[8], valveStatus[9],valveStatus[10]);
//			Serial.println(buffer);
			break;

		case 3:
			sprintf(buffer,"M:%d Pr:%d fTh:%d Rth:%d", motorStatus, PRess, flag_Th, Th_readPin);
			Serial.println(buffer);
			break;

		case 4:
			sprintf(buffer,"s%d c%d El:%d", sectorCurrently, sectorChanged, timeSector);
			Serial.println(buffer);
			//			sprintf(buffer,"Setor Atual: %d, Tempo restante: %d", stateSector, timeSector);
			break;

		case 5:
			sprintf(buffer,"W%d B%d E%d P%d", flag_WDRF, flag_BORF, flag_EXTRF, flag_PORF);
			Serial.println(buffer);
			break;

		case 6:
			sprintf(buffer,"LE%d K%d Th%d K%d", lastError, k1_readPin, Th_readPin, k3_readPin);
			Serial.println(buffer);
			break;

		case 7:
			for(i=0; i<nLog; i++)
			{
				sprintf(buffer2,"%.2d:d%d:L%.2d-%.2d:%.2d;",i, loadStatus[i], loadLog[i], hourLog[i], minuteLog[i]);
				Serial.print(buffer2);
				Serial.println("");
			}
			break;

		case 8:
			Serial.print("I");
			Serial.print(1000*calcIrms());
			Serial.println("");
			break;

		case 21:
			Serial.print("0x0");
			Serial.println(1,HEX);
			break;

		case 22:
			Serial.print("0x0");
			Serial.println(2,HEX);
			break;

		case 23:
			Serial.print("0x0");
			Serial.println(3,HEX);
			break;

		case 24:
			Serial.print("0x0");
			Serial.println(4,HEX);
			break;

		case 25:
			Serial.print("0x0");
			Serial.println(5,HEX);
			break;

		case 26:
			Serial.print("0x0");
			Serial.println(6,HEX);
			break;

		case 27:
			Serial.print("0x0");
			Serial.println(7,HEX);
			break;

		case 28:
			Serial.print("0x0");
			Serial.println(8,HEX);
			break;

		case 29:
			Serial.print("0x0");
			Serial.println(9,HEX);
			break;

		case 30:
			Serial.print("Done");
			break;

		default:
			break;
	}
	memset(buffer,0,sizeof(char));
}

uint16_t timeSectorMemory(uint8_t sectorPrivate)
{
	return 60*timeSectorVectorMin[sectorPrivate-1];
}

// Alarms
uint8_t valveActVerify(uint8_t sectorPrivate)
{
	float I0a=0.0, I0b=0.0, I0c=0.0, Ia=0.0, Ib=0.0, Ic=0.0;
	int I0m=0, Im=0;

	uint8_t delayTime = 10;
	uint8_t valveOk = 0;

	if(valveSequence[sectorPrivate-1])
	{
		wdt_reset();
		I0a = calcIrms();	// Read currently current;
		_delay_ms(delayTime);
		I0b = calcIrms();	// Read currently current;
		_delay_ms(delayTime);
		I0c = calcIrms();	// Read currently current;
		_delay_ms(delayTime);

		I0m = (int) (1.05*1000.0*(I0a+I0b+I0c)/3.0);
//		Serial.print("I0m: ");
//		Serial.println(I0m);

		valveInstr(sectorPrivate, 1, 0);
		_delay_ms(5*delayTime);

		Ia = calcIrms();	// Read currently current;
		_delay_ms(delayTime);
		Ib = calcIrms();	// Read currently current;
		_delay_ms(delayTime);
		Ic = calcIrms();	// Read currently current;
		_delay_ms(delayTime);

		Im = (int) (1000.0*(Ia+Ib+Ic)/3.0);
//		Serial.print("Im: ");
//		Serial.println(Im);

		valveInstr(sectorPrivate, 0, 0);

		if(Im > ((int) I0m*Iconst))
		{
			valveOk = 1;
		}
		else
		{
			valveOk = 0;
		}
	}
	else
	{
		valveOk = 0;
	}

	return valveOk;
}
void valveOpenVerify()
{
	wdt_reset();
	if(motorStatus)
	{
		uint8_t status = 0,i;

		for(i=0;i<=11;i++)
			status |= valveStatus[i];

		if(!status)
		{
			turnAll_OFF();
			summary_Print(21);
			lastError = 0x01;
			eeprom_write_byte((uint8_t *)(addr_lastError), lastError);
//			enableSIM900_Send = 1;
		}
	}

	wdt_reset();
	if(motorStatus)
	{
		float Ia=0.0, Ib=0.0, Ic=0.0;
		int Im=0;

		Ia = calcIrms();	// Read currently current;
		_delay_ms(100);
		Ib = calcIrms();	// Read currently current;
		_delay_ms(100);
		Ic = calcIrms();	// Read currently current;
		_delay_ms(100);
		Im = (int) (1000.0*(Ia+Ib+Ic)/3.0);

//		Serial.print("Im = ");
//		Serial.println(Im);

		if(Im<Imin)
		{
			stateMode = 0; //manual;

//			sprintf(buffer,"Desligado no s[%.2d]!",stateSector);
//			SIM900_sendSMS(buffer);
			turnAll_OFF();
			summary_Print(23);
//			Serial.println("Im sensor Down!");

			lastError = 0x03;
			eeprom_write_byte((uint8_t *)(addr_lastError), lastError);
//			enableSIM900_Send = 1;
		}
	}
}
void thermalSafe()
{
	if(motorStatus)
	{
		if(Th_readPin)
		{
			summary_Print(3);
			uint16_t countThermal = 50000;
//			Serial.println("Thermal Start");
			while(Th_readPin && countThermal)
			{
				countThermal--;
			}
//			Serial.println("Thermal Stop");
			if(!countThermal)
			{
				flag_Th = 1;
				turnAll_OFF();
				summary_Print(22);
//				Serial.println("Thermal Safe executed!");
				lastError = 0x02;
				eeprom_write_byte((uint8_t *)(addr_lastError), lastError);
//				enableSIM900_Send = 1;

//				strcpy(buffer,"Rele Sobrecarga!");
//				SIM900_sendSMS(buffer);
//				eeprom_write_byte(( uint8_t *)(addr_stateMode), stateMode);
			}
			else
				flag_Th = 0;
		}
		else
		{
			flag_Th = 0;
		}
	}
}
void pipeBrokenSafe()
{
	if(motorStatus)
	{
		if(flag_BrokenPipeVerify)
		{
			if(PRess<minPRess)
			{
				turnAll_OFF();
				summary_Print(24);
//				Serial.println("PRessure Down!");
				lastError = 0x04;
				eeprom_write_byte((uint8_t *)(addr_lastError), lastError);
//				enableSIM900_Send = 1;
			}
		}
	}

}
void pSafe()
{
	if(motorStatus)
	{
		if(PRess>=70)
		{
			turnAll_OFF();
			summary_Print(25);
//			Serial.println("PRessure HIGH!");
			lastError = 0x05;
			eeprom_write_byte((uint8_t *)(addr_lastError), lastError);
//			enableSIM900_Send = 1;
		}
	}
}

void process_Working()
{
	if(flag_timeOVF)
	{
		// Local
		uint8_t flag_sectorActiveFound = 0;
		uint8_t sectorNext = sectorCurrently + 1;

		// identifica o proximo setor da sequencia e se está ativo.
		if(sectorNext < sectorMax)
		{
			do{
				if(valveActVerify(sectorNext))
				{
					flag_sectorActiveFound = 1;

					valveInstr(sectorCurrently, 0, 1);
					valveInstr(sectorNext, 1, 1);
					sectorCurrently = sectorNext;
					timeSector = timeSectorMemory(sectorCurrently);
					flag_timeOVF = 0;

					if(!k1_readPin)
					{
						motor_start();
					}
				}
				else
				{
					// Incrementa e procura o proximo.
					sectorNext++;
				}
			}while(!flag_sectorActiveFound && (sectorNext < sectorMax));
		}

		if(sectorNext >= sectorMax)
		{
			turnAll_OFF();
			Serial.println("Done!");
			lastError = 0x10;
			_delay_ms(200);
//			eeprom_write_byte()
		}
	}
}

//void process_Working()
//{
//	uint16_t timeAux = 0;
//
//	// 1- Check valve working before start motor
//	if(!motorStatus)
//	{
//		do{
//			stateSector = verifyNextValve(stateSector);
//			timeAux = timeSectorMemory(stateSector);
//
//			if(stateSector==0) // In the case verifyNextVale() returns 0 with all broken valves;
//				timeAux = 1;
//		}while(timeAux==0);
//
//		if(stateSector>0)
//		{
//			valveInstr(stateSector,1);
//			motor_start();
//
////			_delay_ms(200);
////			GLCD.Init();
//
//			flag_sector = 0;
//			flag_timeOVF = 0;
//		}
//		else
//		{
////			Serial.println("Out of order!");
//			stateMode = 0; //manual;
//		}
//	}
//
//	// 2- Verifica se pode trocar de setor
//	if(!flag_sector)
//	{
//		flag_sector = 1;
//
//		uint8_t i=0;
//		for(i=1;i<stateSector;i++)		// Desliga todos setores anteriores.
//		{
//			valveInstr(i,0);
//		}
//		valveInstr(stateSector,1);
//
//		if(stateMode == 2) // Automatic
//			timeSector = timeSectorSet;
//		else
//			timeSector = timeSectorMemory(stateSector);
//
//		summary_Print(4);
////		sprintf(buffer,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d \n Sector[%.2d]: ON!",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), stateSector);
////		Serial.println(buffer);
////		SIM900_sendSMS(buffer);
//	}
//
//	// Encerra o processo;
//	if(flag_timeOVF)
//	{
//		flag_sector = 0;
//		flag_timeOVF = 0;
//		timeSector = 10;		// Para nao gerar interrupcao e voltar aqui de novo pulando setor
//
//		stateSector = verifyNextValve(stateSector+1);
//	}
//}
void process_Programmed()
{
	if(((tm.Hour == HourOn) && (tm.Minute == MinOn)))
	{
		flag_timeMatch = 1;
	}

	if(((tm.Hour == HourOff) && (tm.Minute == MinOff)) || !periodState)
	{
		flag_timeMatch = 0;
	}

	if(flag_timeMatch)
	{
		process_Working();
	}
	else
	{
		// Over time detected!!!!
		if(motorStatus)
		{
			turnAll_OFF();
			summary_Print(27);
			lastError = 0x07;
			eeprom_write_byte((uint8_t *)(addr_lastError), lastError);
//			enableSIM900_Send = 1;
		}
	}
}
void process_Mode()
{
	switch(stateMode)
	{
		case 0:	// Manual Process
//			process_Manual();
			break;

		case 1: // nightMode
			process_Programmed();
			break;

//		case 2:
//			process_Working();
//			break;

//		case 3: //onlyOneSector. Turn on at night only one sector
//			process_Programmed();
//			break;

//		case 4: //valveTesting
			// Automatic Process
//			process_valveTest();
//			break;
	}
}

void periodVerify0()
{
	if (((tm.Hour == HourOn) && (tm.Minute >= MinOn)) || (tm.Hour > HourOn)
			|| (tm.Hour < HourOff)
			|| ((tm.Hour == HourOff) && (tm.Minute < MinOff)))
	{
		periodState = 1; //greenTime;
		flag04 = 1;
		flag05 = 0;
	}

	if (((tm.Hour == HourOff) && (tm.Minute >= MinOff))
			|| ((tm.Hour > HourOff) && (tm.Hour < HourOn))
			|| ((tm.Hour == HourOn) && (tm.Minute < MinOn)))
	{
		periodState = 0; // redTime;
		flag04 = 0;
		flag05 = 1;
	}
}

void refreshVariables()
{
//	wdt_reset();
//	temperature = getAirTemperature();
//	Serial.println(temperature);
//	Soil Humidity Sensor
//	sprintf(buffer,"Hum. Solo: %.4d",soilSensorRead());
//	GLCD.CursorTo(0,7);
//	GLCD.print(buffer);

	motorStatus = k1_readPin;

	if(motorStatus)
		thermalSafe();

	if(flag_timePipeB)
	{
		flag_timePipeB = 0;
		count_timePipeB = timePipeB;

		if(motorStatus)
		{
			flag_BrokenPipeVerify = 1;
		}
	}

//	if(flag_30s)
//	{
//		flag_30s = 0;
//		SIM900_checkAlive();
//	}

	if (flag_1s)
	{
		flag_1s = 0;

		PRess = get_Pressure();

		if(motorStatus)
		{
//			pSafe();				// Verify maximum pressure;
//			pipeBrokenSafe();		// Verify pipe low pressure;
			valveOpenVerify();		// AND op with all output valves;
		}

		RTC.read(tm);				// RTC fetch;
		periodVerify0();			// Period time refresh;
	}

//	if(flag_5min)
//	{
//		flag_5min = 0;
//		count_5min = 300;
//
//		tempNow = getAirTemperature();
//	}
}
void refreshTimeSectors()
{
	int i;
	for(i=0;i<11;i++)
		timeSectorVectorMin[i] = eeprom_read_byte((uint8_t *)(i+addr_timeSector));

	for(i=0;i<11;i++)
		valveSequence[i] = eeprom_read_byte((uint8_t *)(i+addr_valveSequence));
}
//void refreshCelPhoneNumber()
//{
//	int i;
//	uint8_t flag_Error01 = 0;
//	char buffer[20];
//	for(i=0;i<11;i++)
//	{
//		// Verify is there is any digit bigger the 9
//		if(eeprom_read_byte((uint8_t *)(i+addr_celNumber))>9)
//		{
//			flag_Error01 = 1;
////			eeprom_write_byte(( uint8_t *)(i+23), 9);
//		}
//	}
//
//	if(!flag_Error01)
//	{
//		for(i=0;i<11;i++)
//			celPhoneNumber[i] = eeprom_read_byte((uint8_t *)(i+addr_celNumber));
//
//		sprintf(celPhoneNumber_str,"%d%d%d%d%d%d%d%d%d%d%d",celPhoneNumber[0],celPhoneNumber[1],celPhoneNumber[2],celPhoneNumber[3],celPhoneNumber[4],celPhoneNumber[5],celPhoneNumber[6],celPhoneNumber[7],celPhoneNumber[8],celPhoneNumber[9],celPhoneNumber[10]);
//	}
//	else
//	{
//		strcpy(celPhoneNumber_str,"27988081875");
//		sprintf(buffer,"Restore Cel N. Error!");
////		SIM900_sendSMS(buffer);
//		Serial.println("Restore Cel N. Error!");
//	}
//}
void refreshStoredData()
{
	stateMode = eeprom_read_byte((uint8_t *)(addr_stateMode));
	lastError = eeprom_read_byte((uint8_t *)(addr_lastError));

	powerOn_min_Standy = eeprom_read_byte((uint8_t *)(addr_standBy_min));
	powerOn_min = powerOn_min_Standy;

//	uint8_t lbyte, hbyte;
//	hbyte = eeprom_read_byte((uint8_t *)(addr_LevelRef+1));
//	lbyte = eeprom_read_byte((uint8_t *)(addr_LevelRef));

//	reservoirLevelRef = ((hbyte << 8) | lbyte);

}

void comm_Bluetooth()
{
	// Rx - Always listening
//	uint8_t j2 =0;
	while((Serial.available()>0))	// Reading from serial
	{
		inChar = Serial.read();

		if(inChar=='$')
		{
			j2 = 0;
			flag_frameStartBT = 1;
//			Serial.println("Frame Start!");
		}

		if(flag_frameStartBT)
			sInstrBluetooth[j2] = inChar;

//		sprintf(buffer,"J= %d",j2);
//		Serial.println(buffer);

		j2++;

		if(j2>=sizeof(sInstrBluetooth))
		{
			memset(sInstrBluetooth,0,sizeof(sInstrBluetooth));
			j2=0;
			summary_Print(28);
		}

		if(inChar==';')
		{
//			Serial.println("Encontrou ; !");
			if(flag_frameStartBT)
			{
//				Serial.println("Frame Stop!");
				flag_frameStartBT = 0;
				rLength = j2;
				j2 = 0;
				enableTranslate_Bluetooth = 1;
			}
		}
	}
//	flag_frameStart = 0;

	if(enableTranslate_Bluetooth)
	{
//		Serial.println("enableTranslate_Bluetooth");
		enableTranslate_Bluetooth = 0;

		char *pi0, *pf0;
		pi0 = strchr(sInstrBluetooth,'$');
		pf0 = strchr(sInstrBluetooth,';');

		if(pi0!=NULL)
		{
			uint8_t l0=0;
			l0 = pf0 - pi0;

			int i;
			for(i=1;i<=l0;i++)
			{
				sInstr[i-1] = pi0[i];
//				Serial.write(sInstr[i-1]);
			}
//			memset(sInstrBluetooth,0,sizeof(sInstrBluetooth));
	//		Serial.println(sInstr);

			enableDecode = 1;
		}
		else
		{
			summary_Print(29);
			Serial.write(pi0[0]);
			Serial.write(pf0[0]);
		}
		memset(sInstrBluetooth,0,sizeof(sInstrBluetooth));
	}
}
//void comm_SIM900()
//{
//	// Rx - Always listening
////	uint8_t j1 =0;
//	while((Serial.available()>0))	// Reading from serial
//	{
//		inChar = Serial.read();
//		Serial.write(inChar);
//
//		if(inChar=='$')
//		{
//			j1 = 0;
//			memset(sInstrSIM900,0,sizeof(sInstrSIM900));
//			flag_frameStartSIM900 = 1;
////			Serial.println("Frame Start!");
//		}
//
//		if(flag_frameStartSIM900)
//		{
//			sInstrSIM900[j1] = inChar;
////			Serial.write(sInstrSIM900[j1]);
//		}
//
//		j1++;
//
//		if(j1>=sizeof(sInstrSIM900))
//		{
//			memset(sInstrSIM900,0,sizeof(sInstrSIM900));
//			j1=0;
//			Serial.println("ZEROU! sIntr SIM900 Buffer!");
//		}
//
//		if(inChar==';')
//		{
//			if(flag_frameStartSIM900)
//			{
////				Serial.println("Frame Stop!");
//				flag_frameStartSIM900 = 0;
//				rLengthSIM900 = j1;
////				j1 = 0;
//				enableTranslate_SIM900 = 1;
//			}
//		}
//
//
//		// Variables for check if SIM900 is alive.
//		flag_SIM900_checkAlive = 0;
//		count_SIM900_timeout = 0;
//		count_30s = 0;
//
////		if(flag_SIM900_checkAlive)
////		{
////			if(inChar=='K')
////			{
////				enableSIM900_checkAliveCompare = 1;
////				flag_SIM900_checkAlive = 0;
////				count_SIM900_timeout = 0;
////				j1 = 0;
////			}
////		}
//	}
//
//	// PC to SIM900
//	while(Serial.available() > 0)
//		Serial.write(Serial.read());
//
//	if(enableTranslate_SIM900)
//	{
//		j1 = 0;
//		enableTranslate_SIM900 = 0;
//
//		char *pi1, *pf1;
//		pi1 = strchr(sInstrSIM900,'$');
//		pf1 = strchr(sInstrSIM900,';');
//
//		if(pi1!=NULL)
//		{
////			Serial.println("pi!=NULL");
//			uint8_t l1=0;
//			l1 = pf1 - pi1;
//
//			int i;
//			for(i=1;i<=l1;i++)
//			{
//				sInstr[i-1] = pi1[i];
////				Serial.write(sInstr[i-1]);
//			}
//			memset(sInstrSIM900,0,sizeof(sInstrSIM900));
//			Serial.println(sInstr);
//
//			enableDecode = 1;
//			enableSIM900_Send = 1;
//		}
//		else
//		{
//			Serial.println("Error 404!");
//			Serial.write(pi1[0]);
//			Serial.write(pf1[0]);
//		}
//	}
//
//	// Special Functions  CHECK ALIVE!
////	if(flag_SIM900_checkAlive)
////	{
////		if(count_SIM900_timeout > 5)
////		{
////			flag_SIM900_checkAlive = 0;
////			count_SIM900_timeout = 0;
////			Serial.println("SIM900 Check Alive TIMEOUT!");
////
////			SIM900_power();
////			if((minute()*60 + second())<90)
////			{
////				sprintf(buffer,"- Vassal Controller Started! -");
////				SIM900_sendSMS(buffer);
////			}
////		}
////	}
//
////	if(enableSIM900_checkAliveCompare)
////	{
////		enableSIM900_checkAliveCompare = 0;
////		j1 = 0;
////
////		char *p;
////		p = strchr(sInstrSIM900,'O');
////
////		if(p[0] == 'O' && p[1] == 'K')
////		{
////			flag_SIM900_died = 0;
////			Serial.println("Alive!");
////		}
////		else
////		{
////			Serial.println("Is DEAD??");
//////			flag_SIM900_died = 1;
////		}
////	}
//}
void comm_SerialPC()
{

}
void comm_SIM900_SerialPC()
{
	// SIM900 to PC
	while(Serial.available() > 0)
		Serial.write(Serial.read());

	// PC to SIM900
	while(Serial.available() > 0)
		Serial.write(Serial.read());
}
void comm_SIM900_Bluetooth()
{
	while(Serial.available() > 0)
		Serial.write(Serial.read());

	while(Serial.available() > 0)
		Serial.write(Serial.read());
}
void comm_SerialPC_BluetoothMOD()
{
	while(Serial.available() > 0)
		Serial.write(Serial.read());

	while(Serial.available() > 0)
		Serial.write(Serial.read());
}

void handleMessage()
{
/*
Comandos do VASSALO

Ínicio do comando sempre com dólar $
Fim do comando sempre com ponto vírgula ;

$0X;				Verificar detalhes - Detalhes simples (tempo).
	$00;			- Mostra o tempo ajustado de todos setores.
		$00:0;		- flag_sendContinuously = 0
		$00:1;		- flag_sendContinuously = 1
	$01;			- Mostra a relação de quais válvulas estão ligadas ou desligadas;
	$02;			- Número do telefone que manda o SMS de retorno;
	$03;			- Variáveis do motor;
	$04;			- Verifica a temperatura instantânea do ambiente;
		$04:04;		- Verifica a temperatura média dos últimos 04 dias.
	$05;			- Motivo da reinicialização;
	$06;			- Mostra setor atual, tempo restante e período de operação;
	$07;			- Liga/Desliga o SIM900;
	$08;			- Reseta SIM900;
	$09;			- Reinicia o sistema.

	$1HHMMSS;		Ajusta o horário do sistema;
	$1123040;		Ajustar a hora para 12:30:40

$2DDMMAAAA;  		Ajusta a data do sistema no formato dia/mês/ano(4 dígitos);
	$201042014;	Ajusta a data para 01 de abril de 2014;

$3X;			Acionamento do motor;
	$31;		liga (CUIDADO! Verifique se há válvula aberta antes de acionar o motor!);
	$30;		desliga;

$4sNN:V;		acionamento das válvulas sem verificação (Não é seguro!);
$4f03:1;		- Liga o setor 3;
	$4f10:0;		- Desliga o setor 4;
	$4f12:1;		- Liga a válvula da fertirriação que enche a caixa;
	$4f13:0;		- Desliga a válvula da fertirigação que esvazia a caixa;

Função 4 com “s”: Testa se o setor 2 está funcionando, caso esteja, mantém o 02 ligado e desliga o setor anterior;

$4sNN:V;		Modo seguro de acionamento (somente para válvulas dos piquetes);
	$4s02:1;	- Liga o setor 2 e desliga setor anterior;
	$4s03:1;	- Liga o setor 3 e desliga setor anterior;
	$4s03:0;	- Desliga o setor 3 se estiver ligado;

$5tNN:MM;		Coloca o tempo em minutos do determinado setor (2 dígitos);
	$5t02:09;		- ajusta para 09 minutos o tempo do setor 02;
	$5t11:54;		- ajusta para 54 minutos o tempo do setor 11;
	$5t09:00;		- zera o setor 9 não deixando ligar à noite;

$6X;		Modo de funcionamento
	$60; 		- Coloca no modo manual (desligado). DESLIGA TODAS AS CARGAS!;
	$61;		- Programa para ligar às 21:30 horas do mesmo dia.
	$62:s01:23;	- Liga a noite somente o setor 1 durante 23 min.
	$63:06;		- Executa automático 1x por 6 minutos cada setor;

	$69:s03;	- Testa o setor 3 se está funcionando e retorna SMS;

$727988081875;		Troca número de telefone

$8;				Reinicializa o display GLCD do painel;
*/

	// Tx - Transmitter
	if(enableDecode)
	{
		enableDecode = 0;

//		int i;
//		for(i=0;i<rLength;i++)
//		{
//			Serial.println(sInstr[i]);
//		}
//		for(i=0;i<rLength;i++)
//		{
//			Serial.println(sInstr[i],HEX);
//		}

		// Getting the opcode
		aux[0] = '0';
		aux[1] = sInstr[0];
		aux[2] = '\0';
		uint8_t opcode = (uint8_t) atoi(aux);

		switch (opcode)
		{
// --------------------------------------------------------------------------------------
			case 0:		// Check status
			{
				aux[0] = '0';
				aux[1] = sInstr[1];
				aux[2] = '\0';
				uint8_t statusCommand = (uint8_t) atoi(aux);

				if(sInstr[2] == ';')
				{
					switch (statusCommand)
					{
//						case 7:
//							SIM900_power();
//							Serial.println("SIM900 Power!");
//						break;

//						case 8:	// Reset SIM900
//							SIM900_reset();
//							flag_SIM900_checkAlive = 0;
//							flag_SIM900_died = 0;
//							count_30s = 0;
//							break;

						case 9:	// Reset system
//								Serial.println("Rebooting system...");
								wdt_enable(WDTO_15MS);
								_delay_ms(100);
							break;

						default:
							summary_Print(statusCommand);
//							flag_sendContinuously = !flag_sendContinuously;
							break;
					}
				}
			}
			break;

// --------------------------------------------------------------------------------------
			case 1:		// Set-up clock
			{
				// Getting the parameters
				aux[0] = sInstr[1];
				aux[1] = sInstr[2];
				aux[2] = '\0';
				tm.Hour = (uint8_t) atoi(aux);

				aux[0] = sInstr[3];
				aux[1] = sInstr[4];
				aux[2] = '\0';
				tm.Minute = (uint8_t) atoi(aux);

				aux[0] = sInstr[5];
				aux[1] = sInstr[6];
				aux[2] = '\0';
				tm.Second = (uint8_t) atoi(aux);

				RTC.write(tm);

				summary_Print(0);
			}
				break;

// -----------------------------------------------------------------
			case 2:		// Set-up date

				// Getting the parameters
				aux[0] = sInstr[1];
				aux[1] = sInstr[2];
				aux[2] = '\0';
				tm.Day = (uint8_t) atoi(aux);

				aux[0] = sInstr[3];
				aux[1] = sInstr[4];
				aux[2] = '\0';
				tm.Month = (uint8_t) atoi(aux);

				char aux2[5];
				aux2[0] = sInstr[5];
				aux2[1] = sInstr[6];
				aux2[2] = sInstr[7];
				aux2[3] = sInstr[8];
				aux2[4] = '\0';
				tm.Year = (uint8_t) (atoi(aux2)-1970);

				RTC.write(tm);

				summary_Print(0);

				break;

// -----------------------------------------------------------------
			case 3:		// Set motor ON/OFF

				uint8_t motorCommand;

				aux[0] = '0';
				aux[1] = sInstr[1];
				aux[2] = '\0';
				motorCommand = (uint8_t) atoi(aux);

				if (motorCommand&&(!motorStatus))
				{
					motor_start();
				}
				else
					motor_stop();

				summary_Print(1);

				break;

// -----------------------------------------------------------------
			case 4:	// ON OFF sectors
				if(sInstr[1] == 'f')
				{
					aux[0] = sInstr[2];
					aux[1] = sInstr[3];
					aux[2] = '\0';
					uint8_t sector = (uint8_t) atoi(aux);

					// sInstr[4] == :
					aux[0] = '0';
					aux[1] = sInstr[5];
					aux[2] = '\0';
					uint8_t sectorCommand = (uint8_t) atoi(aux);

					valveInstr(sector, sectorCommand, 1);

					summary_Print(4);
				}

				if(sInstr[1] == 's')
				{
					aux[0] = sInstr[2];				// Get sector number
					aux[1] = sInstr[3];
					aux[2] = '\0';
					uint8_t sector = (uint8_t) atoi(aux);

					aux[0] = '0';
					aux[1] = sInstr[5];				// Get status, 1-ON or 0-OFF
					aux[2] = '\0';
					uint8_t sectorCommand = (uint8_t) atoi(aux);

//					if(enableSIM900_Send)
//					{
//						wdt_reset();
//						_delay_ms(5000);
//						wdt_reset();
//					}

					if(sectorCommand)
					{
						if(valveActVerify(sector))
						{
							if(!sectorCurrently)
							{
								valveInstr(sectorCurrently, 0, 1);
							}

							valveInstr(sector, sectorCommand, 1);
							sectorChanged = 1;
							sectorCurrently = sector;
						}
						else
						{
							sectorChanged = 0;
						}
					}
					else
					{
						valveInstr(sector, sectorCommand, 1);
						sectorChanged = 1;

						if(sector == sectorCurrently)
						{
							sectorCurrently = 0;
						}
					}
					summary_Print(4);

//					if(sectorChanged)
//					{
//						if(sectorCommand)
//						{

//						}
//						else
//						{
//							sectorCurrently = 0;
//						}
//					}
//						valveInstr(stateSector, 0);
//						stateSector = sector;
//						valveInstr(sectorCurrently, 0);
				}
				break;

// -----------------------------------------------------------------
			case 5:
//				5t01:23;
				if(sInstr[1] == 't')
				{
					aux[0] = sInstr[2];
					aux[1] = sInstr[3];
					aux[2] = '\0';
					uint8_t sector = (uint8_t) atoi(aux);

					// sInstr[4] == :
					uint8_t sectorTimeChange;
					aux[0] = sInstr[5];
					aux[1] = sInstr[6];
					aux[2] = '\0';
					sectorTimeChange = (uint8_t) atoi(aux);

					if(!sector)
					{
						int i;
						for(i=0;i<sectorMax;i++)
						{
							eeprom_write_byte((uint8_t *)(i+addr_timeSector), sectorTimeChange);
						}
					}
					else
					{
						eeprom_write_byte(( uint8_t *)(sector-1+addr_timeSector), sectorTimeChange);
					}
					refreshTimeSectors();
					summary_Print(1);
				}
				//	5s02:0;
				if(sInstr[1] == 's')
				{
					aux[0] = sInstr[2];
					aux[1] = sInstr[3];
					aux[2] = '\0';
					uint8_t sector = (uint8_t) atoi(aux);

					// sInstr[4] == :
					aux[0] = '0';
					aux[1] = sInstr[5];
					aux[2] = '\0';
					uint8_t sectorSequenceChange = (uint8_t) atoi(aux);

					if(!sector)
					{
						int i;
						for(i=0;i<sectorMax;i++)
						{
							eeprom_write_byte((uint8_t *)(i+addr_valveSequence), sectorSequenceChange);
						}
					}
					else
					{
						eeprom_write_byte((uint8_t *)(sector-1+addr_valveSequence), sectorSequenceChange);
					}
					refreshTimeSectors();
					summary_Print(2);
				}

				break;

// -----------------------------------------------------------------
			case 6:
			{
				// 6x;
				// 63:sxx;
				aux[0] = '0';
				aux[1] = sInstr[1];
				aux[2] = '\0';
				uint8_t setCommand = (uint8_t) atoi(aux);

				switch (setCommand)
				{
					case 0:
						turnAll_OFF();
						break;

					case 1:
						stateMode = 1; //nightMode;
						eeprom_write_byte(( uint8_t *)(addr_stateMode), stateMode);
						break;

//					case 2:
//						// Turn on just one sector at night.
//						//	62:s01:30;
//						if((sInstr[2] == ':')&&(sInstr[3] == 's'))
//						{
//							aux[0] = sInstr[4];
//							aux[1] = sInstr[5];
//							aux[2] = '\0';
//							onlyValve = (uint8_t) atoi(aux);
//
//							if(sInstr[6] == ':')
//							{
//								aux[0] = sInstr[7];
//								aux[1] = sInstr[8];
//								aux[2] = '\0';
//								timeSectorSet = 60*((uint16_t) atoi(aux));
//
//								stateMode = 3; //onlyOneSector;
//								eeprom_write_byte(( uint8_t *)(addr_stateMode), stateMode);
//							}
//						}
//						if(sInstr[2] == ':')
//						{
//							aux[0] = sInstr[3];
//							aux[1] = sInstr[4];
//							aux[2] = '\0';
//							timeSectorSet = 60*((uint16_t) atoi(aux));
//						}
//						stateMode = 2; //automatic;
//						eeprom_write_byte(( uint8_t *)(addr_stateMode), stateMode);
//
//						break;
//
//					case 3:
//						if(sInstr[2] == ':')
//						{
//
//							aux[0] = sInstr[3];
//							aux[1] = sInstr[4];
//							aux[2] = '\0';
//							timeSectorSet = 60*((uint16_t) atoi(aux));
//
//						}
//						stateMode = 2; //automatic;
//						break;
//
//					case 9:	// Testing mode
//	//				69:s01;
//					if((sInstr[2] == ':')&&(sInstr[3] == 's'))
//					{
//						aux[0] = sInstr[4];
//						aux[1] = sInstr[5];
//						aux[2] = '\0';
//						valveOnTest = (uint8_t) atoi(aux);
//
//						stateMode = 4;//valveTesting;
//						eeprom_write_byte(( uint8_t *)(addr_stateMode), stateMode);
//
////						sprintf(buffer,"Testing Sector[%.2d]...",valveOnTest);
////						SIM900_sendSMS(buffer);
//					}
//					break;
//					default:
//						Serial.println("Ni");
//						break;
				}
				summary_Print(0);
			}
			break;


// -----------------------------------------------------------------
			case 7:
			// 7:27988081875;
//
////				if(sInstr[1] == ':')
////				{
//				int i;
//				uint8_t flag_numError;
//				flag_numError = 0;
//
//				for(i=0;i<11;i++)
//				{
//					aux[0] = '0';
//					aux[1] = sInstr[i+1];
//					aux[2] = '\0';
//					celPhoneNumber[i] = (uint8_t) atoi(aux);
//					if(celPhoneNumber[i] >9)
//					{
//						flag_numError = 1;
//					}
//				}
//
//				if(!flag_numError)
//				{
//					for(i=0;i<11;i++)
//					{
//						eeprom_write_byte(( uint8_t *)(i+addr_celNumber), celPhoneNumber[i]);
//					}
//
////					refreshCelPhoneNumber();
////					summary_Print(2);
//				}
//				else
//				{
////					summary_Print(9);
//				}

				break;

// -----------------------------------------------------------------
			case 8:
//				GLCD.Init();
			break;

// -----------------------------------------------------------------
			case 9: // internet stuffs
				// 9x;
				uint8_t setCommandConnection;
				aux[0] = '0';
				aux[1] = sInstr[1];
				aux[2] = '\0';
				setCommandConnection = (uint8_t) atoi(aux);

				switch(setCommandConnection)
				{
					case 0:
//						Serial.println("Starting GPRS Conn...!");
//						SIM900_GPRS_Connect();
						break;

					case 1:
//						SIM900_getIpAddress();
						break;

					case 2:
//						Serial.println("Starting TCP Server...");
//						SIM900_TCP_Server_Start();
						break;

					case 3:
//						Serial.println("Stoping GPRS Conn...!");
//						SIM900_GPRS_Diconnect();

						break;

					case 4:
//						SIM900_sendmail();
						break;

					default:
//						Serial.println("N4");
						break;
				}
				break;

// -----------------------------------------------------------------
			default:
				summary_Print(10);
				break;
		}
		memset(sInstr,0,sizeof(sInstr));
	}
}

ISR(TIMER1_COMPA_vect)
{
//	if(flag_SIM900_checkAlive)
//	{
//		count_SIM900_timeout++;
//	}
//	else
//	{
//		if(count_30s > 30)
//		{
//			count_30s = 0;
//			count_SIM900_timeout = 0;
//
//			flag_30s = 1;
//		}
//		else
//			count_30s++;
//	}

	if(!count_timePipeB)
		flag_timePipeB = 1;
	else
		count_timePipeB--;

	if(stateMode)
	{
		if(timeSector == 0)
			flag_timeOVF = 1;
		else
			timeSector--;
	}

	flag_1s =1;
}

// Test functions
void get_adc()
{

//	ADCSRB &= ~(1<<MUX5);				// To select ADC0;

//	ADMUX &= ~(1<<MUX4);				// Select ADC0
	ADMUX &= ~(1<<MUX3);
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX1);
	ADMUX |=  (1<<MUX0);

	ADCSRA |= (1<<ADSC);				// Start conversion;
	while (bit_is_set(ADCSRA, ADSC));	// wait until conversion done;

	uint8_t low, high;
	low  = ADCL;
	high = ADCH;

//	int value = (high << 8) | low;
//	Serial.println(value);
	_delay_ms(200);


//	return ((high << 8) | low);
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
	init_valves();
	init_contactors();
	init_ADC();
//	init_SIM900();
	init_Timer1_1Hz();
	init_WDT();

	Serial.begin(38400);				// Debug
	Serial.println("_V_");				// Welcome!

	refreshTimeSectors();				// Refresh variables
	refreshStoredData();
//	refreshCelPhoneNumber();

	while (1)
	{
		// Refresh all variables to compare and take decisions;
		wdt_reset();
		refreshVariables();

		// Bluetooth communication
		wdt_reset();
		comm_Bluetooth();

		// Message Manipulation
		wdt_reset();
		handleMessage();

		// Main process.
		wdt_reset();
		process_Mode();

//		if(freeMemory()<50)
//		{
//			Serial.println("Down!");
//		}
		//	char buffer[15];
		//	sprintf(buffer,"RAM: %d",freeMemory());
		//	Serial.println(buffer);

////	SIM900 <--> uC
//		wdt_reset();
//		comm_SIM900();
	}

//	// Use only this while to setup bluetooth
//	while(1)
//	{
//		wdt_reset();
//		comm_SerialPC_BluetoothMOD();
//	}
}
