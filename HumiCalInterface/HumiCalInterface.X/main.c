/* File:   main.c  
 * Main file for Rev 2 HumiCal Interface Board
 * 
 * Project:     HumiCalInterface
 * Author:      Jim Sedgwick
 *
 * This firmware runs on HumiCal Interface Board Rev 2.0 
 * Humidity sensors are loaded on "rack" boards.
 * The Interface Board serves as the go-between that allows the PC to communicate with 
 * and adjust digital pots on sensor boards.
 * Essentially this firmware is an RS232-I2C converter.
 * 
 * The Interface board also sets multiplexers to select which one of 256 sensors  
 * to communicate with. Analog multiplexers also route the sensor analog output 
 * to a meter. Relays on Interface Board control sensor power.
 * Also I2C commands may be sent to turn on indicator LEDs on racks.
 *
 * Receives ASCII commands at 1200 baud via RS232. 
 * Communicates with AD5172 digital pots and slave rack multiplexers via I2C.
 * Turns relays on and off as necessary to "burn" fuses on pots to make calibration adjustments permanent.
 * Interface Board may control up to four racks with 32 sensors each or 256 sensors total.
 * 
 * Created on June 19, 2020 using Microchip XC8 Compiler V1.33 for PIC 18F4550
 * Adapted from Setra Master V2.0 (Interface Board) firmware 
 * Original firmware was created 9/24/2008 on MPLAB IDE probably using High Tech compiler.
 * 
 * New version editor/IDE: MPLABX V5.15
 * 
 * ConfigBits.h contains CONFIG register settings
 * 
 * 
 * 6-19-20 JBS: Created in Warwick, tested RS232 RX/TX at 1200 Baud
 * 6-20-20 JBS: Added watchdog timer set to 4096 for four second timeout approx
 * TODO: Look at compiler optimization setting.
 */

#include "ConfigBits.h"
#include <xc.h>
#include "DELAY16.H"
#include "Humidity Cal Master.h"
#include "AD5172_I2C.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h> 
#include <math.h>

#define BREQ_OUT		PORTCbits.RC2
#define VOLTAGE_SELECT 	PORTAbits.RA0
#define BANK_1_POWER	PORTAbits.RA1
#define BANK_2_POWER	PORTAbits.RA2
#define BANK_3_POWER	PORTAbits.RA3
#define BANK_4_POWER	PORTAbits.RA5

#define LATCH_RELAY_RESET PORTEbits.RE0
#define LATCH_RELAY_SET PORTEbits.RE1

#define BANK_1_ENABLE	PORTBbits.RB7
#define BANK_2_ENABLE	PORTBbits.RB6
#define BANK_3_ENABLE	PORTBbits.RB5
#define BANK_4_ENABLE	PORTBbits.RB4

#define BOARD_A_SELECT	PORTDbits.RD7
#define BOARD_B_SELECT	PORTDbits.RD6
#define BOARD_C_SELECT	PORTDbits.RD5
#define BOARD_D_SELECT	PORTDbits.RD4

#define COMPRESSOR_ON_PUSHBUTTON PORTBbits.RB3
#define COMPRESSOR_OFF_PUSHBUTTON PORTBbits.RB2

#define CompressorOnLED	PORTDbits.RD3

unsigned char timeout=0x00;
unsigned char UARTflag = FALSE;
int sensorNumber=1;
int potValue=0;
int potSelect=POT1;
int multiplexChannel=0;

unsigned char UARTbuffer[MAXBUFFER+1];
unsigned char outBuffer[MAXBUFFER+1]= {1,2,3,4,5,6,7,8};
unsigned char inBuffer[MAXBUFFER+1];

unsigned char errorFlag; // , WCOLflag;
unsigned char OTP_Validation_Flag;
unsigned char slavePIC = 0x00;
unsigned char boardSelect = 0x00;


void initializePorts(void);
void putch (unsigned char byte);
unsigned char  parseInString (void);
int setMultiplexerAndBankRelays(unsigned char boardSelect, int sensorNumber);
void setProgramSupply(unsigned char mode);
void turnCompressorOn();
void turnCompressorOff(); 

void main(void) 
{
unsigned char ch,i,testByte; 
int inByte1, inByte2, inByte3, inByte4, inByte5, inputData;
unsigned char command=0;
unsigned char CompressorFlag=FALSE;
unsigned char testByte = 'A';

	initializePorts();	
    
	LATCH_RELAY_RESET = 0;
	LATCH_RELAY_SET = 0;

	setProgramSupply(NORMAL_SUPPLY); 		// Select 5.0 volt output	
	BANK_1_POWER = 0;						// Power up state is ON.
	BANK_2_POWER = 0;						// 
	BANK_3_POWER = 0;
	BANK_4_POWER = 0;

	BANK_1_ENABLE = 0;	
	BANK_2_ENABLE = 0;
	BANK_3_ENABLE = 0;
	BANK_4_ENABLE = 0;

	BOARD_A_SELECT = 0;
	BOARD_B_SELECT = 0;
	BOARD_C_SELECT = 0;
	BOARD_D_SELECT = 0;

	PORTD = 0x00;
	CompressorOnLED = 0;
	
	printf("\r\rStarting HumiCal Rev 2 Interface Board at 1200 Baud. Compiled 6/20/2020");
	initialize_I2C();
	command=NO_COMMAND;
	printf("\r\rI2C initialized. Watchdog timer enabled: 4 second timeout\r\r");

	// Get previously stored state of latching relay:
	if(TRUE==EEPROM_READ(0x00))
		CompressorOnLED = 1;
	else
		CompressorOnLED = 0;
	
	while(1) 
	{
        ClrWdt(); // Reset watchdog timer
        
		if(0==COMPRESSOR_ON_PUSHBUTTON) 
		{
			CompressorFlag=TRUE;
			EEPROM_WRITE(0x00,CompressorFlag);
			CompressorOnLED = 1;			
			LATCH_RELAY_SET = 1;
			DelayMs(100);					
		}
		else if(0==COMPRESSOR_OFF_PUSHBUTTON) 
		{
			CompressorFlag=FALSE;
			EEPROM_WRITE(0x00,CompressorFlag);
			CompressorOnLED = 0;
			LATCH_RELAY_RESET = 1;					
			DelayMs(100);
		}
		else
		{
			LATCH_RELAY_RESET = 0;
			LATCH_RELAY_SET = 0;
		}
	
		if(TRUE==UARTflag)
		{
			UARTflag = FALSE;			
			errorFlag=0;	// errorFlag may contain errors from either I2C routines or parsing routine			
			command = parseInString();
			
			if(OFF==command)
			{
				setProgramSupply(OFF);
				printf ("Program supply OFF.\r");
			}
			else if (COMPRESSOR_ON==command)			
			{
				printf ("Compressor ON.\r");
				turnCompressorOn();
			}				
			else if (COMPRESSOR_OFF==command)			
			{
				printf ("Compressor OFF.\r");
				turnCompressorOff();
			}
			else setMultiplexerAndBankRelays(boardSelect, sensorNumber);

			if(TESTCOM==command)
			{
				printf ("COM PORT WORKS\r");
				command=0;
			}
			else if(OTP_SUPPLY==command)
			{
				setProgramSupply(OTP_SUPPLY);
				printf ("Program supply ON.\r");
			}				
			else if(NORMAL_SUPPLY==command)
			{
				setProgramSupply(NORMAL_SUPPLY);
				printf ("5 volt supply ON.\r");
			}
			else if (INIT_I2C==command)
			{
				initialize_I2C();
				printf ("I2C reinitialized\r");
			}
			else if(NO_COMMAND==command)
			{				
				printf(">%c #S%d: \r", boardSelect, sensorNumber);
			}
			else if(WRITE==command)
			{						
				AD5172_Write((unsigned char)potSelect, (unsigned char)potValue);
				if (0==errorFlag) inputData = AD5172_Read();
				if(TIMEOUT==errorFlag) printf("I2C ERROR TIMEOUT\r"); //,(int)errorFlag);
				else if(errorFlag)
				{
					printf("I2C ERROR\r"); // ,(int)errorFlag);
					AD5172_Stop();
				}				
				else if(FUSE_BLOWN==OTP_Validation_Flag)printf(">%c #S%d: Pot value=%d, FUSE BLOWN\r", boardSelect, sensorNumber, inputData);
				else if(FUSE_READY==OTP_Validation_Flag)printf(">%c #S%d: Pot value=%d, FUSE READY\r", boardSelect, sensorNumber, inputData);
				else printf("FUSES BAD\r");
			}
			else if(READ==command)
			{						
				AD5172_Set((unsigned char)potSelect);
				if (0==errorFlag) inputData = AD5172_Read();
				if(TIMEOUT==errorFlag)printf("I2C ERROR TIMEOUT\r"); // ,(int)errorFlag);
				else if(errorFlag)
				{
					printf("I2C ERROR\r"); // ,(int)errorFlag);
					AD5172_Stop();
				}
				else if(FUSE_BLOWN==OTP_Validation_Flag)printf(">%c #S%d: Pot value=%d, FUSE BLOWN\r", boardSelect, sensorNumber, inputData);
				else if(FUSE_READY==OTP_Validation_Flag)printf(">%c #S%d: Pot value=%d, FUSE READY\r", boardSelect, sensorNumber, inputData);
				else printf("FUSES BAD\r");
			}
			else if(PROGRAM==command)
			{		
				if(0==VOLTAGE_SELECT)
					printf("PROGRAMMING SUPPLY IS OFF ");
				else
				{	
					AD5172_Program((unsigned char)potSelect);
					if(TIMEOUT==errorFlag) printf("I2C ERROR TIMEOUT\r"); // $$$$ ,(int)errorFlag);
					else if(errorFlag) printf("I2C ERROR\r"); // ,(int)errorFlag);
					else if(POT1==potSelect) printf(">%c #S%d: PROGRAM Pot1\r", boardSelect, sensorNumber);
 					else if(POT2==potSelect) printf(">%c #S%d: PROGRAM Pot2\r", boardSelect, sensorNumber);
				}
			}
			else if(BAD_COMMAND==command)
			{					
				printf ("BAD COMMAND ");
				errorFlag=0xFF;
			}
			else if(SET_LEDS==command)
			{
				if (0==slavePIC) 
					printf(">%c: ERROR - BAD RACK NUMBER.\r", boardSelect);
				else
				{				
					i2c_SlaveWrite(slavePIC, 5);
					DelayMs(100);					
					i2c_SlaveRead(slavePIC, 5);
					inByte1=(int)inBuffer[0];
					inByte2=(int)inBuffer[1];
					inByte3=(int)inBuffer[2];
					inByte4=(int)inBuffer[3];	
					inByte5=(int)inBuffer[4];	
					if (0==errorFlag)				
						printf(">%c: OK, %d, %d, %d, %d, %d,\r", boardSelect, inByte1,inByte2,inByte3,inByte4,inByte5);
					else
						printf(">%c: NO RACK RESPONDING.\r", boardSelect);
				}
			}
			DelayMs(20);				
		}
	}	
}

void initializePorts(void)
{
	// Initialize ports
	//ADCON1= 0x06;		// All digital I/O	

	//ADCON1 = 0b00001010;	// AN0-AN4 are analog inputs, GND and VCC are references.
	ADCON1 = 0b00001111;	// Set up Port A as all digital.
	ADCON2 = 0b01111111;  	// Use FRC internal oscillator for A/D clock, left justified result
	ADCON0 = 0b00000000;	// Initialize A/D converter, turn it off for now, set for sensorNumber 0


	PORTA = 0b00000000;	// RA1,RA2,RA3,RA5 are BANK POWER RELAY outputs. 
	TRISA = 0b00000000; // All outputs
	TRISB = 0b00001111; // Port B is half input for I2C.

	RBPU = 0;			// Enable Port B pullups
	PORTC = 0x00;		// RS485 Enable and BREQ off for now.
	TRISC = 0b10110001; // Inputs: RC0, RC4, RC5, RC7 = Rx, RC4 = SDI	
	PORTD = 0x00;		// Set multiplexers to 0x00 and disable all board selects
	TRISD = 0x00;		// Port D is all output
	PORTE = 0x00;		// Make sure that all SPI pins start out low
	TRISE = 0b11111000; // 
//	PSPMODE = 0;		// Use PORT D for I/O

// Set up Timer 1 for interrupts every 0.056 seconds
	T1CON=0x00;		// Clear timner control register
	T1CKPS0=0;		// 1:4 ratio
	T1CKPS1=1;
	TMR1CS=0;		// Use internal clock	
	TMR1IF=0;		// Clear interrupt flag
	TMR1IE=1;		// Enable interrupts
	TMR1ON=1;		// Start Timer 1

	// sci_Init(19200, 0);

	BRGH = 0;	// low baud rate 
	SPBRG = 239;		// Set baud rate for 1200 baud @ 18.432 Mhz clock

	SYNC = 0;	// asynchronous 
	SPEN = 1;	// enable serial port pins 
	CREN = 1;	// enable reception 
	SREN = 0;	// no effect 
	TXIE = 0;	// disable tx interrupts 
	RCIE = 0;	// disable rx interrupts 
	TX9  = 0;	// 8- or 9-bit transmission 
	RX9  = 0;	// 8- or 9-bit reception 
	TXEN = 1;	// enable the transmitter 
 
	INTCON = 0x00;  // First, clear all interrupts
	PIE1 = 0;       // Clear all peripheral interrupts
	SSPIE = 0;      // Disable SSP interrupts

	TXIE = 0;       // Disable UART Tx interrupts 
	RCIE = 1;		// Enab;ed UART Rx interrupts
    
	PEIE = 1;       // Enable peripheral interrupts.
	TMR1IE = 1;		// Enable timer 1 interrupts.
	TMR2IE = 0;     // Disable timer 2 interrupts.
	GIE = 1;        // Enable global interrupts
}

/* For writinga dn reading to internal EEPROM on PIC18F4550
unsigned char EpromRead(unsigned char address)
{
	unsigned char dataRead;

	dataRead=EEPROM_READ(address);
	return dataRead;
}

void EpromWrite(unsigned char address, unsigned char data)

{
	EEPROM_WRITE(address,data);
}
*/

static void interrupt
isr (void)
{
unsigned char dummy, ch, temp;
static unsigned char i=0;

	if (OERR == 1)		// If overrun occurs, flush buffer
   	{					// and reset receive enable.
		CREN = 0;
		CREN = 1;
		dummy = RCREG;
		dummy = RCREG;
	}

	if (RCIF == 1)
	{
		RCIF = 0;				
		temp = RCREG;	
		ch=toupper(temp); // Make all alpha characters uppercase

		if('>'==ch)i=0;		

		if(i<MAXBUFFER)
		{
			UARTbuffer[i]=ch;
			i++;
		}		

		if('\r'==ch)
		{
			i = 0;
			UARTflag = TRUE;
		}
	}

	// Timer 1 interrupt - timer rolls over every 0.1137 seconds at 18.432 Mhz
	if(1==TMR1IF)
	{
		TMR1IF=0;
		timeout++;
	}
}


// void sci_PutByte(unsigned char byte)

void putch (unsigned char byte)		
{
	while(!TXIF)	/* set when register is empty */
		continue;
	TXREG = byte;	
	return;
}


// This routine parses the the ASCII command string
// received by the UART, beginning at '>'.
// It returns a single byte which is the command.
// If an 
//
// SYNTAX: To write a value of 54 
// to the balance pot (Pot 1) 
// on sensor tip #12 on board A:
// >A12 WRITE POT1 54
#define MAXSTRING 32
unsigned char  parseInString(void)
{
unsigned char i, j, k, ch;
char alphaString[32]="\0";
char numString[32]="\0";
int value;
unsigned char punctuationChar=0;
unsigned char command;
	
	i=0; // Fetch position of next character.
	punctuationChar=0;
	command=0;
	value=0;
	potValue=0;
	k=0;
	OTP_Validation_Flag=0;

	// All incoming command strings must begin with a right arrow '>'.
	// Check first character and make sure.
	// Otherwise, it's a BAD COMMAND error.	
	if('>'!=UARTbuffer[0]) 
	{		
		UARTbuffer[0]=0;
		return(BAD_COMMAND);
	}			
	UARTbuffer[0]=0;
	
	// Second chracter should select board A,B,C,D:
	boardSelect = UARTbuffer[1];
	if('A'==boardSelect)
	{		
		slavePIC = 0xA0;
	}
	else if('B'==boardSelect)
	{		
		slavePIC = 0xB0;
	}
	else if('C'==boardSelect)
	{		
		slavePIC = 0xC0;
	}
	else if('D'==boardSelect)
	{		
		slavePIC = 0xD0;
	}
	else if('X'==boardSelect) // This case means no specific board is selected.
	{		
		slavePIC = 0x00;
	}
	else 
	{		
		errorFlag=0xFF;
		boardSelect = 0;
		slavePIC = 0x00;
		command=BAD_COMMAND;
		return(0);
	}
	
	// Now that we have checked first two characters, proceed with the third character (i=2).
	punctuationChar='>'; // This indicates that a sensor number is probably coming up next
	i=2;
	do{
		ch=UARTbuffer[i];

		if ('\r'==ch)
		{				
			break;
		}		
		// If a digit is encountered, get all digits and compute integer:
		else if (isdigit(ch))
		{					
			j=0;			
			do{			
				numString[j]=ch;
				j++; 
				if(j>=MAXSTRING)return(BAD_COMMAND);
				i++;
				if(i>=MAXBUFFER)return(BAD_COMMAND);
				ch=UARTbuffer[i];				
			}while(isdigit(ch));
			
			if(j<MAXSTRING)
			{
				numString[j]='\0';
				value=atoi(numString);
			}

			if ('#'==punctuationChar) // If number is preceded by "#S" this must be the sensor number
			{
				sensorNumber=value;
				numString[0]='\0';
				punctuationChar=0;
			}
			else if (WRITE==command)
			{
				potValue=value;

			}
			else if (SET_LEDS==command)
			{	
				if(k<MAXBUFFER)(unsigned char)outBuffer[k]=value;
				k++;
			}		
		}

		// If character is the start of a string i.e: "START_CUE",
		// get complete string, store in array, and terminate it.
		// Note: first character in string must be a letter, but
		// numbers can follow as well, ie: "POT1"
		else if(isalpha(ch))
		{
			j=0;					
			do{					
				alphaString[j]=ch;
				j++; 
				if(j>=MAXSTRING)return(BAD_COMMAND);
				i++;
				if(i>=MAXBUFFER)return(BAD_COMMAND);
				ch=UARTbuffer[i];
			}while((isalpha(ch))||('_'==ch)||(isdigit(ch)));	

			if(j<MAXSTRING)
			{
				alphaString[j]='\0';
			}			

			if(0==strcmp("WRITE", alphaString))			
				command=WRITE;							
			else if(0==strcmp("COM", alphaString))			
				command=TESTCOM;							
			else if(0==strcmp("READ", alphaString))
				command=READ;
			else if(0==strcmp("PROG", alphaString))
				command=PROGRAM;
			else if(0==strcmp("POT1", alphaString))
				potSelect=POT1;
			else if(0==strcmp("POT2", alphaString))
				potSelect=POT2;
			else if(0==strcmp("OTP", alphaString))
				command=OTP_SUPPLY;
			else if(0==strcmp("NORM", alphaString))
				command=NORMAL_SUPPLY;
			else if(0==strcmp("OFF", alphaString))			
				command=OFF;
			else if (0==strcmp("I2C", alphaString))	
				command=INIT_I2C;
			else if(0==strcmp("COMPRESSOR_OFF", alphaString))
				command=COMPRESSOR_OFF;
			else if(0==strcmp("COMPRESSOR_ON", alphaString))
				command=COMPRESSOR_ON;
			else if(0==strcmp("LEDS", alphaString))			
			{
				command=SET_LEDS;
				sensorNumber=0;	// No specific sensor selected
			}
		}
		else if (ispunct(ch))
		{
			punctuationChar=ch;
			if('#'==ch)
			{
				i++; 	// If character is a pound sign "#", 
					 	// then the next character should be the letter S.
						// If it isn't an S, that's an error:
				if(i<MAXBUFFER)	ch=UARTbuffer[i];
				if('S'!=ch) return(BAD_COMMAND);				
			}					 
		}
		else if (isspace(ch))
		{
			;
		}
		i++;		
	}while((i<MAXBUFFER)&&('\r'!=ch));

	return(command);
}


// This routine turns on the programming 
// supply relay for PROGRAM MODE
// and turns it off for NORMAL_MODE.
// For OFF mode, it turns off all the
// bank relays and bank enables also.
void setProgramSupply(unsigned char mode)
{
	if (OFF==mode)
	{
		VOLTAGE_SELECT = 0;	
		BANK_1_POWER=0;		// POWER DOWN: sensor tip boards are switched off when BANK POWER RELAYS outputs = 0.
		BANK_2_POWER=0;
		BANK_3_POWER=0;
		BANK_4_POWER=0;

		BANK_1_ENABLE=0;
		BANK_2_ENABLE=0;
		BANK_3_ENABLE=0;
		BANK_4_ENABLE=0;
	}
	else if (OTP_SUPPLY==mode)
	{
		VOLTAGE_SELECT=1;	// Switch voltage selector relay to programming supply: 5.4 volts.
		BANK_1_POWER=1;		// POWER UP: set BANK POWER RELAYS to 1 to turn sensor tip boards ON.
		BANK_2_POWER=1;
		BANK_3_POWER=1;
		BANK_4_POWER=1;
	}
	else
	{
		VOLTAGE_SELECT=0;	// Switch voltage selector relay to programming supply: 5.4 volts.
		BANK_1_POWER=1;		// POWER UP: set BANK POWER RELAYS to 1 to turn sensor tip boards ON.
		BANK_2_POWER=1;
		BANK_3_POWER=1;
		BANK_4_POWER=1;
	}
}

// 	This routine determines the bank number and multiplex channel for the selected sensor.
// 	It then turns on the appropriate relay and multiplexer,
// 	and sets the multiplexer channel for the desired sensor.
// 	The sensor number is 1 through 32. There are four banks, 1 through 4, 
// 	and eight multiplex channels, 0 through 7.
//
//	Sensor Number = 0 means no sensor has been selected, as in LED command.
//
// 	This routine returns FALSE if an invalid sensor number is entered.
// 	Otherwise it returns TRUE.
// 	"sensorNumber" is the selected sensor.
#define RELAYTIME 10
int setMultiplexerAndBankRelays(unsigned char boardSelect, int sensorNumber)
{
int quotient, dividend, remainder, divisor, temp,  bank;
unsigned char channel;

	if ('A'==boardSelect)
	{
		BOARD_A_SELECT = 1;
		BOARD_B_SELECT = 0;
		BOARD_C_SELECT = 0;
		BOARD_D_SELECT = 0;					
	}
	else if ('B'==boardSelect)
	{
		BOARD_A_SELECT = 0;
		BOARD_B_SELECT = 1;
		BOARD_C_SELECT = 0;
		BOARD_D_SELECT = 0;
	}
	else if ('C'==boardSelect)
	{
		BOARD_A_SELECT = 0;
		BOARD_B_SELECT = 0;
		BOARD_C_SELECT = 1;
		BOARD_D_SELECT = 0;
	}
	else if ('D'==boardSelect)
	{
		BOARD_A_SELECT = 0;
		BOARD_B_SELECT = 0;
		BOARD_C_SELECT = 0;
		BOARD_D_SELECT = 1;
	}
	else // If no rack is selected, that's an error:
	{
		BOARD_A_SELECT = 0;
		BOARD_B_SELECT = 0;
		BOARD_C_SELECT = 0;
		BOARD_D_SELECT = 0;
		return(FALSE);
	}				

	if(sensorNumber>MAXSENSOR)
	{
		errorFlag=(unsigned char)errorFlag|0x10; // invalid sensor number
		return(FALSE);
	}
	if(sensorNumber<0)
	{
		errorFlag=(unsigned char)(errorFlag|0x10); // invalid sensor number
		return(FALSE);
	}

	dividend=sensorNumber-1;	
	quotient=dividend/8;
	bank=quotient+1;
	temp=quotient*8;
	remainder=dividend-temp;

	multiplexChannel=remainder;	// multiplex Channel is 0 through 7	
	channel=(unsigned char)(multiplexChannel&0x0F);
	PORTD=(PORTD&0xF0)|channel;	// Set the PORTD address bits for the multiplexers.
	//printf("SensorNumber=%d, Bank=%d, Multiplex Channel=%d\r", sensorNumber, bank, multiplexChannel);

	if (0==sensorNumber)  // If sensor is 0, do nothing with multiplexers:
	{
		;
	}
	else if(1==bank)
	{		
		BANK_2_ENABLE=0;
		BANK_3_ENABLE=0;
		BANK_4_ENABLE=0;
		DelayMs(RELAYTIME);

		BANK_1_ENABLE=1;

		BANK_1_POWER=1;	// POWER UP: set BANK POWER RELAYS to 1 to turn sensor tip boards ON.
		BANK_2_POWER=1;
		BANK_3_POWER=1;
		BANK_4_POWER=1;
	}
	else if(2==bank)
	{
		BANK_1_ENABLE=0;
		BANK_3_ENABLE=0;
		BANK_4_ENABLE=0;
		DelayMs(RELAYTIME);

		BANK_2_ENABLE=1;		

		BANK_1_POWER=1;	// POWER UP: set BANK POWER RELAYS to 1 to turn sensor tip boards ON.
		BANK_2_POWER=1;
		BANK_3_POWER=1;
		BANK_4_POWER=1;
	}
	else if(3==bank)
	{		
		BANK_1_ENABLE=0;
		BANK_2_ENABLE=0;
		BANK_4_ENABLE=0;
		DelayMs(RELAYTIME);

		BANK_3_ENABLE=1;

		BANK_1_POWER=1;	// POWER UP: set BANK POWER RELAYS to 1 to turn sensor tip boards ON.
		BANK_2_POWER=1;
		BANK_3_POWER=1;
		BANK_4_POWER=1;	
	}
	else if(4==bank)	
	{
		BANK_1_ENABLE=0;
		BANK_2_ENABLE=0;
		BANK_3_ENABLE=0;
		DelayMs(RELAYTIME);

		BANK_4_ENABLE=1;

		BANK_1_POWER=1;	// POWER UP: set BANK POWER RELAYS to 1 to turn sensor tip boards ON.
		BANK_2_POWER=1;
		BANK_3_POWER=1;
		BANK_4_POWER=1;
	}

	return(TRUE);
}


void turnCompressorOn()
{
	EEPROM_WRITE(0x00,TRUE);
	CompressorOnLED = 1;			
	LATCH_RELAY_SET = 1;
	DelayMs(200);
	LATCH_RELAY_SET = 0;
}

void turnCompressorOff() 
{	
	EEPROM_WRITE(0x00,FALSE);
	CompressorOnLED = 0;
	LATCH_RELAY_RESET = 1;					
	DelayMs(200);
	LATCH_RELAY_RESET = 0;
}

