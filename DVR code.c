/**

//----------------------------------------------------------------------------ARIK INTENAM MIR----------------------------------------------------------------------------------------
//---------------------------------------------------------------------------n9637567----------------------------------------------------------------------------------------
//---------------------------------------------------------------------------EGB240 Assignment 02----------------------------------------------------------------------------------------
//---------------------------------------------------------------------------31/05/19----------------------------------------------------------------------------------------



 * main.c - EGB240 Digital Voice Recorder Skeleton Code
 *
 * This code provides a skeleton implementation of a digital voice 
 * recorder using the Teensy microcontroller and QUT TensyBOBv2 
 * development boards. This skeleton code demonstrates usage of
 * the EGB240DVR library, which provides functions for recording
 * audio samples from the ADC, storing samples temporarily in a 
 * circular buffer, and reading/writing samples to/from flash
 * memory on an SD card (using the FAT file system and WAVE file
 * format. 
 *
 * This skeleton code provides a recording implementation which 
 * samples CH0 of the ADC at 8-bit, 15.625kHz. Samples are stored 
 * in flash memory on an SD card in the WAVE file format. The 
 * filename is set to "EGB240.WAV". The SD card must be formatted 
 * with the FAT file system. Recorded WAVE files are playable on 
 * a computer.
 * 
 * LED4 on the TeensyBOBv2 is configured to flash as an 
 * indicator that the programme is running; a 1 Hz, 50 % duty
 * cycle flash should be observed under normal operation.
 *
 * A serial USB interface is provided as a secondary control and
 * debugging interface. Errors will be printed to this interface.
 *
 * Version: v1.0
 *    Date: 10/04/2016
 *  Author: Mark Broadmeadow
 *  E-mail: mark.broadmeadow@qut.edu.au
 */  

 /************************************************************************/
 /* INCLUDED LIBRARIES/HEADER FILES                                      */
 /************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>

#include "serial.h"
#include "timer.h"
#include "wave.h"
#include "buffer.h"
#include "adc.h"
#include "lib/fatfs/ff.h"
#include "lib/fatfs/diskio.h"


/************************************************************************/
/* ENUM DEFINITIONS                                                     */
/************************************************************************/
enum {
	DVR_STOPPED,
	DVR_RECORDING,
	DVR_PLAYING
};

/************************************************************************/
/* GLOBAL VARIABLES                                                     */
/************************************************************************/
uint16_t pageCount = 0;	// Page counter - used to terminate recording
uint16_t newPage = 0;	// Flag that indicates a new page is available for read/write
uint8_t stop = 0;		// Flag that indicates playback/recording is complete
uint16_t counter = 0;
volatile uint16_t speed = 0;

/************************************************************************/
/* FUNCTION PROTOTYPES                                                  */
/************************************************************************/
void pageFull();
void pageEmpty();
void PWM_mode_ON();

/************************************************************************/
/* INITIALISATION FUNCTIONS                                             */
/************************************************************************/

// Initialise PLL (required by USB serial interface, PWM)
void pll_init() {
	PLLFRQ = 0x6A; // PLL = 96 MHz, USB = 48 MHz, TIM4 = 64 MHz
}

// Configure system clock for 16 MHz
void clock_init() {
	CLKPR = 0x80;	// Prescaler change enable
	CLKPR = 0x00;	// Prescaler /1, 16 MHz
}
 
void userio_init()
{
	PORTD &= 0x0F; // LEDs turned off 
	DDRD |= 0xF0; //ENABLING THE LED 1-4 as outputs
	DDRF &= 0x8F; // pushbuttons 1-3
	DDRB |= (1<<PINB6);
}


// Initialise DVR subsystems and enable interrupts
void init() {
	cli();			// Disable interrupts
	clock_init();	// Configure clocks
	pll_init();     // Configure PLL (used by Timer4 and USB serial)
	serial_init();	// Initialise USB serial interface (debug)
	timer_init();	// Initialise timer (used by FatFs library)
	buffer_init(pageFull, pageEmpty);  // Initialise circular buffer (must specify callback functions)
	adc_init();		// Initialise ADC
	userio_init(); // inititalize the LEDs
	sei();			// Enable interrupts
	
	// Must be called after interrupts are enabled
	wave_init();	// Initialise WAVE file interface
}

void PWM_mode_ON()
{
	cli();
	CLKPR = 0x80;	// Prescaler change enable
	CLKPR = 0x00;	// Prescaler /1, 16 MHz
	
	DDRF &= 0b10001111;    // Pushbuttons 1 to 3 - PORTF 6-4 as inputs
	DDRD |= 0b11110000;		// Set PORTD 7-4 as outputs (LEDs)
	DDRB |= 0b01000000;	   // JOUT - PORTB 6 as an output
	
	TCCR4B = 0b00000100;
	TCCR4A = 0x21;   
	OCR4C = 255;
	OCR4B = 255 *0.5;
	TIMSK4 = 0x04;
	TCNT4 = 0x00;
	serial_init();
	sei();
	
}
void PWM_mode_OFF()
{
	TCNT4 = 0x00;
	TCCR4A = 0x00;
	TIMSK4 = 0x00;
	OCR4B = 0;
}
ISR(TIMER4_OVF_vect) 
{
	//newPage++;
	OCR4B = (0x3FF)&newPage;
	uint8_t fidgit = buffer_dequeue();		//dequeue here
	OCR4B = fidgit ;
}


/************************************************************************/
/* CALLBACK FUNCTIONS FOR CIRCULAR BUFFER                               */
/************************************************************************/

// CALLED FROM BUFFER MODULE WHEN A PAGE IS FILLED WITH RECORDED SAMPLES
void pageFull() {
	if(!(--pageCount)) {
		// If all pages have been read
		adc_stop();		// Stop recording (disable new ADC conversions)
		stop = 1;		// Flag recording complete
	} else {
		newPage = 1;	// Flag new page is ready to write to SD card
	}
}

// CALLED FROM BUFFER MODULE WHEN A NEW PAGE HAS BEEN EMPTIED
void pageEmpty() {
	if(!(--pageCount)){
		stop =1;
	}else{
		newPage=1;
	}
}

/************************************************************************/
/* RECORD/PLAYBACK ROUTINES                                             */
/************************************************************************/

// Initiates a record cycle
void dvr_record() {
	buffer_reset();		// Reset buffer state
	counter=0;
	pageCount = 305;	// Maximum record time of 10 sec
	newPage = 0;		// Clear new page flag
	wave_create();		// Create new wave file on the SD card
	adc_start();		// Begin sampling
	PORTD |= 0x60;
}
void dvr_playback()
{
	buffer_reset();
	pageCount = counter;
	PORTD |= 0x10;
	wave_open();
	wave_read(buffer_writePage(),1024);
	newPage = 0;
	PWM_mode_ON();
}

// TODO: Implement code to initiate playback and to stop recording/playback.

/************************************************************************/
/* MAIN LOOP (CODE ENTRY)                                               */
/************************************************************************/
int main(void) {
	uint8_t state = DVR_STOPPED;	// Start DVR in stopped state
	uint8_t pb = 0x00 ;
	
	// Initialisation
	init();
	//PORTD |= (1<<PIND4); // TURNS ON THE LED4
	
	// Loop forever (state machine)
	for(;;) {
		pb = ~PINF;

		// Switch depending on state
		switch (state) {
			case DVR_STOPPED:
			PORTD |= 0x40; //led3
			
			if(pb & (1<<PINF5)){
				printf("Recording...");	// Output status to console
				dvr_record();			// Initiate recording
				state = DVR_RECORDING;	// Transition to "recording" state
				PORTD &= 0xBF;
			}
			else if(pb & (1<<PINF4))
			{
				printf("Playing...");
				dvr_playback();
				state = DVR_PLAYING;
				PORTD &= 0xBF;
			}
			
			break;
			case DVR_RECORDING:
			// TODO: Implement stop functionality
			if (pb & (1<<PINF6)) {
				pageCount = 1;	// Finish recording last page
			}
			
			// Write samples to SD card when buffer page is full
			if (newPage) {
				newPage = 0;
				counter++;	// Acknowledge new page flag
				wave_write(buffer_readPage(), 512);
				} else if (stop) {
				// Stop is flagged when the last page has been recorded
				stop = 0;							// Acknowledge stop flag
				wave_write(buffer_readPage(), 512);	// Write final page
				wave_close();
				adc_stop();						// Finalise WAVE file
				printf("DONE!\n");					// Print status to console
				PORTD &= 0x0F;
				state = DVR_STOPPED;				// Transition to stopped state
			}
			
			break;
			case DVR_PLAYING:
			if(newPage)
			{
				newPage = 0;
				wave_read(buffer_writePage(),512);
			}else if (stop||(pb & 1<<PINF6 ))
			{
				stop = 0;
				wave_close();
				PWM_mode_OFF();
				//MIGHT HAVE TO DO THE PWM_STOP MODE FUNCTION
				printf("RECORDING DONE");
				PORTD &= 0b11101111;
				PORTD &= 0b01111111;
				state = DVR_STOPPED;
			}
			
			break;
			default:
			// Invalid state, return to valid idle state (stopped)
			printf("ERROR: State machine in main entered invalid state!\n");
			state = DVR_STOPPED;
			PORTD |= (1<<PIND4);
			PORTD &= (0b10011111);
			break;

		} // END switch(state)
		
	} // END for(;;)

}



