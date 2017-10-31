// dac.c
// This software configures DAC output
// Runs on LM4F120 or TM4C123
// Program written by: put your names here
// Date Created: 3/6/17 
// Last Modified: 3/9/17 
// Lab number: 6
// Hardware connections
// TO STUDENTS "REMOVE THIS LINE AND SPECIFY YOUR HARDWARE********

#include <stdint.h>
#include "tm4c123gh6pm.h"
// Code files contain the actual implemenation for public functions
// this file also contains an private functions and private data

// **************DAC_Init*********************
// Initialize 4-bit DAC, called once 
// Input: none
// Output: none
// Note: This subroutine is written based on the assumption that Port B, bits 0-5 are used for the DAC
void DAC_Init(void){
	unsigned long volatile delay;
	SYSCTL_RCGCGPIO_R |= 0x02; //Turn on Port B
	delay = 10;
	
	//Port B initialization
	GPIO_PORTB_AMSEL_R &= ~0x0F;
	GPIO_PORTB_DIR_R |= 0x0F;
	GPIO_PORTB_AFSEL_R &= ~0x0F;
	GPIO_PORTB_DEN_R |= 0x0F;
}

// **************DAC_Out*********************
// output to DAC
// Input: 4-bit data, 0 to 15 
// Output: none
// Note: This subroutine is written based on the assumption that Port B is used for the DAC
void DAC_Out(uint32_t data){
	GPIO_PORTB_DATA_R &= ~0x0F;
	GPIO_PORTB_DATA_R = data;
}
