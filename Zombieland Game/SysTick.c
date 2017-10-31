// SysTick.c
// Taken from Lab 8

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "ST7735.h"
#include "ADC.h"
	
extern int ADCMail;
extern int ADCStatus;

void SysTick_Init(void){
	NVIC_ST_CTRL_R = 0;	//Disable SysTick during setup
	NVIC_ST_RELOAD_R = 2000000;	//2000000 = 80MHz/40Hz
	NVIC_ST_CURRENT_R = 0;	//Any value written to CURRENT clears
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & 0x00FFFFFF) | 0x40000000; //Sets priority to 2--changed from 0x20000000
	NVIC_ST_CTRL_R |= 0x07;	//Enables SysTick with internal clock and interrupts enabled
}

