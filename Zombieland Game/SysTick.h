// SysTick.c

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "ADC.h"
	
extern int ADCMail;
extern int ADCStatus;

void SysTick_Init(void);
void SysTick_Handler(void);
