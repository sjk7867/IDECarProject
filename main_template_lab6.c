/*
 * Main module for testing the PWM Code for the K64F
 * 
 * Author:  
 * Created:  
 * Modified: Carson Clarke-Magrab <ctc7359@rit.edu> 
 */

#include "MK64F12.h"
#include "uart.h"
#include "PWM.h"

void delay(int del);
void en_motors();

int main(void) {
	// Initialize UART and PWM
	uart_init();
	FTM0_init();
	FTM3_init();
	// Print welcome over serial
	uart_put("Running... \n\r");
	en_motors();
	
		FTM0_set_duty_cycle(0,10000,1,1);
		FTM0_set_duty_cycle(0,10000,1,0);
	
	//FTM3_set_duty_cycle(1.5,50,1);
	//FTM0_set_duty_cycle(30,10000,1);
	//FTM0_set_duty_cycle(30,10000,0);
	for(;;){  //then loop forever
		FTM0_set_duty_cycle(50,10000,1,1);
		FTM0_set_duty_cycle(50,10000,1,0);
		FTM3_set_duty_cycle(1.3,50,1);
		delay(10);
		FTM3_set_duty_cycle(1.5,50,1);
		delay(10);
		FTM3_set_duty_cycle(2,50,1);
		FTM0_set_duty_cycle(0,10000,1,1);
		FTM0_set_duty_cycle(0,10000,1,0);
		delay(10);
	}
		
	return 0;
}


void delay(int del){
	int i;
	for (i=0; i<del*50000; i++){
		// Do nothing
	}
}

void en_motors(){
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	// Configure the Signal Multiplexer for GPIO
  PORTB_PCR2 |= PORT_PCR_MUX(1);
  PORTB_PCR3 |= PORT_PCR_MUX(1);
	// Switch the GPIO pins to output mode
	GPIOB_PDDR = (1<<2) | (1<<3);
	//Enable Motors
	GPIOB_PSOR = (1<<2)|(1<<3); //A
	//GPIOB_PCOR |= (1<<3); //B
}
