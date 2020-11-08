/*
 * Pulse-Width-Modulation Code for K64
 * PWM signal can be connected to output pins PC3 and PC4
 * 
 * Author: Brent Dimmig <bnd8678@rit.edu>
 * Modified by: Carson Clarke-Magrab <ctc7359@rit.edu>
 * Created: 2/20/2014
 * Modified: 2/25/2020
 */
#include "MK64F12.h"
#include "pwm.h"

/*From clock setup 0 in system_MK64f12.c*/
#define DEFAULT_SYSTEM_CLOCK    20485760u 
#define FTM0_MOD_VALUE          (DEFAULT_SYSTEM_CLOCK/10000)
#define FTM3_MOD_VALUE          ((DEFAULT_SYSTEM_CLOCK/128)/10000)


/*
 * Change the motor duty cycle and frequency
 *
 * @param duty_cycle (0 to 100)
 * @param frequency (~1000 Hz to 20000 Hz)
 * @param dir: 1 for pin C4 active, else pin C3 active 
 * @param motor: 1 for left, elce right
 */
void FTM0_set_duty_cycle(unsigned int duty_cycle, unsigned int frequency, int dir, int motor)
{
	// Calculate the new cutoff value
	uint16_t mod = (uint16_t) (((DEFAULT_SYSTEM_CLOCK / frequency) * duty_cycle) / 100);
  
	// Set outputs 
	if(motor){
		if(dir) {
				FTM0_C3V = mod; 
				FTM0_C2V = 0;
		} else {
				FTM0_C2V = mod; 
				FTM0_C3V = 0;
		}
	}else
			if(dir) {
				FTM0_C1V = mod; 
				FTM0_C0V = 0;
		} else {
				FTM0_C0V = mod; 
				FTM0_C1V = 0;
		}

	// Update the clock to the new frequency
	FTM0_MOD = (DEFAULT_SYSTEM_CLOCK / frequency);
}


/*
 * Change the motor duty cycle and frequency
 *
 * @param duty_cycle (0 to 100)
 * @param frequency (50 Hz to 20000 Hz)
 * @param en: 1 for pin C8 active, else pin C8 not active 
 */
void FTM3_set_duty_cycle(float duty_cycle, unsigned int frequency,int en)
{
	if (duty_cycle>7.4){
		duty_cycle=7.4;
	}else if(duty_cycle<4.3){
		duty_cycle=4.3;
	}
	// Calculate the new cutoff value
	uint16_t mod = (uint16_t) ((((DEFAULT_SYSTEM_CLOCK/128.0) / frequency) * duty_cycle) / 100.0);
  
	// Set outputs 
	if (en){
	    FTM3_C4V = mod;
	} else {
			FTM3_C4V = 0;
	}
	// Update the clock to the new frequency
	FTM3_MOD = ((DEFAULT_SYSTEM_CLOCK/128) / frequency);
}


/*
 * Initialize the FlexTimer for PWM
 */
void FTM0_init()
{
	// 12.2.13 Enable the clock to the FTM0 Module
	SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;
	
	// Enable clock on PORT C so it can output the PWM signals
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	
	// 11.4.1 Route the output of FTM channel 0 to the pins
	// Use drive strength enable flag to high drive strength
    PORTC_PCR3 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK; //Ch2
    PORTC_PCR4 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK; //Ch3
	  PORTC_PCR1 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK; //Ch2
    PORTC_PCR2 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK; //Ch3
	
	// 39.3.10 Disable Write Protection
	FTM0_MODE |= FTM_MODE_WPDIS_MASK;
	
	// 39.3.4 FTM Counter Value
	// Initialize the CNT to 0 before writing to MOD
	FTM0_CNT = 0;
	
	// 39.3.8 Set the Counter Initial Value to 0
	FTM0_CNTIN = 0;
	
	// 39.3.5 Set the Modulo resister
	FTM0_MOD = FTM0_MOD_VALUE;

	// 39.3.6 Set the Status and Control of both channels
	// Used to configure mode, edge and level selection
	// See Table 39-67,  Edge-aligned PWM, High-true pulses (clear out on match)
	FTM0_C3SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM0_C3SC &= ~FTM_CnSC_ELSA_MASK;
	
	// See Table 39-67,  Edge-aligned PWM, Low-true pulses (clear out on match)
	FTM0_C2SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM0_C2SC &= ~FTM_CnSC_ELSA_MASK;

	// See Table 39-67,  Edge-aligned PWM, Low-true pulses (clear out on match)
	FTM0_C0SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM0_C0SC &= ~FTM_CnSC_ELSA_MASK;
	
		// See Table 39-67,  Edge-aligned PWM, Low-true pulses (clear out on match)
	FTM0_C1SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM0_C1SC &= ~FTM_CnSC_ELSA_MASK;
	// 39.3.3 FTM Setup
	// Set prescale value to 1 
	// Chose system clock source
	// Timer Overflow Interrupt Enable
	FTM0_SC = FTM_SC_PS(0) | FTM_SC_CLKS(1); 
}


/*
 * Initialize the FlexTimer for PWM
 */
void FTM3_init()
{
	// 12.2.13 Enable the clock to the FTM1 Module
	SIM_SCGC3 |= SIM_SCGC3_FTM3_MASK;
	
	// Enable clock on PORT C so it can output the PWM signals
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	
	// 11.4.1 Route the output of FTM channel 1 to the pins
	// Use drive strength enable flag to high drive strength
    PORTC_PCR8 = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK; //Ch4 pg.251
	
	// 39.3.10 Disable Write Protection
	FTM3_MODE |= FTM_MODE_WPDIS_MASK;
	
	// 39.3.4 FTM Counter Value
	// Initialize the CNT to 0 before writing to MOD
	FTM3_CNT = 0;
	
	// 39.3.8 Set the Counter Initial Value to 0
	FTM3_CNTIN = 0;
	
	// 39.3.5 Set the Modulo resister
	FTM3_MOD = FTM3_MOD_VALUE;

	// 39.3.6 Set the Status and Control of both channels
	// Used to configure mode, edge and level selection
	// See Table 39-67,  Edge-aligned PWM, High-true pulses (clear out on match)
	
	// See Table 39-67,  Edge-aligned PWM, Low-true pulses (clear out on match)
	FTM3_C4SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM3_C4SC &= ~FTM_CnSC_ELSA_MASK;
	
	// 39.3.3 FTM Setup
	// Set prescale value to 128
	// Chose system clock source
	// Timer Overflow Interrupt Enable
	FTM3_SC = FTM_SC_PS(0x7) | FTM_SC_CLKS(1); 
}

void en_motors(void){
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

void init_motors(void){
	FTM0_init();
	FTM3_init();
	en_motors();
}

