#include "MK64F12.h"
#include "stdio.h"
//#include "uart.h"

// Pixel counter for camera logic
// Starts at -2 so that the SI pulse occurs
//		ADC reads start
int pixcnt = -2;
// clkval toggles with each FTM interrupt
int clkval = 0;
// line stores the current array of camera data
uint16_t line[128];

// These variables are for streaming the camera
//	 data over UART
int debugcamdata = 1;
int capcnt = 0;
char str[100];

int maxsmooth(void);
int minsmooth(void);


uint16_t ADC0VAL;

/* ADC0 Conversion Complete ISR  */
void ADC0_IRQHandler(void) {
	// Reading ADC0_RA clears the conversion complete flag
	ADC0VAL=ADC0_RA;
	
}

/* 
* FTM2 handles the camera driving logic
*	This ISR gets called once every integration period
*		by the periodic interrupt timer 0 (PIT0)
*	When it is triggered it gives the SI pulse,
*		toggles clk for 128 cycles, and stores the line
*		data from the ADC into the line variable
*/
void FTM2_IRQHandler(void){ //For FTM timer
	// Clear interrupt
	FTM2_SC &= ~FTM_SC_TOF_MASK;
	// Toggle clk
	//GPIOB_PSOR |= (1<<9);
	clkval=!clkval;
	if(clkval==0){
		GPIOB_PCOR=(1 <<9);
	}else{
		GPIOB_PSOR = (1<<9);
	}
	if ((pixcnt >= 2) && (pixcnt < 256)) {
		if (!clkval) {	// check for falling edge
			// ADC read (note that integer division is 
			//  occurring here for indexing the array)
			line[pixcnt/2] = ADC0VAL;
		}
		pixcnt += 1;
	} else if (pixcnt < 2) {
		if (pixcnt == -1) {
			GPIOB_PSOR |= (1 << 23); // SI = 1
		} else if (pixcnt == 1) {
			GPIOB_PCOR |= (1 << 23); // SI = 0
			// ADC read
			line[0] = ADC0VAL;
		} 
		pixcnt += 1;
	} else {
		GPIOB_PCOR |= (1 << 9); // CLK = 0
		clkval = 0; // make sure clock variable = 0
		pixcnt = -2; // reset counter
		// Disable FTM2 interrupts (until PIT0 overflows
		//   again and triggers another line capture)
		FTM2_SC &=~FTM_SC_TOIE_MASK;
	}
	return;
}

/* PIT0 determines the integration period
*		When it overflows, it triggers the clock logic from
*		FTM2. Note the requirement to set the MOD register
* 	to reset the FTM counter because the FTM counter is 
*		always counting, I am just enabling/disabling FTM2 
*		interrupts to control when the line capture occurs
*/
void PIT0_IRQHandler(void){
	if (debugcamdata) {
		// Increment capture counter so that we can only 
		//	send line data once every ~2 seconds
		capcnt += 1;
	}
	// Clear interrupt
	PIT_TFLG0 |=PIT_TFLG_TIF_MASK;
	
	// Setting mod resets the FTM counter
	FTM2->MOD = 0x69; //10us
	FTM2_CNT = 0;
	// Enable FTM2 interrupts (camera)
	FTM2_SC |= FTM_SC_TOIE_MASK;
	
	return;
}


/* Initialization of FTM2 for camera */
void init_FTM2(void){
	// Enable clock
	SIM_SCGC6 |= SIM_SCGC6_FTM2_MASK;

	// Disable Write Protection
	FTM2_MODE |= FTM_MODE_WPDIS_MASK;
	FTM2_MODE |= FTM_MODE_FTMEN_MASK;
	
	// Set output to '1' on init
	FTM2_MODE |= FTM_MODE_INIT_MASK;
	
	// Initialize the CNT to 0 before writing to MOD
	FTM2_CNT &=FTM_CNT_COUNT(0);
	
	// Set the Counter Initial Value to 0
	FTM2_CNTIN &= FTM_CNTIN_INIT(0);
	
	// Set the period (~10us)
	FTM2->MOD = 0x69; //10us
	//FTM2->MOD = (DEFAULT_SYSTEM_CLOCK/(1<<7))/100000; //10us
	
	// 50% duty
	//FTM2_C0V=1/2(FTM2_MOD)
	FTM2_C0V = FTM_CnV_VAL(0x67);
	
	// Set edge-aligned mode
	FTM2_QDCTRL &= ~FTM_QDCTRL_QUADEN_MASK; //quadrature decoder disable
	FTM2_C0SC |= FTM_CnSC_MSB_MASK;
	
	
	// Enable High-true pulses
	// ELSB = 1, ELSA = 0
	FTM2_C0SC |= FTM_CnSC_ELSB_MASK; //ELSB=1
	FTM2_C0SC &= ~FTM_CnSC_ELSA_MASK; //ELSA=0

	// Enable hardware trigger from FTM2
	FTM2_EXTTRIG |= FTM_EXTTRIG_CH0TRIG_MASK; //enable hardware trigger 0
	
	// Don't enable interrupts yet (disable)
	FTM2_SC &= ~FTM_SC_TOIE_MASK;
	
	// No prescalar, system clock
	FTM2_SC |= FTM_SC_PS(0); //no prescalar
	FTM2_SC |= FTM_SC_CLKS(0x1);
	
	// Set up interrupt
	NVIC_EnableIRQ(PIT0_IRQn);
	
	return;
}

/* Initialization of PIT timer to control 
* 		integration period
*/
void init_PIT(void){
	// Setup periodic interrupt timer (PIT)
	
	// Enable clock for timers
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	
	// Enable timers to continue in debug mode
	PIT_MCR &= ~PIT_MCR_MDIS_MASK; //enable module
	PIT_MCR &= ~PIT_MCR_FRZ_MASK;
	
	// PIT clock frequency is the system clock
	// Load the value that the timer will count down from
	//Less than 100ms
	PIT_LDVAL0 = PIT_LDVAL_TSV(0xF4240);
	
	// Enable timer interrupts
	PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;
	
	// Enable the timer
	PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;
	
	// Clear interrupt flag
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;

	// Enable PIT interrupt in the interrupt controller
	NVIC_EnableIRQ(FTM2_IRQn);
	
	return;
}


/* Set up pins for GPIO
* 	PTB9 		- camera clk
*		PTB23		- camera SI
*		PTB22		- red LED
*/
void init_GPIO(void){
	// Enable LED and GPIO so we can see results
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	PORTB_PCR22 |= PORT_PCR_MUX(1);//RED LED
	PORTB_PCR23 |= PORT_PCR_MUX(1);//camera SI
  PORTB_PCR9 |= PORT_PCR_MUX(1);//camera clk
	GPIOB_PDDR = (1<<22)|(1<<23)|(1<<9);
	
	return;
}

/* Set up ADC for capturing camera data */
void init_ADC0(void) {
    unsigned int calib;
   // Turn on ADC0
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;
	// Single ended 16 bit conversion, no clock divider
	ADC0_SC1A &= ~ADC_SC1_DIFF_MASK; //diff 0 (single ended)
	ADC0_CFG1 |= ADC_CFG1_MODE(0x3);// 16 bit conversion
	ADC0_CFG1 |= ADC_CFG1_ADIV(0x0);//no clock divider
	ADC0_SC1A &= ~ADC_SC1_ADCH_MASK;
	ADC0_SC1A |= ADC_SC1_ADCH(0x1);
	    
    // Do ADC Calibration for Singled Ended ADC. Do not touch.
    ADC0_SC3 = ADC_SC3_CAL_MASK;
    while ( (ADC0_SC3 & ADC_SC3_CAL_MASK) != 0 );
    calib = ADC0_CLP0; calib += ADC0_CLP1; calib += ADC0_CLP2;
    calib += ADC0_CLP3; calib += ADC0_CLP4; calib += ADC0_CLPS;
    calib = calib >> 1; calib |= 0x8000;
    ADC0_PG = calib;
    
    // Select hardware trigger.
	ADC0_SC2 |= ADC_SC2_ADTRG_MASK;
	
	
	// Set up FTM2 trigger on ADC0
	SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL(0xA);// FTM2 select
	SIM_SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN_MASK;// Alternative trigger en.
	SIM_SOPT7 &= ~SIM_SOPT7_ADC0PRETRGSEL_MASK;// Pretrigger A
	
	ADC0_SC1A |= ADC_SC1_AIEN_MASK;
	// Enable NVIC interrupt
		NVIC_EnableIRQ(ADC0_IRQn);

}

void init_camera(void){
	init_GPIO(); // For CLK and SI output on GPIO
	init_FTM2(); // To generate CLK, SI, and trigger ADC
	init_ADC0();
	init_PIT();	// To trigger camera read based on integration time
}

uint16_t smoothed_line[128];
float smoother[5]={0.2,0.2,0.2,0.2,0.2};

uint16_t edge_line[128];
float edger[2]={-1,1};


void line_averager(void){
	
	for(int i =0; i<128;i++){
		float sum=0.0;	
		if (i<4){
			sum=line[i];
		}else{
		for (int j=4;j>=0;j--){
			sum += smoother[j]*line[i-j];
		}
	}
		smoothed_line[i]=sum;
	}
}

void edge_finder(void){
	for(int i =0; i<128;i++){
		int sum=0;
		if(smoothed_line[i]>((maxsmooth()-minsmooth())/2)){
			sum=1;
		}else{
			sum=0;
		}
		edge_line[i]=sum;
	}
}

int maxsmooth(void){
	int max=smoothed_line[0];
	for(int i=0; i<128;i++){
		if (smoothed_line[i]>max){
			max=smoothed_line[i];
		}
	}
	return max;
}

int minsmooth(void){
	int min=INT16_MAX;
	for(int i=0; i<128;i++){
		if (smoothed_line[i]<min){
			min=smoothed_line[i];
		}
	}
	return min;
}

int leftedge(void){
	for (int i=0;i<127;i++){
		if ((edge_line[i])^(edge_line[i+1])){
			return i;
		}
	}
}
int rightedge(void){
	for (int i=127; i>0;i--){
		if ((edge_line[i])^(edge_line[i-1])){
			return i;
		}
	}
}




