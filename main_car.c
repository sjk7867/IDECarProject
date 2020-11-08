/*
    Title: CMPE460 Car - Main Control
    Description: Does stuff
    Authors: Andrew Pasek
    Date: 10/26/2020
*/

#include "MK64F12.h"
#include "uart.h"         // Main serial functionality
#include "PWM.h"					// Main motor/servo functionality
//#include "camera.h"				// Main Camera
#include <stdio.h>        // General funcitonality
#include <string.h>       // Useful for string operations and memset



int maxsmooth(void);
int minsmooth(void);


uint16_t ADC0VAL;

void ADC_IRQHandler(void);
void FTM2_IRQHandler(void);
void PIT0_IRQHandler(void);
void init_FTM2(void);
void init_PIT(void);
void init_GPIO(void);
void init_ADC0(void);
void init_camera(void);
void getline(uint16_t *newline);
void line_averager(void);
void edge_finder(void);
int maxsmooth(void);
int minsmooth(void);
int leftedge(void);
int rightedge(void);
int maxedge(void);
int minedge(void);

float turncenter=5.8;
float turnright=7.4;
float turnleft=4.3;

//BLE TX=PTB10=P6
//BLE RX=PTB11=P5

//pos=100 centered

//pos>100 left side

//pos<100 right side

/*void line_averager(void);
void edge_finder(void);
int maxsmooth(void);
int minsmooth(void);

uint16_t smoothed_line[128];
float smoother[5]={0.2,0.2,0.2,0.2,0.2};

uint16_t edge_line[128];
float edger[2]={-1,1};

//uint16_t* line_m;
uint16_t line_m[128];

char str_m[100];
int capcnt_m;*/
void init(void){
	uart_init();
	//init_camera();
	init_motors();
}


// Default System clock value
// period = 1/20485760  = 4.8814395e-8
#define DEFAULT_SYSTEM_CLOCK 20485760u 
// Integration time (seconds)
// Determines how high the camera values are
// Don't exceed 100ms or the caps will saturate
// Must be above 1.25 ms based on camera clk 
//	(camera clk is the mod value set in FTM2)
#define INTEGRATION_TIME .0075f

void init_FTM2(void);
void init_GPIO(void);
void init_PIT(void);
void init_ADC0(void);
void FTM2_IRQHandler(void);
void PIT1_IRQHandler(void);
void ADC0_IRQHandler(void);
void line_averager(void);
void edge_finder(void);
int maxsmooth(void);
int minsmooth(void);


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


uint16_t smoothed_line[128];
float smoother[5]={0.2,0.2,0.2,0.2,0.2};

float edge_line[128];
float edger[3]={1,0,-1};


uint16_t	threshold=30;
//uint16_t middle=64;

// ADC0VAL holds the current ADC value
uint16_t ADC0VAL;

int left;
int right;


int maxsmoth;
int minsmoth;

int setmid=64;

int main(void)
{
	int i;
	init();
	init_GPIO(); // For CLK and SI output on GPIO
	init_FTM2(); // To generate CLK, SI, and trigger ADC
	init_ADC0();
	init_PIT();	// To trigger camera read based on integration time
				FTM3_set_duty_cycle(5.8,50,1);

	for(;;) {


	
		uart0_put("LEFT:");
		uart0_putnumU(left);
		uart0_put("RIGHT:");
		uart0_putnumU(right);
		
		int pos=((right-left)/2)+left;
		uart0_put("pos:");
		uart0_putnumU(pos);
		uart3_put("pos:");
		uart3_putnumU(pos);
		uart3_put("\n");
		uart0_putnumU(maxsmoth);
	/*	if ((maxsmoth<30000)&&(maxsmoth>1000)){
			FTM0_set_duty_cycle(0,10000,1,1);
			FTM0_set_duty_cycle(0,10000,1,0);
			uart3_put("BREAK");
			uart0_put("BREAK");
			break;
		}*/
		FTM0_set_duty_cycle(30,10000,1,1);
		FTM0_set_duty_cycle(30,10000,1,0);
		
		if (pos<62){
		//	uart3_put("right");
			int turnammount=((64-pos)/2);
			uart0_put("right");
			FTM3_set_duty_cycle((turncenter+0.2*turnammount),50,1);
		}else if(pos>66){
		//	uart3_put("left");
			int turnammount=((pos-64)/2);
			uart0_put("left");
			FTM3_set_duty_cycle(turncenter-0.2*turnammount,50,1);
		}else{
			//uart3_put("straight");
			uart0_put("straight");
			FTM3_set_duty_cycle(5.8,50,1);
		}
		
			uart0_put("\n\r");
		/*
			line_averager();
			edge_finder();
		int max=maxsmooth();
		int min=minsmooth();

		int left=leftedge();
		int right=rightedge();
		int middle=(((right-left)/2));
		//uart3_put("driving");
		//uart3_put("max:");
		//uart3_putnumU(max);
		//uart3_put("min:");
		//uart3_putnumU(min);
		uart3_put("Left:");
		uart3_putnumU(left);
		uart3_put("right:");
		uart3_putnumU(right);
		uart0_put("max:");
		uart0_putnumU(max);
		uart0_put("min:");
		uart0_putnumU(min);
		//uart0_put("right:");
		//uart0_putnumU(right);
		
		if ((max<17500)&&(max>0)){
			FTM0_set_duty_cycle(0,10000,1,1);
			FTM0_set_duty_cycle(0,10000,1,0);
			uart3_put("BREAK");
			uart0_put("BREAK");
			break;
		}
		//uart3_put("middle:");
		uart0_put("middle:");
		if(middle>0){
		uart3_putnumU(middle);
		uart0_putnumU(middle);
		FTM0_set_duty_cycle(30,10000,1,1);
		FTM0_set_duty_cycle(30,10000,1,0);
		if (middle>70){
			//uart3_put("right");
			uart0_put("right");
			FTM3_set_duty_cycle(7.0,50,1);
		}else if(middle<60){
			//uart3_put("left");
			uart0_put("left");
			FTM3_set_duty_cycle(4.5,50,1);
		}else{
			//uart3_put("straight");
			uart0_put("straight");
			FTM3_set_duty_cycle(5.8,50,1);
		}
	}
				uart3_put("\n\r");
				uart0_put("\n\r");
*/
/*
		if (debugcamdata) {

			// Every 2 seconds
			//if (capcnt >= (2/INTEGRATION_TIME)) {
			if (capcnt >= (150)) {
				GPIOB_PCOR |= (1 << 22);
				// send the array over uart
				sprintf(str,"%i\n\r",-1); // start value
				uart0_put(str);
				for (i = 0; i < 127; i++) {
					sprintf(str,"%f\n", (float)edge_line[i]);
					uart0_put(str);
				}
				sprintf(str,"%i\n\r",-2); // end value
				uart0_put(str);
				capcnt = 0;
				GPIOB_PSOR |= (1 << 22);
			}
		}*/
			/*for (int i=0; i<128;i++){
				if (edge_line[i]==1){
					uart0_putnumU(i);
					uart_put("\n\r");
				}
			}*/
		
	} //for
} //main


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
		
		//FULL line
		line_averager();
		edge_finder();
		
		
		//EDGES
	left=maxedge();	
	right=minedge();
		
		//MIN/MAX
		maxsmoth=maxsmooth();
		minsmoth=minsmooth();
		
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
void init_FTM2(){
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
	ADC0_SC1A |= ADC_SC1_ADCH(0x0);
	    
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




void line_averager(void){
	
	for(int i =0; i<128;i++){
		float sum=0.0;	
		if (i<3){
			sum=line[i];
		}else if(i>125){
			sum=line[i];
		}else{
		for (int j=4;j>=0;j--){
			sum += smoother[j]*line[(2+i)-j];
		}
	}
		smoothed_line[i]=sum;
	}
}

void edge_finder(void){
	for(int i =0; i<128;i++){
		float sum=0.0;	
		if (i<1){
			sum=smoothed_line[i];
		}else{
			//if i>125 pass through
		for (int j=2;j>=0;j--){
			sum += edger[j]*smoothed_line[(1+i)-j];
		}
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


//LEFT
int maxedge(void){
	int max=0;
	int j=10;
	for(int i=10; i<128;i++){
		if (edge_line[i]>max){
			max=edge_line[i];
			j=i;
		}
	}
	return j;
}

//RIGHT
int minedge(void){
	float min=INT16_MAX;
	int j=10;
	for(int i=10; i<120;i++){
		if (edge_line[i]<min){
			min=edge_line[i];
			j=i;
		}
	}
	return j;
}


/*int leftedge(void){
	for(int i=0;i<127;i++){
		if(edge_line[i]^edge_line[i+1]){
			return i;
		}
	}
}
int rightedge(void){
	for(int i=127;i>0;i--){
		if(edge_line[i]^edge_line[i-1]){
			return i;
		}
	}
}*/