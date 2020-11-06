#ifndef UART_H
#define UART_H


void uart_init(void);

uint8_t uart0_getchar()
{
/* Wait until there is space for more data in the receiver buffer*/
	while(!(UART0_S1 & (1<<5))){}
		return UART0_D;

	/* Return the 8-bit data from the receiver */
}

void uart0_putchar(char ch)
{
/* Wait until transmission of previous bit is complete */

	while(!(UART0_S1 & (1<<7))){}
		UART0_D=ch;	
/* Send the character */

}

void uart0_put(char *ptr_str){
	/*use putchar to print string*/
	while(*ptr_str)
		uart0_putchar(*ptr_str++);
}

uint8_t uart3_getchar()
{
/* Wait until there is space for more data in the receiver buffer*/
	while(!(UART3_S1 & (1<<5))){}
		return UART3_D;

	/* Return the 8-bit data from the receiver */
}

void uart3_putchar(char ch)
{
/* Wait until transmission of previous bit is complete */

	while(!(UART3_S1 & (1<<7))){}
		UART3_D=ch;	
/* Send the character */

}

void uart3_put(char *ptr_str){
	/*use putchar to print string*/
	while(*ptr_str)
		uart3_putchar(*ptr_str++);
}

void uart3_putnumU(int i){
	
	while(!(UART3_S1 & (1<<7))){}
	int digit=1000000000;
	int i10 = 10*i;
	if(0==i){
		uart3_putchar(i+'0');
	}
	else{
		while(digit>i10){
			digit /=10;
		}
		while(digit /=10){
			uart3_putchar(((i/digit)%10)+'0');
		}
	}
}

void uart0_putnumU(int i){
	
	while(!(UART0_S1 & (1<<7))){}
	int digit=1000000000;
	int i10 = 10*i;
	if(0==i){
		uart0_putchar(i+'0');
	}
	else{
		while(digit>i10){
			digit /=10;
		}
		while(digit /=10){
			uart0_putchar(((i/digit)%10)+'0');
		}
	}
}



#endif
