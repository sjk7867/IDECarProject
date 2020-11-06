/*
    Title: CMPE460 Lab4 - Bluetooth Low Energy 
    Description: UART driver to send and recieve characters
    Authors: Brandon Key - Andrew Pasek
    Date: 9/16/2020
*/


#include "MK64F12.h"
#include "uart.h"         // Main serial funcitonality
#include <stdio.h>        // General funcitonality
#include <string.h>       // Useful for string operations and memset

#define CHAR_COUNT 80     // Size of a console line

void uart_init(void);

int main()
{

    // Initialize UART0 and UART3
    uart_init();
    
    // Display startup message
    
    // Declare and reset buffers and print prompts
    /* Control loop */
    while(1){
				 if ((UART0_S1 & (1<<5))){
					 int data = UART0_D;
					 uart0_putnumU((data-48)*11);
					 uart3_putnumU((data-48)*11);
				 }
				 if ((UART3_S1 & (1<<5))){
            /* NOTE: Never block in this statement. */
            //char PH[30];
            // Retrieve the character
            char data= UART3_D;
						uart0_putchar(data);
					  //uart0_put("\n");
					 
            } 
        }
    }     
