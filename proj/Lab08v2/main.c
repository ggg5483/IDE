/**
 * ******************************************************************************
 * @file    : main.c
 * @brief   : 
 * @details : 
 * 
 * @author Alexander Hamadeh
 * @author Garrett Geyer
 * @date 2/17/2026
 * ******************************************************************************
*/

#include <ti/devices/msp/msp.h>
#include "timers.h"
#include "uart_extras.h"
#include "adc12.h"
#include "uart.h"

/**
* @brief turn a 32 bit unsigned number into string decimal representation
* @param buf should be size 11 for max 32 bit number
*/
void hex_to_dec(uint32_t val, char *buf){
	//zero case
	if(val == 0){
		buf[0] = '0';
		buf[1] = '\0';
		return;
	}
	
	//intereperet from lsb
	int i = 0;
	buf[0] = '\0';
	while(val > 0){
		buf[i++] = '0' + val % 10;
		val = val/10;
	} // while
	
	//reverse string
	for(int j = 0; j <i/2; j++){
		char temp = buf[j];
		buf[j] = buf[i - 1 - j];
		buf[i - 1 - j] = temp;
	}
}

/**
 * Lab 5 – Part 2: ADC + Timer + UART
 */
int main(void)
{
    /* Initialize UART0 for printing */
    UART0_init();
		
		//TIMG0_init(0xFFFF, 0);
		TIMG6_init(16000, 0);

    /* Initialize ADC0 (channel 0 ? MEMRES0) */
    ADC0_init();
	
	  UART0_put("\e[2J\e[H");
	  UART0_put("\e[48;5;231m\e[38;5;232m"); // background white, text dark

    while (1) {
        __WFI();   /* Sleep until interrupt */
    }
}

void TIMG6_IRQHandler(void){
    /* Clear timer interrupt */
    TIMG6->CPU_INT.ICLR = GPTIMER_CPU_INT_ICLR_Z_CLR;

    /* Trigger ADC conversion and read value */
    uint32_t adcVal = ADC0_getVal();

    /* Print decimal */
    UART0_put("ADC (dec): ");
    UART0_printDec(adcVal);
    UART0_put("   \r\n");

	}
