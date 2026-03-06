/**
 * ******************************************************************************
 * @file    : main.c
 * @brief   : Does motors
 * @details : Yay motors
 * 
 * @author Alex Hamadeh
 * @author Garrett Geyer
 * @date 3/3/26
 * ******************************************************************************
*/

#include <ti/devices/msp/msp.h>
#include "lab6/timers.h"
#include "lab2/uart.h"
#include "uart_extras.h"

int main(void){
	UART0_init();
	TIMA0_PWM_init(0, 3200, 0, 0.2);
	
	for(;;){
		UART0_printFloat(TIMA0->COUNTERREGS.CTR);
		UART0_put("\r\n");
	}
	return 0;
}