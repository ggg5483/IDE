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
	TIMA1_PWM_init(0, 3200, 0, 0.2);
	
	for(;;){
		for(int i = 0; i < 100; i++){
			TIMA0_PWM_DutyCycle(0, (((double)i)/((double)100)));
			for(volatile int _ = 0; _ < 100000; _++){};
		}
		for(int i = 0; i < 100; i++){
			TIMA0_PWM_DutyCycle(0, (((double)(100-i))/((double)100)));
			for(volatile int _ = 0; _ < 100000; _++){};
		}
	}
	return 0;
}