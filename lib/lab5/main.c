/**
 * ******************************************************************************
 * @file    : main.c
 * @brief   : 
 * @details : 
 * 
 * @author Alex Hamadeh
 * @author Garrett Geyer
 * @date 2/17/2026
 * ******************************************************************************
*/

#include <ti/devices/msp/msp.h>
#include "timers.h"
#include "switches.h"
#include "lab2/uart.h"
#include "lab1/leds.h"

/*global LED states*/
enum LED1_state{TOGGLE, OFF};
enum LED1_state LED1 = OFF;
int LED2 = 0;
unsigned long ms = 0;

static int hex_to_ascii(unsigned char c) {
    if (c >= 0 && c <= 9) return '0' + c;
    if (c >= 0xa && c <= 0xf) return 'a' - 0xa + c;
    return -1; // Error indicator
}

int main(void){
	S1_init_interrupt();
	S2_init_interrupt();
	//TIMG0_init(0xFFFF, 0);
	TIMG6_init(64000, 0);
	TIMG12_init(0xFFFF);
	LED1_init();
	LED2_init();
	UART0_init();
	UART0_put("\e[2J\e[H");
	UART0_put("DOING TIMER SHANANEGANERIES!!!!\n\r");
	
	LED1_set(1);
	LED2_set(LED2_WHITE);

	for(;;){
		int cnt_val = TIMG6->COUNTERREGS.CTR;
		UART0_putchar(hex_to_ascii((cnt_val >> 12) & 0xF));
		UART0_putchar(hex_to_ascii((cnt_val >> 8) & 0xF));
		UART0_putchar(hex_to_ascii((cnt_val >> 4) & 0xF));
		UART0_putchar(hex_to_ascii((cnt_val >> 0) & 0xF));
		UART0_put("\n\r");
		//for(volatile int j = 0; j < 100; j++){}
	}
	return 0;
}

#ifndef TIM_INTS
	#define TIM_INTS true
#endif
#if TIM_INTS
void TIMG0_IRQHandler(void){
	
}

void TIMG6_IRQHandler(void){
	switch(LED1){
		case OFF:
			LED1_set(0);
			break;
		case TOGGLE:
			LED1_toggle();
			break;
		default:
			LED1_set(0);
			break;
	}
}


void TIMG12_IRQHandler(void){
	ms++;
	if(LED2 == 0x7){
		LED2 = 0;
	} else {
		LED2++;
	}
	LED2_set(LED2);
}

#endif //TIM_INTS
void GROUP1_IRQHandler(void){
	/*clear interrupt*/
	
	/*handle interrupt*/
	switch(GPIOA->CPU_INT.IIDX){
		case GPIO_CPU_INT_IIDX_STAT_DIO18:
			/*SW1*/
			/*clear interrupt*/
			GPIOA->CPU_INT.ICLR |= (1 << 18);
		

		
			//switch LED1 state
			switch(LED1){
				case OFF:
					LED1 = TOGGLE;
					break;
				case TOGGLE:
					LED1 = OFF;
					break;
				default:
					break;
			}
			break;
			
		default:
			break;
	}
	
	switch(GPIOB->CPU_INT.IIDX){
		case GPIO_CPU_INT_IIDX_STAT_DIO21:
			/*SW2*/
			/*clear interrupt*/
			GPIOB->CPU_INT.ICLR |= (1 << 21);
		

		
		
			break;
		default:
			break;
	}
	
}

