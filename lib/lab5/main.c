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
enum LED1_state{TOGGLE, OFF, ON};
enum LED1_state LED1 = OFF;
int LED2 = 0;
unsigned long ms = 0;
int timing = 0;
char buf[11];

//turn off extra printing
#define verbose false

	

/**
 * @brief turn a hex byte into ASCII value
*/
static int hex_to_ascii(uint8_t c) {
    if (c >= 0 && c <= 9) return '0' + c;
    if (c >= 0xa && c <= 0xf) return 'a' - 0xa + c;
    return -1; // Error indicator
}

void UART0_puthex(uint32_t val){
		UART0_putchar(hex_to_ascii((val >> 28) & 0xF));
		UART0_putchar(hex_to_ascii((val >> 24) & 0xF));
		UART0_putchar(hex_to_ascii((val >> 20) & 0xF));
		UART0_putchar(hex_to_ascii((val >> 16) & 0xF));
		UART0_putchar(hex_to_ascii((val >> 12) & 0xF));
		UART0_putchar(hex_to_ascii((val >> 8)  & 0xF));
		UART0_putchar(hex_to_ascii((val >> 4)  & 0xF));
		UART0_putchar(hex_to_ascii((val >> 0)  & 0xF));
}

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



int main(void){
	S1_init_interrupt();
	S2_init_interrupt();
	//TIMG0_init(0xFFFF, 0);
	TIMG6_init(16000, 0);
	
	LED1_init();
	LED2_init();
	UART0_init();

	UART0_put("\e[2J\e[H");
	UART0_put("Ready!\n\r");

	TIMG12_init(32);
	
	LED1_set(1);
	LED2_set(LED2_WHITE);
	


	
	

	int last_timing = 0;
	for(;;){

		//infinite loop debug printing

		for(volatile int j = 0; j < 10000; j++){}
	
		
		if(timing==1){
			for(int i = 0; i < 11; i++) buf[i] = '\0';
			hex_to_dec(ms, buf);
			UART0_put(buf);
			UART0_put("\r");
			if(LED2 != 0x7) LED2++;
			last_timing = 1;
		} else {
			LED2=0;
			if (last_timing == 1){
				//edge case when interrupted in the middle of printing
				UART0_put("\e[2k");
			}
			last_timing = 0;
		}
		LED2_set(LED2);
	}
	return 0;
}

#ifndef TIM_INTS
	#define TIM_INTS true
#endif // TIM_INTS
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
		case ON:
			LED1_set(1);
			break;
		default:
			LED1_set(0);
			break;
	}
}


void TIMG12_IRQHandler(void){
	//with clock set to 1 kHz this should be mS
	ms++;
	
}
#endif //TIM_INTS


void GROUP1_IRQHandler(void){
	/*handle interrupt*/
	switch(GPIOA->CPU_INT.IIDX){
		case GPIO_CPU_INT_IIDX_STAT_DIO18:
			/*SW1*/
			/*clear interrupt*/
			GPIOA->CPU_INT.ICLR |= (1 << 18);
		
			#if verbose
			UART0_put("SWITCH 1 PRESSED !!!!!\r\n");
			#endif // verbose
		

		
			//switch to toggle LED1 variable state
			switch(LED1){
				case OFF:
					LED1 = TOGGLE;
					// enable timer
					TIMG12->COUNTERREGS.CTRCTL |= GPTIMER_CTRCTL_EN_ENABLED;
					break;
				case TOGGLE:
					LED1 = OFF;
					// disable timer
					TIMG12->COUNTERREGS.CTRCTL &= ~GPTIMER_CTRCTL_EN_ENABLED;
					LED1_set(0);
					break;
				default:
					break;
			} // switch LED1
			break;
			
		default:
			break;
	} // switch IIDX
	
	switch(GPIOB->CPU_INT.IIDX){
		case GPIO_CPU_INT_IIDX_STAT_DIO21:
			/*SW2*/
			/*clear interrupt*/
			GPIOB->CPU_INT.ICLR |= (1 << 21);
			#if verbose
			UART0_put("SWITCH 2 RELEASED !!!!!!\r\n");
			#endif // verbose
			
			if(timing){
				uint32_t end_time = ms;
				timing = 0;
				//\x1B[?25h show cursor
				UART0_put("\rStopping timer!\r\nYour Time was:\r\n");
				hex_to_dec(end_time, buf);
				UART0_put(buf);
				UART0_put(" mS\r\n\x1B[?25h");


			} else {
				//\x1B[?25l hide cursor
				UART0_put("Starting timer!\r\n\x1B[?25l");
				timing = 1;
				ms = 0;
			}
		
		
			break;
		default:
			break;
	}
	
}

