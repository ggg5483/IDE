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
#include "uart_extras.h"
#include "adc12.h"
#include "lab2/uart.h"
#include "lab1/leds.h"

/*
* MAIN 	- which main is used
* 	1		-	Part 1 main, basic timer and LED and buttons
* 	2		-	Part 2 main
*		3		-	Part 3 main
*/
#define MAIN (2)

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

/*
* MAIN 1 - switch interrupts interfacing with timers and LEDs
*/
#if MAIN == 1
/*global variables*/
enum LED1_state{TOGGLE, OFF, ON};
enum LED1_state LED1 = OFF;
int LED2 = 0;
unsigned long ms = 0;
int timing = 0;
char buf[11];

//turn on extra printing
#define verbose false

/*LED2_fast
* true  : LED2 changes at 1 kHz
* fasle : LED2 changes at 1/LED2_slow_ms kHz
*/
#define LED2_fast false
#define LED2_slow_ms 100

int main(void){
	S1_init_interrupt();
	S2_init_interrupt();
	//TIMG0_init(0xFFFF, 0);
	TIMG6_init(16000, 0);
	
	LED1_init();
	LED2_init();
	UART0_init();

	UART0_put("\e[2J\e[H");
	UART0_put("\e[48;5;231m\e[38;5;232m"); // background white, text dark
	
	UART0_put("Ready!\n\r");


	TIMG12_init(32);
	
	LED1_set(1);
	LED2_set(LED2_WHITE);
	


	
	
	timing = 0;
	int last_timing = 0;
	for(;;){

		//infinite loop debug printing

		//for(volatile int j = 0; j < 10000; j++){}
	
		
		if(timing==1){
			for(int i = 0; i < 11; i++) buf[i] = '\0';
			hex_to_dec(ms, buf);
			UART0_put(buf);
			UART0_put(" mS\r");
			
			last_timing = 1;
		} else {
			LED2=0;
			if (last_timing == 1){
				//edge case when interrupted in the middle of printing
				UART0_put("\x1b[2K\r"); //clear line, set sursor to start of line
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
	
	//LED2 sequence
	#if LED2_fast
	if(timing){
	#else
	if(timing && (ms%LED2_slow_ms==1)){
	#endif //LED2_fast
		switch (LED2) {
			case LED2_OFF:
				LED2 = LED2_RED;
				break;
			case LED2_RED:
				LED2 = LED2_GREEN;
				break;
			case LED2_GREEN:
				LED2 = LED2_BLUE;
				break;
			case LED2_BLUE:
				LED2 = LED2_CYAN;
				break;
			case LED2_CYAN:
				LED2 = LED2_MAGENTA;
				break;
			case LED2_MAGENTA:
				LED2 = LED2_YELLOW;
				break;
			case LED2_YELLOW:
				LED2 = LED2_WHITE;
				break;
			case LED2_WHITE:
				break;
			default:
				LED2 = LED2_WHITE;
				break;
			
		} // switch LED2
		
	} // if  timing
}
#endif //TIM_INTS

/*
* GPIO handler
*/
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
					TIMG6->COUNTERREGS.CTRCTL |= GPTIMER_CTRCTL_EN_ENABLED;
					break;
				case TOGGLE:
					LED1 = OFF;
					// disable timer
					TIMG6->COUNTERREGS.CTRCTL &= ~GPTIMER_CTRCTL_EN_ENABLED;
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
			if(ms != 0){// ms != 0 stops some buggy bouncing behaivor we have
				if(timing){ 
					uint32_t end_time = ms;
					timing = 0;
					//\x1B[?25h show cursor
					UART0_put("\x1b[38;5;232m"); // color dark
					UART0_put("\rStopping timer!\r\nYour Time was:\r\n");
					
					hex_to_dec(end_time, buf);
					UART0_put("\x1b[38;5;208m"); // color pink
					UART0_put(buf);
					UART0_put("\x1b[38;5;232m"); // color dark
					UART0_put(" mS\r\n\x1B[?25h");
					ms = 0;


				} else {
					//\x1B[?25l hide cursor
					UART0_put("Starting timer!\r\n\x1B[?25l");
					UART0_put("\x1b[38;5;22m"); //color green
					timing = 1;
					ms = 0;
				}
			
			
				break;
			default:
				break;
		} // if timing
	} //if ms
	
}
#endif // MAIN 1
/**************************************************************************************************/

#if MAIN == 2
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
    UART0_put("   ");

//    /* Print hex */
//    UART0_put("ADC (hex): ");
//    UART0_printHex(adcVal);
//    UART0_put("\r\n");
}


#endif // MAIN 2
/**************************************************************************************************/

#if MAIN == 3
int main(void){
	return 0;
}

void TIMG6_IRQHandler(void){
	
}

#endif // MAIN 3

