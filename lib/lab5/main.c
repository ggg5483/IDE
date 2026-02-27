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
#include "camera.h"

/*
* MAIN 	- which main is used
* 	1		-	Part 1 main, basic timer and LED and buttons
* 	2		-	Part 2 main, Analog to Digital Converter
*		3		-	Part 3 main, DE Car Camera
*/
#define MAIN (3)

/*
* COMP 	- which hardware is being used in Part 2 ADC
* 	1		-	Part 1, Photocell output
* 	2		-	Part 2, TMP36 temperature sensor output
*/
#define COMP (1)

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

	UART0_put("\e[2J\e[H");									//CLEAR screen
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

/* Photocell output (1) */
#if COMP == 1
    /* Print decimal */
    UART0_put("ADC (dec): ");
    UART0_printDec(adcVal);
    UART0_put("   \r\n");

    /* Print hex */
    UART0_put("ADC (hex): 0x");

    const char hexDigits[] = "0123456789ABCDEF";
		
		/* 12 bit to hex val conversion */
    UART0_putchar(hexDigits[(adcVal >> 8) & 0xF]);
    UART0_putchar(hexDigits[(adcVal >> 4) & 0xF]);
    UART0_putchar(hexDigits[(adcVal >> 0) & 0xF]);

    UART0_put("   \r\n");
		#endif // COMP 1
	
/* TMP36 temperature sensor output (2) */	
#if COMP == 2		
    /* Convert ADC to millivolts */
    double voltage_mV = ((double)adcVal * 3300.0) / 4095.0;

    /* TMP36: Convert mV to °C */
    double tempC = (voltage_mV - 500.0) / 10.0;

    /* Convert °C to °F */
    double tempF = (tempC * 9.0 / 5.0) + 32.0;

		/* Print C */
    UART0_put("ADC (C) = ");
    UART0_printFloat(tempC);
		UART0_put("\r\n");

    /* Print F */
    UART0_put("ADC (F) = ");
    UART0_printFloat(tempF);
    UART0_put("\r\n");
		#endif // COMP 2
}


#endif // MAIN 2
/**************************************************************************************************/

#if MAIN == 3
/* Local camera data */
static volatile uint16_t cameraData[128];
static volatile uint16_t pixelCounter = 0;
static volatile uint8_t  cameraData_complete = 0;

int main(void)
{
    UART0_init();
    Camera_init();
	

    
		UART0_put("\e[48;5;231m\e[38;5;232m"); // background white, text dark
		UART0_put("\e[2J\e[H"); // clear screen
		UART0_put("Camera ready\r\n");
    while (1) {
				
				
        if (cameraData_complete) {
            volatile uint16_t *data = cameraData;
            UART0_put("-1\r\n");

            for (int i = 0; i < 128; i++) {
                UART0_printDec(data[i]);
                UART0_put("\r\n");
            }
            UART0_put("-2\r\n");
						//UART0_put("\e[2J\e[H"); // clear screen
						
						cameraData_complete = 0;
						
						
        }
    }
}

/**
 * @brief TIMG6 ISR – controls SI pulse and integration time
*/
void TIMG6_IRQHandler(void)
{
    /* Clear interrupt */
    //TIMG6->CPU_INT.ICLR = GPTIMER_CPU_INT_ICLR_Z_CLR;

    /* Ensure CLK disabled */
    TIMG0->COUNTERREGS.CTRCTL &= ~GPTIMER_CTRCTL_EN_ENABLED;

    /* start new frame*/
		cameraData_complete = 0;
	
    pixelCounter = 0U;

    /* Start capture sequence */
    GPIOA->DOUTCLR31_0 = CAM_CLK_MASK;
    GPIOA->DOUTSET31_0 = CAM_SI_MASK;

    GPIOA->DOUTSET31_0 = CAM_CLK_MASK;
    GPIOA->DOUTCLR31_0 = CAM_CLK_MASK;

    GPIOA->DOUTCLR31_0 = CAM_SI_MASK;

    /* Enable CLK timer */
    TIMG0->COUNTERREGS.CTRCTL |= GPTIMER_CTRCTL_EN_ENABLED;
}


/**
 * @brief TIMG0 ISR – drives CLK and samples ADC
*/
void TIMG0_IRQHandler(void)
{
    /* Clear interrupt */
    //TIMG0->CPU_INT.ICLR = GPTIMER_CPU_INT_ICLR_Z_CLR;

		#if 1
    /* Pulse CLK */
    GPIOA->DOUTSET31_0 = CAM_CLK_MASK;

    /* Sample ADC */
    uint32_t adcVal = ADC0_getVal();

    if (pixelCounter < 128U) {
        cameraData[pixelCounter++] = (uint16_t)adcVal;
    }

    GPIOA->DOUTCLR31_0 = CAM_CLK_MASK;

    /* End of frame */
    if (pixelCounter >= 128U) {
        cameraData_complete = 1U;
        pixelCounter = 0U;

        /* Disable CLK */
        TIMG0->COUNTERREGS.CTRCTL &= ~GPTIMER_CTRCTL_EN_ENABLED;
    }
		#else
		GPIOA->DOUTTGL31_0 = CAM_CLK_MASK;
		GPIOA->DOUTTGL31_0 = CAM_SI_MASK;
		#endif
}

#endif // MAIN 3

