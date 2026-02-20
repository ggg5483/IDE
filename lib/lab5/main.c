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


#include "timers.h"
#include "switches.h"
//#include "uart.c"
#include "lab1/leds.h"



int main(void){
	S1_init_interrupt();
	S2_init_interrupt();
	TIMG0_init(0xFFFF, 0);
	TIMG6_init(0x00FF, 0);
	TIMG12_init(0xFFFF);
	LED1_init();
	LED2_init();

	

	for(;;){}
	return 0;
}

void TIMG0_IRQHandler(void){
	
}
void TIMG6_IRQHandler(void){
	
}
void TIMG12_IRQHandler(void){
}
void GROUP1_IRQHandler(void){
}

