/**
 * ******************************************************************************
 * @file    : main.c
 * @brief   : main code file
 * @details : Switches and leds and stuff
 * 
 * @author  Garrett Geyer
 * @date 		1/16/26
 * ******************************************************************************
*/


#include <ti/devices/msp/msp.h> 
#include "leds.h"
#include "switches.h"

int main(void){
	
	//initialize
	LED1_init();
	LED2_init();
	S1_init();
	S2_init();
	
	LED1_set(0);
	LED2_set(0x0);

	
	
	

	#if 1 //debug
	
	int times_led = 0;
	
	while(true){

		
		LED1_set(S1_pressed());
		
		if (S2_pressed()){
			
			LED2_set(LED2_YELLOW);
			while(S2_pressed()){};
				
			switch (times_led) {

				case 0:	
					LED2_set(LED2_RED);
					for(volatile long i = 0;i <2000000; i+=1 ){}
					LED2_set(LED2_GREEN);
					for(volatile long i = 0;i <2000000; i+=1 ){}
					LED2_set(LED2_BLUE);
					for(volatile long i = 0;i <2000000; i+=1 ){}
					times_led += 1;
					break;
				case 1:
					LED2_set(LED2_CYAN);
					for(volatile long i = 0;i <2000000; i+=1 ){}
					LED2_set(LED2_MAGENTA);
					for(volatile long i = 0;i <2000000; i+=1 ){}
					LED2_set(LED2_YELLOW);
					for(volatile long i = 0;i <2000000; i+=1 ){}
					times_led += 1;
					break;
				case 2:
					LED2_set(LED2_WHITE);
					for(volatile long i = 0;i <2000000; i+=1 ){}
					times_led = 0;
					break;
				default:
					times_led = 0;
					break;
					
				}//switch
		} else {
			LED2_set(0x0);
		}//if sw_2
	
	}//while true
	#endif //debug
	
	
	return 0;
}

void hold_on(void){
	for(volatile long i = 0;i <2000000; i+=1 ){}
	}
