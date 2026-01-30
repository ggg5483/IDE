/**
 * ******************************************************************************
 * @file    : main.c
 * @brief   : UART tester file
 * @details : UART stuff
 * 
 * @author Garrett Geyer 
 * @date 1/23/26
 * ******************************************************************************
*/

#include <ti/devices/msp/msp.h>
#include "uart.h"
#include "lab1/leds.h"

/*
macro to choose between main functions
1 - basic print sentence
2 - UART0 echo
3 - UART1 color input
4 - UART0 UART1 'chat room'
*/
#define MAIN 4

#if MAIN == 1
int main(void){
	UART0_init();
	UART0_put("IDE: Lab 2 Demonstration by Garrett\r\n");
	return 0;
}
#endif //main 1

#if MAIN == 2
#define BUF_SIZE 10

int main(void){
	UART0_init();
	//clear screen, set cursor top left
	//https://www.climagic.org/mirrors/VT100_Escape_Codes.html
	UART0_put("\e[2J\e[H");
	
	while(1){ 
		UART0_put("SENTENCE NOOOOOOOWWWWW!!!!!!\r\n");
		
		char sentence[BUF_SIZE+1];
		int i = 0;
		volatile char ch;
		
		while((ch = UART0_getchar()) != '\r'){
			UART0_putchar(ch);
			
			//backspace / DEL handling
			if(ch == '\b' || ch == 0x7F){
				if (i != 0) {i = i - 1;}
			} else {
				sentence[i++] = ch;
			} //endif
			
			if(i == 10) {break;}
		}		
		sentence[i] = '\0';
		
		UART0_put("\r\n");
		UART0_put(sentence);
		UART0_put("\r\n");
	} //while 1
	return 0;
}

#endif //main 2




#if MAIN == 3
int main(void){
	UART0_init();
	UART1_init();
	
	LED2_init();
	
	//clear screen, set cursor top left
	UART0_put("\e[2J\e[H");
	
	UART0_put("color application\r\n");
	UART1_put("color application\r\n");
	
	LED2_set(LED2_BLUE);
	
	while(1){
		char rec = UART1_getchar();
		UART0_put("recieved:\r\n");
		UART0_putchar(rec);
		UART0_put("\r\n");
		
		UART1_putchar(rec);
		
		switch (rec){
			case '0':
				LED2_set(LED2_OFF);
				break;
			case '1':
				LED2_set(LED2_RED);
				break;
			case '2':
				LED2_set(LED2_BLUE);
				break;
			case '3':
				LED2_set(LED2_GREEN);
				break;
			default:
				break;
		}//switch
	}//while
	
	return 0;
}

#endif //main 3

#if MAIN == 4

#define BUF_SIZE 64

int main(void){
	UART0_init();
	UART1_init();

	UART0_put("\e[2J\e[HCHAT NOW!\r\n");
	UART1_put("CHAT NOW\r\n");
	
	
	char sentenceBT[BUF_SIZE];
	char sentencePC[BUF_SIZE];
	int i = 0;
	int j = 0;
	volatile char ch;
	
	sentenceBT[0] = '\0';
	sentencePC[0] = '\0';
	
	while(1){
		
		
		if(UART0_dataAvailable()){

			
			if((ch = UART0_getchar()) != '\r'){
				UART0_putchar(ch);
				
				//backspace / DEL handling
				if(ch == '\b' || ch == 0x7F){
					if (i != 0) {i = i - 1;}
				} else {
					sentencePC[i++] = ch;
				} //endif
				
				if(i == (BUF_SIZE-1)) {break;}
			}	else {
				//add null terminator
				sentencePC[i] = '\0';
				
				//print
				UART0_put("\033[2K\r");
				UART0_put("pc>");
				UART0_put(sentencePC);
				UART0_put("\r\n");
				
				UART1_put("pc>");
				UART1_put(sentencePC);
				UART1_put("\r\n");
				
				i = 0;
			}//if
		}//UART0
		
		if(UART1_dataAvailable()){

			
			if(((ch = UART1_getchar()) != 0xD)){
				//UART1_putchar(ch);
				
				//backspace / DEL handling
				if(ch == '\b' || ch == 0x7F){
					if (j != 0) {j = j - 1;}
				} else {
					sentenceBT[j++] = ch;
				} //endif
				
				if(j == (BUF_SIZE-1)) {break;}
			}		
			else {
				//add null terminator
				sentenceBT[i] = '\0';
				
				//print
				UART0_put("phone>");
				UART0_put(sentenceBT);
				UART0_put("\r\n");
				

				UART1_put("phone>");
				UART1_put(sentenceBT);
				UART1_put("\r\n");
				
				j = 0;
			}
		}//UART1
		
	}//while loop
	
	return 0;
}
#endif //main 4
