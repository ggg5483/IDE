/**
 * ******************************************************************************
 * @file    : uart.c
 * @brief   : UART module file
 * @details : UART initialization and interaction
 * 
 * @author Garrett Geyer
 * @date 1/23/26
 * ******************************************************************************
*/
#include <ti/devices/msp/msp.h> 
#include <sysctl.h>
#include "uart.h"

#ifndef DEBUG
#define DEBUG false
#endif //ifndef debug	

//these are just placeholders for the actual clock and sampling for now, to change these you need to modify the code
#define SAMPLING 16
#define CLK_RATE 32000000

//this will actually change the baud rate
#define BAUD_RATE 9600

/**
 * @brief Initialize UART0
*/
void UART0_init(void){
	//if not reset
	if (UART0->GPRCM.STAT & UART_GPRCM_STAT_RESETSTKY_MASK){
		//reset
		UART0->GPRCM.RSTCTL = UART_RSTCTL_KEY_UNLOCK_W | UART_RSTCTL_RESETASSERT_ASSERT;
		//enable
		UART0->GPRCM.PWREN = UART_PWREN_KEY_UNLOCK_W | UART_PWREN_ENABLE_ENABLE;
	} //end if
	
	//config IOMUX RX, input enabled
	IOMUX->SECCFG.PINCM[IOMUX_PINCM22] &= ~IOMUX_PINCM_PF_MASK; //clear pf
	IOMUX->SECCFG.PINCM[IOMUX_PINCM22] = 		IOMUX_PINCM_PC_CONNECTED 
																				| IOMUX_PINCM22_PF_UART0_RX 
																				| IOMUX_PINCM_INENA_ENABLE;
	
	//config IOMUX TX
	IOMUX->SECCFG.PINCM[IOMUX_PINCM21] &= ~IOMUX_PINCM_PF_MASK; //clear pf
	IOMUX->SECCFG.PINCM[IOMUX_PINCM21] = 		IOMUX_PINCM_PC_CONNECTED
																				| IOMUX_PINCM21_PF_UART0_TX;
	
	//select BUSCLK
	UART0->CLKSEL = UART_CLKSEL_BUSCLK_SEL_ENABLE;
	
	//clcok division = 1
	//set to 0
	UART0->CLKDIV = UART_CLKDIV_RATIO_DIV_BY_1;
	
	//disable UART clear ENABLE bit of CTL0
	UART0->CTL0 &= ~UART_CTL0_ENABLE_ENABLE; 
	
	//set oversampling to 16x
	//clear both HSE bits
	UART0->CTL0 &= ~UART_CTL0_HSE_MASK;
	
	//set the Transmit Enable, Receive Enable, and FIFO enable
	UART0->CTL0 |= UART_CTL0_TXE_ENABLE | UART_CTL0_RXE_ENABLE | UART_CTL0_FEN_ENABLE;
	
	//set baud rate 9600
	//BRD = UART Clock / (Oversampling x Baud rate) 
	float clk = CLK_RATE;
	clk = clk /(SAMPLING*BAUD_RATE);
	UART0->IBRD = (int) clk;
	
	clk = clk - (int) clk;
	clk = (clk * 64) + 0.5;
	UART0->FBRD = (int) clk;
	
	//8 data 1 stop 0 parity
	UART0->LCRH |= UART_LCRH_WLEN_DATABIT8;
	UART0->LCRH &= ~UART_LCRH_STP2_ENABLE;
	UART0->LCRH &= ~UART_LCRH_PEN_ENABLE;
	
	
	// enable UART enable ENABLE bit of CTL0
	UART0->CTL0 |= UART_CTL0_ENABLE_ENABLE; 
}


/**
 * @brief Put a character over UART0
 * @param[in] ch - Character to print
*/
void UART0_putchar(char ch){
	//wait untill fifo not full
	while((UART0->STAT & UART_STAT_TXFF_MASK)) {
		#if DEBUG
		volatile int testing = UART0->STAT;
		testing = UART0->STAT & UART_STAT_TXFF_MASK;
		#endif //debug
	} //while
		
	//transmit data
	UART0->TXDATA = ch;
	
}


/**
 * @brief Retrieve a single character from UART0
*/
char UART0_getchar(void){
	while((UART0->STAT & UART_STAT_RXFE_MASK)){
		#if DEBUG
		volatile int testing = UART0->STAT;
		testing = testing & UART_STAT_RXFE_MASK;
		#endif //DEBUG
	} //while
	
	return UART0->RXDATA & UART_RXDATA_DATA_MASK;

}

/**
* @brief checks if data is available
*/
int UART0_dataAvailable(void){
	return ((UART0->STAT & UART_STAT_RXFE_MASK) == 0 ? 1 : 0);
}

/**
 * @brief Send a full character string over UART0 terminates at NULL char
 * @param[in] ptr_str - Pointer to the string to print
*/
void UART0_put(char *ptr_str){
	int i = 0;
	char ch = ptr_str[i++];
	while(ch != '\0'){
		UART0_putchar(ch);
		ch = ptr_str[i++];
	}
}

/**
 * @brief Initialize UART1
*/
void UART1_init(void){
	//if not reset
	if (UART1->GPRCM.STAT & UART_GPRCM_STAT_RESETSTKY_MASK){
		//reset
		UART1->GPRCM.RSTCTL = UART_RSTCTL_KEY_UNLOCK_W | UART_RSTCTL_RESETASSERT_ASSERT;
		//enable
		UART1->GPRCM.PWREN = UART_PWREN_KEY_UNLOCK_W | UART_PWREN_ENABLE_ENABLE;
	} //end if
	
	//config IOMUX RX, input enabled
	IOMUX->SECCFG.PINCM[IOMUX_PINCM20] &= ~IOMUX_PINCM_PF_MASK; //clear pf
	IOMUX->SECCFG.PINCM[IOMUX_PINCM20] = 		IOMUX_PINCM_PC_CONNECTED 
																				| IOMUX_PINCM20_PF_UART1_RX 
																				| IOMUX_PINCM_INENA_ENABLE;
	
	//config IOMUX TX
	IOMUX->SECCFG.PINCM[IOMUX_PINCM19] &= ~IOMUX_PINCM_PF_MASK; //clear pf
	IOMUX->SECCFG.PINCM[IOMUX_PINCM19] = 		IOMUX_PINCM_PC_CONNECTED
																				| IOMUX_PINCM19_PF_UART1_TX;
	
	//select BUSCLK
	UART1->CLKSEL = UART_CLKSEL_BUSCLK_SEL_ENABLE;
	
	//clcok division = 1
	//set to 0
	UART1->CLKDIV = UART_CLKDIV_RATIO_DIV_BY_1;
	
	//disable UART clear ENABLE bit of CTL0
	UART1->CTL0 &= ~UART_CTL0_ENABLE_ENABLE; 
	
	//set oversampling to 16x
	//clear both HSE bits
	UART1->CTL0 &= ~UART_CTL0_HSE_MASK;
	
	//set the Transmit Enable, Receive Enable, and FIFO enable
	UART1->CTL0 |= UART_CTL0_TXE_ENABLE | UART_CTL0_RXE_ENABLE | UART_CTL0_FEN_ENABLE;
	
	//set baud rate 9600
	//BRD = UART Clock / (Oversampling x Baud rate) 
	float clk = CLK_RATE;
	clk = clk /(SAMPLING*BAUD_RATE);
	UART1->IBRD = (int) clk;
	
	clk = clk - (int) clk;
	clk = (clk * 64) + 0.5;
	UART1->FBRD = (int) clk;
	
	//8 data 1 stop 0 parity
	UART1->LCRH |= UART_LCRH_WLEN_DATABIT8;
	UART1->LCRH &= ~UART_LCRH_STP2_ENABLE;
	UART1->LCRH &= ~UART_LCRH_PEN_ENABLE;
	
	
	// enable UART enable ENABLE bit of CTL0
	UART1->CTL0 |= UART_CTL0_ENABLE_ENABLE; 
}


/**
 * @brief Put a character over UART1
 * @param[in] ch - Character to print
*/
void UART1_putchar(char ch){
	//wait untill fifo not full
	while((UART1->STAT & UART_STAT_TXFF_MASK)) {
		#if DEBUG
		volatile int testing = UART1->STAT;
		testing = UART1->STAT & UART_STAT_TXFF_MASK;
		#endif //debug
	} //while
		
	//transmit data
	UART1->TXDATA = ch;
	
}


/**
 * @brief Retrieve a single character from UART1
*/
char UART1_getchar(void){
	while((UART1->STAT & UART_STAT_RXFE_MASK)){
		#if DEBUG
		volatile int testing = UART1->STAT;
		testing = testing & UART_STAT_RXFE_MASK;
		#endif //DEBUG
	} //while
	
	return UART1->RXDATA & UART_RXDATA_DATA_MASK;

}

/**
* @brief checks if data is available
*/
int UART1_dataAvailable(void){
	return ((UART1->STAT & UART_STAT_RXFE_MASK) == 0 ? 1 : 0);
}

/**
 * @brief Send a full character string over UART1 terminates at NULL char
 * @param[in] ptr_str - Pointer to the string to print
*/
void UART1_put(char *ptr_str){
	int i = 0;
	char ch = ptr_str[i++];
	while(ch != '\0'){
		UART1_putchar(ch);
		ch = ptr_str[i++];
	}
}

