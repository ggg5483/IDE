/**
 * ******************************************************************************
 * @file    : i2c.c
 * @brief   : I2C module header file
 * @details : I2C initialization and interaction
 * @note    : TI documentation switches terminology around, code is behind
 *             Controller <-> Master
 *             Target     <-> Slave
 *             This changes register access from what the documentation specifies
 *
 * @author 
 * @date 
 * ******************************************************************************
*/


#include <ti/devices/msp/msp.h> 
#include <stdint.h>
#include "i2c.h"

/**
 * @brief Initialize I2C1
 * @param[in] targetAddress - Passed by OLED file. I2C target/listener address
 * @note OLED file looks for this exact prototype
*/
void I2C1_init(uint16_t targetAddress){
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
 * @brief Sends a single character byte over I2C1
 * @param[in] ch - Byte to send
*/
void I2C1_putchar(unsigned char ch){
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
 * @brief Send full character string over I2C until limit is reached
 * @param[in] data - String pointer to data to send
 * @param[in] data_size - Amount of bytes to transmit
*/
void I2C1_put(unsigned char *data, uint16_t data_size){
	int i = 0;
	char ch = data[i++];
	while(ch != '\0'){
		I2C1_putchar(ch);
		ch = data[i++];
		if (i = data_size){break;}//this may need testing
	}
}


