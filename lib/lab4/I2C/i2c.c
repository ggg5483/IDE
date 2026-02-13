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
 * @author  GArrett
 * @author  Alex
 * @date 
 * ******************************************************************************
*/


#include <ti/devices/msp/msp.h> 
#include <stdint.h>
#include <ti/devices/msp/peripherals/hw_i2c.h>
#include "i2c.h"

#ifndef DEBUG
#define DEBUG false
#endif

/**
 * @brief Initialize I2C1
 * @param[in] targetAddress - Passed by OLED file. I2C target/listener address
 * @note OLED file looks for this exact prototype
*/
void I2C1_init(uint16_t targetAddress){
		//if not reset
	if (!(I2C1->GPRCM.PWREN & I2C_PWREN_ENABLE_MASK)){
		//reset
		I2C1->GPRCM.RSTCTL = I2C_RSTCTL_KEY_UNLOCK_W | I2C_RSTCTL_RESETASSERT_ASSERT | I2C_RSTCTL_RESETSTKYCLR_CLR;
		//enable
		I2C1->GPRCM.PWREN = I2C_PWREN_KEY_UNLOCK_W | I2C_PWREN_ENABLE_ENABLE;
	} //end if
	
	//config SDA, input enabled
	//IOMUX->SECCFG.PINCM[IOMUX_PINCM16] &= ~IOMUX_PINCM_PF_MASK; //clear pf
	IOMUX->SECCFG.PINCM[IOMUX_PINCM16] = 		IOMUX_PINCM_PC_CONNECTED 
																				| IOMUX_PINCM16_PF_I2C1_SDA 
																				| IOMUX_PINCM_INENA_ENABLE;
	
	//config SCL
	//IOMUX->SECCFG.PINCM[IOMUX_PINCM15] &= ~IOMUX_PINCM_PF_MASK; //clear pf
	IOMUX->SECCFG.PINCM[IOMUX_PINCM15] = 		IOMUX_PINCM_PC_CONNECTED
																				| IOMUX_PINCM15_PF_I2C1_SCL
																				| IOMUX_PINCM_INENA_ENABLE;
	
	//select BUSCLK
	I2C1->CLKSEL = I2C_CLKSEL_BUSCLK_SEL_ENABLE;
	
	//clcok division = 1
	//set to 0
	I2C1->CLKDIV = I2C_CLKDIV_RATIO_DIV_BY_1;
	
	//disable analog glitch suppresion
	I2C1->GFCTL &= ~(I2C_GFCTL_AGFEN_ENABLE);
	
	//clear controller control register
	I2C1->MASTER.MCTR = 0;
	
	//set clock timer period
	I2C1->MASTER.MTPR = 0x1F;
	
	
	//RX trigger FIFO contains >= 1 byte
	I2C1->MASTER.MFIFOCTL = I2C_MFIFOCTL_TXTRIG_EMPTY | I2C_MFIFOCTL_RXTRIG_LEVEL_1;
	// TX trigger FIFO is empty
	
	//I2C1->MASTER.MFIFOCTL |= ~I2C_MFIFOCTL_RXTRIG_MASK;
	
	//disable clock streaching
	I2C1->MASTER.MCR &= ~(I2C_MCR_CLKSTRETCH_MASK);
	
	//set target address
	//I2C1->MASTER.MCR &= ~I2C_MSA_SADDR_MASK;
	I2C1->MASTER.MCR |= (targetAddress << I2C_MSA_SADDR_OFS);
	
	//enable I2C controller
	I2C1->MASTER.MCTR |= I2C_MCR_ACTIVE_ENABLE;
	
	
	
}


/**
 * @brief Sends a single character byte over I2C1
 * @param[in] ch - Byte to send
*/
void I2C1_putchar(unsigned char ch){
	//wait until tx fifo not full
	while(!((I2C1->MASTER.MFIFOSR & I2C_MFIFOSR_TXFIFOCNT_MASK) >=  I2C_MFIFOSR_TXFIFOCNT_MINIMUM)) {
		
	} //while
		
	//transmit data
	I2C1->MASTER.MTXDATA = ch;
}


/**
 * @brief Send full character string over I2C until limit is reached
 * @param[in] data - String pointer to data to send
 * @param[in] data_size - Amount of bytes to transmit
*/
void I2C1_put(unsigned char *data, uint16_t data_size){
	
	/*
	set MSA dir transmit
	set number of bytes for transaction in MCTR
	start enable and stop enable MCTR and enable burst run.
	for loop
		put charectors
	
	wait till not idle
	turn off burst run
	*/
	
	I2C1->MASTER.MSA |= I2C_MSA_DIR_TRANSMIT;
	
	
	//I2C1->MASTER.MCTR &= ~I2C_MCTR_MBLEN_MASK;
	I2C1->MASTER.MCTR |= (((unsigned int) data_size << I2C_MCTR_MBLEN_OFS) & I2C_MCTR_MBLEN_MAXIMUM);
	I2C1->MASTER.MCTR |= I2C_MCTR_BURSTRUN_ENABLE | I2C_MCTR_START_ENABLE | I2C_MCTR_STOP_ENABLE;
	//I2C1->MASTER.MCTR |= ;
	
	for(int i = 0; i < data_size; i++){
		I2C1_putchar(data[i]);
		
	}
	//wait until idle
	while(!(I2C1->MASTER.MSR & I2C_MSR_IDLE_MASK)){
	
	}
		
	//turn off burst run
	I2C1->MASTER.MCTR &= ~(I2C_MCTR_MBLEN_MASK | I2C_MCTR_BURSTRUN_ENABLE);
		
	//direction recieve
	//I2C1->MASTER.MSA |= I2C_MSA_DIR_MASK;
	
}


