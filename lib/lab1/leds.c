
 /* ******************************************************************************
 * @file    : leds.c
 * @brief   : LEDs module  file
 * @details : LED initialization and interaction
 * 
 * @author Garrett Geyer
 * @date 1/16/26
 * ******************************************************************************
*/


#include <ti/devices/msp/msp.h> 
#include "leds.h"

/**
 * @brief Initialze LED1
 * @hint You might want to check out the schematics in the MSP User Guide
 *       The IOMUX has a hardware inversion bit
*/
void LED1_init(void){
	
	

	
	
	//check if enabled first
			/*	0h = The peripheral has not been reset since this bit was last cleared
					by RESETSTKYCLR in the RSTCTL register
					1h = The peripheral was reset since the last bit clear
			*/
	
	
	if (!(GPIOA->GPRCM.STAT & GPIO_STAT_RESETSTKY_MASK)){
	
			//reset then enable power for gpio A
		GPIOA->GPRCM.RSTCTL = (GPIO_RSTCTL_KEY_UNLOCK_W | GPIO_RSTCTL_RESETASSERT_ASSERT);
		//GPIOA->GPRCM.RSTCTL = (0xb1000000 | 0x02 | 0x01);

		
	}//if GPIOA RESET

	
	//enable
	GPIOA->GPRCM.PWREN = GPIO_PWREN_KEY_UNLOCK_W | GPIO_PWREN_ENABLE_ENABLE;
	//key and enable

	
	//GPIOx is enabled, set individual pin
	
	/*
	PINCMX
	
	LED1 1
	LED2 	R 57
				G 58
				B	50
	*/
	//pulldown and io direction
	
	/*
	use I/O bit 7
	Peripheral is “Connected"
	1h = The output latch of the dataflow will be “transparent"
	*/
	IOMUX->SECCFG.PINCM[IOMUX_PINCM1] |= (0x80 | 0x01 );

	
	/*
	Input bit 18
	0h = Input enable is disabled.
	1h = Input enable is enabled
	*/
	//IOMUX->SECCFG.PINCM[ ] |= 0x00040000 ;
	
	/*high drive strength
	1h = High Drive Strength
	*/
	//IOMUX->SECCFG.PINCM[IOMUX_PINCM1] |= 0x00100000 ;
	
	/*
	Pulldown bit 16
	1h = Pull down is enabled
	*/
	//IOMUX->SECCFG.PINCM[ ] |= 0x00010000 ;
	
	/*
	Pullup bit 17
	1h = Pull up is enabled
	*/
	//IOMUX->SECCFG.PINCM[ ] |= 0x00020000 ;
	
	/*
	Invert bit 26
	Data inversion selection
	0h = Data inversion is disabled.
	1h = Data inversion is enabled
	*/
	IOMUX->SECCFG.PINCM[IOMUX_PINCM1] |= (1 << 26) ;

	
	//Enable output
	GPIOA->DOESET31_0 |= (1 << 0);
	
	//Set output
	GPIOA->DOUTCLR31_0 |= (1 << 0);
		
	
		
		

	
	
}


/**
 * @brief Initialize LED2
 * @note You must account for each LED color
*/
void LED2_init(void){
		//check if enabled
	
		//if not reset, reset
	if (!(GPIOB->GPRCM.STAT & GPIO_STAT_RESETSTKY_MASK)){
		
		//reset
		GPIOB->GPRCM.RSTCTL = GPIO_RSTCTL_KEY_UNLOCK_W | GPIO_RSTCTL_RESETASSERT_ASSERT;
		//RESETSTKYCLR clears the but so this code isn't run again

	}//if gpiob RESET
	
	
	//enable
	GPIOB->GPRCM.PWREN = GPIO_PWREN_KEY_UNLOCK_W | GPIO_PWREN_ENABLE_ENABLE;
	//key | enable

	
	//GPIOx is enabled, set individual pin
	
	//pulldown and io direction
	/*
	use I/O bit 7
	Peripheral is “Connected"
	1h = The output latch of the dataflow will be “transparent"
	R
	*/
	IOMUX->SECCFG.PINCM[IOMUX_PINCM57] |= (0x80 | 0x01 );
	/*
	use I/O bit 7
	Peripheral is “Connected"
	1h = The output latch of the dataflow will be “transparent"
	G
	*/
	IOMUX->SECCFG.PINCM[IOMUX_PINCM58] |= (0x80 | 0x01 );
	/*
	use I/O bit 7
	Peripheral is “Connected"
	1h = The output latch of the dataflow will be “transparent"
	B
	*/
	IOMUX->SECCFG.PINCM[IOMUX_PINCM50] |= (0x80 | 0x01 );
	
	//Enable output
	GPIOB->DOESET31_0 |= (1 << 26);
	GPIOB->DOESET31_0 |= (1 << 27);
	GPIOB->DOESET31_0 |= (1 << 22);
	
	//Set output
	GPIOB->DOUTCLR31_0 |= (1 << 26);
	GPIOB->DOUTCLR31_0 |= (1 << 27);
	GPIOB->DOUTCLR31_0 |= (1 << 22);
	
	
		
};


/**
 * @brief Set LED1 output state
 * @note ON, OFF, TOGGLE
 * @param state 1 = on 0 = off
*/
void LED1_set(int state){
	
		if (state & 0x1){
			GPIOA->DOUTSET31_0 |= (0x01);
		}  else {
			GPIOA->DOUTCLR31_0 |= (0x01);
		}
}



/**
 * @brief Set LED2 output state
 * @note RED, GREEN, BLUE, CYAN, MAGENTA, YELLOW, WHITE, OFF
* @param colors bit 2:R, 1:G, 0:B
*/
void LED2_set(int colors){
	
	//red
	if (colors & 0x04){
		GPIOB->DOUTSET31_0 |= ( 0x1 << 26 );
	} else {
		GPIOB->DOUTCLR31_0 |= ( 0x1 << 26 );
	}
	
	//green
	if (colors & 0x02){
		GPIOB->DOUTSET31_0 |= ( 0x1 << 27 );
	} else {
		GPIOB->DOUTCLR31_0 |= ( 0x1 << 27 );
	}
	
	//blue
	if (colors & 0x01){
		GPIOB->DOUTSET31_0 |= ( 0x1 << 22 );
	} else {
		GPIOB->DOUTCLR31_0 |= ( 0x1 << 22 );
	}
	
	
}



