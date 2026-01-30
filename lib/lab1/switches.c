/**
 * ******************************************************************************
 * @file    : switches.c
 * @brief   : Switches module  file
 * @details : Switches initialization and interaction
 * 
 * @author 
 * @date 		1/16/26
 * ******************************************************************************
*/

#if 0 //redundant include
#include <../source/ti/devices/msp/peripherals/hw_iomux.h>
#include <ti\devices\msp\peripherals\hw_gpio.h>
#include <../source/ti/devices/msp/m0p/mspm0g350x.h>
#endif



#include <ti/devices/msp/msp.h> 
#include "switches.h"

/**
 * @brief Switch 1 polling initialization
 * @hint You might want to check out the schematics in the MSP User Guide
 *       The IOMUX has a hardware inversion bit
*/
void S1_init(void){
	/*
	GPIO A
	input, pull down
	
	
PINCMX

		SW1 40 PA18 DIN
		SW2 49 PB21
	
	
	*/
	
	if (!(GPIOA->GPRCM.STAT & GPIO_STAT_RESETSTKY_MASK)){
		//reset
		GPIOA->GPRCM.RSTCTL = GPIO_RSTCTL_KEY_UNLOCK_W | GPIO_RSTCTL_RESETASSERT_ASSERT;// | 0x01;
		//key | RESETASSERT
		
		
	}//if GPIOA reste
	
	//enable
	GPIOA->GPRCM.PWREN = GPIO_PWREN_KEY_UNLOCK_W | GPIO_PWREN_ENABLE_ENABLE;
	//key | enable
	
	
	//GPIOx is enabled, set individual pin
	
	
	/*
	use I/O bit 7
	Peripheral is “Connected"
	1h = The output latch of the dataflow will be “transparent"
	R
	*/
	IOMUX->SECCFG.PINCM[IOMUX_PINCM40] |= (0x80 | 0x01 );
	
	/*
	Input bit 18
	0h = Input enable is disabled.
	1h = Input enable is enabled
	*/
	IOMUX->SECCFG.PINCM[IOMUX_PINCM40] |= 0x00040000 ;
	
	/*
	Pulldown bit 16
	1h = Pull down is enabled
	*/
	IOMUX->SECCFG.PINCM[IOMUX_PINCM40] |= (1 << 16) ;
	
	
	
}


/**
 * @brief Switch 2 polling initialization
*/
void S2_init(void){
	/*
	GPIO B
	input, pull down
	*/
	
	if (!(GPIOB->GPRCM.STAT & GPIO_STAT_RESETSTKY_MASK)){
		//reset
		GPIOB->GPRCM.RSTCTL = GPIO_RSTCTL_KEY_UNLOCK_W | GPIO_RSTCTL_RESETASSERT_ASSERT;
		//key & RESETASSERT
		

	}//if GPIOB RESET
	
	//GPIOx is enabled, set individual pin
	//enable
	GPIOB->GPRCM.PWREN = GPIO_PWREN_KEY_UNLOCK_W | GPIO_PWREN_ENABLE_ENABLE;
	//key and enable
	
	
	/*
	use I/O bit 7
	Peripheral is “Connected"
	1h = The output latch of the dataflow will be “transparent"
	R
	*/
	IOMUX->SECCFG.PINCM[IOMUX_PINCM49] |= (0x80 | 0x01 );
	
	
	/*
	Input bit 18
	0h = Input enable is disabled.
	1h = Input enable is enabled
	*/
	IOMUX->SECCFG.PINCM[IOMUX_PINCM49] |= 0x00040000 ;
		
	/*
	Pullup bit 17
	1h = Pull up is enabled
	*/
	IOMUX->SECCFG.PINCM[IOMUX_PINCM49] |= 0x00020000 ;
	
	/*
	Invert bit 26
	Data inversion selection
	0h = Data inversion is disabled.
	1h = Data inversion is enabled
	*/
	IOMUX->SECCFG.PINCM[IOMUX_PINCM49] |= (1 << 26) ;



}


/**
 * @brief Check if switch 1 was pressed
 * @return True(1)/False(0) if switch 1 was pressed
*/
int S1_pressed(void){
	return (GPIOA->DIN31_0 >> 18) && 0x1;
}


/**
 * @brief Check if switch 2 was pressed
 * @return True(1)/False(0) if switch 2 was pressed
*/
int S2_pressed(void){
	return (GPIOB->DIN31_0 >> 21) && 0x1;
}


