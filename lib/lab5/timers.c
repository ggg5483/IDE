/**
 * ******************************************************************************
 * @file    : timers.c
 * @brief   : Timers module header file
 * @details : Timers initialization and interaction
 * 
 * @author 
 * @date 
 * ******************************************************************************
*/

#include "timers.h"
#include <ti/devices/msp/msp.h>
#include <stdint.h>

/*
timg0 16
g6 17
g12 21
*/
/**
 * @brief Helper function to power on timer
*/
void TIMG_power(GPTIMER_Regs *TIM){
	if (!(TIM->GPRCM.PWREN & GPTIMER_PWREN_ENABLE_MASK)){
		//reset
		TIM->GPRCM.RSTCTL = (GPTIMER_RSTCTL_KEY_UNLOCK_W | GPTIMER_RSTCTL_RESETASSERT_ASSERT | GPTIMER_RSTCTL_RESETSTKYCLR_CLR);
		//enable
		TIM->GPRCM.PWREN = (GPTIMER_PWREN_KEY_UNLOCK_W | GPTIMER_PWREN_ENABLE_ENABLE);
	}
}


/**
 * @brief Timer G0 module initialization. General purpose timer
 * @note Timer G0 is in Power Domain 0. Check page 3 of the Data Sheet
*/
void TIMG0_init(uint32_t period, uint32_t prescaler){
	TIMG_power(TIMG0);
	
	//seelct clock source
	//80 mhz
	//count reg load sets period
	TIMG0->CLKSEL = GPTIMER_CLKSEL_BUSCLK_SEL_ENABLE;
	//slkdiv ratio
	TIMG0->CLKDIV = 0;
	//precaler
	TIMG0->COMMONREGS.CPS = prescaler;
	//enable timclock
	TIMG0->COMMONREGS.CCLKCTL = 1;
	
	
	//config down
	//periodic
	//counter val after enable
	TIMG0->COUNTERREGS.CTRCTL &= ~(GPTIMER_CTRCTL_CVAE_MASK | GPTIMER_CTRCTL_CM_MASK | GPTIMER_CTRCTL_REPEAT_MASK);
	TIMG0->COUNTERREGS.CTRCTL |= (GPTIMER_CTRCTL_CVAE_LDVAL | GPTIMER_CTRCTL_CM_DOWN | GPTIMER_CTRCTL_REPEAT_REPEAT_1);
	
	
	
	//set LOAD to period
	
	
	
}


/**
 * @brief Timer G6 module initialization. General purpose timer
*/
void TIMG6_init(uint32_t period, uint32_t prescaler){
	TIMG_power(TIMG6);
	
	TIMG6->CLKSEL = GPTIMER_CLKSEL_BUSCLK_SEL_ENABLE;
	//slkdiv ratio
	TIMG6->CLKDIV = 0;
	//precaler
	TIMG6->COMMONREGS.CPS = prescaler;
	//enable timclock
	TIMG6->COMMONREGS.CCLKCTL = 1;
	
}


/**
 * @brief Timer G12 module initialization. General purpose timer
 * @note Timer G12 has no prescaler
*/
void TIMG12_init(uint32_t period){
	TIMG_power(TIMG12);
	
	TIMG12->CLKSEL = GPTIMER_CLKSEL_BUSCLK_SEL_ENABLE;
	//slkdiv ratio
	TIMG12->CLKDIV = 0;
	//enable timclock
	TIMG12->COMMONREGS.CCLKCTL = 1;
	
}



