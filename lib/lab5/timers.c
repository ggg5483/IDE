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
	
	
	/*config down, periodic, counter val after enable is LOAD val */
	TIMG0->COUNTERREGS.CTRCTL &= ~(GPTIMER_CTRCTL_CVAE_MASK | GPTIMER_CTRCTL_CM_MASK | GPTIMER_CTRCTL_REPEAT_MASK);
	TIMG0->COUNTERREGS.CTRCTL = (GPTIMER_CTRCTL_CVAE_LDVAL | GPTIMER_CTRCTL_CM_DOWN | GPTIMER_CTRCTL_REPEAT_REPEAT_1);

	/*set LOAD to period*/
	TIMG0->COUNTERREGS.LOAD = period;
	
	/*setup interrupts*/
	/*disable interrupts */
	__disable_irq();
	
	/*clear zero event*/
	TIMG0->CPU_INT.ICLR |= GPTIMER_CPU_INT_ICLR_REPC_CLR;
	
	/*enable zero event*/
	TIMG0->CPU_INT.IMASK |= GPTIMER_CPU_INT_IMASK_REPC_SET;
	
	/*start timer*/
	TIMG0->COUNTERREGS.CTRCTL |= GPTIMER_CTRCTL_EN_ENABLED;
	
	/*register interrupt*/
	NVIC_EnableIRQ(TIMG0_INT_IRQn);
	
	/*enable interupts */
	__enable_irq();
}


/**
 * @brief Timer G6 module initialization. General purpose timer
*/
void TIMG6_init(uint32_t period, uint32_t prescaler){
	TIMG_power(TIMG6);
	
	//seelct clock source
	//TIMG6->CLKSEL = GPTIMER_CLKSEL_BUSCLK_SEL_ENABLE;
	TIMG6->CLKSEL = GPTIMER_CLKSEL_LFCLK_SEL_ENABLE;
	//slkdiv ratio
	TIMG6->CLKDIV = 0;
	//precaler
	TIMG6->COMMONREGS.CPS = prescaler;
	//enable TIMCLK
	TIMG6->COMMONREGS.CCLKCTL = GPTIMER_CCLKCTL_CLKEN_ENABLED;
	
	/*config down, periodic, counter val after enable is LOAD val */
	//TIMG6->COUNTERREGS.CTRCTL &= ~(GPTIMER_CTRCTL_CVAE_MASK | GPTIMER_CTRCTL_CM_MASK | GPTIMER_CTRCTL_REPEAT_MASK);
	TIMG6->COUNTERREGS.CTRCTL = (GPTIMER_CTRCTL_CVAE_LDVAL | GPTIMER_CTRCTL_CM_DOWN | GPTIMER_CTRCTL_REPEAT_REPEAT_1);

	/*set LOAD to period*/
	TIMG6->COUNTERREGS.LOAD = period;
	
	/*setup interrupts*/
	/*disable interrupts */
	__disable_irq();
	
	/*clear zero event*/
	TIMG6->CPU_INT.ICLR |= GPTIMER_CPU_INT_IMASK_Z_MASK;
	
	/*enable zero event*/
	TIMG6->CPU_INT.IMASK |= GPTIMER_CPU_INT_IMASK_Z_SET;
	
	/*start timer*/
	TIMG6->COUNTERREGS.CTRCTL |= GPTIMER_CTRCTL_EN_ENABLED;
	
	/*register interrupt*/
	NVIC_EnableIRQ(TIMG6_INT_IRQn);
	
	/*enable interupts */
	__enable_irq();
}


/**
 * @brief Timer G12 module initialization. General purpose timer
 * @note Timer G12 has no prescaler
*/
void TIMG12_init(uint32_t period){
	TIMG_power(TIMG12);
	
	//seelct clock source
	TIMG12->CLKSEL = GPTIMER_CLKSEL_BUSCLK_SEL_ENABLE;
	//TIMG12->CLKSEL = GPTIMER_CLKSEL_LFCLK_SEL_ENABLE;
	//slkdiv ratio
	TIMG12->CLKDIV = 0;
	//enable timclock
	TIMG12->COMMONREGS.CCLKCTL = 1;
	
	/*config down, periodic, counter val after enable is LOAD val */
	//TIMG12->COUNTERREGS.CTRCTL &= ~(GPTIMER_CTRCTL_CVAE_MASK | GPTIMER_CTRCTL_CM_MASK | GPTIMER_CTRCTL_REPEAT_MASK);
	TIMG12->COUNTERREGS.CTRCTL = (GPTIMER_CTRCTL_CVAE_LDVAL | GPTIMER_CTRCTL_CM_DOWN | GPTIMER_CTRCTL_REPEAT_REPEAT_1);

	/*set LOAD to period*/
	TIMG12->COUNTERREGS.LOAD = period;
	
	/*setup interrupts*/
	/*disable interrupts */
	__disable_irq();
	
	/*clear zero event*/
	TIMG12->CPU_INT.ICLR |= GPTIMER_CPU_INT_ICLR_Z_CLR;
	
	/*enable zero event*/
	TIMG12->CPU_INT.IMASK |= GPTIMER_CPU_INT_IMASK_Z_SET;
	
	/*start timer*/
	TIMG12->COUNTERREGS.CTRCTL |= GPTIMER_CTRCTL_EN_ENABLED;
	
	/*register interrupt*/
	NVIC_EnableIRQ(TIMG12_INT_IRQn);
	
	/*enable interupts */
	__enable_irq();
}



