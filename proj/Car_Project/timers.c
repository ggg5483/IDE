/**
 * ******************************************************************************
 * @file    : timers.c
 * @brief   : Timers module header file
 * @details : Timers initialization and interaction
 * 
 * @author Alex Hamadeh
 * @author Garrett Geyer
 * @date 3/3/26
 * ******************************************************************************
*/

#include "./timers.h"
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
	//power domain 0
	//40 MHz bus clock
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
	//TIMG0->COUNTERREGS.CTRCTL &= ~(GPTIMER_CTRCTL_CVAE_MASK | GPTIMER_CTRCTL_CM_MASK | GPTIMER_CTRCTL_REPEAT_MASK);
	TIMG0->COUNTERREGS.CTRCTL = (GPTIMER_CTRCTL_CVAE_LDVAL | GPTIMER_CTRCTL_CM_DOWN | GPTIMER_CTRCTL_REPEAT_REPEAT_1);

	/*set LOAD to period*/
	TIMG0->COUNTERREGS.LOAD = period;
	
	/*setup interrupts*/
	/*disable interrupts */
	__disable_irq();
	
	/*clear zero event*/
	TIMG0->CPU_INT.ICLR |= GPTIMER_CPU_INT_ICLR_Z_MASK;
	
	/*enable zero event*/
	TIMG0->CPU_INT.IMASK |= GPTIMER_CPU_INT_IMASK_Z_SET;
	
	/*start timer*/
	TIMG0->COUNTERREGS.CTRCTL |= GPTIMER_CTRCTL_EN_ENABLED;
	
	/*register interrupt*/
	NVIC_EnableIRQ(TIMG0_INT_IRQn);
	
	/*enable interupts */
	__enable_irq();
}

// Use slow clock 32 kHz or bus clock 80 MHz
#ifndef TIMG6_FAST
#define TIMG6_FAST true
#endif
/**
 * @brief Timer G6 module initialization. General purpose timer
*/
void TIMG6_init(uint32_t period, uint32_t prescaler){
	// power Domain 1 
	TIMG_power(TIMG6);
	
	//seelct clock source
	#if TIMG6_FAST
	TIMG6->CLKSEL = GPTIMER_CLKSEL_BUSCLK_SEL_ENABLE;
	#else
	TIMG6->CLKSEL = GPTIMER_CLKSEL_LFCLK_SEL_ENABLE;
	#endif // TIMG6_FAST
	
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

// Use slow clock 32 kHz or bus clock 80 MHz
#ifndef TIMG12_FAST
#define TIMG12_FAST true
#endif
/**
 * @brief Timer G12 module initialization. General purpose timer
 * @note Timer G12 has no prescaler
*/
void TIMG12_init(uint32_t period){
	//
	TIMG_power(TIMG12);
	
	//seelct clock source
	#if TIMG12_FAST
		TIMG12->CLKSEL = GPTIMER_CLKSEL_BUSCLK_SEL_ENABLE;
	#else
		TIMG12->CLKSEL = GPTIMER_CLKSEL_LFCLK_SEL_ENABLE;
	#endif
	
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

/**
 * @brief Timer A0 module PWM initialization
 * @param[in] pin - Timer PWM output pin / channel
 * @param[in] period - Timer load value
 * @param[in] prescaler - Timer prescale value
 * @param[in] percenetDutyCycle - PWM duty cycle positive
 * @note Store period to be able to adjust duty cycle percentage later
*/
void TIMA0_PWM_init(uint8_t pin, uint32_t period, uint32_t prescaler, double percentDutyCycle){
	TIMG_power(TIMA0);
	
	//seelct clock source
	TIMA0->CLKSEL = GPTIMER_CLKSEL_BUSCLK_SEL_ENABLE;
	//TIMA0->CLKSEL = GPTIMER_CLKSEL_LFCLK_SEL_ENABLE;
	
	//slkdiv ratio
	TIMA0->CLKDIV = 0;
	//enable timclock
	TIMA0->COMMONREGS.CCLKCTL = 1;
	
	/*config down, periodic, counter val after enable is LOAD val */
	//TIMA0->COUNTERREGS.CTRCTL &= ~(GPTIMER_CTRCTL_CVAE_MASK | GPTIMER_CTRCTL_CM_MASK | GPTIMER_CTRCTL_REPEAT_MASK);
	TIMA0->COUNTERREGS.CTRCTL = (GPTIMER_CTRCTL_CVAE_LDVAL | GPTIMER_CTRCTL_CM_DOWN | GPTIMER_CTRCTL_REPEAT_REPEAT_1);

	/*set LOAD to period*/
	TIMA0->COUNTERREGS.LOAD = period;
	
	/*select channel */
	//TIMA0->FPUB_0 = 1 + pin;
	
	/*set CCP direction*/
	TIMA0->COMMONREGS.CCPD = 0x7;
	
	/*set IO */
	//channel (pin) 0
	//PB8 PINCM25
	IOMUX->SECCFG.PINCM[IOMUX_PINCM25] = IOMUX_PINCM25_PF_TIMA0_CCP0 | IOMUX_PINCM_PC_CONNECTED;
	//channel (pin) 1
	//PA22 PINCM47
	IOMUX->SECCFG.PINCM[IOMUX_PINCM47] = IOMUX_PINCM47_PF_TIMA0_CCP1 | IOMUX_PINCM_PC_CONNECTED;

	
	/*set CCCTL reg*/
	TIMA0->COUNTERREGS.CCCTL_01[pin] = 0;
	
	/*set CCP output settings */
	TIMA0->COUNTERREGS.CCACT_01[pin] = GPTIMER_CCACT_01_LACT_CCP_HIGH | GPTIMER_CCACT_01_CDACT_CCP_LOW;
	
	/* set duty cycle */
	TIMA0_PWM_DutyCycle(pin, percentDutyCycle);
	
	/*start timer*/
	TIMA0->COUNTERREGS.CTRCTL |= GPTIMER_CTRCTL_EN_ENABLED;
	
	
}


/**
 * @brief Timer A1 module PWM initialization
 * @param[in] pin - Timer PWM output pin / channel
 * @param[in] period - Timer load value
 * @param[in] prescaler - Timer prescale value
 * @param[in] percenetDutyCycle - PWM duty cycle positive
 * @note Store period to be able to adjust duty cycle percentage later
*/
void TIMA1_PWM_init(uint8_t pin, uint32_t period, uint32_t prescaler, double percentDutyCycle){
	TIMG_power(TIMA1);
	
	//seelct clock source
	TIMA1->CLKSEL = GPTIMER_CLKSEL_BUSCLK_SEL_ENABLE;

	
	//seelct clock source
	//TIMA1->CLKSEL = GPTIMER_CLKSEL_BUSCLK_SEL_ENABLE;
	TIMA1->CLKSEL = GPTIMER_CLKSEL_LFCLK_SEL_ENABLE;
	
	//slkdiv ratio
	TIMA1->CLKDIV = 0;
	//enable timclock
	TIMA1->COMMONREGS.CCLKCTL = 1;
	
	/*config down, periodic, counter val after enable is LOAD val */
	//TIMA1->COUNTERREGS.CTRCTL &= ~(GPTIMER_CTRCTL_CVAE_MASK | GPTIMER_CTRCTL_CM_MASK | GPTIMER_CTRCTL_REPEAT_MASK);
	TIMA1->COUNTERREGS.CTRCTL = (GPTIMER_CTRCTL_CVAE_LDVAL | GPTIMER_CTRCTL_CM_DOWN | GPTIMER_CTRCTL_REPEAT_REPEAT_1);

	/*set LOAD to period*/
	TIMA1->COUNTERREGS.LOAD = period;
	
	/*select channel */
	//TIMA1->FPUB_0 = 1 + pin;
	
	/*set CCP direction*/
	TIMA1->COMMONREGS.CCPD = 0x7;
	
	/*set IO */
	//PB4
	IOMUX->SECCFG.PINCM[IOMUX_PINCM17] = IOMUX_PINCM17_PF_TIMA1_CCP0 | IOMUX_PINCM_PC_CONNECTED;
	
	/*set CCCTL reg*/
	TIMA1->COUNTERREGS.CCCTL_01[pin] = 0;
	
	/*set CCP output settings */
	TIMA1->COUNTERREGS.CCACT_01[pin] = GPTIMER_CCACT_01_LACT_CCP_HIGH | GPTIMER_CCACT_01_CDACT_CCP_LOW;
	
	/* set duty cycle */
	TIMA1_PWM_DutyCycle(pin, percentDutyCycle);
	
	/*start timer*/
	TIMA1->COUNTERREGS.CTRCTL |= GPTIMER_CTRCTL_EN_ENABLED;
	
}


/**
 * @brief Change PWM duty cycle for all Timer A0 channels
 * @param[in] pin - Timer PWM output pin / channel
 * @param[in] percentDutyCycle - Duty cycle to change to
*/
void TIMA0_PWM_DutyCycle(uint8_t pin, double percentDutyCycle){
	TIMA0->COUNTERREGS.CC_01[pin] = (int) ((1-percentDutyCycle) * TIMA0->COUNTERREGS.LOAD);
}


/**
 * @brief Change PWM duty cycle for all Timer A1 channels
 * @param[in] pin - Timer PWM output pin / channel
 * @param[in] percentDutyCycle - Duty cycle to change to
*/
void TIMA1_PWM_DutyCycle(uint8_t pin, double percentDutyCycle){
	TIMA1->COUNTERREGS.CC_01[pin] = (int) ((1-percentDutyCycle) * TIMA1->COUNTERREGS.LOAD);
	
}



