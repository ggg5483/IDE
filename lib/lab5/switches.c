/**
 * ******************************************************************************
 * @file    : switches.h
 * @brief   : Switches module header file
 * @details : Switches initialization and interaction
 * 
 * @author 
 * @date 
 * ******************************************************************************
*/

#include <ti/devices/msp/msp.h>
#include "switches.h"

/**
 * @brief Switch 1 polling initialization
 * @hint You might want to check out the schematics in the MSP User Guide
 *       The IOMUX has a hardware inversion bit
*/
void S1_init(void){
	
	if ((GPIOA->GPRCM.PWREN & GPIO_PWREN_ENABLE_MASK) == 0U) {
		GPIOA->GPRCM.RSTCTL = GPIO_RSTCTL_RESETASSERT_ASSERT | GPIO_RSTCTL_KEY_UNLOCK_W | GPIO_RSTCTL_RESETSTKYCLR_CLR;
		GPIOA->GPRCM.PWREN = GPIO_PWREN_ENABLE_MASK | GPIO_PWREN_KEY_UNLOCK_W;
	}
	
  
	/* Configure PA18 as GPIO input with pull-up (active-high per table) */
  IOMUX->SECCFG.PINCM[IOMUX_PINCM40] |= (1U | IOMUX_PINCM_PC_CONNECTED);
	IOMUX->SECCFG.PINCM[IOMUX_PINCM40] |= IOMUX_PINCM_INENA_ENABLE;
	IOMUX->SECCFG.PINCM[IOMUX_PINCM40] |= IOMUX_PINCM_PIPD_ENABLE;
	
	}


/**
 * @brief Switch 2 polling initialization
*/
void S2_init(void){
	
	if ((GPIOB->GPRCM.PWREN & GPIO_PWREN_ENABLE_MASK) == 0U) {
		GPIOB->GPRCM.RSTCTL = GPIO_RSTCTL_RESETASSERT_ASSERT | GPIO_RSTCTL_KEY_UNLOCK_W | GPIO_RSTCTL_RESETSTKYCLR_CLR;
		GPIOB->GPRCM.PWREN = GPIO_PWREN_ENABLE_MASK | GPIO_PWREN_KEY_UNLOCK_W;
	}
  
	/* Configure PA18 as GPIO input with pull-up (active-high per table) */
  IOMUX->SECCFG.PINCM[IOMUX_PINCM49] |= (1U | IOMUX_PINCM_PC_CONNECTED);
	IOMUX->SECCFG.PINCM[IOMUX_PINCM49] |= IOMUX_PINCM_INENA_ENABLE;
	IOMUX->SECCFG.PINCM[IOMUX_PINCM49] |= IOMUX_PINCM_PIPU_ENABLE;
	
}


/**
 * @brief Check if switch 1 was pressed
 * @return True(1)/False(0) if switch 1 was pressed
*/
int S1_pressed(void){

	/* S1 is active-high: pressed -> input reads 1 */
	return ((GPIOA->DIN31_0 & (1U << 18)) != 0U) ? 1 : 0;
}


/**
 * @brief Check if switch 2 was pressed
 * @return True(1)/False(0) if switch 2 was pressed
*/
int S2_pressed(void){
	
	/* S2 is active-low: pressed -> input reads 0 */
	return ((GPIOB->DIN31_0 & (1U << 21)) == 0U) ? 1 : 0;
}


/**
 * @brief Switch 1 interrupt initialization
 * @note Use NVIC_EnableIRQ() to register IRQn with the NVIC
 *       Check out `cmsis_armclang.h`
 * @hint Keep the polarity in mind
*/
void S1_init_interrupt(void){

		/*disable interrupts */
		__disable_irq();
	
    if ((GPIOA->GPRCM.PWREN & GPIO_PWREN_ENABLE_MASK) == 0U) {
        GPIOA->GPRCM.RSTCTL = GPIO_RSTCTL_RESETASSERT_ASSERT |
                              GPIO_RSTCTL_KEY_UNLOCK_W |
                              GPIO_RSTCTL_RESETSTKYCLR_CLR;
        GPIOA->GPRCM.PWREN  = GPIO_PWREN_ENABLE_MASK |
                              GPIO_PWREN_KEY_UNLOCK_W;
    }

    /* Configure PA18 as GPIO input */
    IOMUX->SECCFG.PINCM[IOMUX_PINCM40] |= (1U | IOMUX_PINCM_PC_CONNECTED);
    IOMUX->SECCFG.PINCM[IOMUX_PINCM40] |= IOMUX_PINCM_INENA_ENABLE;
    IOMUX->SECCFG.PINCM[IOMUX_PINCM40] |= IOMUX_PINCM_PIPD_ENABLE;   // pull-down

		#if 0 //commented out as i don't know what this does
    /* Configure interrupt: S1 is active-high rising edge */
    GPIOA->IEV31_0  |=  (1U << 18);   // rising edge
    GPIOA->IS31_0   &= ~(1U << 18);   // edge-sensitive
    GPIOA->IBE31_0  &= ~(1U << 18);   // single-edge
		#endif

    /* Clear any stale interrupt */
    GPIOA->CPU_INT.ICLR |= (1U << 18);

    /* Enable interrupt in GPIO module */
    GPIOA->CPU_INT.IMASK |= (1U << 18);

		/*Set POlairty*/
		GPIOA->POLARITY31_16 |= GPIO_POLARITY31_16_DIO18_RISE;
		
    /* Enable NVIC interrupt for GPIOA */
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
		
		/*enable interupts */
		__enable_irq();
}


/**
 * @brief Switch 2 interrupt initialization
 * @note Use NVIC_EnableIRQ() to register IRQn with the NVIC
 *       Check out `cmsis_armclang.h`
*/
void S2_init_interrupt(void){

		/*disable interrupts */
		__disable_irq();
	
    /* Power on GPIOB if needed */
    if ((GPIOB->GPRCM.PWREN & GPIO_PWREN_ENABLE_MASK) == 0U) {
        GPIOB->GPRCM.RSTCTL = GPIO_RSTCTL_RESETASSERT_ASSERT |
                              GPIO_RSTCTL_KEY_UNLOCK_W |
                              GPIO_RSTCTL_RESETSTKYCLR_CLR;
        GPIOB->GPRCM.PWREN  = GPIO_PWREN_ENABLE_MASK |
                              GPIO_PWREN_KEY_UNLOCK_W;
    }

    /* Configure PB21 as GPIO input */
    IOMUX->SECCFG.PINCM[IOMUX_PINCM49] |= (1U | IOMUX_PINCM_PC_CONNECTED);
    IOMUX->SECCFG.PINCM[IOMUX_PINCM49] |= IOMUX_PINCM_INENA_ENABLE;
    IOMUX->SECCFG.PINCM[IOMUX_PINCM49] |= IOMUX_PINCM_PIPU_ENABLE;   // pull-up

		#if 0 //commented out as i don't know what this does
    /* Configure interrupt: S2 is active-low ? falling edge */
    GPIOB->IEV31_0  &= ~(1U << 21);   // falling edge
    GPIOB->IS31_0   &= ~(1U << 21);   // edge-sensitive
    GPIOB->IBE31_0  &= ~(1U << 21);   // single-edge
		#endif
		
    /* Clear any stale interrupt */
    GPIOB->CPU_INT.ICLR = (1U << 21);

    /* Enable interrupt in GPIO module */
    GPIOB->CPU_INT.IMASK |= (1U << 21);
		
		/*Set POlairty*/
		GPIOB->POLARITY31_16 |= GPIO_POLARITY31_16_DIO21_RISE;

    /* Enable NVIC interrupt for GPIOB */
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
		

		/*enable interupts */
		__enable_irq();
}
	
