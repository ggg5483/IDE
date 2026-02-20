/**
 * ******************************************************************************
 * @file    : adc12.c
 * @brief   : ADC module header file
 * @details : ADC initialization and interaction
 * @note    : ADC does not require IOMUX interaction
 * 
 * @author 
 * @date 
 * ******************************************************************************
*/

#include "adc12.h"

#include <stdint.h>

/**
 * @brief Initialize ADC0
*/
void ADC0_init(void){

}


/**
 * @brief Retrieve the value from the ADC0
 * @note The ADC channel in use is set during initialization
 *       The channel is not the same as where the module stores the value
 * @return ADC0 processed value
*/
uint32_t ADC0_getVal(void){
	return 0;
}



