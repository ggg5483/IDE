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

#include <stdint.h>

/**
 * @brief Timer G0 module initialization. General purpose timer
 * @note Timer G0 is in Power Domain 0. Check page 3 of the Data Sheet
*/
void TIMG0_init(uint32_t period, uint32_t prescaler){}


/**
 * @brief Timer G6 module initialization. General purpose timer
*/
void TIMG6_init(uint32_t period, uint32_t prescaler){}


/**
 * @brief Timer G12 module initialization. General purpose timer
 * @note Timer G12 has no prescaler
*/
void TIMG12_init(uint32_t period){}


