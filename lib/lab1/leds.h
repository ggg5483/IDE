/**
 * ******************************************************************************
 * @file    : leds.h
 * @brief   : LEDs module header file
 * @details : LED initialization and interaction
 * 
 * @author 
 * @date 1/16/26
 * ******************************************************************************
*/

#ifndef _LEDS_H_
#define _LEDS_H_

#define LED2_OFF 0x0
#define LED2_RED 0x4
#define LED2_GREEN 0x2
#define LED2_BLUE 0x1
#define LED2_CYAN 0x3
#define LED2_MAGENTA 0x5
#define LED2_YELLOW 0x6
#define LED2_WHITE 0x7

/**
 * @brief Initialze LED1
 * @hint You might want to check out the schematics in the MSP User Guide
 *       The IOMUX has a hardware inversion bit
*/
void LED1_init(void);


/**
 * @brief Initialize LED2
 * @note You must account for each LED color
*/
void LED2_init(void);


/**
 * @brief Set LED1 output state
 * @note ON, OFF, TOGGLE
*/
void LED1_set();


/**
 * @brief Set LED2 output state
 * @note RED, GREEN, BLUE, CYAN, MAGENTA, YELLOW, WHITE, OFF
*/
void LED2_set();


#endif // _LEDS_H_
