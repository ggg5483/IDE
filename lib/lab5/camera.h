/**
 * ******************************************************************************
 * @file    : camera.h
 * @brief   : Camera module header file
 * @details : Parallax TSL1401-DB Linescan Camera initialization and interaction
 * @note    : Reserves the use of Timers G0 (CLK) and G6 (SI)
 *            and ADC0 channel 0 
 *
 * @author Alexander Hamadeh
 * @author Garrett Geyer
 * @date 
 * ******************************************************************************
*/

#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <stdint.h>

/* SI = PA28, CLK = PA12 */
#define CAM_SI_MASK   (1UL << 28)
#define CAM_CLK_MASK  (1UL << 12)

/**
 * @brief Initialize camera associated components
*/
void Camera_init(void);


/**
 * @brief Checks whether camer data is ready to retrieve
 * @note Make sure to check all data is available
 * @return True(1)/False(0) if camera data is ready
*/
uint8_t Camera_isDataReady(void);


/**
 * @brief Retrieves pointer to camera data array
 * @return Pointer to global data array stored locally in this file
*/
uint16_t* Camera_getData(void);

/**
 * @brief Checks whether camera data is ready to retrieve
 * @return True(1)/False(0)
*/
uint8_t Camera_isDataReady(void);

/**
 * @brief Retrieves pointer to camera data array
 * @return Pointer to global data array
*/
uint16_t* Camera_getData(void);

#endif // _CAMERA_H_
