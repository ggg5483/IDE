/**
 * ******************************************************************************
 * @file    : camera.c
 * @brief   : Camera module header file
 * @details : Parallax TSL1401-DB Linescan Camera initialization and interaction
 * @note    : Reserves the use of Timers G0 (CLK) and G6 (SI)
 *            and ADC0 channel 0 
 *
 * @author 
 * @date 
 * ******************************************************************************
*/

#include "camera.h"
#include <ti/devices/msp/msp.h>
#include <stdint.h>
#include "timers.h"
#include "adc12.h"

static volatile uint16_t cameraData[128];
static volatile uint16_t pixelCounter = 0;
static volatile uint8_t  cameraData_complete = 0;

/**
 * @brief Initialize camera associated components
*/
void Camera_init(void)
{
	
		/*turn on GPIOA */
		if (!(GPIOA->GPRCM.PWREN & GPIO_PWREN_ENABLE_MASK)){
			//reset
			GPIOA->GPRCM.RSTCTL = (GPIO_RSTCTL_KEY_UNLOCK_W | GPIO_RSTCTL_RESETASSERT_ASSERT | GPIO_RSTCTL_RESETSTKYCLR_CLR);
			//enable
			
		}
		GPIOA->GPRCM.PWREN = (GPIO_PWREN_KEY_UNLOCK_W | GPIO_PWREN_ENABLE_ENABLE);
		
    /* Configure SI and CLK pins as GPIO outputs */
    IOMUX->SECCFG.PINCM[IOMUX_PINCM34] = IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM34_PF_GPIOA_DIO12;
    IOMUX->SECCFG.PINCM[IOMUX_PINCM3] = IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM3_PF_GPIOA_DIO28;

		/*enable output*/
		GPIOA->DOESET31_0 |= CAM_SI_MASK;
		GPIOA->DOESET31_0 |= CAM_CLK_MASK;
	
		/*clear output*/
    GPIOA->DOUTCLR31_0 = CAM_SI_MASK;
    GPIOA->DOUTCLR31_0 = CAM_CLK_MASK;

    /* Initialize ADC0 */
    ADC0_init();

    /* TIMG0: 100 kHz clock */
    TIMG0_init(319, 0);
 
    /* Disable CLK timer initially */
    TIMG0->COUNTERREGS.CTRCTL &= ~GPTIMER_CTRCTL_EN_ENABLED;

    /* TIMG6: integration time ~7.5ms */
    TIMG6_init(60000, 3);

    cameraData_complete = 0U;
    pixelCounter = 0U;
}

/**
 * @brief Checks whether camera data is ready to retrieve
 * @return True(1)/False(0)
*/
uint8_t Camera_isDataReady(void)
{
    return cameraData_complete;
}

/**
 * @brief Retrieves pointer to camera data array
 * @return Pointer to global data array
*/
uint16_t* Camera_getData(void)
{
    cameraData_complete = 0U;
    return (uint16_t*)cameraData;
}
#if MAIN == 3

#endif // main == 3
