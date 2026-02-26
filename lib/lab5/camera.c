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

/* SI = PA28, CLK = PA12 */
#define CAM_SI_MASK   (1UL << 28)
#define CAM_CLK_MASK  (1UL << 12)

/* Local camera data */
static volatile uint16_t cameraData[128];
static volatile uint16_t pixelCounter = 0;
static volatile uint8_t  cameraData_complete = 0;

/* GPIO bit-set and bit-clear operation*/
static inline void CAM_SI_HIGH(void)  { GPIOA->DOUTSET31_0 = CAM_SI_MASK; }
static inline void CAM_SI_LOW(void)   { GPIOA->DOUTCLR31_0 = CAM_SI_MASK; }
static inline void CAM_CLK_HIGH(void) { GPIOA->DOUTSET31_0 = CAM_CLK_MASK; }
static inline void CAM_CLK_LOW(void)  { GPIOA->DOUTCLR31_0 = CAM_CLK_MASK; }

/**
 * @brief Initialize camera associated components
*/
void Camera_init(void)
{
    /* Configure SI and CLK pins as GPIO outputs */
    IOMUX->SECCFG.PINCM[28] = 0U;
    IOMUX->SECCFG.PINCM[12] = 0U;

    CAM_SI_LOW();
    CAM_CLK_LOW();

    /* Initialize ADC0 */
    ADC0_init();

    /* TIMG0: 100 kHz clock */
    TIMG0_init(799U, 0U);

    /* Disable CLK timer initially */
    TIMG0->COUNTERREGS.CTRCTL &= ~GPTIMER_CTRCTL_EN_ENABLED;

    /* TIMG6: integration time ~7.5ms */
    TIMG6_init(239U, 0U);

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

/**
 * @brief TIMG6 ISR – controls SI pulse and integration time
*/
void TIMG6_IRQHandler(void)
{
    /* Clear interrupt */
    TIMG6->CPU_INT.ICLR = GPTIMER_CPU_INT_ICLR_Z_CLR;

    /* Ensure CLK disabled */
    TIMG0->COUNTERREGS.CTRCTL &= ~GPTIMER_CTRCTL_EN_ENABLED;

    /* Do not start new frame if previous not completed */
    if (cameraData_complete)
        return;

    pixelCounter = 0U;

    /* Start capture sequence */
    CAM_CLK_LOW();
    CAM_SI_HIGH();

    CAM_CLK_HIGH();
    CAM_CLK_LOW();

    CAM_SI_LOW();

    /* Enable CLK timer */
    TIMG0->COUNTERREGS.CTRCTL |= GPTIMER_CTRCTL_EN_ENABLED;
}

/**
 * @brief TIMG0 ISR – drives CLK and samples ADC
*/
void TIMG0_IRQHandler(void)
{
    /* Clear interrupt */
    TIMG0->CPU_INT.ICLR = GPTIMER_CPU_INT_ICLR_Z_CLR;

    /* Pulse CLK */
    CAM_CLK_HIGH();

    /* Sample ADC */
    uint32_t adcVal = ADC0_getVal();

    if (pixelCounter < 128U) {
        cameraData[pixelCounter++] = (uint16_t)adcVal;
    }

    CAM_CLK_LOW();

    /* End of frame */
    if (pixelCounter >= 128U) {
        cameraData_complete = 1U;
        pixelCounter = 0U;

        /* Disable CLK */
        TIMG0->COUNTERREGS.CTRCTL &= ~GPTIMER_CTRCTL_EN_ENABLED;
    }
}
