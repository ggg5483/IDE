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
#include <ti/devices/msp/msp.h>
#include <stdint.h>
#include <stdbool.h>

#define ADC_SAMPLE_DIVIDER 8U      // sampling clock divider = 8
#define ADC_CHANNEL        0U      // ADC0 channel 0 (PA27)
#define ADC_RESULT_MASK    0x0FFFU // 12-bit ADC

void ADC0_init(void)
{
    /* Reset and power on the module, as usual. */
    if (!(ADC->GPRCM.PWREN & ADC12_PWREN_ENABLE_MASK)) {
        ADC->GPRCM.RSTCTL = ADC12_RSTCTL_KEY_UNLOCK_W | ADC12_RSTCTL_RESETASSERT_ASSERT;
        ADC->GPRCM.PWREN  = ADC12_PWREN_KEY_UNLOCK_W | ADC12_PWREN_ENABLE_ENABLE;
    }

    /* Select Ultra Low Power clock and highest possible frequency range. (TRM Section 12.2.5 "ADC Clocking") */
    ADC->CLKCFG = ADC12_CLKCFG_SAMPCLK_ULPCLK;         // choose ULPCLK
    ADC->CLKCFG = ADC12_CLKCFG_CCONRUN_MASK;           // highest range

    /* Power down behavior: keep ADC on after conversion (TRM 12.2.7) */
    ADC->PWRDN &= ~ADC12_CTL0_PWRDN_MASK;              // ensure ADC remains on after conversion
    ADC->PWRDN |= ADC12_CTL0_PWRDN_MANUAL;    

    /* Sampling period: set sampling clock divider to factor of 8 (TRM 12.2.9) */
    ADC->SCLKDIV = (ADC12_CTL0_SCLKDIV_DIV_BY_8 & ADC12_CTL0_SCLKDIV_MASK);

    /* Conversion mode: single channel, single conversion, auto software trigger (TRM 12.2.10) */
    ADC->CTL1 = 0; // clear control
    ADC->CTL1 |= ADC12_CTL1_CONSEQ_SINGLE;             // single conversion mode
    ADC->CTL1 |= ADC12_CTL1_SAMPMODE_AUTO;             // software auto trigger
    ADC->CTL1 &= ~ADC12_CTL1_CONSEQ_SEQUENCE;          // ensure not continuous

    /* Map channel 0 to memory result register 0 */
    ADC->CHMAP[0] = ADC_CHANNEL;                       // CHMAP[0] -> channel for RESULT0

}

uint32_t ADC0_getVal(void)
{
    uint32_t val = 0;

    /* Enable ADC */
    ADC->CTL1 |= ADC12_CTL1_SAMPMODE_AUTO;

    /* Start a software conversion (TRM: use SW trigger) "can't figure out macros"*/
    ADC->SWTRIG = ADC12_SWTRIG_START;

    /* Wait for conversion to complete: poll BUSY flag "can't figure out macros"*/
    while (ADC->STAT & ADC12_STAT_BUSY_MASK) {
    }

    /* Read result register 0 and mask to 12 bits "can't figure out macros"*/
    val = (ADC->RESULT[0] & ADC12_RESULT_MASK);

    /* Optionally leave ADC powered on (per power-down behavior) */
    return val;
}

