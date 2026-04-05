/**
 * ******************************************************************************
 * @file    : adc12.c
 * @brief   : ADC module header file
 * @details : ADC initialization and interaction
 * @note    : ADC does not require IOMUX interaction
 *
 * @author Alexander Hamadeh
 * @author Garrett Geyer
 * @date 
 * ******************************************************************************
*/

#include "adc12.h"
#include <ti/devices/msp/msp.h>
#include <stdint.h>

/**
 * @brief Initialize ADC0
*/
void ADC0_init(void)
{
    /* Reset + power enable */
    if ((ADC0->ULLMEM.GPRCM.PWREN & ADC12_PWREN_ENABLE_MASK) == 0U) {

        ADC0->ULLMEM.GPRCM.RSTCTL =
              ADC12_RSTCTL_KEY_UNLOCK_W
            | ADC12_RSTCTL_RESETASSERT_ASSERT
            | ADC12_RSTCTL_RESETSTKYCLR_CLR;

        ADC0->ULLMEM.GPRCM.PWREN =
              ADC12_PWREN_KEY_UNLOCK_W
            | ADC12_PWREN_ENABLE_ENABLE;
    }

    /* Sample clock = ULPCLK */
    ADC0->ULLMEM.GPRCM.CLKCFG =
          ADC12_CLKCFG_KEY_UNLOCK_W
        | ADC12_CLKCFG_SAMPCLK_ULPCLK
        | ADC12_CLKCFG_CCONRUN_ENABLE;

    /* CTL0: manual power-down, ENC off for config, SCLKDIV = ÷8 */
    ADC0->ULLMEM.CTL0 = 0U;
    ADC0->ULLMEM.CTL0 |= ADC12_CTL0_PWRDN_MANUAL;
    ADC0->ULLMEM.CTL0 |= ADC12_CTL0_SCLKDIV_DIV_BY_8;

    /* CTL1: software trigger, AUTO sample mode, single conversion */
    ADC0->ULLMEM.CTL1 = 0U;
    ADC0->ULLMEM.CTL1 |= ADC12_CTL1_TRIGSRC_SOFTWARE;
    ADC0->ULLMEM.CTL1 |= ADC12_CTL1_SAMPMODE_AUTO;
    ADC0->ULLMEM.CTL1 |= ADC12_CTL1_CONSEQ_SINGLE;
    ADC0->ULLMEM.CTL1 |= ADC12_CTL1_AVGN_DISABLE;
    ADC0->ULLMEM.CTL1 |= ADC12_CTL1_AVGD_SHIFT0;

    /* CTL2: unsigned, 12-bit, start/end = MEMCTL0 */
    ADC0->ULLMEM.CTL2 = 0U;
    ADC0->ULLMEM.CTL2 |= ADC12_CTL2_DF_UNSIGNED;
    ADC0->ULLMEM.CTL2 |= ADC12_CTL2_RES_BIT_12;
    ADC0->ULLMEM.CTL2 |= ADC12_CTL2_STARTADD_ADDR_00;
    ADC0->ULLMEM.CTL2 |= ADC12_CTL2_ENDADD_ADDR_00;

    /* MEMCTL0 ? channel 0 */
    ADC0->ULLMEM.MEMCTL[0] = 0U;
		
}

/**
 * @brief Retrieve ADC0 value
 * @return 12-bit ADC result
*/
uint32_t ADC0_getVal(void)
{
  	/* Enable conversions */
    ADC0->ULLMEM.CTL0 |= ADC12_CTL0_ENC_ON;
	
    /* Start conversion */
    ADC0->ULLMEM.CTL1 |= ADC12_CTL1_SC_START;

    /* Poll until MEMRES0 contains a valid result */
    while (1) {
        uint32_t mem = ADC0->ULLMEM.MEMRES[0];

        /* Valid when:
           - masked result is non-zero, OR
           - result is exactly zero (valid for GND) */
        if ((mem & ADC12_PERIPHERALREGIONSVT_SVTMEM_MEMRES_DATA_MASK) != 0U || mem == 0U)
            break;
    }

    return (ADC0->ULLMEM.MEMRES[0] & ADC12_PERIPHERALREGIONSVT_SVTMEM_MEMRES_DATA_MASK);
}


