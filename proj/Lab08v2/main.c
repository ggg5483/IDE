/**
 * ******************************************************************************
 * @file    : main.c
 * @brief   : 
 * @details : 
 * 
 * @author Alexander Hamadeh
 * @author Garrett Geyer
 * @date 2/17/2026
 * ******************************************************************************
*/

#include <ti/devices/msp/msp.h>
#include "lab8/timers.h"
#include "uart_extras.h"
#include "lab8/adc12.h"
#include "lab8/uart.h"
#include <stdint.h>
#include <stdbool.h>

/* Configuration */
#define SAMPLE_RATE_HZ        200U                   /* ISR sampling frequency (Hz) */
#define WINDOW_SAMPLES        (SAMPLE_RATE_HZ / 10U) /* 0.1s window -> SAMPLE_RATE_HZ * 0.1 */
#define MIN_PEAK_SEP_MS       40U                    /* ignore peaks closer than this (ms) (avoid double counts) */
#define TIMER_CLOCK_HZ        80000000U              /* timer input clock for TIMG6_init */
#define SCALE_TO_BPM          600U                   /* 60 / 0.1 = 600 */
#define THRESHOLD 						2200U 									/*permanent threshold value*/
 
/* compile-time allowed size check */
#if WINDOW_SAMPLES < 1 || WINDOW_SAMPLES > 256
#error "WINDOW_SAMPLES must be between 1 and 256"
#endif

/* Forward declaration for ISR to avoid missing-prototype warning */
void TIMG6_IRQHandler(void);

/* Circular buffer to hold the most recent WINDOW_SAMPLES ADC values */
static uint16_t buf[WINDOW_SAMPLES];
static uint32_t head = 0;            /* index of oldest sample in buffer */
static uint32_t count = 0;           /* number of samples currently in buffer */

/* Peak detection state */
static uint16_t prev = 0;            /* previous ADC sample */
static bool rising = false;          /* whether signal was rising */
static uint32_t lastPeakIdx = 0;     /* sample index of last detected peak */
static uint32_t idx = 0;             /* running sample index */
static uint32_t peaks = 0;           /* peaks counted in current window */
static uint16_t winMax = 0;          /* running max in window */
static uint16_t winMin = 0x0FFF;     /* running min in window */

/* Push a new sample into the circular window buffer and update min/max */
static void push(uint16_t v){
    if (count < WINDOW_SAMPLES){
        buf[(head + count) % WINDOW_SAMPLES] = v;
        count++;
    } else {
        buf[head] = v;
        head = (head + 1) % WINDOW_SAMPLES;
    }
    if (v > winMax) winMax = v;
    if (v < winMin) winMin = v;
}

int main(void)
{
    UART0_init(); /* enable UART for printing */

	  UART0_put("\033[48;5;231m\033[38;5;232m"); // background white, text dark
		UART0_put("\033[2J\033[H"); //clear screen
		UART0_put("UART INITIALIZED!\r\n");
    /* compute timer period to achieve SAMPLE_RATE_HZ and start TIMG6 */

    TIMG6_init(32, 0);

    ADC0_init(); /* initialize ADC0 */

    /* seed previous sample and window with one reading */
    prev = (uint16_t)(ADC0_getVal() & 0x0FFF);
    push(prev);

    /* main loop sleeps; work happens in TIMG6 ISR */
    while (1) __WFI();
}

void TIMG6_IRQHandler(void){
    /* clear timer interrupt */
    TIMG6->CPU_INT.ICLR = GPTIMER_CPU_INT_ICLR_Z_CLR;

    /* read ADC (12-bit) */
    uint16_t sample = (uint16_t)(ADC0_getVal() & 0x0FFF);
	
		#if 0
		UART0_printDec(sample);
		UART0_put("\r\n");
		#endif
    /* add to sliding window and update min/max */
    push(sample);

    /* threshold = midpoint between window max and min (very simple) */
    //uint16_t threshold = (uint16_t)(((uint32_t)winMax + (uint32_t)winMin) / 2U);
		uint16_t threshold = THRESHOLD;
    /* minimum separation between peaks in samples to avoid double-counting */
    uint32_t minSepSamples = (SAMPLE_RATE_HZ * MIN_PEAK_SEP_MS) / 1000U;

	
		if(sample > prev){
			rising = true;
			#if 0
			UART0_put("rising!\r\n");
			UART0_printDec(idx);
			#endif
			/*rising*/
		} else if ((sample < prev) && rising && (sample > threshold)){

			#if 0
			UART0_put("peak!\r\n");
			#endif
			/*peak*/
			//if( lastPeakIdx != 0 ){
					float time = (float) idx - (float) lastPeakIdx;
					#if 0
					UART0_printFloat(time);
					UART0_put("peak!\r\n");
					#endif
					//bpm = 1000/time * 60
					if(time > 100){
						time =  1000/time;
						time *= 60;
						UART0_put("BPM: ");
						UART0_printDec((int)time);
						UART0_put("\r\n");
						idx = 0;
					}
			//}
			lastPeakIdx = idx;
			rising = false;
		}
		prev = sample;
		idx++;
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		#if 0
    /* detect a local maximum: previously rising and now falling */
    if (sample > prev) {
        rising = true;
    } else if (rising && (sample < prev)) {
        uint32_t sinceLast = idx - lastPeakIdx;
				if( lastPeakIdx != 0 ){
							float time = (float) idx - (float) lastPeakIdx;
							//bpm = 1000/time * 60
							time =  1000/time;
							time *= 60;
							UART0_put("BPM: ");
							UART0_printDec((int)time);
							UART0_put("\r\n");
				}
        /* count peak only if it exceeds threshold and is sufficiently separated */
        if ((prev > threshold) && (sinceLast >= minSepSamples)) {
            peaks++;
						
            lastPeakIdx = idx;
        }
        rising = false;
    }

    prev = sample;
    idx++;
	
    /* when we've collected WINDOW_SAMPLES samples, compute BPM and print */
    if ((idx % WINDOW_SAMPLES) == 0) {
        /* BPM = peaks * SCALE_TO_BPM (SCALE_TO_BPM = 60 / 0.1 = 600) */
        uint32_t bpm = peaks * SCALE_TO_BPM;

				
				if(bpm > 0){
					/* print only the required output */
					UART0_put("BPM: ");
					UART0_printDec((int)bpm);
					UART0_put("\r\n");
				}
				
        /* reset counters and window stats for next measurement window */
        peaks = 0;
        head = 0;
        count = 0;
        winMax = 0;
        winMin = 0x0FFF;
        lastPeakIdx = idx;
    }
		#endif
}
