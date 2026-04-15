/**
 * ******************************************************************************
 * @file    : main.c
 * @brief   : heartbeat monitor
 * @details : monitors a heart beat via connected light sensor
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
#define SAMPLE_RATE_HZ        1000U                   /* ISR sampling frequency (Hz) */
#define WINDOW_SAMPLES        (SAMPLE_RATE_HZ / 10U)  /* 0.1s window -> SAMPLE_RATE_HZ * 0.1 */
#define MIN_PEAK_SEP_MS       10                      /* ignore peaks closer than this (ms) (avoid double counts)  */
#define TIMER_CLOCK_HZ        80e6U                   /* timer input clock for TIMG6_init */
#define SCALE_TO_BPM          600U                    /* 60 / 0.1 = 600 */
#define THRESHOLD 						700 									  /*permanent threshold value*/
#define PEAKS_AVG_NUM					5			              	/*number of peaks to average*/
 
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
static uint32_t bpm = 0;					   /*constant bpm*/
static uint32_t ms = 0;							 /*ms counter*/
static uint32_t last_peak_ms = 0;		 /*ms of last peak*/
static uint32_t peaks = 0;           /* peaks counted in current window */
static uint16_t sample = 0;
static uint32_t period = 0;

static uint16_t winMax = 0;          /* running max in window */
static uint16_t winMin = 0x0FFF;     /* running min in window */
static uint32_t lastPeakIdx = 0;     /* sample index of last detected peak */
static uint32_t idx = 0;             /* running sample index */

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

	  UART0_put("\033[?25h\033[48;5;231m\033[38;5;232m"); // show cursor, background white, text dark
		UART0_put("\033[2J\033[H"); //clear screen
		UART0_put("UART INITIALIZED!\r\n");
    /* compute timer period to achieve SAMPLE_RATE_HZ and start TIMG6 */

    TIMG6_init(32, 0);
		TIMG12_init(1500000);

    ADC0_init(); /* initialize ADC0 */

    /* seed previous sample and window with one reading */
    prev = (uint16_t)(ADC0_getVal() & 0x0FFF);
    push(prev);

    /* main loop sleeps; work happens in TIMG6 ISR */
		int last_samp = 0;
		UART0_put("\033[?25l"); //hide cursor
    while (1){
			/* read ADC (12-bit) */
			
			#if 0
			UART0_put("\r\ntime since sample: ");
			UART0_printDec(ms-last_samp);
			last_samp = ms;
			UART0_put("       \033[F");
			#endif
			
			/*print BPM*/
			UART0_put("          \rBPM: ");
			UART0_printDec((int) bpm);
			
			//debug printing
			#if 0 //disable all
			#if 1
			UART0_put("\r\nADC val: ");
			UART0_printDec(sample);
			UART0_put("       \033[F");
			#endif
			#if 1
			UART0_put("\r\n\nms val: ");
			UART0_printDec(ms);
			UART0_put("         \033[F\033[F");
			#endif
			#if 1
			UART0_put("\r\n\n\nlast_peak_ms val: ");
			UART0_printDec(last_peak_ms);
			UART0_put("       \033[F\033[F\033[F");
			#endif
			#if 1
			UART0_put("\r\n\n\n\n");
			if(rising){
				UART0_put(" rising!   ");
			} else {
				UART0_put(" falling!   ");
			}
			UART0_put("\033[F\033[F\033[F\033[F");
			#endif
			#if 1
			UART0_put("\r\n\n\n\n\npeaks : ");
			UART0_printDec(peaks);
			UART0_put("      \033[F\033[F\033[F\033[F\033[F");
			#endif
			#if 1
			UART0_put("\r\n\n\n\n\n\nperiod : ");
			UART0_printDec(period);
			UART0_put("      \033[F\033[F\033[F\033[F\033[F\033[F");
			#endif
			#endif //disable all
			
		}
}

static int times_risen = 0;

void TIMG12_IRQHandler(void){
		TIMG12->CPU_INT.ICLR = GPTIMER_CPU_INT_ICLR_Z_CLR;
		sample = (uint16_t)(ADC0_getVal() & 0x0FFF);
	
}
void TIMG6_IRQHandler(void){
    /* clear timer interrupt */
    TIMG6->CPU_INT.ICLR = GPTIMER_CPU_INT_ICLR_Z_CLR;


		if(sample > prev){
			if (times_risen < 1){
				times_risen++;
			} else {	
				rising = true;
			}

		} else if ((prev-sample)<500 && (sample < prev) && rising && (sample > THRESHOLD) && ((ms - last_peak_ms) > MIN_PEAK_SEP_MS)){ //increase MIN_PEAK_SEP to avoid double count?
		//} else if ((sample < prev) && rising ){//&& (sample > THRESHOLD) && ((ms - last_peak_ms) > MIN_PEAK_SEP_MS)){
			/*peak*/
			peaks++;

			if((peaks % PEAKS_AVG_NUM == 0) && ms > MIN_PEAK_SEP_MS){
				
				period = ms / PEAKS_AVG_NUM;

				// update bpm, do calculations as a float

				bpm = (60000/period);
				if(bpm < 50|| bpm >250){
					bpm = 0;
				}
				ms = 0;
				peaks = 0;
			}

			last_peak_ms = ms;
			rising = false;
			times_risen = 0;
		}
		prev = sample;
		ms++;
		

}
