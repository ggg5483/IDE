/**
 * car_main.c — IDE Car Project
 * Hybrid Steering (Servo & Differential Motors)
 *
 * OVERVIEW:
 * ---------
 * Control system uses both:
 *   1. Servo steering (TIMA1) — primary steering
 *   2. Differential motor torque (TIMA0) — secondary steering
 *
 * The servo handles large, smooth directional changes.
 * The differential motor split stabilizes the car and adds corrective torque.
 *
 * The camera provides a 128-pixel line scan. White track = high ADC values.
 * Filter threshold, binarize, compute centroid, filter it, and then fed into a PID.
 *
 * Throttle changes smoothly and never exceeds 50% (course rule).
 * A "watchdog" function stops the car if the track is lost for too long.
**/

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <ti/devices/msp/msp.h>

#include "switches.h"
#include "camera.h"
#include "adc12.h"
#include "timers.h"
#include "uart_extras.h"

/* ============================================================
 *                     CONFIGURATION CONSTANTS
 * ============================================================ */

/* Camera geometry */
#define CAMERA_PIXELS           128
#define CAMERA_CENTER           68.0f     // ideal centroid (accounting for off centered camera) increase if to the right of center, decrease if to the left of center
#define CENTER_DEADBAND         3.0f      // acceptable error before slowing/turning (for intersection/wavy)

/* Filtering */
#define FILTER_WINDOW           5         // moving-average window
#define NO_TRACK_LIMIT          10        // watchdog threshold (frames)

/* PID gains (tuned for hybrid control) */
#define KP  0.020f
#define KI  0.0005f
#define KD  0.001f

/* Servo PWM (TIMA1) */
#define SERVO_PERIOD_TICKS      640 
#define SERVO_CH                0
#define SERVO_MIN_US            5.0f    // full left
#define SERVO_CENTER_US         90.0f    // straight/init
#define SERVO_MAX_US            175.0f   // full right
#define STEARING_RAMP           0.01f    // smooth stearing

/* Motor PWM (TIMA0) */
#define MOTOR_PERIOD_TICKS      3200    
#define LEFT_CH                 0
#define RIGHT_CH                2

/* Throttle rules */
#define INIT_THROTTLE           0.00f
#define MAX_THROTTLE            0.45f     // never exceed 50%
#define TURN_THROTTLE           0.30f     // slowest throttle when steering
#define THROTTLE_RAMP_UP        0.005f    // smooth acceleration/deceleration
#define THROTTLE_RAMP_DOWN      0.05f     // smooth acceleration/deceleration

/* Differential steering scaling */
#define DIFF_SCALE              0.02f     // PID ? torque split
#define DIFF_MAX                0.20f     // ±20% torque redistribution (max variance in motor assisted turning)

/* Thresholding */
#define THRESH_FACTOR           0.55f     // adaptive threshold factor (to account for different light levels)

/* Motor Controller Enable */
#define LEFT_EN_MASK   (1U << 19)         // PB19
#define RIGHT_EN_MASK  (1U << 22)         // PA22

/* ============================================================
 *                CAMERA CENTROID + THRESHOLDING
 * ============================================================ */

/**
 * Compute centroid of the white region.
 * Steps:
 *   1. Compute mean + max of raw ADC values
 *   2. Adaptive threshold = mean + (max - mean)*THRESH_FACTOR
 *   3. Binarize pixels ("band pass filter")
 *   4. Compute centroid of all '1' pixels
 *
 * Returns:
 *   - centroid index (0–127)
 *   - -1 if no white region found
 */
static int compute_centroid(uint16_t *raw, uint8_t bin[128]) {
    uint32_t sum = 0, maxv = 0;

    for (int i = 0; i < 128; i++) {
        sum += raw[i];
        if (raw[i] > maxv) maxv = raw[i];
    }

    float mean = sum / 128.0f;
    float thr = mean + (maxv - mean) * THRESH_FACTOR;

    uint32_t wsum = 0, isum = 0;

    for (int i = 0; i < 128; i++) {
        if (raw[i] >= thr) {
            bin[i] = 1;
            wsum++;
            isum += i;
        } else {
            bin[i] = 0;
        }
    }

    if (wsum == 0) return -1;  // no track detected
    return (int)((float)isum / wsum + 0.5f);
}

/* ============================================================
 *                      Helper Functions
 * ============================================================ */

static void DC_ENABLE(void){
	if ((GPIOB->GPRCM.PWREN & GPIO_PWREN_ENABLE_MASK) == 0U) {
        GPIOB->GPRCM.RSTCTL = GPIO_RSTCTL_RESETASSERT_ASSERT |
                              GPIO_RSTCTL_KEY_UNLOCK_W |
                              GPIO_RSTCTL_RESETSTKYCLR_CLR;
        GPIOB->GPRCM.PWREN  = GPIO_PWREN_ENABLE_MASK |
                              GPIO_PWREN_KEY_UNLOCK_W;
    }

    // Power on GPIOA if needed
    if ((GPIOA->GPRCM.PWREN & GPIO_PWREN_ENABLE_MASK) == 0U) {
        GPIOA->GPRCM.RSTCTL = GPIO_RSTCTL_RESETASSERT_ASSERT |
                              GPIO_RSTCTL_KEY_UNLOCK_W |
                              GPIO_RSTCTL_RESETSTKYCLR_CLR;
        GPIOA->GPRCM.PWREN  = GPIO_PWREN_ENABLE_MASK |
                              GPIO_PWREN_KEY_UNLOCK_W;
    }

    // Configure PB19 as GPIO output (L DC ENABLE)
    IOMUX->SECCFG.PINCM[IOMUX_PINCM45] |= (1U | IOMUX_PINCM_PC_CONNECTED);   // PB19 = pincm45
    GPIOB->DOESET31_0 |= LEFT_EN_MASK;   // enable output driver
    GPIOB->DOUTSET31_0 = LEFT_EN_MASK;   // drive HIGH

    // Configure PA22 as GPIO output (R DC ENABLE)
    IOMUX->SECCFG.PINCM[IOMUX_PINCM47] |= (1U | IOMUX_PINCM_PC_CONNECTED);   // PA22 = PINCM47
    GPIOA->DOESET31_0 |= RIGHT_EN_MASK;  // enable output driver
    GPIOA->DOUTSET31_0 = RIGHT_EN_MASK;  // drive HIGH
}

static void delay_1ms(void){
    for(volatile int i = 0; i < 10000; i++){}
}

// Convert angle (0–180°) to duty cycle for a 1–2 ms pulse
double servo_angle_to_duty(int angle){
    double minPulse = 0.001;   // 1 ms
    double maxPulse = 0.002;   // 2 ms
    double pulse = minPulse + (angle / 180.0) * (maxPulse - minPulse);
    return pulse / 0.020;      // divide by 20 ms period
}

/* ============================================================
 *                          MAIN LOOP
 * ============================================================ */
int main(void) {
    
    int angle = SERVO_CENTER_US;
    int no_track = 0;
    float throttle = TURN_THROTTLE;
  	uint8_t bin[CAMERA_PIXELS];
    int offCen = 0;
    
    /* PID state */
    float pid_error = 0.0f;
    float pid_prev_error = 0.0f;
    float pid_integral = 0.0f;
    float pid_derivative = 0.0f;
    float pid_output = 0.0f;
    
        /* Camera/ADC Init */
				ADC0_init();
				Camera_init();

				/* DC Init */
				TIMA0_PWM_init(LEFT_CH, MOTOR_PERIOD_TICKS, 0, INIT_THROTTLE );
				TIMA0_PWM_init(RIGHT_CH, MOTOR_PERIOD_TICKS, 0, INIT_THROTTLE );
    
				/* Enable DC motors */
				DC_ENABLE();
				
				/* SERVO Init */
        TIMA1_PWM_init(SERVO_CH, SERVO_PERIOD_TICKS, 0, servo_angle_to_duty(SERVO_CENTER_US));
        
        /* roughly 5s delay before starting to run the car */
        for(volatile int i = 0; i < 500; i++) {
            delay_1ms();
        }
        
        /* start motors */
        TIMA0_PWM_DutyCycle(LEFT_CH, TURN_THROTTLE);   
        TIMA0_PWM_DutyCycle(RIGHT_CH, TURN_THROTTLE);
    
        /* start stearing loop */
        while(1){
            /* Wait for camera frame */
            if (!Camera_isDataReady()) {
                            delay_1ms();
            }
                        
						/* Retrieves camera data */
            uint16_t *raw = Camera_getData();

            /* Compute centroid */
            int cen = compute_centroid(raw, bin);
                       
						/* PID error based on centroid offset */
						pid_error = (CAMERA_CENTER - cen);
						
						/* PID accumulate */
						pid_integral += pid_error;
						pid_derivative = pid_error - pid_prev_error;
						pid_prev_error = pid_error;

						/* PID output */
						pid_output = (KP * pid_error) + (KI * pid_integral) + (KD * pid_derivative);

						/* Servo steering using PID */
						angle = SERVO_CENTER_US + (pid_output * 2.5f); //adjust for how responsive we want the stearing control
						if (angle < SERVO_MIN_US) {angle = SERVO_MIN_US;}
            else if (angle > SERVO_MAX_US) {angle = SERVO_MAX_US;}
            TIMA1_PWM_DutyCycle(SERVO_CH, servo_angle_to_duty(angle));

						/* Throttle control using PID magnitude */
						float absErr = fabsf(pid_error);

						/* Normalize error (0.0 to 1.0) */
						float norm = absErr / CAMERA_CENTER;
						if (norm > 1.0f) norm = 1.0f;

						/* Compute target throttle:
						- MAX_THROTTLE on straights
						- TURN_THROTTLE in hard turns
						*/
						float target = MAX_THROTTLE - (norm * (MAX_THROTTLE - TURN_THROTTLE));

						/* Smooth ramp toward target */
						if (throttle < target) {
							throttle += THROTTLE_RAMP_UP;
							if (throttle > target) throttle = target;
						} else {
							throttle -= THROTTLE_RAMP_DOWN;
							if (throttle < target) throttle = target;
						}

						/* Apply throttle */
						TIMA0_PWM_DutyCycle(LEFT_CH, throttle);
						TIMA0_PWM_DutyCycle(RIGHT_CH, throttle);

                        
						/* Track end/carpet stop check, checks if NO_TRACK_LIMIT number of consecutive frames logged had no track data*/
						if (cen == 0) {
							no_track++;
							if (no_track>NO_TRACK_LIMIT) {
								/* Track has ended or been lost, end/idle program */
								TIMA0_PWM_DutyCycle(LEFT_CH, INIT_THROTTLE);   
								TIMA0_PWM_DutyCycle(RIGHT_CH, INIT_THROTTLE);  
								TIMA1_PWM_DutyCycle(SERVO_CH, servo_angle_to_duty(SERVO_CENTER_US));
								break;
							}
                            
						} else { 
							no_track = 0; 
						}
						
        }
        
}



//int main(void) {
//	
//		int angle = SERVO_CENTER_US;
//		int no_track = 0;
//	  float throttle = TURN_THROTTLE;
//	  uint8_t bin[CAMERA_PIXELS];
//	  int offCen = 0;
//	
//		/* Camera/ADC Init */
//    ADC0_init();
//  	Camera_init();

//    /* DC Init */
//    TIMA0_PWM_init(LEFT_CH, MOTOR_PERIOD_TICKS, 0, INIT_THROTTLE );
//    TIMA0_PWM_init(RIGHT_CH, MOTOR_PERIOD_TICKS, 0, INIT_THROTTLE );
//	
//		/* Enable DC motors */
//		DC_ENABLE();
//	
//    /* SERVO Init */
//    TIMA1_PWM_init(SERVO_CH, SERVO_PERIOD_TICKS, 0, servo_angle_to_duty(SERVO_CENTER_US));
//		
//		/* roughly 5s delay before starting to run the car */
//		for(volatile int i = 0; i < 500; i++) {
//			delay_1ms();
//		}
//		
//		/* start motors */
//		TIMA0_PWM_DutyCycle(LEFT_CH, TURN_THROTTLE);   
//    TIMA0_PWM_DutyCycle(RIGHT_CH, TURN_THROTTLE);
//	
//		/* start stearing loop */
//	  while(1){
//            /* Wait for camera frame */
//            if (!Camera_isDataReady()) {
//							delay_1ms();
//            }
//						
//						/* Retrieves camera data */
//            uint16_t *raw = Camera_getData();

//            /* Compute centroid and centroid error */
//            int cen = compute_centroid(raw, bin);
//						float cenError = (CAMERA_CENTER-cen)/CAMERA_CENTER;
//						
//						/* Check if centroid is centered accounting for deadband */
//						if (cen>CAMERA_CENTER-CENTER_DEADBAND && cen<CAMERA_CENTER+CENTER_DEADBAND) {
//							/* If car centered, speed up dc motors */
//							if (throttle < MAX_THROTTLE) {throttle = throttle+THROTTLE_RAMP_UP;}
//							TIMA0_PWM_DutyCycle(LEFT_CH, throttle);   
//							TIMA0_PWM_DutyCycle(RIGHT_CH, throttle);   
//							TIMA1_PWM_DutyCycle(SERVO_CH, servo_angle_to_duty(SERVO_CENTER_US));
//						} else if (throttle < TURN_THROTTLE+THROTTLE_RAMP_DOWN) {
//							throttle = throttle-(throttle-TURN_THROTTLE);
//						} else {
//							throttle = throttle-THROTTLE_RAMP_DOWN;
//						}
//						
//						/* turning is now based off of error percentage from center for more responsive stearing */
//            if (cen<CAMERA_CENTER-CENTER_DEADBAND || cen>CAMERA_CENTER+CENTER_DEADBAND) {
//							TIMA0_PWM_DutyCycle(LEFT_CH, throttle);   
//							TIMA0_PWM_DutyCycle(RIGHT_CH, throttle);   
//							angle = SERVO_CENTER_US+(2.5*(SERVO_CENTER_US*cenError)); //swap + to - if stearing is backwards
//							if (angle < SERVO_MIN_US) {angle=SERVO_MIN_US;}
//							else if (angle > SERVO_MAX_US) {angle=SERVO_MAX_US;}
//							TIMA1_PWM_DutyCycle(SERVO_CH, servo_angle_to_duty(angle));
//						}
//						
//						/* Track end/carpet stop check, checks if NO_TRACK_LIMIT number of consecutive frames logged had no track data*/
//						if (cen == 0) {
//							no_track++;
//							if (no_track>NO_TRACK_LIMIT) {
//								/* Track has ended or been lost, end/idle program */
//								TIMA0_PWM_DutyCycle(LEFT_CH, INIT_THROTTLE);   
//						   	TIMA0_PWM_DutyCycle(RIGHT_CH, INIT_THROTTLE);  
//								TIMA1_PWM_DutyCycle(SERVO_CH, servo_angle_to_duty(SERVO_CENTER_US));
//								break;
//						  }
//							
//						} else { 
//							no_track = 0; 
//						}

//		}
//		
//}
