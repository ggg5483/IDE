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
 * The PID output drives:
 *   - Servo angle (primary)
 *   - Differential torque split (secondary)
 *
 * Throttle changes smoothly and never exceeds 50% (course rule).
 * A "watchdog" function stops the car if the track is lost for too long.
 *
 * --------------------------------------------------------------------------------------------------------------------------- *
 * Work needs to be done for timers: for camera, currently TIMG0: 100 kHz clock (319, 0) needs to be set to output pin to PA12
 * and TIMG6: integration time ~7.5ms (60000, 3), needs to be set to output pin to PA28 Camera SI.
 *
 * and the following for motors:
//    // DC
//    // 10 kHz period = 100 µs 100 µs * 80 MHz = 8000 timer counts was (3200)
//      TIMA0_PWM_init(0 (L), MOTOR_PERIOD_TICKS, 0, 0.0);// 0% duty cycle off initilization
//      TIMA0_PWM_init(2 (R), MOTOR_PERIOD_TICKS, 0, 0.0);// 0% duty cycle off initilization
//    Need to set lefts timer output pin to PB8
//          Set enable pin PB19
//    Need to set lefts timer output pin to PB17
//          Set enable pin PA22

//    // SERVO
//    // 20 ms period = 640 counts at 32 kHz was (640)
//    TIMA1_PWM_init(SERVO_CH (0), SERVO_PERIOD_TICKS, 0, servo_frac); straight off initilization
//    Need to set this timers output pin to PB4
 *
 * Work needs to be done on adc12 to make sure it's data recieving pin is set to PA27
 * --------------------------------------------------------------------------------------------------------------------------- *
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
#define CAMERA_CENTER           64.0f     // ideal centroid
#define CENTER_DEADBAND         3.0f      // acceptable error before slowing

/* Filtering */
#define FILTER_WINDOW           5         // moving-average window
#define NO_TRACK_LIMIT          6         // watchdog threshold (frames)

/* PID gains (tuned for hybrid control) */
#define KP  0.020f
#define KI  0.0005f
#define KD  0.001f

/* Servo PWM (TIMA1) */
#define SERVO_PERIOD_TICKS      640 
#define SERVO_CH                0
#define SERVO_MIN_US            180.0f   // full left
#define SERVO_CENTER_US         90.0f    // straight/init
#define SERVO_MAX_US            0.0f     // full right
#define STEARING_RAMP           0.07f    // smooth stearing

/* Motor PWM (TIMA0) */
#define MOTOR_PERIOD_TICKS      3200    
#define LEFT_CH                 0
#define RIGHT_CH                2

/* Throttle rules */
#define INIT_THROTTLE           0.00f
#define MAX_THROTTLE            0.50f     // never exceed 50%
#define TURN_THROTTLE           0.30f     // slowest throttle when steering
#define THROTTLE_RAMP           0.03f     // smooth acceleration/deceleration

/* Differential steering scaling */
#define DIFF_SCALE              0.02f     // PID ? torque split
#define DIFF_MAX                0.20f     // ±20% torque redistribution (max variance in motor assisted turning)

/* Thresholding */
#define THRESH_FACTOR           0.55f     // adaptive threshold factor (to account for different light levels)

/* Motor Controller Enable (CAN'T FIND THE DUMB MACRO)*/
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
 *                MOVING-AVERAGE FILTER FOR CENTROID
 * ============================================================ */

//typedef struct {
//    int buf[FILTER_WINDOW];
//    int idx, count;
//} CentroidFilter;

///** Initialize filter buffer */
//static void cf_init(CentroidFilter *f) {
//    for (int i = 0; i < FILTER_WINDOW; i++) f->buf[i] = -1;
//    f->idx = 0;
//    f->count = 0;
//}

///** Push new centroid value into filter */
//static void cf_push(CentroidFilter *f, int v) {
//    f->buf[f->idx] = v;
//    f->idx = (f->idx + 1) % FILTER_WINDOW;
//    if (f->count < FILTER_WINDOW) f->count++;
//}

///** Compute filtered centroid (ignoring -1 entries) */
//static float cf_get(CentroidFilter *f) {
//    int sum = 0, n = 0;
//    for (int i = 0; i < FILTER_WINDOW; i++) {
//        if (f->buf[i] >= 0) {
//            sum += f->buf[i];
//            n++;
//        }
//    }
//    return (n == 0) ? -1.0f : (float)sum / n;
//}


/* ============================================================
 *                          MAIN LOOP
 * ============================================================ */

///* Small busy delay used for ms waits */
//static void busy_delay_ms(uint32_t ms) {
//    volatile uint32_t outer = ms;
//    while (outer--) {
//        volatile uint32_t inner = 12000U; // approximate inner loop for ~1ms; adjust if needed
//        while (inner--) __asm__("nop");
//    }
//}

///* Debounced read of S1 (active-high). Returns 1 if pressed (stable). */
//static int read_start_button_debounced(void) {
//    if (!S1_pressed()) return 0;
//    busy_delay_ms(50);
//    return S1_pressed() ? 1 : 0;
//}

///* Wait until S1 is pressed (blocking). Returns after a stable press is detected. */
//static void wait_for_start_press(void) {
//    while (1) {
//        if (read_start_button_debounced()) {
//            return;
//        }
//        busy_delay_ms(10);
//    }
//}

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

static void delay_10ms(void){
    for(volatile int i = 0; i < 100000; i++){}
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



int main(void) {
	
		int angle = SERVO_CENTER_US;
		int no_track = 0;
	  float throttle = 0.0;
	  uint8_t bin[CAMERA_PIXELS];
	
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
		
		/* small 5s delay before starting to run the car */
		for(volatile int i = 0; i < 500; i++) {
			delay_10ms();
		}
		
		/* start motors */
		TIMA0_PWM_DutyCycle(LEFT_CH, TURN_THROTTLE);   
    TIMA0_PWM_DutyCycle(RIGHT_CH, TURN_THROTTLE);
	
		/* start stearing loop */
	  while(1){
            /* Wait for camera frame */
            if (!Camera_isDataReady()) {
							delay_10ms();
            }
						
						/* Retrieves camera data */
            uint16_t *raw = Camera_getData();

            /* Compute centroid */
            int cen = compute_centroid(raw, bin);
						
						/* Check if centroid is centered accounting for deadband */
						if (cen<CAMERA_CENTER+CENTER_DEADBAND && cen>CAMERA_CENTER-CENTER_DEADBAND) {
							/* If car not centered, so slow down dc motors */
							if (throttle < MAX_THROTTLE) {throttle = throttle+THROTTLE_RAMP;}
							TIMA0_PWM_DutyCycle(LEFT_CH, throttle);   
							TIMA0_PWM_DutyCycle(RIGHT_CH, throttle);   
							TIMA1_PWM_DutyCycle(SERVO_CH, servo_angle_to_duty(SERVO_CENTER_US));
						} else if (throttle < TURN_THROTTLE+THROTTLE_RAMP) {
							throttle = throttle-(throttle-TURN_THROTTLE);
						} else {
							throttle = throttle-THROTTLE_RAMP;
						
						/* Check if right turn */
						/* If so turn servos right */
            if (cen>CAMERA_CENTER+CENTER_DEADBAND) {
							TIMA0_PWM_DutyCycle(LEFT_CH, throttle);   
							TIMA0_PWM_DutyCycle(RIGHT_CH, throttle);   
							angle = angle-STEARING_RAMP;
							if (angle < SERVO_MIN_US) {angle=SERVO_MIN_US;}
							TIMA1_PWM_DutyCycle(SERVO_CH, servo_angle_to_duty(angle));
						}
						
						/* Check if left turn */
						/* If so turn servos left */
						if (cen<CAMERA_CENTER-CENTER_DEADBAND) {
							TIMA0_PWM_DutyCycle(LEFT_CH, throttle);   
							TIMA0_PWM_DutyCycle(RIGHT_CH, throttle);   
							angle = angle+STEARING_RAMP;
							if (angle > SERVO_MAX_US) {angle=SERVO_MAX_US;}
              TIMA1_PWM_DutyCycle(SERVO_CH, servo_angle_to_duty(angle));
						}
						
						/* Track end/carpet stop check, checks if NO_TRACK_LIMIT number of consecutive frames logged had no track data*/
						if (cen < 0) {
							no_track++;
							if (no_track>NO_TRACK_LIMIT) {
								/* Track has ended or been lost, end/idle program */
								TIMA0_PWM_DutyCycle(LEFT_CH, INIT_THROTTLE);   
						   	TIMA0_PWM_DutyCycle(RIGHT_CH, INIT_THROTTLE);  
								break;
						  }
							
						} else { 
							no_track = 0; 
						}

						delay_1ms();
		}
		
}





//    /* ---------- Initialize subsystems ---------- */

//    ADC0_init();
//    Camera_init();

//    /* Motor PWM (TIMA0) */
//    TIMA0_PWM_init(LEFT_CH, MOTOR_PERIOD_TICKS, 0, 0.0);
//    TIMA0_PWM_init(RIGHT_CH, MOTOR_PERIOD_TICKS, 0, 0.0);
//	
//		//---------------------------------------Motor Enable------------------------------------------------
//	  // Power on GPIOB if needed
//    if ((GPIOB->GPRCM.PWREN & GPIO_PWREN_ENABLE_MASK) == 0U) {
//        GPIOB->GPRCM.RSTCTL = GPIO_RSTCTL_RESETASSERT_ASSERT |
//                              GPIO_RSTCTL_KEY_UNLOCK_W |
//                              GPIO_RSTCTL_RESETSTKYCLR_CLR;
//        GPIOB->GPRCM.PWREN  = GPIO_PWREN_ENABLE_MASK |
//                              GPIO_PWREN_KEY_UNLOCK_W;
//    }

//    // Power on GPIOA if needed
//    if ((GPIOA->GPRCM.PWREN & GPIO_PWREN_ENABLE_MASK) == 0U) {
//        GPIOA->GPRCM.RSTCTL = GPIO_RSTCTL_RESETASSERT_ASSERT |
//                              GPIO_RSTCTL_KEY_UNLOCK_W |
//                              GPIO_RSTCTL_RESETSTKYCLR_CLR;
//        GPIOA->GPRCM.PWREN  = GPIO_PWREN_ENABLE_MASK |
//                              GPIO_PWREN_KEY_UNLOCK_W;
//    }

//    // Configure PB19 as GPIO output (L DC ENABLE)
//    IOMUX->SECCFG.PINCM[IOMUX_PINCM45] |= (1U | IOMUX_PINCM_PC_CONNECTED);   // PB19 = pincm45
//    GPIOB->DOESET31_0 |= LEFT_EN_MASK;   // enable output driver
//    GPIOB->DOUTSET31_0 = LEFT_EN_MASK;   // drive HIGH

//    // Configure PA22 as GPIO output (R DC ENABLE)
//    IOMUX->SECCFG.PINCM[IOMUX_PINCM47] |= (1U | IOMUX_PINCM_PC_CONNECTED);   // PA22 = PINCM47
//    GPIOA->DOESET31_0 |= RIGHT_EN_MASK;  // enable output driver
//    GPIOA->DOUTSET31_0 = RIGHT_EN_MASK;  // drive HIGH
//		//---------------------------------------------------------------------------------------------------
//		
//    /* Servo PWM (TIMA1) — start centered */
//    float servo_frac = pulse_to_frac(SERVO_CENTER_US);
//    TIMA1_PWM_init(SERVO_CH, SERVO_PERIOD_TICKS, 0, servo_frac);

//    /* Control state variables */
//    float throttle = 0.0f;
//    float target_throttle = MAX_THROTTLE;

//    float integ = 0, prev_err = 0;

//    CentroidFilter filt;
//    cf_init(&filt);

//    int no_track = 0;
//    uint8_t bin[128];

//    /* ============================================================
//     *                     CONTROL LOOP
//     * ============================================================ */

//    /* Initialize start switch S1 (polling mode) */
//    S1_init();

//    /* Ensure motors and servo are safe at boot */
//    TIMA0_PWM_DutyCycle(LEFT_CH, 0.0);
//    TIMA0_PWM_DutyCycle(RIGHT_CH, 0.0);
//    TIMA1_PWM_DutyCycle(SERVO_CH, servo_frac);

//    typedef enum { STATE_IDLE = 0, STATE_ARMING, STATE_RUNNING } system_state_t;
//    system_state_t state = STATE_IDLE;

//    while (1) {

//        if (state == STATE_IDLE) {
//            /* Idle: wait for start button press */
//            wait_for_start_press();

//            /* When pressed, move to arming state (countdown) */
//            state = STATE_ARMING;

//            /* 5 second arming delay with periodic feedback */
//            uint32_t elapsed = 0;
//            while (elapsed < 5000U) {
//                busy_delay_ms(1000);
//                elapsed += 1000U;
//            }

//            /* Reset control integrators and filters before running */
//            integ = 0.0f;
//            prev_err = 0.0f;
//            cf_init(&filt);
//            no_track = 0;

//            /* Set initial throttle target and ramp from zero */
//            target_throttle = MAX_THROTTLE;
//            state = STATE_RUNNING;
//        }

//        if (state == STATE_RUNNING) {

//            /* Wait for camera frame */
//            if (!Camera_isDataReady()) {
//                busy_delay_ms(1);
//                continue;
//            }
//            uint16_t *raw = Camera_getData();

//            /* Compute centroid */
//            int cen = compute_centroid(raw, bin);

//            /* Watchdog tracking */
//            if (cen < 0) no_track++;
//            else { no_track = 0; cf_push(&filt, cen); }

//            float fcen = cf_get(&filt);
//            bool on_track = (fcen >= 0);

//            /* If track lost/too long ? stop car and return to IDLE */
//            if (no_track >= NO_TRACK_LIMIT) {

//                target_throttle = 0;
//                while (throttle > 0.001f) {
//                    throttle = ramp(throttle, target_throttle, THROTTLE_RAMP);
//                    TIMA0_PWM_DutyCycle(LEFT_CH, (double) throttle);
//                    TIMA0_PWM_DutyCycle(RIGHT_CH, (double) throttle);
//                    busy_delay_ms(10);
//                }

//                /* Center servo for safety */
//                servo_frac = pulse_to_frac(SERVO_CENTER_US);
//                TIMA1_PWM_DutyCycle(SERVO_CH, (double) servo_frac);

//                /* Reset integrators and filter to avoid stale state */
//                integ = 0.0f;
//                prev_err = 0.0f;
//                cf_init(&filt);
//                no_track = 0;

//                /* Move to IDLE and wait for next start press */
//                state = STATE_IDLE;
//                continue;
//            }

//            /* Compute steering error */
//            float err = on_track ? (CAMERA_CENTER - fcen) : 0;

//            /* PID update */
//            integ += err;
//            float deriv = err - prev_err;
//            prev_err = err;

//            float pid = KP*err + KI*integ + KD*deriv;


//            /* ============================================================
//             *                PRIMARY STEERING — SERVO
//             * ============================================================ */

//            /* Convert PID ? servo pulse width */
//            float servo_us = SERVO_CENTER_US + pid * 6.0f;  // 6us per pixel

//            /* Clamp to servo limits */
//            if (servo_us < SERVO_MIN_US) servo_us = SERVO_MIN_US;
//            if (servo_us > SERVO_MAX_US) servo_us = SERVO_MAX_US;

//            /* Smooth servo motion */
//            servo_frac = ramp(servo_frac, pulse_to_frac(servo_us), 0.001f);
//            TIMA1_PWM_DutyCycle(SERVO_CH, (double) servo_frac);


//            /* ============================================================
//             *          SECONDARY STEERING — DIFFERENTIAL TORQUE
//             * ============================================================ */

//            /* PID ? differential torque split */
//            float diff = pid * DIFF_SCALE;
//            if (diff > DIFF_MAX) diff = DIFF_MAX;
//            if (diff < -DIFF_MAX) diff = -DIFF_MAX;


//            /* ============================================================
//             *                     THROTTLE CONTROL
//             * ============================================================ */

//            /* Slow down when steering significantly */
//            if (on_track && fabsf(err) <= CENTER_DEADBAND)
//                target_throttle = MAX_THROTTLE;
//            else
//                target_throttle = TURN_THROTTLE;

//            /* Smooth throttle changes */
//            throttle = ramp(throttle, target_throttle, THROTTLE_RAMP);


//            /* ============================================================
//             *                     MOTOR OUTPUT
//             * ============================================================ */

//            /* Apply differential steering */
//            float L = throttle * (1 - diff);
//            float R = throttle * (1 + diff);

//            /* Enforce 50% max rule */
//            float maxLR = fmaxf(L, R);
//            if (maxLR > MAX_THROTTLE) {
//                float s = MAX_THROTTLE / maxLR;
//                L *= s;
//                R *= s;
//            }

//            /* Prevent negative torque */
//            if (L < 0) L = 0;
//            if (R < 0) R = 0;

//            /* Output PWM */
//            TIMA0_PWM_DutyCycle(LEFT_CH, (double) L);
//            TIMA0_PWM_DutyCycle(RIGHT_CH, (double) R);
//        }

//        /* small delay to avoid tight loop */
//        busy_delay_ms(1);
//    }
//}
