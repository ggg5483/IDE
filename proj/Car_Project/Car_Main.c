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
 * Work needs to be done for timers to have two for camera "currently TIMG0: 100 kHz clock (319, 0) and TIMG6: integration time ~7.5ms (60000, 3)",
 * and the following for motors:
//    // DC
//    // 10 kHz period = 100 µs 100 µs * 80 MHz = 8000 timer counts was (3200) "might have altered timers?"
//    // Now (8000)
//		TIMA0_PWM_init(0 (L), MOTOR_PERIOD_TICKS, 0, 0.0);// 0% duty cycle off initilization
//		TIMA0_PWM_init(1 (R), MOTOR_PERIOD_TICKS, 0, 0.0);// 0% duty cycle off initilization

//    // SERVO
//    // 20 ms period = 640 counts at 32 kHz was (640)
//    // Now 50Hz (1600000)
//    TIMA1_PWM_init(SERVO_CH (0), SERVO_PERIOD_TICKS, 0, servo_frac); straight off initilization
**/

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <ti/devices/msp/msp.h>

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
#define NO_TRACK_LIMIT          6         // watchdog threshold

/* PID gains (tuned for hybrid control) */
#define KP  0.020f
#define KI  0.0005f
#define KD  0.001f

/* Servo PWM (TIMA1) */
#define SERVO_PERIOD_TICKS      1600000UL // 50 Hz PWM
#define SERVO_CH                0
#define SERVO_MIN_US            1000.0f   // full left
#define SERVO_CENTER_US         1500.0f   // straight
#define SERVO_MAX_US            2000.0f   // full right
#define SERVO_PERIOD_US         20000.0f  // 20 ms period

/* Motor PWM (TIMA0) */
#define MOTOR_PERIOD_TICKS      8000UL    
#define LEFT_CH                 0
#define RIGHT_CH                1

/* Throttle rules */
#define MAX_THROTTLE            0.50f     // never exceed 50%
#define TURN_THROTTLE           0.20f     // slow throttle when steering
#define THROTTLE_RAMP           0.005f    // smooth acceleration/deceleration

/* Differential steering scaling */
#define DIFF_SCALE              0.02f     // PID ? torque split
#define DIFF_MAX                0.20f     // ±20% torque redistribution (max variance in motor assisted turning)

/* Thresholding */
#define THRESH_FACTOR           0.55f     // adaptive threshold factor (to account for wiggly track will need to be adjusted for intersections)


/* ============================================================
 *                     UTILITY FUNCTIONS
 * ============================================================ */

/**
 * Smoothly move a value toward a target by a fixed step.
 * Used for throttle and servo smoothing.
 */
static float ramp(float cur, float tgt, float step) {
    if (cur < tgt) { cur += step; if (cur > tgt) cur = tgt; }
    else if (cur > tgt) { cur -= step; if (cur < tgt) cur = tgt; }
    return cur;
}

/**
 * Convert a servo pulse width (microseconds) into a PWM duty fraction.
 */
static float pulse_to_frac(float us) {
    return us / SERVO_PERIOD_US;
}


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

typedef struct {
    int buf[FILTER_WINDOW];
    int idx, count;
} CentroidFilter;

/** Initialize filter buffer */
static void cf_init(CentroidFilter *f) {
    for (int i = 0; i < FILTER_WINDOW; i++) f->buf[i] = -1;
    f->idx = 0;
    f->count = 0;
}

/** Push new centroid value into filter */
static void cf_push(CentroidFilter *f, int v) {
    f->buf[f->idx] = v;
    f->idx = (f->idx + 1) % FILTER_WINDOW;
    if (f->count < FILTER_WINDOW) f->count++;
}

/** Compute filtered centroid (ignoring -1 entries) */
static float cf_get(CentroidFilter *f) {
    int sum = 0, n = 0;
    for (int i = 0; i < FILTER_WINDOW; i++) {
        if (f->buf[i] >= 0) {
            sum += f->buf[i];
            n++;
        }
    }
    return (n == 0) ? -1.0f : (float)sum / n;
}


/* ============================================================
 *                          MAIN LOOP
 * ============================================================ */

int main(void) {

    /* ---------- Initialize subsystems ---------- */

    ADC0_init();
    Camera_init();

    /* Motor PWM (TIMA0) */
    TIMA0_PWM_init(LEFT_CH, MOTOR_PERIOD_TICKS, 0, 0.0);
    TIMA0_PWM_init(RIGHT_CH, MOTOR_PERIOD_TICKS, 0, 0.0);

    /* Servo PWM (TIMA1) — start centered */
    float servo_frac = pulse_to_frac(SERVO_CENTER_US);
    TIMA1_PWM_init(SERVO_CH, SERVO_PERIOD_TICKS, 0, servo_frac);

    /* Control state variables */
    float throttle = 0.0f;
    float target_throttle = MAX_THROTTLE;

    float integ = 0, prev_err = 0;

    CentroidFilter filt;
    cf_init(&filt);

    int no_track = 0;
    uint8_t bin[128];
		
		//need to add button functionality to start the car instead of starting right off flashing


    /* ============================================================
     *                     CONTROL LOOP
     * ============================================================ */
    while (1) {

        /* Wait for camera frame */
        if (!Camera_isDataReady()) continue;
        uint16_t *raw = Camera_getData();

        /* Compute centroid */
        int cen = compute_centroid(raw, bin);

        /* Watchdog tracking */
        if (cen < 0) no_track++;
        else { no_track = 0; cf_push(&filt, cen); }

        float fcen = cf_get(&filt);
        bool on_track = (fcen >= 0);

        /* If track lost/too long ? stop car */
        if (no_track >= NO_TRACK_LIMIT) {
            target_throttle = 0;
            throttle = ramp(throttle, target_throttle, THROTTLE_RAMP);
            TIMA0_PWM_DutyCycle(LEFT_CH, throttle);
            TIMA0_PWM_DutyCycle(RIGHT_CH, throttle);
            continue;
        }

        /* Compute steering error */
        float err = on_track ? (CAMERA_CENTER - fcen) : 0;

        /* PID update */
        integ += err;
        float deriv = err - prev_err;
        prev_err = err;

        float pid = KP*err + KI*integ + KD*deriv;


        /* ============================================================
         *                PRIMARY STEERING — SERVO
         * ============================================================ */

        /* Convert PID ? servo pulse width */
        float servo_us = SERVO_CENTER_US + pid * 6.0f;  // 6us per pixel

        /* Clamp to servo limits */
        if (servo_us < SERVO_MIN_US) servo_us = SERVO_MIN_US;
        if (servo_us > SERVO_MAX_US) servo_us = SERVO_MAX_US;

        /* Smooth servo motion */
        servo_frac = ramp(servo_frac, pulse_to_frac(servo_us), 0.001f);
        TIMA1_PWM_DutyCycle(SERVO_CH, servo_frac);


        /* ============================================================
         *          SECONDARY STEERING — DIFFERENTIAL TORQUE
         * ============================================================ */

        /* PID ? differential torque split */
        float diff = pid * DIFF_SCALE;
        if (diff > DIFF_MAX) diff = DIFF_MAX;
        if (diff < -DIFF_MAX) diff = -DIFF_MAX;


        /* ============================================================
         *                     THROTTLE CONTROL
         * ============================================================ */

        /* Slow down when steering significantly */
        if (on_track && fabsf(err) <= CENTER_DEADBAND)
            target_throttle = MAX_THROTTLE;
        else
            target_throttle = TURN_THROTTLE;

        /* Smooth throttle changes */
        throttle = ramp(throttle, target_throttle, THROTTLE_RAMP);


        /* ============================================================
         *                     MOTOR OUTPUT
         * ============================================================ */

        /* Apply differential steering */
        float L = throttle * (1 - diff);
        float R = throttle * (1 + diff);

        /* Enforce 50% max rule */
        float maxLR = fmaxf(L, R);
        if (maxLR > MAX_THROTTLE) {
            float s = MAX_THROTTLE / maxLR;
            L *= s;
            R *= s;
        }

        /* Prevent negative torque */
        if (L < 0) L = 0;
        if (R < 0) R = 0;

        /* Output PWM */
        TIMA0_PWM_DutyCycle(LEFT_CH, L);
        TIMA0_PWM_DutyCycle(RIGHT_CH, R);
    }
}
