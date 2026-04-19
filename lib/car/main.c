/**
 * @file main.c
 *
 * @brief autonomous car code. Uses line scan camera to control stearing.
 *
 * @author Alexander Hamadeh
 * @author Garrett Geyer
 * @date 
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
#define CAMERA_CENTER           64.0f     // ideal centroid (accounting for off centered camera) increase if to the right of center, decrease if to the left of center (point camera slightly to the right)
#define CENTER_DEADBAND         3.0f      // acceptable error before slowing/turning (for intersection/wavy)

/* Filtering */
#define FILTER_WINDOW           5         // moving-average window
#define NO_TRACK_LIMIT          10        // watchdog threshold (frames)

/* PID gains (tuned for hybrid control) */
#define KP  1.50f
#define KI  0.00f
#define KD  0.5f
#define PID_SCALER 2.7f

/* Servo PWM (TIMA1) */
#define SERVO_PERIOD_TICKS      640 
#define SERVO_CH                0
#define SERVO_MIN_US            5.0f    // full left
#define SERVO_CENTER_US         90.0f    // straight/init
#define SERVO_MAX_US            175.0f   // full right
#define STEARING_RAMP           0.01f    // smooth stearing

/* Motor PWM (TIMA0) */
#define MOTOR_EN true			//compile-time motor disable
#define MOTOR_PERIOD_TICKS      3200    
#define LEFT_CH                 0
#define RIGHT_CH                2

/* Throttle rules */
#define INIT_THROTTLE           0.00f
#define MAX_THROTTLE            0.43f     // never exceed 50%
#define TURN_THROTTLE           0.22f     // slowest throttle when steering
#define THROTTLE_RAMP_UP        0.001f    // smooth acceleration/deceleration
#define THROTTLE_RAMP_DOWN      0.1f     // smooth acceleration/deceleration

/* Differential steering scaling */
#define DIFF_SCALE              0.02f     // PID ? torque split
#define DIFF_MAX                0.20f     // ±20% torque redistribution (max variance in motor assisted turning)

/* Thresholding */
#define THRESH_FACTOR           0.55f     // adaptive threshold factor (to account for different light levels)

/* Motor Controller Enable */
#define LEFT_EN_MASK   (1U << 19)         // PB19
#define RIGHT_EN_MASK  (1U << 22)         // PA22

/* Carpet Stopping */
#define CARPET_STOP false								//use carpet stopping?
	
/* UART */
#define UART_EN true
#if UART_EN
#include "uart.h"
#include "uart_extras.h"
#define BUF_SIZE 20

#endif //UART_EN
/* ============================================================
 *             				   notes
 * ============================================================ */

/**
4/19/26
battery position affect steering - in back of car, to little tracksion of turning wheel -> moved to center of car


battery voltage significantly changes car response.
7.89v - P2D0.5I0, scaler 2.7, Max T 0.43, turn T 0.22
worked well

swap in 8.25v
can still stay on the track, but starts to have back wheels rotate to outside of turn

wiped down back wheels -> stopped drifting, started understeering
Wiped down all four wheels -> goes back do drifting but much less, more drifting is caused by track sliding

Wanted to get max_throttle to 0.5 - > caused more oscillations, turned kp to 1.5 helped, causes occasional snake-skipping(two u-turns, might skip the second.
turned off carpet stopping to stop this from stopping the car.
Increase turn throttle to 0.25 - > helped with track sliding, turns a little wider but still makes it.

battery fell to 7.7v over ~1.5 hours. Final values 7.7v, PID 1.5, 0.5, 0, MAX 0.5, TURN 0.25, scaler 2.7

*/


/* ============================================================
 *             				   GLOBAL VARIABLES
 * ============================================================ */

		/* PID changable vals */
		float kp = KP;
		float ki = KI;
		float kd = KD;
		float pid_scaler = PID_SCALER;

    float max_throttle = MAX_THROTTLE;
    float turn_throttle = TURN_THROTTLE;

	/* UART values */
	#if UART_EN
		float kp_saved = KP;
		float ki_saved = KI;
		float kd_saved = KD;
		float pid_scaler_saved = PID_SCALER;
    float max_throttle_saved = MAX_THROTTLE;
    float turn_throttle_saved = TURN_THROTTLE;
		char buf[BUF_SIZE];
	#endif //UART_EN

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

/**
* @brief helper function to handle UART terminal
* @note causes 4 warnings i don't know how to fix
*/
#if UART_EN
void handle_uart(char ch){
	switch (ch) {
			case 'h':
			case 'H':
				UART1_put("UART terminal commands:\r\nh : help\r\n<p|i|d>XXXXX : set k<> to XX.XXX\r\nq : stop\r\ns : save values\r\nr : restore values\r\nv : display values\r\nx<XXXX>: set pid_scaler to X.XXX\r\ng : start car if stopped\r\ntXXX : max throttle to 0.xxx\r\ncXXX : turn/corner throttle to 0.xxx\r\n");
				break;
			case 's':
			case 'S':
				//UART1_put("saved\r\n")
				ki_saved = ki;
				kp_saved = kp;
				kd_saved = kd;
				pid_scaler_saved = pid_scaler;
        max_throttle_saved = max_throttle;
        turn_throttle_saved = turn_throttle;
				break;
			case 'r':
			case 'R':
				//UART1_put("restored\r\n")
				ki = ki_saved;
				kp = kp_saved;
				kd = kd_saved;
				pid_scaler = pid_scaler_saved;
        max_throttle = max_throttle_saved;
        turn_throttle = turn_throttle_saved;
				break;
			case 'v':
			case 'V':
				UART1_put("KP\r\n");
				UART1_printFloat(kp);
				UART1_put("\r\nKD\r\n");
				UART1_printFloat(kd);
				UART1_put("\r\nKI\r\n");
				UART1_printFloat(ki);
        UART1_put("\r\npid_scaler\r\n");
				UART1_printFloat(pid_scaler);
        UART1_put("\r\nmax_throttle\r\n");
				UART1_printFloat(max_throttle);
        UART1_put("\r\nturn_throttle\r\n");
				UART1_printFloat(turn_throttle);
				UART1_put("\r\n");
				break;
			case 'p':
			case 'P':
				UART1_get(&buf, BUF_SIZE);
				kp = (float) str_to_int(buf) / (float) 1000;
				//UART1_put(buf);
				break;
			case 'i':
			case 'I':
				UART1_get(&buf, BUF_SIZE);
				ki = (float) str_to_int(buf) / (float) 1000;
				//UART1_put(buf);
				break;
			case 'd':
			case 'D':
				UART1_get(&buf, BUF_SIZE);
				kd = (float) str_to_int(buf) / (float) 1000;
				//UART1_put(buf);
				break;
			case 'x':
			case 'X':
				UART1_get(&buf, BUF_SIZE);
				pid_scaler = (float) str_to_int(buf) / (float) 1000;
        break;
      case 't':
      case 'T':
        UART1_get(&buf, BUF_SIZE);
				max_throttle = (float) str_to_int(buf) / (float) 1000;
        break;
      case 'c':
      case 'C':
        UART1_get(&buf, BUF_SIZE);
				turn_throttle = (float) str_to_int(buf) / (float) 1000;
        break;
        break;
			case '\r':
			case '\n':
				break;
			default:
				break;
			
		}//switch
}
#endif // UART_EN

/* ============================================================
 *                          MAIN LOOP
 * ============================================================ */
/**
*choosing main function
* 1 - main
* 2 - testing/debugging main
*/
#define MAIN (1)

#if MAIN == 1
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
        
				/* UART init */
				#if UART_EN
				//UART0_init(); //connected putty terminal
				UART1_init(); // bluetooth
				UART1_put("\033[?25h\033[48;5;231m\033[38;5;232m"); // show cursor, background white, text dark
				UART1_put("\033[2J\033[H"); //clear screen
				UART1_put("UART INITIALIZED!\r\n");
				
				#endif //UART_EN
				
        /* roughly 5s delay before starting to run the car */
        for(volatile int i = 0; i < 500; i++) {
            delay_1ms();
        }
        
        /* start motors */
				#if MOTOR_EN
        TIMA0_PWM_DutyCycle(LEFT_CH, TURN_THROTTLE);   
        TIMA0_PWM_DutyCycle(RIGHT_CH, TURN_THROTTLE);
				#endif //MOTOR_EN
				
				main_loop:				
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
						pid_output = (kp * pid_error) + (ki * pid_integral) + (kd * pid_derivative);

						/* Servo steering using PID */
						angle = SERVO_CENTER_US + (pid_output * pid_scaler); //adjust for how responsive we want the stearing control
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
						float target = max_throttle - (norm * (max_throttle - turn_throttle));

						/* Smooth ramp toward target */
						if (throttle < target) {
							throttle += THROTTLE_RAMP_UP;
							if (throttle > target) throttle = target;
						} else {
							throttle -= THROTTLE_RAMP_DOWN;
							if (throttle < target) throttle = target;
						}

						/* Apply throttle */
						#if MOTOR_EN
						TIMA0_PWM_DutyCycle(LEFT_CH, throttle);
						TIMA0_PWM_DutyCycle(RIGHT_CH, throttle);
						#endif //MOTOR_EN
                        
						/* Track end/carpet stop check, checks if NO_TRACK_LIMIT number of consecutive frames logged had no track data*/
						#if CARPET_STOP
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
						#endif //CARPET_STOP
						
						/*Uart for BT communication*/
						#if UART_EN
						//UART1_put("HELLO WORLD");
						if(UART1_dataAvailable()){
							char ch = UART1_getchar();
              if(ch == 'q')goto uart_stop;
							handle_uart(ch);
							
						}// if data available
						#endif //UART_EN
						
						
        } // while 1, main driving loop
				
				//option to restart car over uart, if both are enabled
				#if UART_EN
        uart_stop:
        TIMA0_PWM_DutyCycle(LEFT_CH, INIT_THROTTLE);   
        TIMA0_PWM_DutyCycle(RIGHT_CH, INIT_THROTTLE);  
        TIMA1_PWM_DutyCycle(SERVO_CH, servo_angle_to_duty(SERVO_CENTER_US));
				while(1){
					if(UART1_dataAvailable()){
							char ch = UART1_getchar();
							if(ch == 'g') goto main_loop;
							handle_uart(ch);
					} ///if
				}//while
				#endif // UART_EN && CARPET_STOP
        
}
#endif // MAIN == 1

#if MAIN == 2
int main(void) {
				/*
				0 - left forewords
				1 - nothing
				2 - right  forewords
				3 - did nothing
	
	
				*/
				int channel = 0;
				//start 0 throttle
				TIMA0_PWM_init(channel, MOTOR_PERIOD_TICKS, 0, INIT_THROTTLE );

				for(volatile int i = 0; i < 500; i++) {
            delay_1ms();
        }
    
				/* Enable DC motors */
				DC_ENABLE();
				
				TIMA0_PWM_DutyCycle(channel, 0.3);
				/* SERVO Init */
        //TIMA1_PWM_init(SERVO_CH, SERVO_PERIOD_TICKS, 0, servo_angle_to_duty(SERVO_CENTER_US));

	return 0;
}

#endif // MAIN == 2
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
