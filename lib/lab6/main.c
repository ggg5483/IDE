/**
 * ******************************************************************************
 * @file    : main.c
 * @brief   : Does motors
 * @details : Yay motors
 * 
 * @author Alex Hamadeh
 * @author Garrett Geyer
 * @date 3/3/26
 * ******************************************************************************
*/

#include <ti/devices/msp/msp.h>
#include "lab6/timers.h"
#include "lab2/uart.h"
#include "uart_extras.h"

//int main(void){
//	UART0_init();
//	TIMA0_PWM_init(0, 3200, 0, 0.2);
//	TIMA1_PWM_init(0, 3200, 0, 0.2);
//	
//	for(;;){
//		for(int i = 0; i < 100; i++){
//			TIMA0_PWM_DutyCycle(0, (((double)i)/((double)100)));
//			for(volatile int _ = 0; _ < 100000; _++){};
//		}
//		for(int i = 0; i < 100; i++){
//			TIMA0_PWM_DutyCycle(0, (((double)(100-i))/((double)100)));
//			for(volatile int _ = 0; _ < 100000; _++){};
//		}
//	}
//	return 0;
//}
	
	
	

static void delay_10ms(void){
    for(volatile int i = 0; i < 100000; i++){}
}

// Convert angle (0–180°) to duty cycle for a 1–2 ms pulse
double servo_angle_to_duty(int angle){
    double minPulse = 0.001;   // 1 ms
    double maxPulse = 0.002;   // 2 ms
    double pulse = minPulse + (angle / 180.0) * (maxPulse - minPulse);
    return pulse / 0.020;      // divide by 20 ms period
}

int main(void){
  UART0_init();
  TIMA0_PWM_init(0, 3200, 0, 0.0);   // Forward channel
  TIMA1_PWM_init(0, 3200, 0, 0.0);   // Reverse channel

  for(;;){

        // ---------------------------------------------------------
        // 1. Forward: 0% ? 100%
        // ---------------------------------------------------------
        for(int i = 0; i <= 100; i++){
            double duty = ((double)i) / 100.0;

            TIMA0_PWM_DutyCycle(0, duty);   // Forward PWM active
            TIMA1_PWM_DutyCycle(0, 0.0);    // Reverse off

            delay_10ms();
        }

        // ---------------------------------------------------------
        // 2. Forward: 100% ? 0%
        // ---------------------------------------------------------
        for(int i = 100; i >= 0; i--){
            double duty = ((double)i) / 100.0;

            TIMA0_PWM_DutyCycle(0, duty);
            TIMA1_PWM_DutyCycle(0, 0.0);

            delay_10ms();
        }

        // ---------------------------------------------------------
        // 3. Reverse: 0% ? 100%
        // ---------------------------------------------------------
        for(int i = 0; i <= 100; i++){
            double duty = ((double)i) / 100.0;

            TIMA1_PWM_DutyCycle(0, duty);   // Reverse PWM active
            TIMA0_PWM_DutyCycle(0, 0.0);    // Forward off

            delay_10ms();
        }

        // ---------------------------------------------------------
        // 4. Reverse: 100% ? 0%
        // ---------------------------------------------------------
        for(int i = 100; i >= 0; i--){
            double duty = ((double)i) / 100.0;

            TIMA1_PWM_DutyCycle(0, duty);
            TIMA0_PWM_DutyCycle(0, 0.0);

            delay_10ms();
        }
    }

    return 0;
}

//---------------------------------------------------------------------------------//

////PART 2 SERVO SWEEP (0, 180)
//int main(void){
//    UART0_init();

//    // 20 ms period = 1,600,000 counts at 80 MHz
//    TIMA0_PWM_init(0, 1600000, 0, 0.0);

//    for(;;){

//        // Sweep 0° ? 180°
//        for(int angle = 0; angle <= 180; angle++){
//            double duty = servo_angle_to_duty(angle);
//            TIMA0_PWM_DutyCycle(0, duty);
//            delay_10ms();
//        }

//        // Sweep 180° ? 0°
//        for(int angle = 180; angle >= 0; angle--){
//            double duty = servo_angle_to_duty(angle);
//            TIMA0_PWM_DutyCycle(0, duty);
//            delay_10ms();
//        }
//    }

//    return 0;
//}

//---------------------------------------------------------------------------------//

////PART 3 SERVO SWEEP (0, 90, 180)
//int main(void){
//    UART0_init();

//    TIMA0_PWM_init(0, 1600000, 0, 0.0);

//    for(;;){

//        // Move to 0°
//        TIMA0_PWM_DutyCycle(0, servo_angle_to_duty(0));
//        for(int i = 0; i < 100; i++) delay_10ms();   // hold position

//        // Move to 90°
//        TIMA0_PWM_DutyCycle(0, servo_angle_to_duty(90));
//        for(int i = 0; i < 100; i++) delay_10ms();

//        // Move to 180°
//        TIMA0_PWM_DutyCycle(0, servo_angle_to_duty(180));
//        for(int i = 0; i < 100; i++) delay_10ms();
//    }

//    return 0;
//}

//---------------------------------------------------------------------------------//

////PART 4 SERVO AND DC
//int main(void){
//    UART0_init();

//    // DC
//    // 10 kHz period = 100 µs
//    // 100 µs * 80 MHz = 8000 timer counts
//    TIMA0_PWM_init(0, 8000, 0, 0.3);   // 30% duty cycle

//    // SERVO
//    // 20 ms period = 1,600,000 counts at 80 MHz
//    TIMA1_PWM_init(0, 1600000, 0, 0.0);

//    // Move servo to 90°
//    double servoDuty = servo_angle_to_duty(90);
//    TIMA1_PWM_DutyCycle(0, servoDuty);

//    for(;;){
//        // Idle loop — PWM hardware continues running
//    }

//    return 0;
//}

