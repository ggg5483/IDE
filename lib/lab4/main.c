/**
 * ******************************************************************************
 * @file    : main.c
 * @brief   : 
 * @details : 
 * 
 * @author Alex Hamadeh
 * @author Garrett Geyer
 * @date 2/6/2026
 * ******************************************************************************
*/

#include <stdint.h>
#include <stdbool.h>
#include <ti/devices/msp/msp.h>
#include <lab4/I2C/oled.h>
#include <lab4/I2C/i2c.h>
#include <lab1/switches.h>

unsigned char OLED_TEXT_ARR[1024];
unsigned char OLED_GRAPH_ARR[1024];
#define MAIN 1

#if MAIN == 1
int main(void)
{
    // 1. Initialize the OLED display
    OLED_Init();
	
	OLED_display_clear();

		OLED_draw_line(1, 1, (unsigned char *)"Hello World");
    // 2. Write “Hello World” on line 1, column 1
    //OLED_Print(1, 1, "Hello World");

    // 3. Write “How are you?” on line 2, column 2
    //OLED_Print(2, 2, "How are you?");
		OLED_draw_line(2, 2, (unsigned char *)"How are you?");
	
    // 4. Write “Goodbye” on line 3, column 3
    //OLED_Print(3, 3, "Goodbye");
		OLED_draw_line(3, 3, (unsigned char *)"Goodbye");
	
	  // 8. Write name "for check off" on line 4 column 4
	  //OLED_Print(4, 4, "Alex & Garrett");
		OLED_draw_line(4, 4, (unsigned char *)"Alex & Garrett");
	
    // 5. Create fake 14-bit camera line data (0 ? 214), 128 samples stored in a uint16_t array
    uint16_t fake_camera_line[128];

    for (int i = 0; i < 128; i++)
    {
        // Spread values from 0 - 214 across 128 pixels
        fake_camera_line[i] = (uint16_t)(100 * i);
    }

    // 6. Display the fake camera data on the OLED
    //OLED_DisplayCameraData(fake_camera_line);

    // 7. Loop forever
    while (1)
    {
			OLED_write_display(OLED_TEXT_ARR);
			for(volatile int j = 0; j <100000000; j++){}
    }

    return 0;
}
#endif

#if MAIN == 2
int main(void)
{
    // 1. Initialize the OLED display
    OLED_Init();
		S1_init();
	
		OLED_display_clear();

		int i = 0;
	
		while(1){
			if(S1_pressed()){
				while(S1_pressed()){;}
				switch(i){
					case 0:
						OLED_ClearTextArr();
						OLED_display_clear();
						OLED_Print(1,1,"     _      _   ");
						OLED_Print(2,1,"   _( )__ _( )_");
						OLED_Print(3,1," _|     _|     _|");
						OLED_Print(4,1,"(_   _ (_   _ (_ ");
						i++;
						break;
					case 1:
						OLED_ClearTextArr();
						OLED_display_clear();
						OLED_Print(1,1,"_|___|___|___|");
						OLED_Print(2,1,"___|___|___|__");
						OLED_Print(3,1,"_|___|___|___|");
						OLED_Print(4,1,"___|___|___|__");
						i++;
						break;
					case 2:
						OLED_ClearTextArr();
						OLED_display_clear();
						OLED_Print(1,1,"_|_     _|_     ");
						OLED_Print(2,1," |       |      ");
						OLED_Print(3,1,"    _|_     _|_ ");
						OLED_Print(4,1,"     |       |  ");
						i = 0;
						break;
					default:
						i = 0;
					break;
				}
				OLED_write_display(OLED_TEXT_ARR);
			
			}
		}
		
	
//    // 5. Create fake 14-bit camera line data (0 ? 214), 128 samples stored in a uint16_t array
//    uint16_t fake_camera_line[128];

//    for (int i = 0; i < 128; i++)
//    {
//        // Spread values from 0 - 214 across 128 pixels
//        fake_camera_line[i] = (uint16_t)(100 * i);
//    }

    // 6. Display the fake camera data on the OLED
    //OLED_DisplayCameraData(fake_camera_line);

    // 7. Loop forever
//    while (1)
//    {
//			OLED_write_display(OLED_TEXT_ARR);
//			for(volatile int j = 0; j <10000; j++){}
//    }

    return 0;
}
#endif

