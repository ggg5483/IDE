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

int main(void)
{
    // 1. Initialize the OLED display
    OLED_Init();

    // 2. Write “Hello World” on line 1, column 1
    OLED_Print(1, 1, "Hello World");

    // 3. Write “How are you?” on line 2, column 2
    OLED_Print(2, 2, "How are you?");

    // 4. Write “Goodbye” on line 3, column 3
    OLED_Print(3, 3, "Goodbye");
	
	  // 8. Write name "for check off" on line 4 column 4
	  OLED_Print(4, 4, "Alex & Garrett");

    // 5. Create fake 14-bit camera line data (0 ? 214), 128 samples stored in a uint16_t array
    uint16_t fake_camera_line[128];

    for (int i = 0; i < 128; i++)
    {
        // Spread values from 0 - 214 across 128 pixels
        fake_camera_line[i] = (uint16_t)(100 * i);
    }

    // 6. Display the fake camera data on the OLED
    OLED_DisplayCameraData(fake_camera_line);

    // 7. Loop forever
    while (1)
    {
    }

    return 0;
}

