# CMPE460 MSPM0
Welcome to CMPE-460: Interface and Digital Electronic (IDE), at the Rochester Insitute of Technology!

This repository contains the files you will need to start developing for
the Texas Instruments **LP-MSPM0G3507** microcontroller development board.

***Please read this in full, so you can set up your project correctly!***

## Repository Structure
| Directory | Description |
| --- | --- |
| `3DPrintedParts` | 3D prints used throughout the course. These will be provided already printed in most cases. |
| `binaries` | MSP binaries used for either hardware testing or initialization. These files require TI's [Uniflash](https://www.ti.com/tool/UNIFLASH) to load to the board. |
| `docs` | Board development documents. Such as official TI docs for the microcontroller. |
| `LabReportTemplateExample` | CE report template example. |
| `source` | MSP source code. Headers and startup files needed to flash to the board. |
| `lib` | Provided driver files required for the class. |
| `proj`[^3] | Where your project code will go! |

## Project Setup
### Keil [^1][^2]
As Keil uVision is a Windows program, the following instructions are intended
for Windows only.
> ⚠️ **Note that these paths assume your project is created under a *new* directory
> within the `proj` sublevel.**
#### Device Setup
The following steps will set up the Keil IDE to build and flash to the MSPM0G3507.

1. In the Pack Installer expand All Devices ⇾ Texas Instruments ⇾ MSPM0G1X0X_G3X0X Series ⇾ MSPM0G350X ⇾ MSPM0G3507
2. Install the `TexasInstruments::MSPM0G1X0X_G3X0X_DFP` package
3. Create a new project in a *new* directory in the `proj` sublevel
4. Select the `MSPM0G3507` target board (Texas Instruments ⇾ MSPM0G1X0X_G3X0X Series ⇾ MSPM0G350X ⇾ MSPM0G3507)
5. Add the `CMSIS CORE` component. There is no `Device` component at this time.
6. Open the "Target Options" menu (the wand).
7. On the **Target** tab, select compiler version 6 for the *ARM Compiler*.
8. On the **C/C++** tab, insert this define into the *Preprocessor Symbols* section: `__MSPM0G3507__`
9. Under the same tab, add the sources directory to the *Include Paths*: `..\..\source`
10. Go to the **Linker** tab and add the *Scatter File*, located here:
   `..\..\source\ti\devices\msp\m0p\linker_files\keil\mspm0g3507.sct`.

      (It is an ASM file, not a C file)

11. Go to the **Debug** tab and select the `CMSIS-DAP Debugger` from the
   dropdown.
12. Click `OK` to close and save these options.
13. Click on the icon of 3 blocks next to the wand to open the "Manage Project
    Items" menu.
14. Under the current target and current source group add the startup file,
    located here:
    `..\..\source\ti\devices\msp\m0p\startup_system_files\keil\startup_mspm0g350x_uvision.s`

##### Setup Errors
There may be a PDSC flash error under this configuration. If this occurs, do the following:

1. Navigate to the **Debug** tab in the "Target Options" menu.
2. Click the *Settings* button next to the algorithm dropdown, where `CMSIS-DAP` is selected.
3. In the menu popup, navigate to the **Pack** tab and __deselect__ the "Enable" checkbox under the *Debug Description* section.
> This options can result in slightly different project configurations. Make sure setup is consistent between computers.

The board may also refuse to flash the code `RDAP` or similar errors. To bypass this, the board can be forcefully reset. These steps will require [Uniflash](https://www.ti.com/tool/UNIFLASH).

1. Open Uniflash with the board connected over USB.
2. The board should be autodetected. If not, manually find it in the list. Start a new session for the board.
3. Navigate to the **Settings & Utilities** section. Attempt to `Factory reset auto`. At this point, try to flash the board again. This reset should fix *most* errors. If you are still having issues, continue on to the next step.
4. Hold S1 for 30 seconds. While S1 is still being held, tap the reset button on the board.
5. At this point you will have roughly 10 seconds to forcefully `Factory reset auto` the board before it locks once again.
6. If you are able to reset the board in this condition, then it should allow your code to be loaded.


#### Driver Setup
These steps will demonstrate how to add the driver files from the `lib`[^4] directory.
1. Open the "Manage Project Items" menu (the 3 blocks).
2. Add a new group called `lib`.
3. With the `lib` group selected, click add files and navigate to the lib
   directory at the repository top level. Make sure the file type shows all
   files. Select (or create) the desired .c files and click `Add`. Then exit and click `OK`.
4. Open the "Target Options" menu.
5. Go to the **C/C++** tab, under the *Inlcude Paths*, add the following:
   `..\..\lib` to direct to the drivers. `..\..\source;..\..\lib` can be used
   to include both of the needed paths.
> ⚠️ If the file you want to add is within another subdirectory, you will need to specify either `#include "<subdirectory>/file.c`, **OR**, you can add the subdirectory path to the *Include Paths* above.
6. Once done, exit by clicking `OK`.
> With these steps, you do *not* need to add the .h files to the directory. Only the .c files are required.

#### Includes
At this point, both the board SDK files and the driver files developed here
should be able to be included in the new project. The includes should look
something like this (yours will differ):
```c
#include <ti/devices/msp/msp.h>
#include "lab1/leds.h" // OR #include "leds.h"
```

[^1]: Steps performed using Keil uVision V5.31.0.0
[^2]: Developed using Keil Device Pack V1.3.1
[^3]: You can rename or move the `proj` directory, but remember if you do, the
    paths listed here may change.
[^4]: This directory can be substituted for any other directory you might want.
    The steps are largely the same.

## Recommended Modules
Setup these specific modules for core functionality on the car. You will work
through this on the labs.

* GPIOA
* GPIOB
* UART0
* UART1
* I2C1
* TIMA0
* TIMA1
* TIMG0
* TIMG6
* TIMG12
* ADC0

### Car Pinout
This class has a car! You get to drive it.

Listed here are the current pinouts for the car. If you want your code to work
on the car, it should interact with these specific pins.

| Module | Label        | Hardware     | Pin Out  |
| ------ | -----------  | ------------ | -------- |
| UART1  | UART1 TX     | HM-10 BLE RX | PA8      |
| UART1  | UART1 RX     | HM-10 BLE TX | PA9, SW1 |
| I2C1   | SCL          | OLED SCL     | PB2      |
| I2C1   | SDA          | OLED SDA     | PB3      |
| TIMA0  | Channel 0    | DC Motor     | PB8      |
| TIMA0  | Channel 1    | DC Motor     | PB12     |
| TIMA0  | Channel 2    | DC Motor     | PB17     |
| TIMA0  | Channel 3    | DC Motor     | PB13     |
| TIMA1  | Channel 0    | Servo Motor  | PB4      |
| TIMG0  | Channel 0    | Camera CLK   | PA12     |
| ADC0   | Channel 0    | Camera AO    | PA27     |
| GPIOA  |              | Camera SI    | PA28     |
| GPIOB  |              | Left DC EN   | PB19     |
| GPIOA  |              | Right DC EN  | PA22     |

> ⚠️ Note: SW1 is a physical header on the board that must be moved to connect PA9 for
> UART1 functionality to work as expected.

### Board Clock Speed
A driver file, `sysctl`, is provided in the `lib` directory to control the default clock speed of the board. By default, the board operates at 32 MHz.
* The `SYSCTL_SYSCLK_set()` function allows changing this default to any of the predefined clock speed enumerations located in the `systctl.h` file. If this function is not called during initialization, the board will run at the default clock speed.
* Getters functions are also provided to retrieve the current operating speed.
   * `SYSCTL_SYSCLK_getMCLK()` — Returns the MCLK frequency.
   * `SYSCTL_SYSCLK_getULPCLK()` — Returns the ULPCLK frequency.

The *set* function only needs to be called once during system initialization; afterwards, the board will continue running at the configured frequency.
> ⚠️ Do not edit the driver source itself. Doing so may render the board unresponsive and require re-flashing. Only use the provided functions.

---

## Labs
Lab driver header files are given in the `lib` directory. These files provide
function prototypes with some explanation on what to implement. Additional files
will also be provided here to help with the labs and/or car, like `sysctl`.

These header files are to serve as a guide. You can modify the prototypes, or add
additional functionality as you see fit. Feel free to move the files around to fit
your project structure as well.

## Good Luck!
And that should be all! Good luck!
