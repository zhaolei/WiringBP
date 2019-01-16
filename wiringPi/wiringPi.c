/*
 * wiringPi:
 *	Arduino compatable (ish) Wiring library for the Raspberry Pi
 *	Copyright (c) 2012 Gordon Henderson
 *	Additional code for pwmSetClock by Chris Hall <chris@kchall.plus.com>
 *
 *	Thanks to code samples from Gert Jan van Loo and the
 *	BCM2835 ARM Peripherals manual, however it's missing
 *	the clock section /grr/mutter/
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

// Revisions:
//	19 Jul 2012:
//		Moved to the LGPL
//		Added an abstraction layer to the main routines to save a tiny
//		bit of run-time and make the clode a little cleaner (if a little
//		larger)
//		Added waitForInterrupt code
//		Added piHiPri code
//
//	 9 Jul 2012:
//		Added in support to use the /sys/class/gpio interface.
//	 2 Jul 2012:
//		Fixed a few more bugs to do with range-checking when in GPIO mode.
//	11 Jun 2012:
//		Fixed some typos.
//		Added c++ support for the .h file
//		Added a new function to allow for using my "pin" numbers, or native
//			GPIO pin numbers.
//		Removed my busy-loop delay and replaced it with a call to delayMicroseconds
//
//	02 May 2012:
//		Added in the 2 UART pins
//		Change maxPins to numPins to more accurately reflect purpose


#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <poll.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>

#include "softPwm.h"
#include "softTone.h"

#include "wiringPi.h"

#ifndef	TRUE
#define	TRUE	(1==1)
#define	FALSE	(1==2)
#endif

// Environment Variables

#define	ENV_DEBUG	"WIRINGPI_DEBUG"
#define	ENV_CODES	"WIRINGPI_CODES"


// Mask for the bottom 64 pins which belong to the Banana Pro
//	The others are available for the other devices

#define	PI_GPIO_MASK	(0xFFFFFFC0)

struct wiringPiNodeStruct *wiringPiNodes = NULL ;

// BCM Magic

#define	BCM_PASSWORD		0x5A000000


// The BCM2835 has 54 GPIO pins.
//	BCM2835 data sheet, Page 90 onwards.
//	There are 6 control registers, each control the functions of a block
//	of 10 pins.
//	Each control register has 10 sets of 3 bits per GPIO pin - the ALT values
//
//	000 = GPIO Pin X is an input
//	001 = GPIO Pin X is an output
//	100 = GPIO Pin X takes alternate function 0
//	101 = GPIO Pin X takes alternate function 1
//	110 = GPIO Pin X takes alternate function 2
//	111 = GPIO Pin X takes alternate function 3
//	011 = GPIO Pin X takes alternate function 4
//	010 = GPIO Pin X takes alternate function 5
//
// So the 3 bits for port X are:
//	X / 10 + ((X % 10) * 3)

// Port function select bits

#define	FSEL_INPT		0b000
#define	FSEL_OUTP		0b001
#define	FSEL_ALT0		0b100
#define	FSEL_ALT1		0b101
#define	FSEL_ALT2		0b110
#define	FSEL_ALT3		0b111
#define	FSEL_ALT4		0b011
#define	FSEL_ALT5		0b010

// Access from ARM Running Linux
//	Taken from Gert/Doms code. Some of this is not in the manual
//	that I can find )-:

#define BCM2708_PERI_BASE	                     0x20000000
#define GPIO_PADS		(BCM2708_PERI_BASE + 0x00100000)
#define CLOCK_BASE		(BCM2708_PERI_BASE + 0x00101000)
#define GPIO_BASE		(BCM2708_PERI_BASE + 0x00200000)
#define GPIO_TIMER		(BCM2708_PERI_BASE + 0x0000B000)
#define GPIO_PWM		(BCM2708_PERI_BASE + 0x0020C000)

#define	PAGE_SIZE		(4*1024)
#define	BLOCK_SIZE		(4*1024)

// PWM
//	Word offsets into the PWM control region

#define	PWM_CONTROL 0
#define	PWM_STATUS  1
#define	PWM0_RANGE  4
#define	PWM0_DATA   5
#define	PWM1_RANGE  8
#define	PWM1_DATA   9

//	Clock regsiter offsets

#define	PWMCLK_CNTL	40
#define	PWMCLK_DIV	41

#define	PWM0_MS_MODE    0x0080  // Run in MS mode
#define	PWM0_USEFIFO    0x0020  // Data from FIFO
#define	PWM0_REVPOLAR   0x0010  // Reverse polarity
#define	PWM0_OFFSTATE   0x0008  // Ouput Off state
#define	PWM0_REPEATFF   0x0004  // Repeat last value if FIFO empty
#define	PWM0_SERIAL     0x0002  // Run in serial mode
#define	PWM0_ENABLE     0x0001  // Channel Enable

#define	PWM1_MS_MODE    0x8000  // Run in MS mode
#define	PWM1_USEFIFO    0x2000  // Data from FIFO
#define	PWM1_REVPOLAR   0x1000  // Reverse polarity
#define	PWM1_OFFSTATE   0x0800  // Ouput Off state
#define	PWM1_REPEATFF   0x0400  // Repeat last value if FIFO empty
#define	PWM1_SERIAL     0x0200  // Run in serial mode
#define	PWM1_ENABLE     0x0100  // Channel Enable

// Timer
//	Word offsets

#define	TIMER_LOAD	(0x400 >> 2)
#define	TIMER_VALUE	(0x404 >> 2)
#define	TIMER_CONTROL	(0x408 >> 2)
#define	TIMER_IRQ_CLR	(0x40C >> 2)
#define	TIMER_IRQ_RAW	(0x410 >> 2)
#define	TIMER_IRQ_MASK	(0x414 >> 2)
#define	TIMER_RELOAD	(0x418 >> 2)
#define	TIMER_PRE_DIV	(0x41C >> 2)
#define	TIMER_COUNTER	(0x420 >> 2)

// Locals to hold pointers to the hardware

static volatile uint32_t *gpio ;
static volatile uint32_t *pwm ;
static volatile uint32_t *clk ;
static volatile uint32_t *pads ;

#ifdef	USE_TIMER
static volatile uint32_t *timer ;
static volatile uint32_t *timerIrqRaw ;
#endif

/*add for BananaPro by LeMaker team*/
// for mmap BananaPro 
#define	MAX_PIN_NUM		(0x40)  //64
#define SUNXI_GPIO_BASE (0x01C20800)
#define MAP_SIZE	(4096*2)
#define MAP_MASK	(MAP_SIZE - 1)
//sunxi_pwm
#define SUNXI_PWM_BASE (0x01c20e00)
#define SUNXI_PWM_CTRL_REG  (SUNXI_PWM_BASE)
#define SUNXI_PWM_CH0_PERIOD  (SUNXI_PWM_BASE + 0x4)
#define SUNXI_PWM_CH1_PERIOD  (SUNXI_PWM_BASE + 0x8)

#define SUNXI_PWM_CH0_EN			(1 << 4)
#define SUNXI_PWM_CH0_ACT_STA		(1 << 5)
#define SUNXI_PWM_SCLK_CH0_GATING	(1 << 6)
#define SUNXI_PWM_CH0_MS_MODE		(1 << 7) //pulse mode
#define SUNXI_PWM_CH0_PUL_START		(1 << 8)

#define SUNXI_PWM_CH1_EN			(1 << 19)
#define SUNXI_PWM_CH1_ACT_STA		(1 << 20)
#define SUNXI_PWM_SCLK_CH1_GATING	(1 << 21)
#define SUNXI_PWM_CH1_MS_MODE		(1 << 22) //pulse mode
#define SUNXI_PWM_CH1_PUL_START		(1 << 23)


#define PWM_CLK_DIV_120 	        0
#define PWM_CLK_DIV_180		1
#define PWM_CLK_DIV_240		2
#define PWM_CLK_DIV_360		3
#define PWM_CLK_DIV_480		4
#define PWM_CLK_DIV_12K		8
#define PWM_CLK_DIV_24K		9
#define PWM_CLK_DIV_36K		10
#define PWM_CLK_DIV_48K		11
#define PWM_CLK_DIV_72K		12

#define GPIO_PADS_BP		(0x00100000)
#define CLOCK_BASE_BP		(0x00101000)
//	addr should 4K*n
//	#define GPIO_BASE_BP		(SUNXI_GPIO_BASE)
#define GPIO_BASE_BP		(0x01C20000)
#define GPIO_TIMER_BP		(0x0000B000)
#define GPIO_PWM_BP		(0x01c20000)  //need 4k*n

static int wiringPinMode = WPI_MODE_UNINITIALISED ;
int wiringPiCodes = FALSE ;
/*end 2014.09.18*/

// Data for use with the boardId functions.
//	The order of entries here to correspond with the PI_MODEL_X
//	and PI_VERSION_X defines in wiringPi.h
//	Only intended for the gpio command - use at your own risk!

const char *piModelNames [6] =
{
  "Unknown",
  "Model A",
  "Model B",
  "Model B+",
  "Compute Module",
  "Banana Pro",    //add for BananaPro by LeMaker team
} ;

const char *piRevisionNames [5] =
{
  "Unknown",
  "1",
  "1.1",
  "1.2",
  "2",
} ;

const char *piMakerNames [5] =
{
  "Unknown",
  "Egoman",
  "Sony",
  "Qusda",
  "LeMaker", //add for BananaPro by LeMaker team
} ;


// Time for easy calculations

static uint64_t epochMilli, epochMicro ;

// Misc

static int wiringPiMode = WPI_MODE_UNINITIALISED ;
static volatile int    pinPass = -1 ;
static pthread_mutex_t pinMutex ;

// Debugging & Return codes

int wiringPiDebug       = FALSE;  // guenter FALSE ;
int wiringPiReturnCodes = FALSE ;

// sysFds:
//	Map a file descriptor from the /sys/class/gpio/gpioX/value

static int sysFds [300] =
{
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
} ;

// ISR Data

static void (*isrFunctions [64])(void) ;


// Doing it the Arduino way with lookup tables...
//	Yes, it's probably more innefficient than all the bit-twidling, but it
//	does tend to make it all a bit clearer. At least to me!

// pinToGpio:
//	Take a Wiring pin (0 through X) and re-map it to the BCM_GPIO pin
//	Cope for 3 different board revisions here.

static int *pinToGpio ;

// Revision 1, 1.1:

static int pinToGpioR1 [64] =
{
  17, 18, 21, 22, 23, 24, 25, 4,	// From the Original Wiki - GPIO 0 through 7:	wpi  0 -  7
   0,  1,				// I2C  - SDA1, SCL1				wpi  8 -  9
   8,  7,				// SPI  - CE1, CE0				wpi 10 - 11
  10,  9, 11, 				// SPI  - MOSI, MISO, SCLK			wpi 12 - 14
  14, 15,				// UART - Tx, Rx				wpi 15 - 16

// Padding:

      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 31
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 63
} ;

// Revision 2:

static int pinToGpioR2 [64] =
{
  17, 18, 27, 22, 23, 24, 25, 4,	// From the Original Wiki - GPIO 0 through 7:	wpi  0 -  7
   2,  3,				// I2C  - SDA0, SCL0				wpi  8 -  9
   8,  7,				// SPI  - CE1, CE0				wpi 10 - 11
  10,  9, 11, 				// SPI  - MOSI, MISO, SCLK			wpi 12 - 14
  14, 15,				// UART - Tx, Rx				wpi 15 - 16
  28, 29, 30, 31,			// Rev 2: New GPIOs 8 though 11			wpi 17 - 20
   5,  6, 13, 19, 26,			// B+						wpi 21, 22, 23, 24, 25
  12, 16, 20, 21,			// B+						wpi 26, 27, 28, 29
   0,  1,				// B+						wpi 30, 31

// Padding:

  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 63
} ;


// physToGpio:
//	Take a physical pin (1 through 26) and re-map it to the BCM_GPIO pin
//	Cope for 2 different board revisions here.
//	Also add in the P5 connector, so the P5 pins are 3,4,5,6, so 53,54,55,56

static int *physToGpio ;

static int physToGpioR1 [64] =
{
  -1,		// 0
  -1, -1,	// 1, 2
   0, -1,
   1, -1,
   4, 14,
  -1, 15,
  17, 18,
  21, -1,
  22, 23,
  -1, 24,
  10, -1,
   9, 25,
  11,  8,
  -1,  7,	// 25, 26

                                              -1, -1, -1, -1, -1,	// ... 31
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 63
} ;

static int physToGpioR2 [64] =
{
  -1,		// 0
  -1, -1,	// 1, 2
   2, -1,
   3, -1,
   4, 14,
  -1, 15,
  17, 18,
  27, -1,
  22, 23,
  -1, 24,
  10, -1,
   9, 25,
  11,  8,
  -1,  7,	// 25, 26

// B+

   0,  1,
   5, -1,
   6, 12,
  13, -1,
  19, 16,
  26, 20,
  -1, 21,

// the P5 connector on the Rev 2 boards:

  -1, -1,
  -1, -1,
  -1, -1,
  -1, -1,
  -1, -1,
  28, 29,
  30, 31,
  -1, -1,
  -1, -1,
  -1, -1,
  -1, -1,
} ;

// gpioToGPFSEL:
//	Map a BCM_GPIO pin to it's Function Selection
//	control port. (GPFSEL 0-5)
//	Groups of 10 - 3 bits per Function - 30 bits per port

static uint8_t gpioToGPFSEL [] =
{
  0,0,0,0,0,0,0,0,0,0,
  1,1,1,1,1,1,1,1,1,1,
  2,2,2,2,2,2,2,2,2,2,
  3,3,3,3,3,3,3,3,3,3,
  4,4,4,4,4,4,4,4,4,4,
  5,5,5,5,5,5,5,5,5,5,
} ;


// gpioToShift
//	Define the shift up for the 3 bits per pin in each GPFSEL port

static uint8_t gpioToShift [] =
{
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
} ;


// gpioToGPSET:
//	(Word) offset to the GPIO Set registers for each GPIO pin

static uint8_t gpioToGPSET [] =
{
   7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
   8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
} ;

// gpioToGPCLR:
//	(Word) offset to the GPIO Clear registers for each GPIO pin

static uint8_t gpioToGPCLR [] =
{
  10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,
} ;


// gpioToGPLEV:
//	(Word) offset to the GPIO Input level registers for each GPIO pin

static uint8_t gpioToGPLEV [] =
{
  13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
  14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
} ;


#ifdef notYetReady
// gpioToEDS
//	(Word) offset to the Event Detect Status

static uint8_t gpioToEDS [] =
{
  16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
  17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
} ;

// gpioToREN
//	(Word) offset to the Rising edge ENable register

static uint8_t gpioToREN [] =
{
  19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,
  20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
} ;

// gpioToFEN
//	(Word) offset to the Falling edgde ENable register

static uint8_t gpioToFEN [] =
{
  22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,
  23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
} ;
#endif


// GPPUD:
//	GPIO Pin pull up/down register

#define	GPPUD	37

// gpioToPUDCLK
//	(Word) offset to the Pull Up Down Clock regsiter

static uint8_t gpioToPUDCLK [] =
{
  38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,
  39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,
} ;


// gpioToPwmALT
//	the ALT value to put a GPIO pin into PWM mode

static uint8_t gpioToPwmALT [] =
{
          0,         0,         0,         0,         0,         0,         0,         0,	//  0 ->  7
          0,         0,         0,         0, FSEL_ALT0, FSEL_ALT0,         0,         0, 	//  8 -> 15
          0,         0, FSEL_ALT5, FSEL_ALT5,         0,         0,         0,         0, 	// 16 -> 23
          0,         0,         0,         0,         0,         0,         0,         0,	// 24 -> 31
          0,         0,         0,         0,         0,         0,         0,         0,	// 32 -> 39
  FSEL_ALT0, FSEL_ALT0,         0,         0,         0, FSEL_ALT0,         0,         0,	// 40 -> 47
          0,         0,         0,         0,         0,         0,         0,         0,	// 48 -> 55
          0,         0,         0,         0,         0,         0,         0,         0,	// 56 -> 63
} ;


// gpioToPwmPort
//	The port value to put a GPIO pin into PWM mode

static uint8_t gpioToPwmPort [] =
{
          0,         0,         0,         0,         0,         0,         0,         0,	//  0 ->  7
          0,         0,         0,         0, PWM0_DATA, PWM1_DATA,         0,         0, 	//  8 -> 15
          0,         0, PWM0_DATA, PWM1_DATA,         0,         0,         0,         0, 	// 16 -> 23
          0,         0,         0,         0,         0,         0,         0,         0,	// 24 -> 31
          0,         0,         0,         0,         0,         0,         0,         0,	// 32 -> 39
  PWM0_DATA, PWM1_DATA,         0,         0,         0, PWM1_DATA,         0,         0,	// 40 -> 47
          0,         0,         0,         0,         0,         0,         0,         0,	// 48 -> 55
          0,         0,         0,         0,         0,         0,         0,         0,	// 56 -> 63

} ;

// gpioToGpClkALT:
//	ALT value to put a GPIO pin into GP Clock mode.
//	On the Pi we can really only use BCM_GPIO_4 and BCM_GPIO_21
//	for clocks 0 and 1 respectively, however I'll include the full
//	list for completeness - maybe one day...

#define	GPIO_CLOCK_SOURCE	1

// gpioToGpClkALT0:

static uint8_t gpioToGpClkALT0 [] =
{
          0,         0,         0,         0, FSEL_ALT0, FSEL_ALT0, FSEL_ALT0,         0,	//  0 ->  7
          0,         0,         0,         0,         0,         0,         0,         0, 	//  8 -> 15
          0,         0,         0,         0, FSEL_ALT5, FSEL_ALT5,         0,         0, 	// 16 -> 23
          0,         0,         0,         0,         0,         0,         0,         0,	// 24 -> 31
  FSEL_ALT0,         0, FSEL_ALT0,         0,         0,         0,         0,         0,	// 32 -> 39
          0,         0, FSEL_ALT0, FSEL_ALT0, FSEL_ALT0,         0,         0,         0,	// 40 -> 47
          0,         0,         0,         0,         0,         0,         0,         0,	// 48 -> 55
          0,         0,         0,         0,         0,         0,         0,         0,	// 56 -> 63
} ;

// gpioToClk:
//	(word) Offsets to the clock Control and Divisor register

static uint8_t gpioToClkCon [] =
{
         -1,        -1,        -1,        -1,        28,        30,        32,        -1,	//  0 ->  7
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1, 	//  8 -> 15
         -1,        -1,        -1,        -1,        28,        30,        -1,        -1, 	// 16 -> 23
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,	// 24 -> 31
         28,        -1,        28,        -1,        -1,        -1,        -1,        -1,	// 32 -> 39
         -1,        -1,        28,        30,        28,        -1,        -1,        -1,	// 40 -> 47
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,	// 48 -> 55
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,	// 56 -> 63
} ;

static uint8_t gpioToClkDiv [] =
{
         -1,        -1,        -1,        -1,        29,        31,        33,        -1,	//  0 ->  7
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1, 	//  8 -> 15
         -1,        -1,        -1,        -1,        29,        31,        -1,        -1, 	// 16 -> 23
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,	// 24 -> 31
         29,        -1,        29,        -1,        -1,        -1,        -1,        -1,	// 32 -> 39
         -1,        -1,        29,        31,        29,        -1,        -1,        -1,	// 40 -> 47
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,	// 48 -> 55
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,	// 56 -> 63
} ;


/*add for BananaPro by LeMaker team*/
//map tableb for BP

static int *physToPin ;

static int upDnConvert[3] = {0, 2, 1};

/* guenter static int pinToGpio_BP [64] =
{
  275,259,
 274,273,
 244,245,
 272,226,
 53,52,
 266,270,
 268,269,
 267,228,
 229, -1,
 -1, -1,
 -1,  35, 
 277,45, 
 39, 37, 
 276,38, 
 44,40,
 257,256,      // ...31

 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,// ... 63
} ; guenter ende */ 

// WiringPiNr. gegeben .. -> Array GPIOx orange pi guenter neu
// A ab 0x00, B ab 0x20, C ab 0x40, D ab 0x50 ......
// 00 - 31 = PA00-PA31
// 32 - 63 = PB00-PB31
// 64 - 95 = PC00-PC31
static int pinToGpio_BP [64] =
{
  1,110,    // 0, 1
   0,  3,    // 2, 3
  68, 71,    // 4  5
   2,  6,    // 6, 7
  12, 11,    // 8, 9
  67, 21,    //10,11
  64, 65,    //12,13
  66, 13,    //14,15
  14, -1,    //16,17
  -1, -1,    //18,19
  -1,  7,    //20,21
   8,  9,    //22,23
  10, 20,    //24,25
 200,201,    //26,27
 198,199,    //28,29
  19, 18,    //30,31

 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,// ... 63
} ;
// guenter neu ende


// guenter ... dieses braucht nicht umgewandelt werden, da kein /sys/class/gpio auf orange pi
static int pinTobcm_BP [64] =
{
#if 0
 257,256,   //map to BCM GPIO0,1
 53,52,       //map to BCM GPIO2,3
 226,35,     //map to BCM GPIO4,5
 277,270,   //map to BCM GPIO6,7
 266,269,   //map to BCM GPIO8,9
 268,267,   //map to BCM GPIO10,11
 276,45,      //map to BCM GPIO12,13
 228,229,   //map to BCM GPIO14,15
 38,275,    //map to BCM GPIO16,17
 259,39,    //map to BCM GPIO18,19
 44, 40,     //map to BCM GPIO20,21
 273,244,    //map to BCM GPIO22,23
 245,272,    //map to BCM GPIO24,25
 37, 274,     //map to BCM GPIO26,27
#endif

 19, 18,    //map to BCM GPIO0,1
 12, 11,    //map to BCM GPIO2,3
  6,  7,    //map to BCM GPIO4,5
  8, 21,    //map to BCM GPIO6,7
 67, 65,    //map to BCM GPIO8,9
 64, 66,    //map to BCM GPIO10,11
200,  9,    //map to BCM GPIO12,13
 13, 14,    //map to BCM GPIO14,15
201,  1,    //map to BCM GPIO16,17
110, 10,    //map to BCM GPIO18,19
198,199,    //map to BCM GPIO20,21
  3, 68,    //map to BCM GPIO22,23
 71,  2,    //map to BCM GPIO24,25
 20,  0,    //map to BCM GPIO26,27
 
  -1,-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,  // 29... 44
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, //45... 60
  -1, -1, -1, -1                                                                   // ...63
} ;
/* guenter 
static int physToGpio_BP [64] =
{
  -1,          // 0
  -1,     -1,     //1, 2
   53,    -1,     //3, 4
   52,    -1,     //5, 6
   226,  228,   //7, 8
  -1,     229,   //9, 10
  275,   259,   //11, 12
  274,   -1,     //13, 14
  273,   244,   //15, 16
  -1,     245,   //17, 18
  268,   -1,     //19, 20
   269,  272,   //21, 22
  267,   266,   //23, 24
  -1,    270,   //25, 26
  257,   256,  //27, 28
  35,    -1,    //29, 30
  277,  276,   //31, 32      
  45,   -1,      //33, 34
  39,   38,    //35, 36
  37,   44,     //37, 38
  -1,   40,    //39, 40
   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, //41-> 55
   -1, -1, -1, -1, -1, -1, -1, -1 // 56-> 63
} ;
guenter ende */

//guenter anfang 
static int physToGpio_BP [64] =
{
  -1,          // 0
  -1,    -1,   // 1, 2
  12,    -1,   // 3, 4
  11,    -1,   // 5, 6
   6,    13,   // 7, 8
  -1,    14,   // 9, 10
   1,   110,   //11, 12
   0,    -1,   //13, 14
   3,    68,   //15, 16
  -1,    71,   //17, 18
  64,    -1,   //19, 20
  65,     2,   //21, 22
  66,    67,   //23, 24
  -1,    21,   //25, 26
  19,    18,   //27, 28
   7,    -1,   //29, 30
   8,   200,   //31, 32
   9,    -1,   //33, 34
  10,   201,   //35, 36
  20,   198,   //37, 38
  -1,   199,   //39, 40
   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, //41-> 55
   -1, -1, -1, -1, -1, -1, -1, -1 // 56-> 63
} ;
// guenter ende



static int syspin [64] =
{
  -1, -1, 2, 3, 4, 5, 6, 7,   //GPIO0,1 used to I2C
  8, 9, 10, 11, 12,13, 14, 15,
  16, 17, 18, 19, 20, 21, 22, 23,
  24, 25, 26, 27, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
} ;

static int edge [64] =
{
  -1, -1, -1, -1, 4, -1, -1, 7,   //support the INT
  8, 9, 10, 11, -1,-1, 14, 15,
  -1, 17, -1, -1, -1, -1, 22, 23,
  24, 25, -1, 27, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
} ;

static int pinToGpioR3 [64] =
{
  17, 18, 27, 22, 23, 24, 25, 4,	// From the Original Wiki - GPIO 0 through 7:	wpi  0 -  7
   2,  3,				// I2C  - SDA0, SCL0				wpi  8 -  9
   8,  7,				// SPI  - CE1, CE0				wpi 10 - 11
  10,  9, 11, 				// SPI  - MOSI, MISO, SCLK			wpi 12 - 14
  14, 15,				// UART - Tx, Rx				wpi 15 - 16
  -1, -1, -1, -1,			// Rev 2: New GPIOs 8 though 11			wpi 17 - 20
   5,  6, 13, 19, 26,			// B+						wpi 21, 22, 23, 24, 25
  12, 16, 20, 21,			// B+						wpi 26, 27, 28, 29
   0,  1,				// B+						wpi 30, 31

// Padding:

  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 63
} ;

static int physToGpioR3 [64] =//head num map to BCMpin
{
  -1,		// 0
  -1, -1,	// 1, 2
   2, -1,
   3, -1,
   4, 14,
  -1, 15,
  17, 18,
  27, -1,
  22, 23,
  -1, 24,
  10, -1,
   9, 25,
  11,  8,
  -1,  7,	// 25, 26
  
  0,   1,   //27, 28
  5,  -1,  //29, 30
  6,  12,  //31, 32
  13, -1, //33, 34
  19, 16, //35, 36
  26, 20, //37, 38
  -1, 21, //39, 40
// Padding:

  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 56
  -1, -1, -1, -1, -1, -1, -1, 	// ... 63
} ;

static int physToPinR3 [64] = //return wiringPI pin
{
  -1,		// 0
  -1, -1,	// 1, 2
   8, -1,  //3, 4
   9, -1,  //5, 6
   7, 15,  //7, 8
  -1, 16, //9,10
  0, 1, //11,12
  2, -1, //13,14
  3, 4, //15,16
  -1, 5, //17,18
  12, -1, //19,20
   13, 6, //21,22
  14, 10, //23, 24
  -1,  11,	// 25, 26
  
  30,   31,   //27, 28
  21,  -1,  //29, 30
  22,  26,  //31, 32
  23, -1, //33, 34
  24, 27, //35, 36
  25, 28, //37, 38
  -1, 29, //39, 40
// Padding:

  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 56
  -1, -1, -1, -1, -1, -1, -1, 	// ... 63
} ;

/* guenter raus
static int BP_PIN_MASK[9][32] =  //[BANK]  [INDEX]
{
 {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PA
 {-1,-1,-1, 3,-1,  5,  6, 7,  8,-1,-1,-1,12,13,-1,-1,-1,-1,-1,-1,20,21,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PB
 {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PC
 {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PD
 {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PE
 {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PF
 {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PG
 {-1,-1,  2,-1, 4,  5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,20,21,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PH
 {0,1,-1,3,-1,-1,-1,-1,-1,-1,10,11,12,13,14,-1,16,17,18,19,20,21,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PI
};
 guenter ende */ 

// guenter anfang ... welche pins werden freigegeben .. -1 = gesperrt
static int BP_PIN_MASK[9][32] =  //[BANK]  [INDEX]
{
 { 0, 1, 2, 3,-1,-1, 6, 7, 8, 9,10,11,12,13,14,-1,-1,-1,18,19,20,21,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PA
 {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PB
 { 0, 1, 2, 3, 4,-1,-1, 7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PC
 {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,14,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PD
 {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PE
 {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PF
 {-1,-1,-1,-1,-1,-1, 6, 7, 8, 9,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PG
 {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PH
 {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PI
};
// guenter ende


static int version=0;
static int pwmmode=0;

/*end 20140918*/

/*
 * Functions
 *********************************************************************************
 */
 
/*add for BananaPro by LeMaker team*/ 
 uint32_t readl(uint32_t addr)
{
	  uint32_t val = 0;
	  uint32_t mmap_base = (addr & ~MAP_MASK);
	  uint32_t mmap_seek = ((addr - mmap_base) >> 2);
	  val = *(gpio + mmap_seek);
	  return val;
	
}
void writel(uint32_t val, uint32_t addr)
{
	  uint32_t mmap_base = (addr & ~MAP_MASK);
	  uint32_t mmap_seek = ((addr - mmap_base) >> 2);
	  *(gpio + mmap_seek) = val;
}
//pwm for BananaPro only for pwm1
void sunxi_pwm_set_enable(int en)
{
	 int val = 0;
	 val = readl(SUNXI_PWM_CTRL_REG);
	 if(en)
	 {
		val |= (SUNXI_PWM_CH1_EN | SUNXI_PWM_SCLK_CH1_GATING);
	 } 
	 else 
	 {
		val &= ~(SUNXI_PWM_CH1_EN | SUNXI_PWM_SCLK_CH1_GATING);
	 }
	  if (wiringPiDebug)
		printf(">>function%s,no:%d,enable? :0x%x\n",__func__, __LINE__, val);
	 writel(val, SUNXI_PWM_CTRL_REG);
	 delay (1) ;
}
void sunxi_pwm_set_mode(int mode)
{
	 int val = 0;
	 val = readl(SUNXI_PWM_CTRL_REG);
	 mode &= 1; //cover the mode to 0 or 1
	 if(mode)
	 { //pulse mode
		val |= ( SUNXI_PWM_CH1_MS_MODE|SUNXI_PWM_CH1_PUL_START);
		pwmmode=1;
	 }
	 else 
	 {  //cycle mode
		val &= ~( SUNXI_PWM_CH1_MS_MODE);
		pwmmode=0;
	 }
	 val |= ( SUNXI_PWM_CH1_ACT_STA);
	   if (wiringPiDebug)
			printf(">>function%s,no:%d,mode? :0x%x\n",__func__, __LINE__, val);
	 writel(val, SUNXI_PWM_CTRL_REG);
	 delay (1) ;
}
void sunxi_pwm_set_clk(int clk)
{
	 int val = 0;
	 
	// sunxi_pwm_set_enable(0);
	 val = readl(SUNXI_PWM_CTRL_REG);
	 //clear clk to 0
	 val &= 0xf801f0;
	 val |= ((clk & 0xf) << 15);  //todo check wether clk is invalid or not
	 writel(val, SUNXI_PWM_CTRL_REG);
	 sunxi_pwm_set_enable(1);
	 if (wiringPiDebug)
		printf(">>function%s,no:%d,clk? :0x%x\n",__func__, __LINE__, val);
	delay (1) ;
}
/**
 * ch0 and ch1 set the same,16 bit period and 16 bit act
 */
uint32_t sunxi_pwm_get_period(void)
{
	 uint32_t period_cys = 0;
	 period_cys = readl(SUNXI_PWM_CH1_PERIOD);//get ch1 period_cys
	 period_cys &= 0xffff0000;//get period_cys
	 period_cys = period_cys >> 16;
	   if (wiringPiDebug)
	  printf(">>func:%s,no:%d,period/range:%d",__func__,__LINE__,period_cys);
	 delay (1) ;
	 return period_cys;
}
uint32_t sunxi_pwm_get_act(void)
{
	 uint32_t period_act = 0;
	 period_act = readl(SUNXI_PWM_CH1_PERIOD);//get ch1 period_cys
	 period_act &= 0xffff;//get period_act
	   if (wiringPiDebug)
	  printf(">>func:%s,no:%d,period/range:%d",__func__,__LINE__,period_act);
	  delay (1) ;
	 return period_act;
}
void sunxi_pwm_set_period(int period_cys)
{
	uint32_t val = 0;
	//all clear to 0
	if (wiringPiDebug)
		printf(">>func:%s no:%d\n",__func__,__LINE__);
	period_cys &= 0xffff; //set max period to 2^16
	period_cys = period_cys << 16;
	val = readl(SUNXI_PWM_CH1_PERIOD);
	val &=0x0000ffff;
	period_cys |= val;
	writel(period_cys, SUNXI_PWM_CH1_PERIOD);
	delay (1) ;

}
void sunxi_pwm_set_act(int act_cys)
{
	uint32_t per0 = 0;
	//keep period the same, clear act_cys to 0 first
	if (wiringPiDebug)
		printf(">>func:%s no:%d\n",__func__,__LINE__);
	per0 = readl(SUNXI_PWM_CH1_PERIOD);
	per0 &= 0xffff0000;
	act_cys &= 0xffff;
	act_cys |= per0;
	writel(act_cys,SUNXI_PWM_CH1_PERIOD);
	delay (1) ;
}
int sunxi_get_gpio_mode(int pin)
{
 uint32_t regval = 0;
 int bank = pin >> 5;
 int index = pin - (bank << 5);
 int offset = ((index - ((index >> 3) << 3)) << 2);
 uint32_t reval=0;
 uint32_t phyaddr = SUNXI_GPIO_BASE + (bank * 36) + ((index >> 3) << 2);
 if (wiringPiDebug)
		printf("func:%s pin:%d,  bank:%d index:%d phyaddr:0x%x\n",__func__, pin , bank,index,phyaddr); 
	if(BP_PIN_MASK[bank][index] != -1)
	 {
			regval = readl(phyaddr);
			if (wiringPiDebug)
				printf("read reg val: 0x%x offset:%d  return: %d\n",regval,offset,reval);
			// reval=regval &(reval+(7 << offset));
			reval=(regval>>offset)&7;
			if (wiringPiDebug)
				printf("read reg val: 0x%x offset:%d  return: %d\n",regval,offset,reval);
			return reval;
	 }
	else 
	 {
		printf("line:%dpin number error\n",__LINE__);
		return reval;
	 } 
}

void sunxi_set_gpio_mode(int pin,int mode)
{
 uint32_t regval = 0;
 int bank = pin >> 5;
 int index = pin - (bank << 5);
 int offset = ((index - ((index >> 3) << 3)) << 2);
 uint32_t phyaddr = SUNXI_GPIO_BASE + (bank * 36) + ((index >> 3) << 2);
	if (wiringPiDebug)
		printf("func:%s pin:%d, MODE:%d bank:%d index:%d phyaddr:0x%x\n",__func__, pin , mode,bank,index,phyaddr); 
	if(BP_PIN_MASK[bank][index] != -1)
	 {
			regval = readl(phyaddr);
			if (wiringPiDebug)
				printf("read reg val: 0x%x offset:%d\n",regval,offset);
			if(INPUT == mode)
			{
				regval &= ~(7 << offset);
				writel(regval, phyaddr);
				regval = readl(phyaddr);
			if (wiringPiDebug)
				printf("Input mode set over reg val: 0x%x\n",regval);
			}
			else if(OUTPUT == mode)
			{
			   regval &= ~(7 << offset);
			   regval |=  (1 << offset);
			   if (wiringPiDebug)
					printf("Out mode ready set val: 0x%x\n",regval);
			   writel(regval, phyaddr);
			   regval = readl(phyaddr);
			   if (wiringPiDebug)
					printf("Out mode set over reg val: 0x%x\n",regval);
		  } 
		  else if(PWM_OUTPUT == mode)
		  {
		   // set pin PWMx to pwm mode
		   regval &= ~(7 << offset);
		   regval |=  (0x2 << offset);
		   if (wiringPiDebug)
				printf(">>>>>line:%d PWM mode ready to set val: 0x%x\n",__LINE__,regval);
		   writel(regval, phyaddr);
		   delayMicroseconds (200);
		   regval = readl(phyaddr);
		   if (wiringPiDebug)
				printf("<<<<<PWM mode set over reg val: 0x%x\n",regval);
		   //clear all reg
		   writel(0,SUNXI_PWM_CTRL_REG); 
		   writel(0,SUNXI_PWM_CH0_PERIOD); 
		   writel(0,SUNXI_PWM_CH1_PERIOD); 

		   //set default M:S to 1/2
		   sunxi_pwm_set_period(1024);
		   sunxi_pwm_set_act(512);
		   pwmSetMode(PWM_MODE_MS);
		   sunxi_pwm_set_clk(PWM_CLK_DIV_120);//default clk:24M/120
		   delayMicroseconds (200);
		  }
	 }
	 else 
	 {
		printf("line:%dpin number error\n",__LINE__);
	 }

	return ;
}

void sunxi_digitalWrite(int pin, int value)
{ 
	 uint32_t regval = 0;
	 int bank = pin >> 5;
	 int index = pin - (bank << 5);
	 uint32_t phyaddr = SUNXI_GPIO_BASE + (bank * 36) + 0x10; // +0x10 -> data reg
	   if (wiringPiDebug)
			printf("func:%s pin:%d, value:%d bank:%d index:%d phyaddr:0x%x\n",__func__, pin , value,bank,index,phyaddr); 
	 if(BP_PIN_MASK[bank][index] != -1)
	 {
			regval = readl(phyaddr);
			if (wiringPiDebug)
				printf("befor write reg val: 0x%x,index:%d\n",regval,index);
		  if(0 == value)
		  {
		   regval &= ~(1 << index);
		   writel(regval, phyaddr);
		   regval = readl(phyaddr);
			 if (wiringPiDebug)
				printf("LOW val set over reg val: 0x%x\n",regval);
		  }
		  else
		  {
		   regval |= (1 << index);
		   writel(regval, phyaddr);
		   regval = readl(phyaddr);
			 if (wiringPiDebug)
				printf("HIGH val set over reg val: 0x%x\n",regval);
		  }
	 }
	 else
	 {
		printf("pin number error\n");
	 }

	 return ;
}

int sunxi_digitalRead(int pin)
{
	uint32_t regval = 0;
	int bank = pin >> 5;
	int index = pin - (bank << 5);
	uint32_t phyaddr = SUNXI_GPIO_BASE + (bank * 36) + 0x10; // +0x10 -> data reg
	if (wiringPiDebug)
			printf("func:%s pin:%d,bank:%d index:%d phyaddr:0x%x\n",__func__, pin,bank,index,phyaddr);
	if(BP_PIN_MASK[bank][index] != -1)
	{
		regval = readl(phyaddr);
		regval = regval >> index;
		regval &= 1;
		if (wiringPiDebug)
			printf("***** read reg val: 0x%x,bank:%d,index:%d,line:%d\n",regval,bank,index,__LINE__);
	 	return regval;
	}
	else
	{
		printf("Sunxi_digitalRead() pin - number error\n");
		return regval;
	}
}

void sunxi_pullUpDnControl (int pin, int pud)
{
	 uint32_t regval = 0;
	 int bank = pin >> 5;
	 int index = pin - (bank << 5);
	 int sub = index >> 4;
	 int sub_index = index - 16*sub;
	 uint32_t phyaddr = SUNXI_GPIO_BASE + (bank * 36) + 0x1c + sub*4; // +0x10 -> pullUpDn reg
	   if (wiringPiDebug)
			printf("func:%s pin:%d,bank:%d index:%d sub:%d phyaddr:0x%x\n",__func__, pin,bank,index,sub,phyaddr); 
	 if(BP_PIN_MASK[bank][index] != -1)
	 {  //PI13~PI21 need check again
			regval = readl(phyaddr);
			if (wiringPiDebug)
				printf("pullUpDn reg:0x%x, pud:0x%x sub_index:%d\n", regval, pud, sub_index);
			regval &= ~(3 << (sub_index << 1));
			regval |= (pud << (sub_index << 1));
			if (wiringPiDebug)
				printf("pullUpDn val ready to set:0x%x\n", regval);
			writel(regval, phyaddr);
			regval = readl(phyaddr);
			if (wiringPiDebug)
				printf("pullUpDn reg after set:0x%x  addr:0x%x\n", regval, phyaddr);
	 }
	 else 
	 {
		printf("pin number error\n");
	 } 
	 delay (1) ;	
	return ;
}
/*end 2014.09.18*/

/*
 * wiringPiFailure:
 *	Fail. Or not.
 *********************************************************************************
 */

int wiringPiFailure (int fatal, const char *message, ...)
{
  va_list argp ;
  char buffer [1024] ;

  if (!fatal && wiringPiReturnCodes)
    return -1 ;

  va_start (argp, message) ;
    vsnprintf (buffer, 1023, message, argp) ;
  va_end (argp) ;

  fprintf (stderr, "%s", buffer) ;
  exit (EXIT_FAILURE) ;

  return 0 ;
}


/*
 * piBoardRev:
 *	Return a number representing the hardware revision of the board.
 *
 *	Revision 1 really means the early Model B's.
 *	Revision 2 is everything else - it covers the B, B+ and CM.
 *
 *	Seems there are some boards with 0000 in them (mistake in manufacture)
 *	So the distinction between boards that I can see is:
 *	0000 - Error
 *	0001 - Not used 
 *	0002 - Model B,  Rev 1,   256MB, Egoman
 *	0003 - Model B,  Rev 1.1, 256MB, Egoman, Fuses/D14 removed.
 *	0004 - Model B,  Rev 2,   256MB, Sony
 *	0005 - Model B,  Rev 2,   256MB, Qisda
 *	0006 - Model B,  Rev 2,   256MB, Egoman
 *	0007 - Model A,  Rev 2,   256MB, Egoman
 *	0008 - Model A,  Rev 2,   256MB, Sony
 *	0009 - Model A,  Rev 2,   256MB, Qisda
 *	000d - Model B,  Rev 2,   512MB, Egoman
 *	000e - Model B,  Rev 2,   512MB, Sony
 *	000f - Model B,  Rev 2,   512MB, Qisda
 *	0010 - Model B+, Rev 1.2, 512MB, Sony
 *	0011 - Pi CM,    Rev 1.2, 512MB, Sony
 *
 *	A small thorn is the olde style overvolting - that will add in
 *		1000000
 *
 *	The Pi compute module has an revision of 0011 - since we only check the
 *	last digit, then it's 1, therefore it'll default to not 2 or 3 for a
 *	Rev 1, so will appear as a Rev 2. This is fine for the most part, but
 *	we'll properly detect the Compute Module later and adjust accordingly.
 *
 *********************************************************************************
 */

static void piBoardRevOops (const char *why)
{
  fprintf (stderr, "piBoardRev: Unable to determine board revision from /proc/cpuinfo\n") ;
  fprintf (stderr, " -> %s\n", why) ;
  fprintf (stderr, " ->  You may want to check:\n") ;
  fprintf (stderr, " ->  http://www.lemaker.org/\n") ;  /*modify for BananaPro by LeMmaker team*/
  exit (EXIT_FAILURE) ;
}

/*add for BananaPro by LeMaker team*/
int isA20(void)
{
  FILE *cpuFd ;
  char line [120] ;
  char *d;
	if ((cpuFd = fopen ("/proc/cpuinfo", "r")) == NULL)
		piBoardRevOops ("Unable to open /proc/cpuinfo") ;
	  while (fgets (line, 120, cpuFd) != NULL)
		{
			if (strncmp (line, "Hardware", 8) == 0)
			break ;
		}
		
	fclose (cpuFd) ;
	if (strncmp (line, "Hardware", 8) != 0)
		piBoardRevOops ("No \"Hardware\" line") ;
	
  for (d = &line [strlen (line) - 1] ; (*d == '\n') || (*d == '\r') ; --d)
    *d = 0 ;
  if (wiringPiDebug)
    printf ("piboardRev: Hardware string: %s\n", line) ;
	
	if (strstr(line,"sun7i") != NULL)			
	{
		if (wiringPiDebug)
		printf ("Hardware:%s\n",line) ;
		return 1 ;
	}
	else
	{
		if (wiringPiDebug)
		printf ("Hardware:%s\n",line) ;
		return 0 ;
	}
}
/*end 2014.09.18*/



/*add for H3 guenter*/
int isH3(void)
{
  FILE *cpuFd ;
  char line [120] ;
  char *d;
	if ((cpuFd = fopen ("/proc/cpuinfo", "r")) == NULL)
		piBoardRevOops ("Unable to open /proc/cpuinfo") ;
	  while (fgets (line, 120, cpuFd) != NULL)
		{
			if (strncmp (line, "Hardware", 8) == 0)
			break ;
		}
		
	fclose (cpuFd) ;
	if (strncmp (line, "Hardware", 8) != 0)
		piBoardRevOops ("No \"Hardware\" line") ;
	
  for (d = &line [strlen (line) - 1] ; (*d == '\n') || (*d == '\r') ; --d)
    *d = 0 ;
  if (wiringPiDebug)
    printf ("piboardRev: Hardware string: %s\n", line) ;
	
	if (strstr(line,"sun8i") != NULL)			//guenter von sun7i auf sun8i
	{
		if (wiringPiDebug)
		printf ("Hardware:%s\n",line) ;
		return 1 ;
	}
	else
	{
		if (wiringPiDebug)
		printf ("Hardware:%s\n",line) ;
		return 0 ;
	}
}
/* guenter ende */





int piBoardRev (void)
{
  FILE *cpuFd ;
  char line [120] ;
  char *c ;
  static int  boardRev = -1 ;

/*add for orange pi guenter */
  if(isH3())			//guenter if(isA20())
  {
	version = BPRVER;
		if (wiringPiDebug)
			printf ("piboardRev:  %d\n", version) ;
		return BPRVER ;
  }
 /*end 2014.09.18*/
 
  if (boardRev != -1)	// No point checking twice
    return boardRev ;

  if ((cpuFd = fopen ("/proc/cpuinfo", "r")) == NULL)
    piBoardRevOops ("Unable to open /proc/cpuinfo") ;

  while (fgets (line, 120, cpuFd) != NULL)
    if (strncmp (line, "Revision", 8) == 0)
      break ;

  fclose (cpuFd) ;

  if (strncmp (line, "Revision", 8) != 0)
    piBoardRevOops ("No \"Revision\" line") ;

// Chomp trailing CR/NL

  for (c = &line [strlen (line) - 1] ; (*c == '\n') || (*c == '\r') ; --c)
    *c = 0 ;
  
  if (wiringPiDebug)
    printf ("piboardRev: Revision string: %s\n", line) ;

// Scan to first digit

  for (c = line ; *c ; ++c)
    if (isdigit (*c))
      break ;

  if (!isdigit (*c))
    piBoardRevOops ("No numeric revision string") ;

// Make sure its long enough

  if (strlen (c) < 4)
    piBoardRevOops ("Bogus \"Revision\" line (too small)") ;
  
// If you have overvolted the Pi, then it appears that the revision
//	has 100000 added to it!

  if (wiringPiDebug)
    if (strlen (c) != 4)
      printf ("piboardRev: This Pi has/is overvolted!\n") ;

// Isolate  last 4 characters:

  c = c + strlen (c) - 4 ;

  if (wiringPiDebug)
    printf ("piboardRev: last4Chars are: \"%s\"\n", c) ;

  if ( (strcmp (c, "0002") == 0) || (strcmp (c, "0003") == 0))
    boardRev = 1 ;
  else
    boardRev = 2 ;

  if (wiringPiDebug)
    printf ("piBoardRev: Returning revision: %d\n", boardRev) ;

  return boardRev ;
}


/*
 * piBoardId:
 *	Do more digging into the board revision string as above, but return
 *	as much details as we can.
 *	This is undocumented and really only intended for the GPIO command.
 *	Use at your own risk!
 *********************************************************************************
 */

void piBoardId (int *model, int *rev, int *mem, int *maker, int *overVolted)
{
  FILE *cpuFd ;
  char line [120] ;
  char *c ;

  (void)piBoardRev () ;	// Call this first to make sure all's OK. Don't care about the result.

  if ((cpuFd = fopen ("/proc/cpuinfo", "r")) == NULL)
    piBoardRevOops ("Unable to open /proc/cpuinfo") ;

  while (fgets (line, 120, cpuFd) != NULL)
    if (strncmp (line, "Revision", 8) == 0)
      break ;

  fclose (cpuFd) ;

  if (strncmp (line, "Revision", 8) != 0)
    piBoardRevOops ("No \"Revision\" line") ;

// Chomp trailing CR/NL

  for (c = &line [strlen (line) - 1] ; (*c == '\n') || (*c == '\r') ; --c)
    *c = 0 ;
  
  if (wiringPiDebug)
    printf ("piboardId: Revision string: %s\n", line) ;

// Scan to first digit

  for (c = line ; *c ; ++c)
    if (isdigit (*c))
      break ;

// Make sure its long enough

  if (strlen (c) < 4)
    piBoardRevOops ("Bogus \"Revision\" line") ;

// If longer than 4, we'll assume it's been overvolted

  *overVolted = strlen (c) > 4 ;
  
// Extract last 4 characters:

  c = c + strlen (c) - 4 ;

// Fill out the replys as appropriate

  /**/ if (strcmp (c, "0002") == 0) { *model = PI_MODEL_B  ; *rev = PI_VERSION_1   ; *mem = 256 ; *maker = PI_MAKER_EGOMAN ; }
  else if (strcmp (c, "0003") == 0) { *model = PI_MODEL_B  ; *rev = PI_VERSION_1_1 ; *mem = 256 ; *maker = PI_MAKER_EGOMAN ; }
  else if (strcmp (c, "0004") == 0) { *model = PI_MODEL_B  ; *rev = PI_VERSION_2   ; *mem = 256 ; *maker = PI_MAKER_SONY   ; }
  else if (strcmp (c, "0005") == 0) { *model = PI_MODEL_B  ; *rev = PI_VERSION_2   ; *mem = 256 ; *maker = PI_MAKER_QISDA  ; }
  else if (strcmp (c, "0006") == 0) { *model = PI_MODEL_B  ; *rev = PI_VERSION_2   ; *mem = 256 ; *maker = PI_MAKER_EGOMAN ; }
  else if (strcmp (c, "0007") == 0) { *model = PI_MODEL_A  ; *rev = PI_VERSION_2   ; *mem = 256 ; *maker = PI_MAKER_EGOMAN ; }
  else if (strcmp (c, "0008") == 0) { *model = PI_MODEL_A  ; *rev = PI_VERSION_2   ; *mem = 256 ; *maker = PI_MAKER_SONY ; ; }
  else if (strcmp (c, "0009") == 0) { *model = PI_MODEL_B  ; *rev = PI_VERSION_2   ; *mem = 256 ; *maker = PI_MAKER_QISDA  ; }
  else if (strcmp (c, "000d") == 0) { *model = PI_MODEL_B  ; *rev = PI_VERSION_2   ; *mem = 512 ; *maker = PI_MAKER_EGOMAN ; }
  else if (strcmp (c, "000e") == 0) { *model = PI_MODEL_B  ; *rev = PI_VERSION_2   ; *mem = 512 ; *maker = PI_MAKER_SONY   ; }
  else if (strcmp (c, "000f") == 0) { *model = PI_MODEL_B  ; *rev = PI_VERSION_2   ; *mem = 512 ; *maker = PI_MAKER_EGOMAN ; }
  else if (strcmp (c, "0010") == 0) { *model = PI_MODEL_BP ; *rev = PI_VERSION_1_2 ; *mem = 512 ; *maker = PI_MAKER_SONY   ; }
  else if (strcmp (c, "0011") == 0) { *model = PI_MODEL_CM ; *rev = PI_VERSION_1_2 ; *mem = 512 ; *maker = PI_MAKER_SONY   ; }
//add for BananaPro by LeMaker team
  else if (strcmp (c, "0000") == 0) { *model = PI_MODEL_BPR;  *rev = PI_VERSION_1_2;  *mem = 1024;  *maker = PI_MAKER_LEMAKER;}
//end 2014.09.30
  else                              { *model = 0           ; *rev = 0              ; *mem =   0 ; *maker = 0 ;               }
}
 


/*
 * wpiPinToGpio:
 *	Translate a wiringPi Pin number to native GPIO pin number.
 *	Provided for external support.
 *********************************************************************************
 */

int wpiPinToGpio (int wpiPin)
{
  return pinToGpio_BP [wpiPin & 63] ;
}


/*
 * physPinToGpio:
 *	Translate a physical Pin number to native GPIO pin number.
 *	Provided for external support.
 *********************************************************************************
 */

int physPinToGpio (int physPin)
{
  return physToGpio_BP [physPin & 63] ;
}

/*
 * physPinToGpio:
 *	Translate a physical Pin number to wiringPi  pin number. add by lemaker team for BananaPi
 *	Provided for external support.
 *********************************************************************************
 */
int physPinToPin(int physPin)
{
	return physToPin [physPin & 63] ;
}

/*
 * setPadDrive:
 *	Set the PAD driver value
 *********************************************************************************
 */

void setPadDrive (int group, int value)
{
  uint32_t wrVal ;
 /*add for BananaPro by LeMaker team*/
  if(BPRVER == version) 
  	return;
 /*end 2014.08.19*/
  if ((wiringPiMode == WPI_MODE_PINS) || (wiringPiMode == WPI_MODE_PHYS) || (wiringPiMode == WPI_MODE_GPIO))
  {
    if ((group < 0) || (group > 2))
      return ;

    wrVal = BCM_PASSWORD | 0x18 | (value & 7) ;
    *(pads + group + 11) = wrVal ;

    if (wiringPiDebug)
    {
      printf ("setPadDrive: Group: %d, value: %d (%08X)\n", group, value, wrVal) ;
      printf ("Read : %08X\n", *(pads + group + 11)) ;
    }
  }
}


/*
 * getAlt:
 *	Returns the ALT bits for a given port. Only really of-use
 *	for the gpio readall command (I think)
 *********************************************************************************
 */

int getAlt (int pin)
{
  int fSel, shift, alt ;

  pin &= 63 ;
   /*add for BananaPro by LeMaker team*/
	if(BPRVER == version)
	{

		//printf("[%s:L%d] the pin:%d  mode: %d is invaild,please check it over!\n", __func__,  __LINE__, pin, wiringPiMode);
		if (wiringPiMode == WPI_MODE_PINS)
			pin = pinToGpio_BP [pin] ;
		else if (wiringPiMode == WPI_MODE_PHYS)
			pin = physToGpio_BP[pin] ;
		else if (wiringPiMode == WPI_MODE_GPIO) 
			pin=pinTobcm_BP[pin];//need map A20 to bcm
		else return 0 ;

		if(-1 == pin)
		{
			printf("[%s:L%d] the pin:%d  mode: %d is invaild,please check it over!\n", __func__,  __LINE__, pin, wiringPiMode);
			return -1;
		}
		alt=sunxi_get_gpio_mode(pin);
		 return alt ;
	}
/*end 2014.08.19*/

  /**/ if (wiringPiMode == WPI_MODE_PINS)
    pin = pinToGpio [pin] ;
  else if (wiringPiMode == WPI_MODE_PHYS)
    pin = physToGpio [pin] ;
  else if (wiringPiMode != WPI_MODE_GPIO)
    return 0 ;

  fSel    = gpioToGPFSEL [pin] ;
  shift   = gpioToShift  [pin] ;

  alt = (*(gpio + fSel) >> shift) & 7 ;

  return alt ;
}


/*
 * pwmSetMode:
 *	Select the native "balanced" mode, or standard mark:space mode
 *********************************************************************************
 */

void pwmSetMode (int mode)
{
   /*add for BananaPro by LeMaker team*/
  if (BPRVER == version)
  {
  		sunxi_pwm_set_mode(mode);
		return;
  }
  /*end 2014.08.19*/
  
  if ((wiringPiMode == WPI_MODE_PINS) || (wiringPiMode == WPI_MODE_PHYS) || (wiringPiMode == WPI_MODE_GPIO))
  {
    if (mode == PWM_MODE_MS)
      *(pwm + PWM_CONTROL) = PWM0_ENABLE | PWM1_ENABLE | PWM0_MS_MODE | PWM1_MS_MODE ;
    else
      *(pwm + PWM_CONTROL) = PWM0_ENABLE | PWM1_ENABLE ;
  }
}


/*
 * pwmSetRange:
 *	Set the PWM range register. We set both range registers to the same
 *	value. If you want different in your own code, then write your own.
 *********************************************************************************
 */

void pwmSetRange (unsigned int range)
{
 /*add for BananaPro by LeMaker team*/
  if (BPRVER == version)
  {
  	  sunxi_pwm_set_period(range);
	  return;
  }
 /*end 2014.08.19*/
  if ((wiringPiMode == WPI_MODE_PINS) || (wiringPiMode == WPI_MODE_PHYS) || (wiringPiMode == WPI_MODE_GPIO))
  {
    *(pwm + PWM0_RANGE) = range ; delayMicroseconds (10) ;
    *(pwm + PWM1_RANGE) = range ; delayMicroseconds (10) ;
  }
}


/*
 * pwmSetClock:
 *	Set/Change the PWM clock. Originally my code, but changed
 *	(for the better!) by Chris Hall, <chris@kchall.plus.com>
 *	after further study of the manual and testing with a 'scope
 *********************************************************************************
 */

void pwmSetClock (int divisor)
{
  uint32_t pwm_control ;
	
 /*add for BananaPro by LeMaker team*/
 if (BPRVER == version)
 {
 	 sunxi_pwm_set_clk(divisor);
	 sunxi_pwm_set_enable(1);
	 return;
 }
 /*end 2014.08.19*/
 
  divisor &= 4095 ;
 	
  if ((wiringPiMode == WPI_MODE_PINS) || (wiringPiMode == WPI_MODE_PHYS) || (wiringPiMode == WPI_MODE_GPIO))
  {
    if (wiringPiDebug)
      printf ("Setting to: %d. Current: 0x%08X\n", divisor, *(clk + PWMCLK_DIV)) ;

    pwm_control = *(pwm + PWM_CONTROL) ;		// preserve PWM_CONTROL

// We need to stop PWM prior to stopping PWM clock in MS mode otherwise BUSY
// stays high.

    *(pwm + PWM_CONTROL) = 0 ;				// Stop PWM

// Stop PWM clock before changing divisor. The delay after this does need to
// this big (95uS occasionally fails, 100uS OK), it's almost as though the BUSY
// flag is not working properly in balanced mode. Without the delay when DIV is
// adjusted the clock sometimes switches to very slow, once slow further DIV
// adjustments do nothing and it's difficult to get out of this mode.

    *(clk + PWMCLK_CNTL) = BCM_PASSWORD | 0x01 ;	// Stop PWM Clock
      delayMicroseconds (110) ;			// prevents clock going sloooow

    while ((*(clk + PWMCLK_CNTL) & 0x80) != 0)	// Wait for clock to be !BUSY
      delayMicroseconds (1) ;

    *(clk + PWMCLK_DIV)  = BCM_PASSWORD | (divisor << 12) ;

    *(clk + PWMCLK_CNTL) = BCM_PASSWORD | 0x11 ;	// Start PWM clock
    *(pwm + PWM_CONTROL) = pwm_control ;		// restore PWM_CONTROL

    if (wiringPiDebug)
      printf ("Set     to: %d. Now    : 0x%08X\n", divisor, *(clk + PWMCLK_DIV)) ;
  }
}


/*
 * gpioClockSet:
 *	Set the freuency on a GPIO clock pin
 *********************************************************************************
 */

void gpioClockSet (int pin, int freq)
{
  int divi, divr, divf ;
/*add for BananaPro by LeMaker team*/
  if (BPRVER == version)
		return;
/*end 2014.08.19*/

  pin &= 63 ;

  /**/ if (wiringPiMode == WPI_MODE_PINS)
    pin = pinToGpio [pin] ;
  else if (wiringPiMode == WPI_MODE_PHYS)
    pin = physToGpio [pin] ;
  else if (wiringPiMode != WPI_MODE_GPIO)
    return ;
  
  divi = 19200000 / freq ;
  divr = 19200000 % freq ;
  divf = (int)((double)divr * 4096.0 / 19200000.0) ;

  if (divi > 4095)
    divi = 4095 ;

  *(clk + gpioToClkCon [pin]) = BCM_PASSWORD | GPIO_CLOCK_SOURCE ;		// Stop GPIO Clock
  while ((*(clk + gpioToClkCon [pin]) & 0x80) != 0)				// ... and wait
    ;

  *(clk + gpioToClkDiv [pin]) = BCM_PASSWORD | (divi << 12) | divf ;		// Set dividers
  *(clk + gpioToClkCon [pin]) = BCM_PASSWORD | 0x10 | GPIO_CLOCK_SOURCE ;	// Start Clock
}


/*
 * wiringPiFindNode:
 *      Locate our device node
 *********************************************************************************
 */

struct wiringPiNodeStruct *wiringPiFindNode (int pin)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  while (node != NULL)
    if ((pin >= node->pinBase) && (pin <= node->pinMax))
      return node ;
    else
      node = node->next ;

  return NULL ;
}


/*
 * wiringPiNewNode:
 *	Create a new GPIO node into the wiringPi handling system
 *********************************************************************************
 */

static void pinModeDummy             (struct wiringPiNodeStruct *node, int pin, int mode)  { return ; }
static void pullUpDnControlDummy     (struct wiringPiNodeStruct *node, int pin, int pud)   { return ; }
static int  digitalReadDummy         (struct wiringPiNodeStruct *node, int pin)            { return LOW ; }
static void digitalWriteDummy        (struct wiringPiNodeStruct *node, int pin, int value) { return ; }
static void pwmWriteDummy            (struct wiringPiNodeStruct *node, int pin, int value) { return ; }
static int  analogReadDummy          (struct wiringPiNodeStruct *node, int pin)            { return 0 ; }
static void analogWriteDummy         (struct wiringPiNodeStruct *node, int pin, int value) { return ; }

struct wiringPiNodeStruct *wiringPiNewNode (int pinBase, int numPins)
{
  int    pin ;
  struct wiringPiNodeStruct *node ;

// Minimum pin base is 64

  if (pinBase < 64)
    (void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: pinBase of %d is < 64\n", pinBase) ;

// Check all pins in-case there is overlap:

  for (pin = pinBase ; pin < (pinBase + numPins) ; ++pin)
    if (wiringPiFindNode (pin) != NULL)
      (void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: Pin %d overlaps with existing definition\n", pin) ;

  node = (struct wiringPiNodeStruct *)calloc (sizeof (struct wiringPiNodeStruct), 1) ;	// calloc zeros
  if (node == NULL)
    (void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: Unable to allocate memory: %s\n", strerror (errno)) ;

  node->pinBase         = pinBase ;
  node->pinMax          = pinBase + numPins - 1 ;
  node->pinMode         = pinModeDummy ;
  node->pullUpDnControl = pullUpDnControlDummy ;
  node->digitalRead     = digitalReadDummy ;
  node->digitalWrite    = digitalWriteDummy ;
  node->pwmWrite        = pwmWriteDummy ;
  node->analogRead      = analogReadDummy ;
  node->analogWrite     = analogWriteDummy ;
  node->next            = wiringPiNodes ;
  wiringPiNodes         = node ;

  return node ;
}


#ifdef notYetReady
/*
 * pinED01:
 * pinED10:
 *	Enables edge-detect mode on a pin - from a 0 to a 1 or 1 to 0
 *	Pin must already be in input mode with appropriate pull up/downs set.
 *********************************************************************************
 */

void pinEnableED01Pi (int pin)
{
  pin = pinToGpio_BP [pin & 63] ;
}
#endif


/*
 *********************************************************************************
 * Core Functions
 *********************************************************************************
 */

/*
 * pinModeAlt:
 *	This is an un-documented special to let you set any pin to any mode
 *********************************************************************************
 */

void pinModeAlt (int pin, int mode)
{
  int fSel, shift ;
 /*add for BananaPro by LeMaker team*/
  if (BPRVER == version)
  {
  		return;
  }
 /*end 2014.08.19*/
 
  if ((pin & PI_GPIO_MASK) == 0)		// On-board pin
  {
    /**/ if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio [pin] ;
    else if (wiringPiMode != WPI_MODE_GPIO)
      return ;

    fSel  = gpioToGPFSEL [pin] ;
    shift = gpioToShift  [pin] ;

    *(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)) | ((mode & 0x7) << shift) ;
  }
}


/*
 * pinMode:
 *	Sets the mode of a pin to be input, output or PWM output
 *********************************************************************************
 */

void pinMode (int pin, int mode)
{
  int    fSel, shift, alt ;
  struct wiringPiNodeStruct *node = wiringPiNodes ;
  int origPin = pin ;
	
/*add for BananaPro by LeMaker team*/
 if(BPRVER == version )
	{
		if (wiringPiDebug)
			printf ("%s,%d,pin:%d,mode:%d\n", __func__, __LINE__,pin,mode) ;
		if ((pin & PI_GPIO_MASK) == 0)		// On-board pin
		  {
				if (wiringPiMode == WPI_MODE_PINS)
						pin = pinToGpio_BP [pin] ;
				else if (wiringPiMode == WPI_MODE_PHYS)
						pin = physToGpio_BP[pin] ;
				else if (wiringPiMode == WPI_MODE_GPIO)
						pin=pinTobcm_BP[pin];//need map A20 to bcm
				else return ;
				
				if (-1 == pin)  /*VCC or GND return directly*/
				{
					//printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
					return;
				}
				
				 if (mode == INPUT)
				 {
					  sunxi_set_gpio_mode(pin,INPUT);
					  wiringPinMode = INPUT;
					  return ;
				}
				else if (mode == OUTPUT)
				{
					  sunxi_set_gpio_mode(pin, OUTPUT); //gootoomoon_set_mode
					  wiringPinMode = OUTPUT;
					  return ;
				}
				else if (mode == PWM_OUTPUT)
				{
					  if(pin != 259)
					  {
						   printf("the pin you choose is not surport hardware PWM\n");
						   printf("you can select PI3 for PWM pin\n");
						   printf("or you can use it in softPwm mode\n");
						   return ;
					  }
					  //printf("you choose the hardware PWM:%d\n", 1);
					  sunxi_set_gpio_mode(pin,PWM_OUTPUT);
					  wiringPinMode = PWM_OUTPUT;
					  return ;
				}
				else
					return ;
		  }
	  else
	  {
		if ((node = wiringPiFindNode (pin)) != NULL)
		  node->pinMode (node, pin, mode) ;
		return ;
	  }
	}
 /*end 2014.08.19*/
 
  if ((pin & PI_GPIO_MASK) == 0)		// On-board pin
  {
    /**/ if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio [pin] ;
    else if (wiringPiMode != WPI_MODE_GPIO)
      return ;

    softPwmStop  (origPin) ;
    softToneStop (origPin) ;

    fSel    = gpioToGPFSEL [pin] ;
    shift   = gpioToShift  [pin] ;

    /**/ if (mode == INPUT)
      *(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)) ; // Sets bits to zero = input
    else if (mode == OUTPUT)
      *(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)) | (1 << shift) ;
    else if (mode == SOFT_PWM_OUTPUT)
      softPwmCreate (origPin, 0, 100) ;
    else if (mode == SOFT_TONE_OUTPUT)
      softToneCreate (origPin) ;
    else if (mode == PWM_TONE_OUTPUT)
    {
      pinMode (origPin, PWM_OUTPUT) ;	// Call myself to enable PWM mode
      pwmSetMode (PWM_MODE_MS) ;
    }
    else if (mode == PWM_OUTPUT)
    {
      if ((alt = gpioToPwmALT [pin]) == 0)	// Not a hardware capable PWM pin
	return ;

// Set pin to PWM mode

      *(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)) | (alt << shift) ;
      delayMicroseconds (110) ;		// See comments in pwmSetClockWPi

      pwmSetMode  (PWM_MODE_BAL) ;	// Pi default mode
      pwmSetRange (1024) ;		// Default range of 1024
      pwmSetClock (32) ;		// 19.2 / 32 = 600KHz - Also starts the PWM
    }
    else if (mode == GPIO_CLOCK)
    {
      if ((alt = gpioToGpClkALT0 [pin]) == 0)	// Not a GPIO_CLOCK pin
	return ;

// Set pin to GPIO_CLOCK mode and set the clock frequency to 100KHz

      *(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)) | (alt << shift) ;
      delayMicroseconds (110) ;
      gpioClockSet      (pin, 100000) ;
    }
  }
  else
  {
    if ((node = wiringPiFindNode (pin)) != NULL)
      node->pinMode (node, pin, mode) ;
    return ;
  }
}


/*
 * pullUpDownCtrl:
 *	Control the internal pull-up/down resistors on a GPIO pin
 *	The Arduino only has pull-ups and these are enabled by writing 1
 *	to a port when in input mode - this paradigm doesn't quite apply
 *	here though.
 *********************************************************************************
 */

void pullUpDnControl (int pin, int pud)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;
	
/*add for BananaPro by LeMaker team*/
if(BPRVER == version)
	{	
		if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
		  {
			   if (wiringPiMode == WPI_MODE_PINS)
					pin = pinToGpio_BP [pin] ;
				else if (wiringPiMode == WPI_MODE_PHYS)
					pin = physToGpio_BP[pin] ;
				else if (wiringPiMode == WPI_MODE_GPIO)
					pin=pinTobcm_BP[pin];//need map A20 to bcm
				else return ;
				if (wiringPiDebug)
					printf ("%s,%d,pin:%d\n", __func__, __LINE__,pin) ;

				if (-1 == pin)
				{
					printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
					return;
				}

				pud = upDnConvert[pud]; // convert wiringpi pud to sunxi pud value
				sunxi_pullUpDnControl(pin, pud);
				return;
		  }
		else	// Extension module
		  {
			if ((node = wiringPiFindNode (pin)) != NULL)
			  node->pullUpDnControl (node, pin, pud) ;
			return ;
		  }
	  }
/*end 2014.08.19*/

  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
  {
    /**/ if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio [pin] ;
    else if (wiringPiMode != WPI_MODE_GPIO)
      return ;

    *(gpio + GPPUD)              = pud & 3 ;		delayMicroseconds (5) ;
    *(gpio + gpioToPUDCLK [pin]) = 1 << (pin & 31) ;	delayMicroseconds (5) ;
    
    *(gpio + GPPUD)              = 0 ;			delayMicroseconds (5) ;
    *(gpio + gpioToPUDCLK [pin]) = 0 ;			delayMicroseconds (5) ;
  }
  else						// Extension module
  {
    if ((node = wiringPiFindNode (pin)) != NULL)
      node->pullUpDnControl (node, pin, pud) ;
    return ;
  }
}


/*
 * digitalRead:
 *	Read the value of a given Pin, returning HIGH or LOW
 *********************************************************************************
 */

int digitalRead (int pin)
{
  char c ;
  struct wiringPiNodeStruct *node = wiringPiNodes ;

	/*add for BananaPro by LeMaker team*/
	if(BPRVER == version)
	{
		if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
			  {
				int _pinMode = getAlt(pin);
				 if (wiringPiMode == WPI_MODE_GPIO_SYS)	// Sys mode
				{
					if(pin==0)
					{
						//printf("%d %s,%d invalid pin,please check it over.\n",pin,__func__, __LINE__);
						return 0;
					}
					if(syspin[pin]==-1)
					{
						//printf("%d %s,%d invalid pin,please check it over.\n",pin,__func__, __LINE__);
						return 0;
					}
					if (sysFds [pin] == -1)
						{
							if (wiringPiDebug)
								printf ("pin %d sysFds -1.%s,%d\n", pin ,__func__, __LINE__) ;
							return LOW;
						}
					if (wiringPiDebug)
						printf ("pin %d :%d.%s,%d\n", pin ,sysFds [pin],__func__, __LINE__) ;
					lseek  (sysFds [pin], 0L, SEEK_SET) ;
					read   (sysFds [pin], &c, 1) ;
					return (c == '0') ? LOW : HIGH ;
				}
				else if (wiringPiMode == WPI_MODE_PINS)
					pin = pinToGpio_BP [pin] ;
				else if (wiringPiMode == WPI_MODE_PHYS)
					pin = physToGpio_BP[pin] ;
				else if (wiringPiMode == WPI_MODE_GPIO)
					pin=pinTobcm_BP[pin];//need map A20 to bcm
				else
				  return LOW ;
				if(-1 == pin){
					printf("[%s:L%d] the pin:%d is invalid, please check it over!\n", __func__,  __LINE__, pin);
					return LOW;
					}
				if(_pinMode == 6){
					// -- Allwinner H3 --
					//ALT2, ie. EINT is enabled!
					//Gotta export the pin if it's not exported already, then read its value that way
					char fName[64];
					struct stat s;
					sprintf(fName, "/sys/class/gpio/gpio%d", pin);
					if(stat(fName, &s) == -1) {
						int tempFd = open("/sys/class/gpio/export", O_WRONLY);
						if(tempFd < 0) return LOW;
						sprintf(fName, "%d\n", pin);
						write(tempFd, fName, strlen(fName));
						close(tempFd);
						}
					sprintf(fName, "/sys/class/gpio/gpio%d/value", pin);
					if(sysFds[pin] == -1){
						if ((sysFds[pin] = open (fName, O_RDWR)) < 0){
							printf("digitalRead(): Failed to open %s, pin not exported?\n", fName);
							return LOW;
							}
						}
					lseek(sysFds[pin], 0L, SEEK_SET);
					read(sysFds[pin], &c, 1);
					return (c == '0') ? LOW : HIGH;
					}
				return sunxi_digitalRead(pin);
			  }
		else
		  {
			if ((node = wiringPiFindNode (pin)) == NULL)
			  return LOW ;
			return node->digitalRead (node, pin) ;
		  }
	  }
	/*end 2014.08.19*/

  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
  {
    /**/ if (wiringPiMode == WPI_MODE_GPIO_SYS)	// Sys mode
    {
      if (sysFds [pin] == -1)
	return LOW ;

      lseek  (sysFds [pin], 0L, SEEK_SET) ;
      read   (sysFds [pin], &c, 1) ;
      return (c == '0') ? LOW : HIGH ;
    }
    else if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio [pin] ;
    else if (wiringPiMode != WPI_MODE_GPIO)
      return LOW ;

    if ((*(gpio + gpioToGPLEV [pin]) & (1 << (pin & 31))) != 0)
      return HIGH ;
    else
      return LOW ;
  }
  else
  {
    if ((node = wiringPiFindNode (pin)) == NULL)
      return LOW ;
    return node->digitalRead (node, pin) ;
  }
}


/*
 * digitalWrite:
 *	Set an output bit
 *********************************************************************************
 */

void digitalWrite (int pin, int value)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;
  /*add for BananaPro by LeMaker team*/
	if(BPRVER == version)
	{	
		if (wiringPiDebug)
		printf ("%s,%d\n", __func__, __LINE__) ;
		if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
		{
			/**/ if (wiringPiMode == WPI_MODE_GPIO_SYS)	// Sys mode
					{
						if (wiringPiDebug)
						{
							printf("%d %s,%d invalid pin,please check it over.\n",pin,__func__, __LINE__);
						}
						if(pin==0)
						{
							//printf("%d %s,%d invalid pin,please check it over.\n",pin,__func__, __LINE__);
							return;
						}
						if(syspin[pin]==-1)
							{
								//printf("%d %s,%d invalid pin,please check it over.\n",pin,__func__, __LINE__);
								return;
							}
						if (sysFds [pin] == -1)
						{
							if (wiringPiDebug)
								printf ("pin %d sysFds -1.%s,%d\n", pin ,__func__, __LINE__) ;
						}
						if (sysFds [pin] != -1)
						  {
							if (wiringPiDebug)
									printf ("pin %d :%d.%s,%d\n", pin ,sysFds [pin],__func__, __LINE__) ;
							if (value == LOW)
							  write (sysFds [pin], "0\n", 2) ;
							else
							  write (sysFds [pin], "1\n", 2) ;
						  }
						return ;
					}
					else if (wiringPiMode == WPI_MODE_PINS)
					   pin = pinToGpio_BP [pin] ;
					else if (wiringPiMode == WPI_MODE_PHYS)
					   pin = physToGpio_BP[pin] ;
					else if (wiringPiMode == WPI_MODE_GPIO)
					    pin=pinTobcm_BP[pin];//need map A20 to bcm
					else  return ;
				   if(-1 == pin){
						//printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
						return ;
					}
					sunxi_digitalWrite(pin, value);		
			}
		  else
		  {
			if ((node = wiringPiFindNode (pin)) != NULL)
			  node->digitalWrite (node, pin, value) ;
		  }
		  return;
	  }
	/*end 2014.08.19*/

  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
  {
    /**/ if (wiringPiMode == WPI_MODE_GPIO_SYS)	// Sys mode
    {
      if (sysFds [pin] != -1)
      {
	if (value == LOW)
	  write (sysFds [pin], "0\n", 2) ;
	else
	  write (sysFds [pin], "1\n", 2) ;
      }
      return ;
    }
    else if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio [pin] ;
    else if (wiringPiMode != WPI_MODE_GPIO)
      return ;

    if (value == LOW)
      *(gpio + gpioToGPCLR [pin]) = 1 << (pin & 31) ;
    else
      *(gpio + gpioToGPSET [pin]) = 1 << (pin & 31) ;
  }
  else
  {
    if ((node = wiringPiFindNode (pin)) != NULL)
      node->digitalWrite (node, pin, value) ;
  }
}


/*
 * pwmWrite:
 *	Set an output PWM value
 *********************************************************************************
 */

void pwmWrite (int pin, int value)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;
	
 /*add for BananaPro by LeMaker team*/
	if(BPRVER == version)
	{	
		 uint32_t a_val = 0;
		 if(pwmmode==1)//sycle
		 {
			sunxi_pwm_set_mode(1);
		 }
		 else
		 {
			//sunxi_pwm_set_mode(0);
		 }
		 if (pin < MAX_PIN_NUM)  // On-Board Pin needto fix me Jim
		 {
		  if (wiringPiMode == WPI_MODE_PINS)
		   pin = pinToGpio_BP [pin] ;
		  else if (wiringPiMode == WPI_MODE_PHYS){
		   pin = physToGpio_BP[pin] ;
		  } else if (wiringPiMode == WPI_MODE_GPIO)
		  pin=pinTobcm_BP[pin];//need map A20 to bcm
		  else
		   return ;
			if(-1 == pin){
				printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
				return ;
				}
		  if(pin != 259){
		   printf("please use soft pwmmode or choose PWM pin\n");
		   return ;
		  }
		  a_val = sunxi_pwm_get_period();
			if (wiringPiDebug)
		   printf("==> no:%d period now is :%d,act_val to be set:%d\n",__LINE__,a_val, value);
		   if(value > a_val){
		   printf("val pwmWrite 0 <= X <= 1024\n");
		   printf("Or you can set new range by yourself by pwmSetRange(range\n");
		   return;
		  }
		  //if value changed chang it
		  sunxi_pwm_set_enable(0);
		  sunxi_pwm_set_act(value);
		  sunxi_pwm_set_enable(1);
		 } else {
		  printf ("not on board :%s,%d\n", __func__, __LINE__) ;
		  if ((node = wiringPiFindNode (pin)) != NULL){
			 if (wiringPiDebug)
					 printf ("Jim find node%s,%d\n", __func__, __LINE__) ;
		   node->digitalWrite (node, pin, value) ;
		  }
		 }
		   if (wiringPiDebug)
		  printf ("this fun is ok now %s,%d\n", __func__, __LINE__) ;
		 
		 return;
  }
 /*end 2014.08.19*/	
 
  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
  {
    /**/ if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio [pin] ;
    else if (wiringPiMode != WPI_MODE_GPIO)
      return ;

    *(pwm + gpioToPwmPort [pin]) = value ;
  }
  else
  {
    if ((node = wiringPiFindNode (pin)) != NULL)
      node->pwmWrite (node, pin, value) ;
  }
}


/*
 * analogRead:
 *	Read the analog value of a given Pin. 
 *	There is no on-board Pi analog hardware,
 *	so this needs to go to a new node.
 *********************************************************************************
 */

int analogRead (int pin)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  if ((node = wiringPiFindNode (pin)) == NULL)
    return 0 ;
  else
    return node->analogRead (node, pin) ;
}


/*
 * analogWrite:
 *	Write the analog value to the given Pin. 
 *	There is no on-board Pi analog hardware,
 *	so this needs to go to a new node.
 *********************************************************************************
 */

void analogWrite (int pin, int value)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  if ((node = wiringPiFindNode (pin)) == NULL)
    return ;

  node->analogWrite (node, pin, value) ;
}


/*
 * pwmToneWrite:
 *	Pi Specific.
 *      Output the given frequency on the Pi's PWM pin
 *********************************************************************************
 */

void pwmToneWrite (int pin, int freq)
{
  int range ;

  if (freq == 0)
    pwmWrite (pin, 0) ;             // Off
  else
  {
    range = 200000 / freq ;
    pwmSetRange (range) ;
    pwmWrite    (pin, range / 2) ;
  }
}



/*
 * digitalWriteByte:
 *	Pi Specific
 *	Write an 8-bit byte to the first 8 GPIO pins - try to do it as
 *	fast as possible.
 *	However it still needs 2 operations to set the bits, so any external
 *	hardware must not rely on seeing a change as there will be a change 
 *	to set the outputs bits to zero, then another change to set the 1's
 *********************************************************************************
 */
static int head2win[8]={11,12,13,15,16,18,22,7}; /*add for BananaPro by lemaker team*/
void digitalWriteByte (int value)
{
  uint32_t pinSet = 0 ;
  uint32_t pinClr = 0 ;
  int mask = 1 ;
  int pin ;

 /*add for BananaPro by LeMaker team*/
 	if(BPRVER == version)
	{
		if (wiringPiMode == WPI_MODE_GPIO_SYS||wiringPiMode == WPI_MODE_GPIO)
		{
			
			for (pin = 0 ; pin < 8 ; ++pin)
			{
				
				pinMode(pin,OUTPUT);
				delay(1);
			  digitalWrite (pinToGpio [pin], value & mask) ;
			  mask <<= 1 ;
			}
			
		}
		else if(wiringPiMode == WPI_MODE_PINS)
		{
			
			for (pin = 0 ; pin < 8 ; ++pin)
			{
				
				pinMode(pin,OUTPUT);
				delay(1);
			  digitalWrite (pin, value & mask) ;
			  mask <<= 1 ;
			}
		}
		else
		{
			for (pin = 0 ; pin < 8 ; ++pin)
			{
				pinMode(head2win[pin],OUTPUT);
				delay(1);
			  digitalWrite (head2win[pin], value & mask) ;
			  mask <<= 1 ;
			}
		}
		return ;
	}
 /*end 2014.08.19*/
 
  /**/ if (wiringPiMode == WPI_MODE_GPIO_SYS)
  {
    for (pin = 0 ; pin < 8 ; ++pin)
    {
      digitalWrite (pin, value & mask) ;
      mask <<= 1 ;
    }
    return ;
  }
  else
  {
    for (pin = 0 ; pin < 8 ; ++pin)
    {
      if ((value & mask) == 0)
	pinClr |= (1 << pinToGpio [pin]) ;
      else
	pinSet |= (1 << pinToGpio [pin]) ;

      mask <<= 1 ;
    }

    *(gpio + gpioToGPCLR [0]) = pinClr ;
    *(gpio + gpioToGPSET [0]) = pinSet ;
  }
}


/*
 * waitForInterrupt:
 *	Pi Specific.
 *	Wait for Interrupt on a GPIO pin.
 *	This is actually done via the /sys/class/gpio interface regardless of
 *	the wiringPi access mode in-use. Maybe sometime it might get a better
 *	way for a bit more efficiency.
 *********************************************************************************
 */

int waitForInterrupt (int pin, int mS)
{
  int fd, x ;
  uint8_t c ;
  struct pollfd polls ;

  /**/ if (wiringPiMode == WPI_MODE_PINS)
    pin = pinToGpio_BP [pin] ;
  else if (wiringPiMode == WPI_MODE_PHYS)
    pin = physToGpio_BP [pin] ;

  if ((fd = sysFds [pin]) == -1)
    return -2 ;

// Setup poll structure

  polls.fd     = fd ;
  polls.events = POLLPRI ;	// Urgent data!

// Wait for it ...

  x = poll (&polls, 1, mS) ;

// Do a dummy read to clear the interrupt
//	A one character read appars to be enough.

  (void)read (fd, &c, 1) ;

  return x ;
}


/*
 * interruptHandler:
 *	This is a thread and gets started to wait for the interrupt we're
 *	hoping to catch. It will call the user-function when the interrupt
 *	fires.
 *********************************************************************************
 */

static void *interruptHandler (void *arg)
{
  int myPin ;

  (void)piHiPri (55) ;	// Only effective if we run as root

  myPin   = pinPass ;
  pinPass = -1 ;

  for (;;)
    if (waitForInterrupt (myPin, -1) > 0)
      isrFunctions [myPin] () ;

  return NULL ;
}


/*
 * wiringPiISR:
 *	Pi Specific.
 *	Take the details and create an interrupt handler that will do a call-
 *	back to the user supplied function.
 *********************************************************************************
 */

int wiringPiISR (int pin, int mode, void (*function)(void))
{
  pthread_t threadId ;
  const char *modeS ;
  char fName   [64] ;
  char  pinS [8] ;
  pid_t pid ;
  int   count, i ;
  char  c ;
  int   bcmGpioPin ;

  if ((pin < 0) || (pin > 63))
    return wiringPiFailure (WPI_FATAL, "wiringPiISR: pin must be 0-63 (%d)\n", pin) ;

  /**/ if (wiringPiMode == WPI_MODE_UNINITIALISED)
    return wiringPiFailure (WPI_FATAL, "wiringPiISR: wiringPi has not been initialised. Unable to continue.\n") ;
  else if (wiringPiMode == WPI_MODE_PINS)
    bcmGpioPin = pinToGpio_BP [pin] ;
  else if (wiringPiMode == WPI_MODE_PHYS)
    bcmGpioPin = physToGpio_BP [pin] ;
  else
    bcmGpioPin = pin ;

	/*add for BananaPro by LeMaker team*/
	
	if(BPRVER == version)
	{
		if(-1 == bcmGpioPin)  /**/
		{
			printf("[%s:L%d] the pin:%d is invalid, please check it over!\n", __func__,  __LINE__, pin);
			return -1;
		}
		if(isH3()){
			// All PA- and PG-pins support interrupts, the others don't
			if(bcmGpioPin > 31 && bcmGpioPin < 192)
				return wiringPiFailure (WPI_FATAL, "wiringPiISR: the specified pin does not support interrupts on this board (Allwinner H3) (%d,%d)\n", pin,bcmGpioPin) ;
		}
		else if(edge[bcmGpioPin]==-1)
			return wiringPiFailure (WPI_FATAL, "wiringPiISR: pin not supported on Banana Pi (%d,%d)\n", pin,bcmGpioPin) ;
	}
	/*end 2014.08.19*/
	
// Now export the pin and set the right edge
//	We're going to use the gpio program to do this, so it assumes
//	a full installation of wiringPi. It's a bit 'clunky', but it
//	is a way that will work when we're running in "Sys" mode, as
//	a non-root user. (without sudo)

  if (mode != INT_EDGE_SETUP)
  {
    /**/ if (mode == INT_EDGE_FALLING)
      modeS = "falling" ;
    else if (mode == INT_EDGE_RISING)
      modeS = "rising" ;
    else
      modeS = "both" ;

    sprintf (pinS, "%d", bcmGpioPin) ;

    if ((pid = fork ()) < 0)	// Fail
      return wiringPiFailure (WPI_FATAL, "wiringPiISR: fork failed: %s\n", strerror (errno)) ;

    if (pid == 0)	// Child, exec
    {
      /**/ if (access ("/usr/local/bin/gpio", X_OK) == 0)
      {
	execl ("/usr/local/bin/gpio", "gpio", "edge", pinS, modeS, (char *)NULL) ;
	return wiringPiFailure (WPI_FATAL, "wiringPiISR: execl failed: %s\n", strerror (errno)) ;
      }
      else if (access ("/usr/bin/gpio", X_OK) == 0)
      {
	execl ("/usr/bin/gpio", "gpio", "edge", pinS, modeS, (char *)NULL) ;
	return wiringPiFailure (WPI_FATAL, "wiringPiISR: execl failed: %s\n", strerror (errno)) ;
      }
      else
	return wiringPiFailure (WPI_FATAL, "wiringPiISR: Can't find gpio program\n") ;
    }
    else		// Parent, wait
      wait (NULL) ;
  }

// Now pre-open the /sys/class node - but it may already be open if
//	we are in Sys mode...

  if (sysFds [bcmGpioPin] == -1)
  {
    sprintf (fName, "/sys/class/gpio/gpio%d/value", bcmGpioPin) ;
    if ((sysFds [bcmGpioPin] = open (fName, O_RDWR)) < 0)
      return wiringPiFailure (WPI_FATAL, "wiringPiISR: unable to open %s: %s\n", fName, strerror (errno)) ;
  }

// Clear any initial pending interrupt

  ioctl (sysFds [bcmGpioPin], FIONREAD, &count) ;
  for (i = 0 ; i < count ; ++i)
    read (sysFds [bcmGpioPin], &c, 1) ;

  isrFunctions [pin] = function ;

  pthread_mutex_lock (&pinMutex) ;
    pinPass = pin ;
    pthread_create (&threadId, NULL, interruptHandler, NULL) ;
    while (pinPass != -1)
      delay (1) ;
  pthread_mutex_unlock (&pinMutex) ;

  return 0 ;
}


/*
 * initialiseEpoch:
 *	Initialise our start-of-time variable to be the current unix
 *	time in milliseconds and microseconds.
 *********************************************************************************
 */

static void initialiseEpoch (void)
{
  struct timeval tv ;

  gettimeofday (&tv, NULL) ;
  epochMilli = (uint64_t)tv.tv_sec * (uint64_t)1000    + (uint64_t)(tv.tv_usec / 1000) ;
  epochMicro = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)(tv.tv_usec) ;
}


/*
 * delay:
 *	Wait for some number of milliseconds
 *********************************************************************************
 */

void delay (unsigned int howLong)
{
  struct timespec sleeper, dummy ;

  sleeper.tv_sec  = (time_t)(howLong / 1000) ;
  sleeper.tv_nsec = (long)(howLong % 1000) * 1000000 ;

  nanosleep (&sleeper, &dummy) ;
}


/*
 * delayMicroseconds:
 *	This is somewhat intersting. It seems that on the Pi, a single call
 *	to nanosleep takes some 80 to 130 microseconds anyway, so while
 *	obeying the standards (may take longer), it's not always what we
 *	want!
 *
 *	So what I'll do now is if the delay is less than 100uS we'll do it
 *	in a hard loop, watching a built-in counter on the ARM chip. This is
 *	somewhat sub-optimal in that it uses 100% CPU, something not an issue
 *	in a microcontroller, but under a multi-tasking, multi-user OS, it's
 *	wastefull, however we've no real choice )-:
 *
 *      Plan B: It seems all might not be well with that plan, so changing it
 *      to use gettimeofday () and poll on that instead...
 *********************************************************************************
 */

void delayMicrosecondsHard (unsigned int howLong)
{
  struct timeval tNow, tLong, tEnd ;

  gettimeofday (&tNow, NULL) ;
  tLong.tv_sec  = howLong / 1000000 ;
  tLong.tv_usec = howLong % 1000000 ;
  timeradd (&tNow, &tLong, &tEnd) ;

  while (timercmp (&tNow, &tEnd, <))
    gettimeofday (&tNow, NULL) ;
}

void delayMicroseconds (unsigned int howLong)
{
  struct timespec sleeper ;
  unsigned int uSecs = howLong % 1000000 ;
  unsigned int wSecs = howLong / 1000000 ;

  /**/ if (howLong ==   0)
    return ;
  else if (howLong  < 100)
    delayMicrosecondsHard (howLong) ;
  else
  {
    sleeper.tv_sec  = wSecs ;
    sleeper.tv_nsec = (long)(uSecs * 1000L) ;
    nanosleep (&sleeper, NULL) ;
  }
}


/*
 * millis:
 *	Return a number of milliseconds as an unsigned int.
 *********************************************************************************
 */

unsigned int millis (void)
{
  struct timeval tv ;
  uint64_t now ;

  gettimeofday (&tv, NULL) ;
  now  = (uint64_t)tv.tv_sec * (uint64_t)1000 + (uint64_t)(tv.tv_usec / 1000) ;

  return (uint32_t)(now - epochMilli) ;
}


/*
 * micros:
 *	Return a number of microseconds as an unsigned int.
 *********************************************************************************
 */

unsigned int micros (void)
{
  struct timeval tv ;
  uint64_t now ;

  gettimeofday (&tv, NULL) ;
  now  = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)tv.tv_usec ;

  return (uint32_t)(now - epochMicro) ;
}


/*
 * wiringPiSetup:
 *	Must be called once at the start of your program execution.
 *
 * Default setup: Initialises the system into wiringPi Pin mode and uses the
 *	memory mapped hardware directly.
 *
 * Changed now to revert to "gpio" mode if we're running on a Compute Module.
 *********************************************************************************
 */

int wiringPiSetup (void)
{
  int   fd ;
  int   boardRev ;
  int   model, rev, mem, maker, overVolted ;
  memset(&sysFds, -1, sizeof(int)*300); // Initialize the filedescriptor - table all to -1

  if (getenv (ENV_DEBUG) != NULL)
    wiringPiDebug = TRUE ;

  if (getenv (ENV_CODES) != NULL)
    wiringPiReturnCodes = TRUE ;

  if (geteuid () != 0)
    (void)wiringPiFailure (WPI_FATAL, "wiringPiSetup: Must be root. (Did you forget sudo?)\n") ;

  if (wiringPiDebug)
    printf ("wiringPi: wiringPiSetup called\n") ;

  boardRev = piBoardRev () ;
	
  if (BPRVER == boardRev)  /*modify for BananaPro by LeMaker team zhaolei*/
  {
  	 	pinToGpio =  pinToGpioR3 ;
		physToGpio = physToGpioR3 ;
		physToPin = physToPinR3;
  }
  else
  {
	  /**/ if (boardRev == 1)	// A, B, Rev 1, 1.1
	  {
	     pinToGpio =  pinToGpioR1 ;
	    physToGpio = physToGpioR1 ;
	  }
	  else 				// A, B, Rev 2, B+, CM
	  {
	     pinToGpio =  pinToGpioR2 ;
	    physToGpio = physToGpioR2 ;
	  }
  }
	
// Open the master /dev/memory device

  if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
    return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno)) ;

	if (BPRVER == boardRev)  /*modify for BananaPro by LeMaker team*/
	{
			// GPIO:
			  gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_BASE_BP);
				//if (wiringPiDebug)
				//	printf("++++ gpio:0x%x\n", gpio);
			  //gpio += 0x21b; //for PD0 cubieboard
				//if (wiringPiDebug)
				//	printf("++++ gpio PDx:0x%x\n", gpio);
			  if ((int32_t)gpio == -1)
				return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (GPIO) failed: %s\n", strerror (errno)) ;

			// PWM

			  pwm = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_PWM_BP) ;
			  if ((int32_t)pwm == -1)
				 return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (PWM) failed: %s\n", strerror (errno)) ;
			 
			// Clock control (needed for PWM)

			  clk = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, CLOCK_BASE_BP) ;
			  if ((int32_t)clk == -1)
				 return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (CLOCK) failed: %s\n", strerror (errno)) ;
			 
			// The drive pads

			  pads = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_PADS_BP) ;
			  if ((int32_t)pads == -1)
				 return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (PADS) failed: %s\n", strerror (errno)) ;

			#ifdef	USE_TIMER
			// The system timer

			  timer = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_TIMER_BP) ;
			  if ((int32_t)timer == -1)
				return wiringPiFailure (WPI_ALMOST,"wiringPiSetup: mmap (TIMER) failed: %s\n", strerror (errno)) ;

			// Set the timer to free-running, 1MHz.
			//	0xF9 is 249, the timer divide is base clock / (divide+1)
			//	so base clock is 250MHz / 250 = 1MHz.

			  *(timer + TIMER_CONTROL) = 0x0000280 ;
			  *(timer + TIMER_PRE_DIV) = 0x00000F9 ;
			  timerIrqRaw = timer + TIMER_IRQ_RAW ;
			#endif		
	}
	else
	{
		// GPIO:
		  gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_BASE) ;
		  if ((int32_t)gpio == -1)
		    return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (GPIO) failed: %s\n", strerror (errno)) ;

		// PWM

		  pwm = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_PWM) ;
		  if ((int32_t)pwm == -1)
		    return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (PWM) failed: %s\n", strerror (errno)) ;
		 
		// Clock control (needed for PWM)

		  clk = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, CLOCK_BASE) ;
		  if ((int32_t)clk == -1)
		    return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (CLOCK) failed: %s\n", strerror (errno)) ;
		 
		// The drive pads

		  pads = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_PADS) ;
		  if ((int32_t)pads == -1)
		    return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (PADS) failed: %s\n", strerror (errno)) ;

#ifdef	USE_TIMER
		// The system timer

		  timer = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_TIMER) ;
		  if ((int32_t)timer == -1)
		    return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (TIMER) failed: %s\n", strerror (errno)) ;

		// Set the timer to free-running, 1MHz.
		//	0xF9 is 249, the timer divide is base clock / (divide+1)
		//	so base clock is 250MHz / 250 = 1MHz.

		  *(timer + TIMER_CONTROL) = 0x0000280 ;
		  *(timer + TIMER_PRE_DIV) = 0x00000F9 ;
		  timerIrqRaw = timer + TIMER_IRQ_RAW ;
#endif
		}
  initialiseEpoch () ;

// If we're running on a compute module, then wiringPi pin numbers don't really many anything...

  piBoardId (&model, &rev, &mem, &maker, &overVolted) ;
  if (model == PI_MODEL_CM)
    wiringPiMode = WPI_MODE_GPIO ;
  else
    wiringPiMode = WPI_MODE_PINS ;

  return 0 ;
}


/*
 * wiringPiSetupGpio:
 *	Must be called once at the start of your program execution.
 *
 * GPIO setup: Initialises the system into GPIO Pin mode and uses the
 *	memory mapped hardware directly.
 *********************************************************************************
 */

int wiringPiSetupGpio (void)
{
  (void)wiringPiSetup () ;

  if (wiringPiDebug)
    printf ("wiringPi: wiringPiSetupGpio called\n") ;

  wiringPiMode = WPI_MODE_GPIO ;

  return 0 ;
}


/*
 * wiringPiSetupPhys:
 *	Must be called once at the start of your program execution.
 *
 * Phys setup: Initialises the system into Physical Pin mode and uses the
 *	memory mapped hardware directly.
 *********************************************************************************
 */

int wiringPiSetupPhys (void)
{
  (void)wiringPiSetup () ;

  if (wiringPiDebug)
    printf ("wiringPi: wiringPiSetupPhys called\n") ;

  wiringPiMode = WPI_MODE_PHYS ;

  return 0 ;
}


/*
 * wiringPiSetupSys:
 *	Must be called once at the start of your program execution.
 *
 * Initialisation (again), however this time we are using the /sys/class/gpio
 *	interface to the GPIO systems - slightly slower, but always usable as
 *	a non-root user, assuming the devices are already exported and setup correctly.
 */

int wiringPiSetupSys (void)
{
  int boardRev ;
  int pin ;
  char fName [128] ;

  if (getenv (ENV_DEBUG) != NULL)
    wiringPiDebug = TRUE ;

  if (getenv (ENV_CODES) != NULL)
    wiringPiReturnCodes = TRUE ;

  if (wiringPiDebug)
    printf ("wiringPi: wiringPiSetupSys called\n") ;

  boardRev = piBoardRev () ;
  if (BPRVER == boardRev)   /*modify for BananaPro by LeMaker team*/
  {
		pinToGpio =  pinToGpioR3 ;
		physToGpio = physToGpioR3 ;
		physToPin = physToPinR3;
  }
  else
  {
	  if (boardRev == 1)
	  {
	     pinToGpio =  pinToGpioR1 ;
	    physToGpio = physToGpioR1 ;
	  }
	  else
	  {
	     pinToGpio =  pinToGpioR2 ;
	    physToGpio = physToGpioR2 ;
	  }
  }
// Open and scan the directory, looking for exported GPIOs, and pre-open
//	the 'value' interface to speed things up for later
    if(BPRVER == boardRev)    /*modify for BananaPro by LeMaker team*/
	{
	  for (pin = 1 ; pin < 32 ; ++pin)
	  {
		sprintf (fName, "/sys/class/gpio/gpio%d/value", pin) ;
		sysFds [pin] = open (fName, O_RDWR) ;
	  }
	}
  else
  {
	  for (pin = 0 ; pin < 64 ; ++pin)
	  {
	    sprintf (fName, "/sys/class/gpio/gpio%d/value", pin) ;
	    sysFds [pin] = open (fName, O_RDWR) ;
	  }
  }
  initialiseEpoch () ;

  wiringPiMode = WPI_MODE_GPIO_SYS ;

  return 0 ;
}
