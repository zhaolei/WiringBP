/************************************************************
  Copyright (C), 2014, lemaker.org.
  FileName:      testLedCon6.c
  Author:         peter
  Version :      1.0
  Date:          2014.10.09 
  Description: be used to debug wiringPI Or GPIO
  Function List: 
  History:
      <author>    <time>     <version >   <desc>
          peter     2014.10.09	  1.0		  create
          
  Reference:
  GPIO0-GPIO27 definitio in the script.fex 
		gpio_pin_0 = port:PI01<1><default><default><default>
		gpio_pin_1 = port:PI00<1><default><default><default>
		gpio_pin_2 = port:PB21<1><default><default><default>
		gpio_pin_3 = port:PB20<1><default><default><default>
		gpio_pin_4 = port:PH02<1><default><default><default>
		gpio_pin_5 = port:PB03<1><default><default><default>
		gpio_pin_6 = port:PI21<1><default><default><default>
		gpio_pin_7 = port:PI14<1><default><default><default>
		gpio_pin_8 = port:PI10<1><default><default><default>
		gpio_pin_9 = port:PI13<1><default><default><default>
		gpio_pin_10 = port:PI12<1><default><default><default>
		gpio_pin_11 = port:PI11<1><default><default><default>
		gpio_pin_12 = port:PI20<1><default><default><default>
		gpio_pin_13 = port:PB13<1><default><default><default>
		gpio_pin_14 = port:PH00<1><default><default><default>
		gpio_pin_15 = port:PH01<1><default><default><default>
		gpio_pin_16 = port:PB06<1><default><default><default>
		gpio_pin_17 = port:PI19<1><default><default><default>
		gpio_pin_18 = port:PI03<1><default><default><default>
		gpio_pin_19 = port:PB07<1><default><default><default>
		gpio_pin_20 = port:PB12<1><default><default><default>
		gpio_pin_21 = port:PB08<1><default><default><default>
		gpio_pin_22 = port:PI17<1><default><default><default>
		gpio_pin_23 = port:PH20<1><default><default><default>
		gpio_pin_24 = port:PH21<1><default><default><default>
		gpio_pin_25 = port:PI16<1><default><default><default>
		gpio_pin_26 = port:PB05<1><default><default><default>
		gpio_pin_27 = port:PI18<1><default><default><default>
***********************************************************/
#include <stdio.h>
#include <wiringPi.h>
#include <stdlib.h>

#define PHY_PIN_MAX 40  /*1,2,3,4 .... 40*/
#define SYS_EXPORT_PIN 27 /*1,2,3,4 ...27*/

#define FAIL -1

#define BCM 1
#define WPI 2
#define PHY 3
#define SYS 4

static unsigned int ARRAY_INT[14] = {7, 8, 10, 11, 13, 15, 16, 18, 19, 21, 22, 23, 24, 26}; //phy pin numbers

void uSage(void)
{
		printf("Usage: testLedCon6 int|bcm|wpi|phy|sys\n");
}

/*
Description: interrupt handles function
 */
void myInterrupt1 (void) { printf("The key1 is detected, and press next key\n"); }
void myInterrupt2 (void) { printf("The key2 is detected, and press next key\n"); }
void myInterrupt3 (void) { printf("The key3 is detected, and press next key\n"); }
void myInterrupt4 (void) { printf("The key4 is detected, and press next key\n"); }
void myInterrupt5 (void) { printf("The key5 is detected, and press next key\n"); }
void myInterrupt6 (void) { printf("The key6 is detected, and press next key\n"); }
void myInterrupt7 (void) { printf("The key7 is detected, and press next key\n"); }
void myInterrupt8 (void) { printf("The key8 is detected, and press next key\n"); }
void myInterrupt9 (void) { printf("The key9 is detected, and press next key\n"); }
void myInterrupt10 (void) { printf("The key10 is detected, and press next key\n"); }
void myInterrupt11 (void) { printf("The key11 is detected, and press next key\n"); }
void myInterrupt12 (void) { printf("The key12 is detected, and press next key\n"); }
void myInterrupt13 (void) { printf("The key13 is detected, and press next key\n"); }
void myInterrupt14 (void) { printf("The key14 is detected, key test over\n"); }

/*
Function: bprTbIntInit
Description: install the interrupt handles function, and only support 14 EINT.
*/
int bprTbIntInit(void) 
{
	  int pinInt;
	  wiringPiSetupPhys();
	 
	  for(pinInt=0; pinInt<14; pinInt++)  //enable the pull up
	  {
	  		pullUpDnControl(ARRAY_INT[pinInt], PUD_UP);
	  }
  
	  wiringPiISR (7, INT_EDGE_FALLING, &myInterrupt1) ;
	  wiringPiISR (8, INT_EDGE_FALLING, &myInterrupt2) ;
	  wiringPiISR (10, INT_EDGE_FALLING, &myInterrupt3) ;
	  wiringPiISR (11, INT_EDGE_FALLING, &myInterrupt4) ;
	  wiringPiISR (13, INT_EDGE_FALLING, &myInterrupt5) ;
	  wiringPiISR (15, INT_EDGE_FALLING, &myInterrupt6) ;
	  wiringPiISR (16, INT_EDGE_FALLING, &myInterrupt7) ;
	  wiringPiISR (18, INT_EDGE_FALLING, &myInterrupt8) ;
	  wiringPiISR (19, INT_EDGE_FALLING, &myInterrupt9) ;
	  wiringPiISR (21, INT_EDGE_FALLING, &myInterrupt10) ;
	  wiringPiISR (22, INT_EDGE_FALLING, &myInterrupt11) ;
	  wiringPiISR (23, INT_EDGE_FALLING, &myInterrupt12) ;
	  wiringPiISR (24, INT_EDGE_FALLING, &myInterrupt13) ;
	  wiringPiISR (26, INT_EDGE_FALLING, &myInterrupt14) ;


}
/*
Function: bprTbExportAllPin
Description: export bcm gpio1-gpio27
*/
void bprTbExportAllPin(void)
{
	 int sysPin;
	 char cmdstr[80] = {'\0'};
	 
	 for(sysPin=1; sysPin <= SYS_EXPORT_PIN; sysPin++)  /*echo 0 > export ->invalid argument*/
	 {
	 	 snprintf(cmdstr, 80, "/usr/local/bin/gpio export %d out", sysPin);
		 system(cmdstr);
	 }
}
/*
Funciton: bprTbAllLedBlink
Description: all leds blink at the same time, and support four operating modes
*/
void bprTbAllLedBlink(int val, int opt)
{
	int pinNum;

	for(pinNum=1; pinNum<=PHY_PIN_MAX; pinNum++)
	{       
		switch(opt)
		{
			case SYS:
			case BCM:
				pinMode(physPinToGpio(pinNum), OUTPUT);
				digitalWrite(physPinToGpio(pinNum), val);
				break;
			case WPI:
				pinMode(physPinToPin(pinNum), OUTPUT);
				digitalWrite(physPinToPin(pinNum), val);
				break;
			case PHY:
				pinMode(pinNum, OUTPUT);
				digitalWrite(pinNum, val);
				break;
			default:
				printf("the BPR test Board 01 init failed!\n");
				break;
		}
	}
}

/*
Function: bprTbEachLedBlink
Description: the led blink one by one, and support four operating modes
*/
void bprTbEachLedBlink(int dly, int opt)
{
	int pin = 1;

	for(;;)
	{
		switch(opt)
		{
			case SYS:
			case BCM:
				digitalWrite(physPinToGpio(pin), HIGH);
				delay(dly);
				digitalWrite(physPinToGpio(pin), LOW);
				delay(dly);
				break;
			case WPI:
				digitalWrite(physPinToPin(pin), HIGH);
				delay(dly);
				digitalWrite(physPinToPin(pin), LOW);
				delay(dly);
				break;
			case PHY:
				digitalWrite(pin, HIGH);
				delay(dly);
				digitalWrite(pin, LOW);
				delay(dly);
				break;
		}
		
		pin = pin +1;
		if(pin > PHY_PIN_MAX)
			pin = 1;
	}

}

int main(int argc, char **argv)
{
        if(argc  != 2)
        {
        	uSage();
		return FAIL;
        }
	
	if (0 == strcasecmp(argv[1], "int"))
	{
		bprTbIntInit();
		while(1);
	}
	else if (0 == strcasecmp(argv[1], "bcm"))
	{
		wiringPiSetupGpio();
		bprTbAllLedBlink(LOW, BCM); //disable all led
		delay(2000); //delay 2s
		bprTbAllLedBlink(HIGH, BCM); //enable all led
		delay(3000);//delay 3s
		bprTbAllLedBlink(LOW, BCM); //disable all led

		
		bprTbEachLedBlink(500, BCM); //led blink one by one
	}
	else if (0 == strcasecmp(argv[1], "wpi"))
	{
		wiringPiSetup();
		bprTbAllLedBlink(LOW, WPI); //disable all led
		delay(2000); //delay 2s
		bprTbAllLedBlink(HIGH, WPI); //enable all led
		delay(3000);//delay 3s
		bprTbAllLedBlink(LOW, WPI); //disable all led
		
		bprTbEachLedBlink(500, WPI); //led blink one by one
	}
	else if (0 == strcasecmp(argv[1], "phy"))
	{
		wiringPiSetupPhys();
		bprTbAllLedBlink(LOW, PHY); //disable all led
		delay(2000); //delay 2s
		bprTbAllLedBlink(HIGH, PHY); //enable all led
		delay(3000);//delay 3s
		bprTbAllLedBlink(LOW, PHY); //disable all led

		
		bprTbEachLedBlink(500, PHY); //led blink one by one
	}
	else if (0 == strcasecmp(argv[1], "sys")) /*can't control the phy pin27 pin28*/
	{
		bprTbExportAllPin();
		wiringPiSetupSys();
		
		bprTbAllLedBlink(LOW, SYS); //disable all led
		delay(2000); //delay 2s
		bprTbAllLedBlink(HIGH, SYS); //enable all led
		delay(3000);//delay 3s
		bprTbAllLedBlink(LOW, SYS); //disable all led

		
		bprTbEachLedBlink(500, SYS); //led blink one by one		
	}
	else
	{
		uSage();
		return FAIL;
	}
}
