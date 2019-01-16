/*
 * blink.c:
 *	Standard "blink" program in wiringPi. Blinks an LED connected
 *	to the first GPIO pin.
 *
 * Copyright (c) 2012-2013 Gordon Henderson. <projects@drogon.net>
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <stdio.h>
#include <wiringPi.h>

// LED Pin - wiringPi pin 0 is BCM_GPIO 17.

#define	LED	0

int leds[] = {16, 15,13, 9};

//16 15 13 9
int main (void)
{

  int i; 
  printf ("Raspberry Pi blink\n") ;

  wiringPiSetup () ;
  pinMode (16, OUTPUT) ;
  pinMode (15, OUTPUT) ;
  pinMode (13, OUTPUT) ;
  pinMode (9, OUTPUT) ;

  for (;;)
  {
    for(i=0; i<4; i++) {
    digitalWrite (leds[i], HIGH) ;	// On
    delay (20) ;		// mS
    digitalWrite (leds[i], LOW) ;	// Off
    delay (20) ;
    }
   delay(800);
  }
  return 0 ;
}
