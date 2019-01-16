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
#include <sys/time.h>

#define	TRIG   15	
#define	ECHO   16	

float disT(void) 
{
  struct timeval tv1;
  struct timeval tv2;

  long start, stop;
  float dis;
  
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  while(!(digitalRead(ECHO) == 1)) {
    gettimeofday(&tv1, NULL);
  }

  while(!(digitalRead(ECHO) == 0)) {
    if(tv2.tv_sec - tv1.tv_sec > 10) break;
    gettimeofday(&tv2, NULL);
  }


  start = tv1.tv_sec * 1000000  + tv1.tv_usec;
  stop = tv2.tv_sec * 1000000  + tv2.tv_usec;

  dis = (float) (stop - start) / 1000000 * 34000 / 2;

  return dis;
}

int main (void)
{
  float dis;

  wiringPiSetup () ;

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  while(1) 
  {
    dis = disT();
    printf("dis: %f \n", dis);
    delay(1200);
  }

  return 0 ;
}
