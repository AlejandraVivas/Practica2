/*
 * GlobalFunctions.c
 *
 *  Created on: Jun 22, 2014
 *      Author: Luis
 */

#include "GlobalFunctions.h"

void delay(uint32 delay)
{
	volatile int counter, counter2;
	
	for(counter2=32; counter2 > 0; counter2--)
	{
		for(counter=delay; counter > 0; counter--); 

	}
}
