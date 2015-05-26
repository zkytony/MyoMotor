#ifndef _HAND_TIME_H
#define _HAND_TIME_H

#define ABSOLUTE_TIME 456126
#define RELATIVE_TIME 456127

#ifdef _WIN32
	#include <time.h>
	#include <Windows.h>
	#include <stdio.h>
	#include <conio.h>
#elif __linux__
	#include "unistd.h"
#endif



#ifdef __cplusplus
extern "C"
{
#endif

	#define _LOCK2aCORE_

	void Time_ms( double *time_in_sec, int time_type);
	void Time_us(double *time_in_sec, int time_type);
	void Delay_ms(double delay_in_sec);
	void Delay_us(double delay_in_sec);
	void TimeReset();
	void TimeInIt();
	void TimeClose();

#ifdef __cplusplus
}
#endif

#endif
