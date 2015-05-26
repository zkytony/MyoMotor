/*	V1.1 
- Dealy_us now uses QPC instead of Waitable timer
- Thread locking to a CPU mechanism is now optional
*/

#include "Htime.h"

// globals
double miliSec_Start; 
double microSec_Start;
double sec_per_tick;
HANDLE hCurrentThread;
LARGE_INTEGER frequency;

//=============================================================================  
// Time with miliseconds(ms) resolution
//=============================================================================
void Time_ms( double *time_in_sec, int time_type)
{	
	clock_t t = clock();
	if(time_type == RELATIVE_TIME)
		*time_in_sec = ((double)t-miliSec_Start)/CLOCKS_PER_SEC;
	else 
		*time_in_sec = (double)t/CLOCKS_PER_SEC;
}	


//=============================================================================  
// Time with microseconds(us) resolution
//=============================================================================
void Time_us( double *time_in_sec, int time_type )
{    
	LARGE_INTEGER counter;
	double total_ticks;
	
#ifdef _LOCK2aCORE_
	DWORD_PTR  dwThreadAffinity;    
	hCurrentThread = GetCurrentThread();

	// force thread on a cpu 
	dwThreadAffinity = SetThreadAffinityMask(hCurrentThread, 4); //using 3rd core
	if(dwThreadAffinity ==0)
	{	printf("\nTime   :> Error getting Thread affinity (%d)\n", GetLastError());
		_getch(); exit(1);
	}

	// retrieves the current value of the high-resolution performance counter
	QueryPerformanceCounter(&counter);
	
	 // reset thread
	SetThreadAffinityMask(hCurrentThread, dwThreadAffinity);
#else
	// retrieves the current value of the high-resolution performance counter
	QueryPerformanceCounter(&counter);

#endif

	// time in micro seconds
	total_ticks		= (double)counter.QuadPart;  

	if(time_type == RELATIVE_TIME)
		*time_in_sec = (sec_per_tick * total_ticks)-microSec_Start;
	else 
		*time_in_sec = (sec_per_tick * total_ticks);
   
}

inline void MPsleep(int time_in_sec)
{
	#ifdef WIN32
		Sleep((DWORD)time_in_sec);
	#elif __linux__
		usleep((int)time_in_sec*1000000);
	#endif
}


//=============================================================================  
// Delay with miliseconds(ms) resolution
//=============================================================================
void Delay_ms(double delay_in_sec)
{  
	#ifdef WIN32
		Sleep((int)(1000*delay_in_sec));	//convert time in milisec
	#elif __linux__
		usleep((int)delay_in_sec*1000000);
	#endif
}


//=============================================================================  
// Delay with 100nanoseconds(100ns) resolution
//=============================================================================

void Delay_us(double delay_in_sec)
{
#ifdef WIN32
	LARGE_INTEGER initial;
	LARGE_INTEGER counter;
	double delay = 0.0;

	#ifdef _LOCK2aCORE_
		DWORD_PTR  dwThreadAffinity;
		// force thread onto processor it is currently on
		DWORD prsr		= GetCurrentProcessorNumber();
		hCurrentThread	= GetCurrentThread();
		dwThreadAffinity = SetThreadAffinityMask(hCurrentThread, 0x1<<prsr);
		if(dwThreadAffinity == 0)
		{	printf("\nTime   :> Error getting Thread affinity (%d)\n", GetLastError());
			_getch(); exit(1);
		}
	#endif

	//Hybrid waiting: lond cheap delay using  Sleep and expensive small using QPC
	QueryPerformanceCounter(&initial);
	if (delay_in_sec > 0.002) 
	{	DWORD milliseconds = (DWORD)((delay_in_sec-0.002) * 1000.0);
		Sleep(milliseconds);
	}
	
	do 
	{	QueryPerformanceCounter(&counter);
		delay	= sec_per_tick * ((double)counter.QuadPart - (double)initial.QuadPart);
	} while (delay < delay_in_sec);

	#ifdef _LOCK2aCORE_
		SetThreadAffinityMask(hCurrentThread, dwThreadAffinity);
	#endif

#elif __linux__
	usleep((int)delay_in_sec*1000000);
#endif
}

//=============================================================================  
// Resets colock to t = 0; Relatived time are measured from this moment
//=============================================================================
void TimeReset()
{
	Time_us( &microSec_Start, ABSOLUTE_TIME );
}



//=============================================================================  
// Prepare Timers and delay: Relatived time are measured from this moment
//=============================================================================
void TimeInIt()
{
	//milisec ------------------------------
	timeBeginPeriod(1);
	Time_ms(&miliSec_Start, ABSOLUTE_TIME);

	//microsec -----------------------------
	// retrieves the frequency of the high-resolution performance counter */
	QueryPerformanceFrequency(&frequency);
	sec_per_tick	= (double)1/(double)frequency.QuadPart;
	Time_us( &microSec_Start, ABSOLUTE_TIME );
}


//=============================================================================  
// Close Timers and delay 
//=============================================================================
void TimeClose()
{
	timeEndPeriod(1);
	CloseHandle(hCurrentThread);
}
