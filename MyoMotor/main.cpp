#define _USEGRAPHICS true

#include "myoData.h"
#include "matplotPP.h"
#include <conio.h>
// For dxl
#include "dynamixel.h"

double us_0 = 0.0;
double us_1 = 0.0;
int GoalPos[2] = { 0, 1023 };
int CommStatus;
double goal_positions[2][conf::REFRESH_LIMIT]; // 0 for claw, 1 for base
int gp_count[2];

int main(int argc, char** argv)
{
	// Open device
	if (dxl_initialize(DEFAULT_PORTNUM, DEFAULT_BAUDNUM) == 0)
	{
		printf("Failed to open USB2Dynamixel!\n");
		printf("Press any key to terminate...\n");
		_getch();
		//return 0;
	}

	bool graphics_exit = false;

	initMyo();

	if(_USEGRAPHICS)
		Graphics_init(argc, argv, 200, 10, 1200, 800, &graphics_exit, "MYO Driver"); //Set up graphics

	// Do your work here 
	while (!graphics_exit && !_kbhit()) 
	{
		Sleep(1000);
	}

	closeMyo();
    
	if(_USEGRAPHICS)
		Graphics_Close();
}
