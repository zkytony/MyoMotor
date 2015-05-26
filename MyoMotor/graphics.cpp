#include "matplotPP.h"
#include "myoData.h"
#include "myo/cxx/Vector3.hpp"

int nMyos = 1; // #Myos expecting to connect
myoData localMyo;

const int trace_n = 500;
extern double base_velocity;

typedef struct _myoDataTrace
{	// orientation trace
	double orientation_timestamp[trace_n];
	double eulerAngle[3][trace_n];
	double quat[4][trace_n];

	// Acceleration trace
	double accel_timestamp[trace_n];
	double accel[3][trace_n];

	// Gyro trace
	double gyro_timestamp[trace_n];
	double gyro[3][trace_n];

	// EMG trace
	double emg_timestamp[trace_n];
	double emgSamples[8][trace_n];
}myoDataTrace;

std::vector<myoDataTrace> trace(nMyos);
int trace_i = 0;

extern double us_0;
extern double us_1;
extern double goal_positions[2][conf::REFRESH_LIMIT];   // 0 for claw, 1 for base
extern int gp_count[2];
dvec gg(1);

void Plot::DISPLAY()
{
	
	char name[50];
	int iMyos=0;
	static bool firstFlag = true;
	if(firstFlag)
	{	while(iMyos!=nMyos)
		{	iMyos = 0;
			for(unsigned int i=0; i<Myos.size(); i++)
				iMyos = iMyos + (int)Myos[i].isConnected;
			printf("\nMYOS: %d paired, %d connected, %d waiting", Myos.size(), iMyos, nMyos-iMyos);
			Sleep(1000);
		}
		firstFlag = false;
	}

	// Layers
	for(iMyos=0; iMyos< nMyos; iMyos++)
	{
		// copy data locally
		myoData_mutex.lock();
		{	localMyo = Myos[iMyos];
		}myoData_mutex.unlock();
		
		// update trace
		for(int i=0; i<3; i++)
		{	trace[iMyos].eulerAngle[i][trace_i] = localMyo.eulerAngle[i];
			trace[iMyos].accel[i][trace_i] = localMyo.accel[i];
			trace[iMyos].gyro[i][trace_i] = localMyo.gyro[i];
		}
		trace[iMyos].quat[0][trace_i] = localMyo.quat.x();
		trace[iMyos].quat[1][trace_i] = localMyo.quat.y();
		trace[iMyos].quat[2][trace_i] = localMyo.quat.z();
		trace[iMyos].quat[3][trace_i] = localMyo.quat.w();
		
		for(int i=0; i<8; i++)
			trace[iMyos].emgSamples[i][trace_i]= localMyo.emgSamples[i];

		trace[iMyos].orientation_timestamp[trace_i]= (double)localMyo.orientation_timestamp;
		trace[iMyos].accel_timestamp[trace_i]= (double)localMyo.accel_timestamp;
		trace[iMyos].gyro_timestamp[trace_i]= (double)localMyo.gyro_timestamp;
		trace[iMyos].emg_timestamp[trace_i]= (double)localMyo.emg_timestamp;

		// plots 
		if(localMyo.isConnected)
		{	sprintf_s(name,"Myo-%d",iMyos);
			layer(name, iMyos==0);

			// status flags =================================
			subplot(3,4,1,""); axis(0); set("BackgroundColor","none");
			axis(5.5, 0.0, 0.0, 5.5);
			// connection status
			if(localMyo.isConnected)
				text(1, 5, "Connected", false, "k", "lg", "k");
			else
				text(1, 5, "Disconnected", false, "k", "lr", "k");
			// lock status
			if(localMyo.isUnlocked)
				text(1, 4, "Unlocked", false, "k", "lg", "k");
			else
				text(1, 4, "Locked", false, "k", "lr", "k");
			// which arm
			if(localMyo.onArm)
			{	if(localMyo.whichArm==libmyo_arm_left)
					text(1, 3, "Left arm", false, "k", "lc", "k");
				else if(localMyo.whichArm==libmyo_arm_right)
					text(1, 3, "Right arm", false, "k", "lm", "k");
			}
			else
				text(1, 3, "Off arm", false, "k", "lr", "k");
			// Pose
			if(localMyo.currentPose == myo::Pose::unknown)
				text(1, 2, "Pose: unknown", false, "k", "lr", "k");
			else
				text(1, 2, localMyo.currentPose.toString(), false, "k", "lg", "k");


			// Orientation =================================
			localMyo.quat.normalized();
			myo::Vector3<double> vec3 = myo::rotate(localMyo.quat, myo::Vector3<double>{1.0f, 0.0f, 0.0f});

			subplot(3,4,2,"orientation");
			plot3({0, vec3.x()}, {0, vec3.y()}, {0, vec3.z()}); set(5);
			set("x"); set(5);
			set("MarkerSize",8);
			set("COLOR", "k");
			axis(-1.5, 1.5, -1.5, 1.5, -1.5, 1.5);

			/*subplot(3,2,3); title("Euler Angles");
			plot(trace[iMyos].eulerAngle[0], trace_n, 2, "r");
			plot(trace[iMyos].eulerAngle[1], trace_n, 2, "g");
			plot(trace[iMyos].eulerAngle[2], trace_n, 2, "b");
			axis(-1.0, trace_n+1.0, -5.0, 5.0);
			subplot(3,2,3); title("Quat");
			plot(trace[iMyos].quat[0], trace_n, 2, "r");
			plot(trace[iMyos].quat[1], trace_n, 2, "g");
			plot(trace[iMyos].quat[2], trace_n, 2, "b");
			plot(trace[iMyos].quat[3], trace_n, 2, "c");
			axis(-1.0, trace_n+1.0, -5.0, 5.0);*/

			// Acceleration ==============================
			subplot(3,2,3); title("Accelerometer");
			plot(trace[iMyos].accel[0], trace_n, 2, "r");
			plot(trace[iMyos].accel[1], trace_n, 2, "g");
			plot(trace[iMyos].accel[2], trace_n, 2, "b");
			axis(-1.0, trace_n+1.0, -5.0, 5.0);

			// gyro ======================================
			subplot(3,2,5); title("Gyroscope");
			plot(trace[iMyos].gyro[0], trace_n, 2, "r");
			plot(trace[iMyos].gyro[1], trace_n, 2, "g");
			plot(trace[iMyos].gyro[2], trace_n, 2, "b");
			axis(-1.0, trace_n+1.0, -1000.0, 1000.0);

			// EMG ========================================
			for(int i=0; i<8; i++)
			{	subplot(8,2,2*(i+1),""); axis("x");
				sprintf_s(name,"EMG%d",i);	ylabel(name);
				plot(trace[iMyos].emgSamples[i], trace_n); 
				axis(-1.0, trace_n+1.0, -160.0, 160.0);
			}
		}
	}

	layer("Goal Position", false);
	int hy;
	hy = subplot(3, 1, 1);
	axis(0, conf::REFRESH_LIMIT, 0, conf::Y_MAX);
	title("Goal Position for BASE motor");
	ylabel("us");

	xlabel("time - " + std::to_string(conf::REFRESH_LIMIT * conf::DURATION) + " ms");
	plot(goal_positions[1], conf::REFRESH_LIMIT);

	hy = subplot(3, 1, 2);
	title("velocity");
	gg.at(0) = base_velocity;
	bar(gg);

	// increment trace counter 
	trace_i = (trace_i==trace_n-1)? 0: trace_i+1;

}