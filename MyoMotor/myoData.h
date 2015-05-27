
#ifndef _MYODATA_H_
#define _MYODATA_H_

#include <myo/myo.hpp>
#include <mutex>
#include "dynamixel.h"
#include "config.h"

#define _USEGRAPHICS			true

// Control table address
#define P_GOAL_POSITION_L		30
#define P_GOAL_POSITION_H		31
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37
#define P_MOVING				46
#define TORQUE_ENABLE			24
#define MOVING_SPEED_L			32
#define MOVING_SPEED_H			33
#define PRESENT_SPEED_L			38
#define PRESENT_SPEED_H			39
#define CW_LIMIT_L				6
#define CW_LIMIT_H				7		
#define CCW_LIMIT_L				8
#define CCW_LIMIT_H				9
#define PRESENT_LOAD_L			40
#define PRESENT_LOAD_H			41
#define PRESENT_VOLT			42
#define PRESENT_TEMP			43
#define RETURN_DELAY			5
#define LED						25

// Defulat setting
#define DEFAULT_PORTNUM		4	// COM3
#define DEFAULT_BAUDNUM		1   // 1Mbps
#define CLAW_MOTOR_ID		1
#define BASE_MOTOR_ID		2

extern double us_0;
extern double us_1;
extern double goal_positions[2][conf::REFRESH_LIMIT];   // 0 for claw, 1 for base
extern int gp_count[2];

typedef struct _myoData
{
	//myoData(){}

	// status flags
	myo::Myo* myo = NULL;
	bool isConnected = false;
	bool isUnlocked = false;
	bool onArm = false;
    myo::Arm whichArm = myo::Arm::armUnknown;

	// Pose data
	uint64_t pose_timestamp = 0;
	myo::Pose currentPose = myo::Pose::unknown;

	// orientation data
	uint64_t orientation_timestamp = 0;
	myo::Quaternion<double> quat = {0.0f, 0.0f, 0.0f, 1.0f};
	double eulerAngle[3]; //= {0.0, 0.0, 0.0};

	// Acceleration data (g)
	uint64_t accel_timestamp = 0;
	myo::Vector3<float> accel {0.0f, 0.0f, 0.0f};

	// Gyro data (deg/s)
	uint64_t gyro_timestamp = 0;
	myo::Vector3<float> gyro {0.0f, 0.0f, 0.0f};

	// EMG data
	uint64_t emg_timestamp = 0;
	double emgSamples[8];

	//MyoData():eulerAngle{0.0, 0.0, 0.0}{}
}myoData;


// Data from all known Myos
extern std::vector<myoData> Myos;

// syncrhonization
extern std::mutex myoData_mutex;	// mutes on myoData
extern std::ofstream dataFile;     // the file that emg data will be stored in
extern double baseVelocity;        // the velocity of the base motor moving
extern double deltaTime;              // how much time has passed since the last engage (in second)
extern int goalPosition_claw;      // the goal position for the claw motor
extern int goalPosition_base;      // the goal position for the base motor
extern bool isMyoData;				// is myoData updating 
extern bool engaged;                // is the base motor engaged for rotation?
extern bool opening;                // is the claw opening?
extern bool baseCCW;               // is the base motor rotating counter clockwise?
extern bool writeLED;              // should the led on the base motor lights up?

// Maps a myo::Myo* to a unique ID starting at 0. It does so by looking for the PAIRED Myo pointer in Myos vector
int identifyMyo(myo::Myo* myo);

// vibrates myo
void vibrateMyo(myo::Myo* myo, myo::Myo::VibrationType duration);

// intitializes, listens to Myos and updates the myoData
void initMyo();

// stop listening to Myos and close connection
void closeMyo();

// given a Myo Pose, return a corresponding integer id
int matchPoseId(myo::Pose pose);

#endif