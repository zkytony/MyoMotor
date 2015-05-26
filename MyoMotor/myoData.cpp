// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.

// This sample illustrates how to interface with multiple Myo armbands and distinguish between them.

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <thread>
#include <fstream>

#include <myo/myo.hpp>
#include "myoData.h"
#include "Htime.h"

std::vector<myoData> Myos;
std::mutex myoData_mutex;

std::thread *MyoListenerThread;
std::ofstream dataFile;

myo::Pose currentPose;
double baseVelocity = 0;      // the velocity of the base motor moving
double deltaTime = 0;            // how much time has passed since the last engage (in second)
double moderator = 0.25;
int goalPosition_claw = 0;    // the goal position for the claw motor
int goalPosition_base = 0;    // the goal position for the base motor
int roll_0 = 0;                //
bool roll_0_is_set = false;    // 
bool isMyoData = true;         // is myoData updating 
bool engaged = false;          // is the base motor engaged for rotation?
bool opening = false;          // is the claw opening?
bool writeLED = true;         // should the led on the base motor lights up?
bool baseCCW = false;         // is the base motor rotating counter clockwise?

class MyoEventsManager : public myo::DeviceListener {
public:
	MyoEventsManager(){}

    // Called once when Myo Connect successfully pairs with a Myo armband
    void onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
    {
        // Print out the MAC address of the armband we paired with.

        // The pointer address we get for a Myo is unique - in other words, it's safe to compare two Myo pointers to
        // see if they're referring to the same Myo.

        // Add the Myo pointer to our list of known Myo devices. This list is used to implement identifyMyo() below so
        // that we can give each Myo a nice short identifier.
        //knownMyos.push_back(myo);
		
		myoData md;
		md.myo = myo;
		myoData_mutex.lock();
		{	Myos.push_back(md);
		}myoData_mutex.unlock();

		// Next we enable EMG streaming on the found Myo.
		myo->setStreamEmg(myo::Myo::streamEmgEnabled);

        // Now that we've added it to our list, get our short ID for it and print it out.
		int i_myo = identifyMyo(myo);
		if(i_myo>=0)
		{	std::cout << std::endl << "Paired with " << i_myo << "." ;
		}
    }

	// onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
    void onUnpair(myo::Myo* myo, uint64_t timestamp)
    {
        // We've lost a Myo. Let's clean up some leftover state.
		int i_myo = identifyMyo(myo);
		if(i_myo>=0)
		{	Myos[i_myo].eulerAngle[0] = 0;
			Myos[i_myo].eulerAngle[1] = 0;
			Myos[i_myo].eulerAngle[2] = 0;
			Myos[i_myo].quat = myo::Quaternion<double>(0,0,0,1);
			Myos[i_myo].onArm = false;
			Myos[i_myo].isUnlocked = false;

			for (int i = 0; i < 8; i++) 
				Myos[i_myo].emgSamples[i] = 0.0;
		}
    }
	
	// Called when a paired Myo has been connected.
    void onConnect(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
    {
		int i_myo = identifyMyo(myo);
		if(i_myo>=0)
		{	myoData_mutex.lock();
			{	Myos[i_myo].isConnected = true;
			}myoData_mutex.unlock();
		}
        std::cout << std::endl << "Myo " << identifyMyo(myo) << " has connected and unlocked.";
    }

	// Called when a paired Myo has been disconnected.
    void onDisconnect(myo::Myo* myo, uint64_t timestamp)
    {
		int i_myo = identifyMyo(myo);
		if(i_myo>=0)
			Myos[i_myo].isConnected = false;
        std::cout << std::endl << "Myo " << identifyMyo(myo) << " has disconnected.";
		myo->lock();
		myo->vibrate(myo::Myo::vibrationLong);
    }

	// Called when a paired Myo recognizes that it is on an arm.
	void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection)
    {
		// Update data buffer
		int i_myo = identifyMyo(myo);
		if(i_myo>=0)
		{
			myoData_mutex.lock();
			{	Myos[i_myo].onArm = true;
				Myos[i_myo].whichArm = arm;
			}myoData_mutex.unlock();
			myo->unlock(myo::Myo::unlockHold);
		}

		dataFile.open("Myo_EMG_data.csv");
		dataFile << "time,id,base_speed,base_goal_position,claw_load,roll_scale,engaged\n";
    }

    // Called when Myo moved from a stable position on a person's arm after it recognized the arm.
    void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
    {
		// Update data buffer
		int i_myo = identifyMyo(myo);
		if(i_myo>=0)
		{
			myoData_mutex.lock();
			{	Myos[i_myo].onArm = false;
			}myoData_mutex.unlock();
			myo->lock();
		}

		dataFile.close();
    }

    // onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
    void onUnlock(myo::Myo* myo, uint64_t timestamp)
    {
		// Update data buffer
		int i_myo = identifyMyo(myo);
		if(i_myo>=0)
		{
			myoData_mutex.lock();
			{	Myos[i_myo].isUnlocked = true;
			}myoData_mutex.unlock();
		}
    }

    // onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
    void onLock(myo::Myo* myo, uint64_t timestamp)
    {
		// Update data buffer
		int i_myo = identifyMyo(myo);
		if(i_myo>=0)
		{
			myoData_mutex.lock();
			{	Myos[i_myo].isUnlocked = false;
			}myoData_mutex.unlock();
		}
    }

	// Called when a paired Myo has provided a new pose.
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
    {	
		currentPose = pose;
		int i_myo = identifyMyo(myo);
		if(i_myo>=0)
		{	Myos[i_myo].currentPose = pose;
			Myos[i_myo].myo->notifyUserAction();
		}

		if (pose == myo::Pose::fist) {
			// engage the motor
			engaged = true;
		}
		else if (pose == myo::Pose::waveOut) {
			// open the claw
			goalPosition_claw = conf::MAX_POS_CLAW;
			opening = true;
			dxl_write_word(CLAW_MOTOR_ID, P_GOAL_POSITION_L, goalPosition_claw);
			engaged = false;
		}
		else if (pose == myo::Pose::waveIn) {
			// close the claw
			goalPosition_claw = 0;
			opening = false;
			dxl_write_word(CLAW_MOTOR_ID, P_GOAL_POSITION_L, goalPosition_claw);
			engaged = false;
		}
		else {
			engaged = false;
		}

		//std::cout << (engaged ? "engaged" : "not") << std::endl;
    }

	// Called when a paired Myo has provided new orientation data.
	void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
    {
        using std::atan2;
        using std::asin;
        using std::sqrt;
        using std::max;
        using std::min;

        // Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
        double roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                           1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
        double pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
        double yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                        1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));

		// Update data buffer
		int i_myo = identifyMyo(myo);
		if(i_myo>=0)
		{
			myoData_mutex.lock();
			{	Myos[i_myo].eulerAngle[0] = roll;
				Myos[i_myo].eulerAngle[1] = pitch;
				Myos[i_myo].eulerAngle[2] = yaw;
				Myos[i_myo].quat = myo::Quaternion<double>((double)quat.x(),(double)quat.y(),(double)quat.z(),(double)quat.w()); ;
			}myoData_mutex.unlock();
		}

		int roll_scale = static_cast<int>((roll + (float)M_PI) / (M_PI * 2.0f) * conf::SENSITIVITY);

		if (engaged) {
			if (!roll_0_is_set) {
				roll_0 = roll_scale;
				roll_0_is_set = true;
			}

			if (writeLED) {
				dxl_write_word(BASE_MOTOR_ID, LED, 1);
				writeLED = false;
			}
			
			// Rotating clockwise
			if (130 < roll_scale && roll_scale <= 250) {
				if (!baseCCW) {
					deltaTime = 0;  // you are changing direction - so start the timing again
					baseCCW = true;
				}
				baseVelocity = (roll_scale - roll_0)*(roll_scale - roll_0)*moderator;
			}
			// rotating counter clockwise
			else if (0 <= roll_scale && roll_scale < 120) {
				if (baseCCW) {
					deltaTime = 0;  // you are changing direction - so start the timing again
					baseCCW = false;
				}
				baseVelocity = -(roll_0 - roll_scale)*(roll_0 - roll_scale)*moderator;
			}
		}
		else {
			baseVelocity = 0.0;
			roll_0_is_set = false;
			if (!writeLED) {
				dxl_write_word(BASE_MOTOR_ID, LED, 0);
				writeLED = true;
			}
			deltaTime = 0;
		}
    }

	// Called when a paired Myo has provided new accelerometer data in units of g.
	void onAccelerometerData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& accel) 
	{
		int i_myo = identifyMyo(myo);
		if(i_myo>=0)
		{
			myoData_mutex.lock();
			{	Myos[i_myo].accel = myo::Vector3<float>(accel[0], accel[1], accel[2]);
			}myoData_mutex.unlock();
		}
	}

	// Called when a paired Myo has provided new gyroscope data in units of deg/s.
	void onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& gyro) 
	{
		int i_myo = identifyMyo(myo);
		if(i_myo>=0)
		{
			myoData_mutex.lock();
			{	Myos[i_myo].gyro = myo::Vector3<float>(gyro[0], gyro[1], gyro[2]);
			}myoData_mutex.unlock();
		}
	}

	// onEmgData() is called whenever a paired Myo has provided new EMG data, and EMG streaming is enabled.
    void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
    {
		int i_myo = identifyMyo(myo);
		if(i_myo>=0)
		{
			myoData_mutex.lock();
			{	for (int i = 0; i < 8; i++)
					Myos[i_myo].emgSamples[i] = (double)emg[i];
			}myoData_mutex.unlock();
		}
    }

	void update() {
		deltaTime += conf::DURATION / 1000.0;      // convert to seconds
		goalPosition_base += (int)(baseVelocity * deltaTime);
		if (goalPosition_base > conf::MAX_POS_BASE) { goalPosition_base = conf::MAX_POS_BASE; }
		if (goalPosition_base < 0) { goalPosition_base = 0; }
		dxl_write_word(BASE_MOTOR_ID, P_GOAL_POSITION_L, goalPosition_base);
	}

	void log() {
		// time,id,base_speed,base_goal_position,claw_load,roll_scale,engaged
		double t;
		Time_ms(&t, RELATIVE_TIME);
		int id = matchPoseId(currentPose);
		int base_speed = dxl_read_word(BASE_MOTOR_ID, PRESENT_SPEED_L);
		int claw_load = dxl_read_word(CLAW_MOTOR_ID, PRESENT_LOAD_L);

		dataFile << t << "," << id << "," << base_speed << "," << goalPosition_base << "," << claw_load << "," << "XX" << "," << (engaged ? "1" : "0") << "\n";
	}
};


// listens to Myos and updates the myoData
void *ListenerMyo()
{
	try 
	{
        myo::Hub hub("com.example.multiple-myosData");

        // Instantiate the PrintMyoEvents class we defined above, and attach it as a listener to our Hub.
        MyoEventsManager manager;
        hub.addListener(&manager);

        while (isMyoData) 
		{	// Process events for 10 milliseconds at a time.
            hub.run(conf::DURATION);
			manager.update();
			manager.log();
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();
    }
	return NULL;
}


// Maps a myo::Myo* to a unique ID starting at 0. It does so by looking for the PAIRED Myo pointer in Myos vector
int identifyMyo(myo::Myo* myo) 
{
    // Walk through the list of Myo devices that we've seen pairing events for.
    for (size_t i = 0; i < Myos.size(); ++i) {
        // If two Myo pointers compare equal, they refer to the same Myo device.
        if (Myos[i].myo == myo) {
            return i;
        }
    }
    return -1;
}


// vibrates myo
void vibrateMyo(myo::Myo* myo, myo::Myo::VibrationType duration)
{
	myo->vibrate(duration);
}


// intitializes myo and launches listener thread
void initMyo()
{
	isMyoData = true;
	engaged = false;
	opening = false;
	goalPosition_claw = 0;
	goalPosition_base = 0;
	MyoListenerThread = new std::thread(ListenerMyo);
	printf("\nListening to Myo events");
}


// stop listening to Myos and close connection
void closeMyo()
{
	for(unsigned int i=0; i<Myos.size(); i++)
	{ 	
		vibrateMyo(Myos[i].myo, myo::Myo::vibrationMedium);
		Myos[i].myo->lock();
	}
	isMyoData = false;
	engaged = false;
	opening = false;
	goalPosition_claw = 0;
	goalPosition_base = 0;
	printf("\nWaiting for the Myo listener thread to join");
	MyoListenerThread->join();
	printf("\nStopped listening to Myo events");
}

// given a Myo Pose, return a corresponding integer id
int matchPoseId(myo::Pose pose) {
	if (pose == myo::Pose::unknown) {
		return -1;
	}
	else if (pose == myo::Pose::rest) {
		return 0;
	}
	else if (pose == myo::Pose::doubleTap) {
		return 1;
	}
	else if (pose == myo::Pose::fingersSpread) {
		return 2;
	}
	else if (pose == myo::Pose::waveIn) {
		return 3;
	}
	else if (pose == myo::Pose::waveOut) {
		return 4;
	}
	else if (pose == myo::Pose::fist) {
		return 5;
	}
	else {
		return -2; // equivolent to ?
	}
}