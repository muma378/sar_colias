/*
* config.h - the argments and headers used in the program
* Auhtor: Yang Xiao
* Date: 13 September 2015
*/

#ifndef __SAR_COLIAS_CONFIG__
#define __SAR_COLIAS_CONFIG__

#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <iostream>
#include <string>
#include <fstream>
#include <libplayerc++/playerc++.h>

using namespace std;
using namespace PlayerCc;

#define ROBOTS_COUNT 10 	// the amount of robots
#define F_MAX 100	// max value of fitness
#define F_MIN 0
#define UPDATE_INTERVAL 300  // millisecond
#define MAX_SPEED 0.35	// m/s
#define MIN_SPEED 0.15
#define PI 3.1415926
// the boundary of trun speed is PI/2
#define TURN_SPEED_BOUND (PI/2)
#define SMALL_RADIAN_ERROR 0.09		// 5 degree
#define COMMUNICATE_RANGE 0.5  // m, range_max robot_detector possesses
#define ACCELATOR 1  // avoid the forward speed keeping so small
#define DETECT_SOURCE_RANGE 1.5 // the distance that source can be detected
#define IR_RANGE 0.15  // IR range
#define IR_INTENSITY_FACTOR 0.0012 // to control the scale of intensity
#define IR_INTENSITY_OFFSET 0.03	// prevent the intensity becoming infinity
#define DEFAULT_MEMORY_SIZE 15
// GATING_FITNESS > (MAX_SPEED*UPDATE_INTERVAL/1000)*(F_MAX/DETECT_SOURCE_RANGE)
#define GATING_FITNESS 	15 // to filter values with big error
#define MIN_STATIONARY_TIME (2000/UPDATE_INTERVAL)	// 2 seconds
#define MAX_STATIONARY_TIME 30
#define DISTANCE_ERROR 0.001  // seen as stationary if the distance robots move less than it

#define GROUP_SIZE 3
#define DEPART_WEIGHT 1
#define CONVERGE_WEIGHT 2

#define TARGETS_COUNT 5
// targets are able to be collected after PREPARE_TIME since program running 
#define PREPARE_TIME (15*1000/UPDATE_INTERVAL)	// 15s	
// the collection of a target takes RESCUE_TIME
#define RESCUE_TIME (3*1000/UPDATE_INTERVAL)	// 3s 
// targets would be seen as found only if the fitness of any robots within
// the target's range is larger than RESCUE_RANGE
// #define RESCUE_RANGE 0.5	//	m
#define RESCUE_FITNESS 50	//	F_MAX/2
#define TARGET_ID_BASE 100  	// the id of targets counted from 101

extern ofstream logfile;
extern ofstream csvfile;
extern int iteration;
extern const string TARGET_CODE;

#endif