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
#include <iostream>
#include <math.h>
#include <libplayerc++/playerc++.h>

using namespace std;
using namespace PlayerCc;

#define ROBOTS_COUNT 3 	// the amount of robots
#define F_MAX 100	// max value of fitness
#define F_MIN 0
#define UPDATE_INTERVAL 300  // millisecond
#define MAX_SPEED 0.35	// m/s
#define PI 3.1415926
// the boundary of trun speed is PI/2
#define MAX_TURN_SPEED 7	// radians/s
#define TURN_SPEED_BOUND (PI/2)
#define SMALL_RADIAN_ERROR 0.09		// 5 degree
#define COMMUNICATE_RANGE 1  // m, range_max robot_detector possesses
#define ACCELATOR 1  // avoid the forward speed keeping so small
#define DETECT_SOURCE_RANGE 1.5  // the distance that source can be detected
#define IR_RANGE 0.15  // IR range
#define IR_INTENSITY_FACTOR 0.0012 // to control the scale of intensity
#define IR_INTENSITY_OFFSET 0.03	// prevent the intensity becoming infinity
#define HISTORY_COUNT 10  // records last 10 poses
#define DEFAULT_MEMORY_SIZE 10
// (MAX_SPEED*UPDATE_INTERVAL/1000)*(DETECT_SOURCE_RANGE/F_MAX)
#define GATING_FITNESS 3	// to filter values with big error
#define MIN_STATIONARY_TIME (2000/UPDATE_INTERVAL)	// 2 seconds
#define MAX_STATIONARY_TIME 30
#define DISTANCE_ERROR 0.001  // seen as stationary if the distance robots move less than it
#define DISTANCE(x, y) (sqrt(x*x+y*y)) 

#define GROUP_SIZE 5
#endif