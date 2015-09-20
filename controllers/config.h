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

#define ROBOTS_COUNT 20 	// the amount of robots
#define F_MAX 15	// max value of fitness
#define UPDATE_INTERVAL 100  // millisecond
#define MAX_SPEED 0.35	// m/s
#define PI 3.1415926
// the boundary of trun speed is PI/2
#define MAX_TURN_SPEED 7	// radians/s
#define TURN_SPEED_BOUND (PI/2)
#define COMMUNICATE_RANGE 1  // m, range_max robot_detector possesses
#define ACCELATOR 1  // avoid the forward speed keeping so small
#define DETECT_SOURCE_RANGE 1.5  // the 
#define IR_RANGE 0.15  // IR range
#define IR_INTENSITY_FACTOR 0.0012 // to control the scale of intensity
#define IR_INTENSITY_OFFSET 0.03	// prevent the intensity becoming infinity

#define DISTANCE(x, y) (sqrt(x*x+y*y)) 

#endif