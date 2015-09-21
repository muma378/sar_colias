/*
* robot.cpp - the class of robot, wraps the interfaces for devices
* implements high-level behaviors 
* Auhtor: Yang Xiao
* Date: 13 September 2015
*/

#include "robot.h"


// initialize devices
Robot::Robot(PlayerClient* client, int index)
	: id_(index+1), 
	  engine_(client, index),
	  bumper_(client, 2*index), 
	  infrared_(client, 2*index+1), 
	  robot_detector_(client, 2*index), 
	  source_detector_(client, 2*index+1){
	// assign fitness a random negative number
	fitness_ = int( - ( rand() % F_MAX ));
}

void Robot::PrintInterfaces(){
	cout << "id: " << id_ << "fitness: " << get_fitness() << endl;
	cout << engine_ << infrared_ << robot_detector_ << endl;
}

void Robot::SetSpeed(double x, double y, double angle){
	// engine_.SetSpeed(x, y, angle);
	// engine_.SetVelHead(x, angle);
	engine_.SetSpeed(x, angle);
}

void Robot::SetSpeed(double x, double angle){
	engine_.SetSpeed(x, angle);
}

void Robot::set_fitness(int value){
	fitness_ = value;
}

int Robot::get_fitness(){
	return fitness_;
}

void Robot::SetVelocity(double x, double y){
	SetVelocity(x, y, COMMUNICATE_RANGE);
}

void Robot::SetVelocity(double x, double y, double normal){
	double turn_speed;
	double forward_speed;
	double small_error = 0.09;	// 5 degree
	// TODO:test if x can be 0
	double radians = atan( y/x );
	// tune the radians when angle is bigger than 90 degree
	if ( x<0 && y<0 ){	// the third quadrant
		radians = radians - PI;
	}
	if ( x<0 && y>0 ){	// the second quadrant
		radians = PI - radians;
	}
	// if it was heading to the target
	if ( abs(radians)<=small_error ){
		turn_speed = 0;
		double distance = sqrt((x*x)+(y*y));
		forward_speed = (distance/normal)*MAX_SPEED*ACCELATOR;
	}else{
		// make sure turns in the max speed but won't be over-tuned
		turn_speed = radians/UPDATE_INTERVAL*1000;
		turn_speed = abs(turn_speed)>TURN_SPEED_BOUND?TURN_SPEED_BOUND\
					  :turn_speed;
		forward_speed = 0;
	}
	SetSpeed(forward_speed, turn_speed);
}

// TODO: fitness shall be effected by many other factors
void Robot::UpdateFitness(){
	int fitness = GetSourceIntensity();
	fitness_ = fitness==0?Gating(fitness):fitness;
	SaveCurrentPlace();
}

// if the value much less or bigger than previous ones
// then leave it alone
int Robot::Gating(int actual){
	int estimate = history_memory_.EstimateFitness();
	estimate = estimate==-1?actual:estimate;	
	int fall = estimate - actual;
	if (abs(fall) > GATING_FITNESS){
		return estimate;
	}else{
		return actual;
	}

}

void Robot::Run(double& forward_speed, double& turn_speed){
	//	TODO: velocity is affected by group, history and obstacles
	
	SetSpeed(0.35, 0);
	VoidObstacles();

}

// TODO: robot reflects when detecting obstacles
// this strategy may be too simple, we will modify it later
void Robot::VoidObstacles(){
	double turn_speed = 0;
	double forward_speed = 0;
	if ( InBumperRange() || GotStuckIn() ){	//	random rotating untill not in the range
		turn_speed = random()%10==0?PI/2:-PI/2;
		// cout << "ROBOT " << id_ << " in bumper range! Rotate speed " << turn_speed << endl;
		forward_speed = - 0.15;
		SetSpeed(forward_speed, turn_speed);
		return;
	}
	double x;
	double y;
	double obstacle_bearing = GetObstacleBearing(x, y);
	if (abs(obstacle_bearing)==PI){
		return;
	}
	if (abs(obstacle_bearing)>=PI/2){
		forward_speed = 0.15;
	}
	if (abs(obstacle_bearing)>0 && abs(obstacle_bearing)<PI/2){	// will hit in the future
		turn_speed = -obstacle_bearing;
	}
	if (obstacle_bearing==0){	// heading obstacle
		int direction = (rand()%2)==0?1:-1;
		turn_speed = direction*PI/2;
	}
	SetSpeed(forward_speed, turn_speed);

}

// get the bearing of obstacle with data read from all irs
double Robot::GetObstacleBearing(double& x, double& y){
	y = 0;
	x = 0;
	for (int i = 0; i < GetIRRangerCount(); ++i){	// only use the front ir rangers
		double intensity = GetIRRangerIntensity(i);
		// cout << intensity << "-" ;
		y += intensity * sin_radians_between_irs_[i];
		x += intensity * cos_radians_between_irs_[i];
	}
	
	double obstacle_bearing = atan(y/x);
	if (x==0 && y==0){	// no obstacles detected
		obstacle_bearing = PI;
	}
	if (x<0 && y<0){	// locates at south-east
		// obstacle_bearing = atan(y/x)-PI;
		obstacle_bearing -= PI;
	}
	if (x<0 && y>0){	// locates at south-west
		// obstacle_bearing = PI+atan(y/x);
		obstacle_bearing += PI;
	}
	// cout << "robot " << id_ << " found a obstacle at" << obstacle_bearing << endl;
	return obstacle_bearing;

}

int Robot::GetIRRangerCount(){
	return infrared_.GetRangeCount();
}

double Robot::GetIRRangerIntensity(int ranger_index){
	if (infrared_.GetIntensity(ranger_index)==0){	//no obstacles detected
		return 0;
	}else{
		double range_reading = infrared_.GetRange(ranger_index) + IR_INTENSITY_OFFSET;
		return IR_INTENSITY_FACTOR/(range_reading*range_reading);
	}
	// return IR_RANGE - infrared_.GetRange(ranger_index);
}

bool Robot::InBumperRange(){
	for (int i = 0; i < bumper_.GetIntensityCount(); ++i){
		if (bumper_.GetIntensity(i)){
			return true;
		}
	}
	return false;
}

bool Robot::GotStuckIn(){
	if (id_==2){
		Place* place = history_memory_.GetPlaceWithMaxFitness();
		// cout << "get the place with max fitness: " << *place << endl;
	}
	return false;
}

void Robot::SaveCurrentPlace(){
	double x = engine_.GetXPos();
	double y = engine_.GetYPos();
	Place* place = new Place(x, y, fitness_);
	if (id_==2){
		// cout << *place << endl;
	}
	// history_memory_.Save( new Place(x, y, fitness_) );
	history_memory_.Save( place );
}

int Robot::GetNeighboursCount(){
	return robot_detector_.GetCount();
}

int Robot::GetNeighbourId(int neighbours_index){
	return robot_detector_.GetFiducialItem(neighbours_index).id;
}

player_pose3d_t Robot::GetNeighbourPose(int neighbours_index){
	return robot_detector_.GetFiducialItem(neighbours_index).pose;
}

player_pose3d_t Robot::GetNeighbourPoseError(int neighbours_index){
	return robot_detector_.GetFiducialItem(neighbours_index).upose;
}

player_fiducial_item_t Robot::GetNeighbour(int neighbours_index){
	return robot_detector_.GetFiducialItem(neighbours_index);
}

// intensity increases as the distance decreases
// if several sources exist at the same time, choose the highest one
int Robot::GetSourceIntensity(){
	player_pose3d_t pose;
	int fitness = 0;
	for (int i = 0; i < source_detector_.GetCount(); ++i){
		pose = source_detector_.GetFiducialItem(i).pose;
		fitness=fitness<GenerateFitness(pose)?GenerateFitness(pose):fitness;
	}
	return fitness;
}

int Robot::GenerateFitness(player_pose3d_t pose){
	return (1-DISTANCE(pose.px, pose.py)/DETECT_SOURCE_RANGE)*F_MAX;
}

Group::Group(Robot* robot)
	: fiducial_robot_(robot),
	  fiducial_robot_id_(robot->id_){
	Initialize(robot);
}

Group::~Group(){}

// does the same thing as the constructor but locally
void Group::ReConstruct(Robot* robot){
	fiducial_robot_ = robot;
	fiducial_robot_id_ = robot->id_;
	Initialize(robot);
}

void Group::Initialize(Robot* robot){
	ClearBitset();
	ResetPose3dT(center_);
	ResetPose3dT(fittest_center_);

	JoinGroup(robot->id_);
	// includes itself
	int neighbour_size = robot->GetNeighboursCount();
	// set the neighbours' corresponding bit 1
	for (int i = 0; i < neighbour_size ; ++i){
		JoinGroup(robot->GetNeighbourId(i));
	}
	group_size_ = neighbour_size + 1;	// includes itself
}

void Group::JoinGroup(const int robot_id){
	if ((robot_id>0) && (robot_id<=ROBOTS_COUNT)){
		members_bitset_[robot_id-1] = 1;
	}else{
		printf("Group::JoinGroup(%d) - index out of range\n", robot_id);
	}
}

// calculates the center and fittest center
void Group::Update(int* fitness_array){
	// cout << "fitness array:\n";
	// for (int i = 0; i < ROBOTS_COUNT; ++i){
	// 	cout << fitness_array[i] << endl;
	// }
	player_pose3d_t pose;
	ResetPose3dT(center_);
	ResetPose3dT(fittest_center_);
	int max_fitness_ = fiducial_robot_->get_fitness();
	int fittest_count = 1;
	for (int i = 0; i < group_size_-1 ; ++i){	// not includes itself
		// cout << "Neighbour pose: " << fiducial_robot_->GetNeighbourPose(i) << endl;
		// cout << "Center: " << center_ << endl;
		pose = fiducial_robot_->GetNeighbourPose(i);
		center_ = AddPose(center_, pose);
		// cout << "add poes: " << center_ << endl;
		int fitness = fitness_array[fiducial_robot_->GetNeighbourId(i)-1];
		// cout << "member id: " << fiducial_robot_->GetNeighbourId(i) << "-fitness: " << fitness << endl;

		if (max_fitness_==fitness){
			fittest_center_ = AddPose(fittest_center_, pose);
			fittest_count++;
		}
		if (max_fitness_<fitness){
			max_fitness_ = fitness;
			fittest_center_ = pose;
			fittest_count = 1;
		}
	}
	center_ = AveragePose(center_, group_size_);
	fittest_center_ = AveragePose(fittest_center_, fittest_count);
	// cout << "center : " << center_ <<endl;
	// cout << "fittest_center_ : "<< fittest_center_<<endl;
}

void Group::WeightGroupSpeed(double& forward_speed, double& turn_speed){
	return;
}

int Group::NextMemberIndex(int robot_index){
	if (robot_index>=-1&&robot_index<ROBOTS_COUNT){
		for (int i = robot_index+1; i < ROBOTS_COUNT; ++i){
			if (members_bitset_.test(i))
				return i;
		}
	}
	return -1;	// no one next to it
}

void Group::FillBitset(){
	members_bitset_.set();	// all 1
}

void Group::ClearBitset(){
	members_bitset_.reset();
}

void Group::ResetPose3dT(player_pose3d_t& p){
	p.px = 0;
	p.py = 0;
	p.pz = 0;
	p.proll = 0;
	p.ppitch = 0;
	p.pyaw = 0;
}
