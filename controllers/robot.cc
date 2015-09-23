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

// the velocity is relative to the standard coordinate system
void Robot::SetAbsoluteVelocity(const Vector2d& absolute_velocity){
	last_velocity_ = absolute_velocity;
	// translate standard to local
	Pose local_pose = GetCurrentPose();
	Pose standard_pose(absolute_velocity);
	local_pose = standard_pose.TranslateToCoordinate(local_pose);
	SetVelocity(local_pose);
}

// the velocity is relative to current coordinate system
void Robot::SetRelativeVelocity(const Vector2d& relative_velocity){
	SetVelocity(relative_velocity);
	// translate local to standard
	Pose standard_pose(0, 0, 0);
	Pose local_pose(relative_velocity, engine_.GetYaw());
	last_velocity_ = local_pose.TranslateToCoordinate(standard_pose);
}

// low-level method, shouldn't be called by other methods
// the parameter velocity is relative to local current coordinate system
void Robot::SetVelocity(const Vector2d& velocity){
	double forward_speed = 0;
	double turn_speed = 0;
	double radians = velocity.Radian();
	if (abs(radians)<=SMALL_RADIAN_ERROR) {
		// forward_speed = velocity.Magnitude()/COMMUNICATE_RANGE*MAX_SPEED*ACCELATOR;
		forward_speed = 0.25;
	} else {
		// make sure turns in the max speed but won't be over-tuned
		turn_speed = radians/UPDATE_INTERVAL*1000;
		turn_speed = abs(turn_speed)>TURN_SPEED_BOUND?TURN_SPEED_BOUND\
					  :turn_speed;
	}
	SetSpeed(forward_speed, turn_speed);
}

// TODO: fitness shall be effected by many other factors
void Robot::UpdateFitness(){
	int fitness = GetSourceIntensity();
	fitness_ = fitness==0?Gating(fitness):fitness;
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

void Robot::Run(Pose& velocity){
	// velocity is affected by group, history and obstacles
	// only one robot in the group
	if (velocity.IsAtOrigin()){
		// and there was a fitness higher than current one
		if (fitness_ < history_memory_.GetMaxFitness()){
			Vector2d vec = *(history_memory_.GetPlaceWithMaxFitness()) - 
						   GetCurrentPlace();
			// fitness is less than the last place
			if (fitness_ < history_memory_.back()->get_fitness())
				vec += RandomUnitVelocity()*10;
			else 
				vec += RandomUnitVelocity();
			if (id_==1)
				cout << "strategy 4 move to" << vec << endl;
			SetAbsoluteVelocity(vec);
		} else {
			if (id_==1)
				cout << "strategy 3 move to" << last_velocity_ << endl;
			// keeps the previous velocity
			SetAbsoluteVelocity(last_velocity_);
		}
	} else {
		if (id_==1)
			cout << "move to " << velocity << endl;
		// velocity += RandomUnitVelocity();
		SetRelativeVelocity(RandomUnitVelocity()+velocity);
	}
	// SetSpeed(0.25, 0);
	VoidObstacles();
	SaveCurrentPlace();
}

Vector2d Robot::RandomUnitVelocity(){
	Vector2d random_vector(random()%100, random()%100);
	return random_vector.Normalize()*0.1;
}

// TODO: robot reflects when detecting obstacles
// this strategy may be too simple, we will modify it later
void Robot::VoidObstacles(){
	double turn_speed = 0;
	double forward_speed = 0;
	if (InBumperRange()){	//	random rotating untill not in the range
		turn_speed = random()%10==0?PI/2:-PI/2;
		// cout << "ROBOT " << id_ << " in bumper range! Rotate speed " << turn_speed << endl;
		forward_speed = - 0.15;
		SetSpeed(forward_speed, turn_speed);
		return;
	}
	Vector2d obstacle_bearing = GetObstacleBearing();
	if (obstacle_bearing.IsAtOrigin()){
		return;	// no obstocles detected
	} else {
		double obstacle_yaw = obstacle_bearing.Radian();
		Vector2d velocity;
		// obstocles detcted locate behind
		if (abs(obstacle_yaw) >= PI/2){ 
			velocity.x_ = 0.15;
		}
		// heading obstacle
		if (abs(obstacle_yaw)>0 && abs(obstacle_yaw)<PI/2){	// will hit in the future
			velocity.y_ = -obstacle_bearing.y_;
			// TODO:to add x
		}
		// facing obstacle
		if (obstacle_yaw==0){	
			velocity.y_ = (rand()%2)==0?1:-1;
		}
		SetRelativeVelocity(velocity);
	}
}

// get the bearing of obstacle with data read from all irs
Vector2d Robot::GetObstacleBearing(){
	Vector2d obstacle_bearing;
	for (int i = 0; i < GetIRRangerCount(); ++i){	// only use the front ir rangers
		double intensity = GetIRRangerIntensity(i);
		// cout << intensity << "-" ;
		obstacle_bearing.x_ += intensity * cos_radians_between_irs_[i];
		obstacle_bearing.y_ += intensity * sin_radians_between_irs_[i];
	}
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

int Robot::GetNeighboursCount(){
	return robot_detector_.GetCount();
}

int Robot::GetNeighbourId(int neighbours_index){
	return robot_detector_.GetFiducialItem(neighbours_index).id;
}

Pose* Robot::GetNeighbourPose(int neighbours_index){
	// TestPoseClass(neighbours_index);
	return new Pose(robot_detector_.GetFiducialItem(neighbours_index).pose);
}

Pose* Robot::GetNeighbourPose(Robot& robot){
	for (int i = 0; i < GetNeighboursCount(); ++i){
		if (GetNeighbourId(i)==robot.id_){
			return GetNeighbourPose(i);
		}
	}
	throw out_of_range("Not in the list of neighbours");
}

Pose* Robot::GetNeighbourPoseError(int neighbours_index){
	return new Pose(robot_detector_.GetFiducialItem(neighbours_index).upose);
}

Pose* Robot::GetSourcePose(int source_index){
	return new Pose(source_detector_.GetFiducialItem(source_index).pose);
}

player_fiducial_item_t Robot::GetNeighbour(int neighbours_index){
	return robot_detector_.GetFiducialItem(neighbours_index);
}

// intensity increases as the distance decreases
// if several sources exist at the same time, choose the highest one
int Robot::GetSourceIntensity(){
	Pose* pose;
	int fitness = 0;
	for (int i = 0; i < source_detector_.GetCount(); ++i){
		pose = GetSourcePose(i);
		fitness=fitness<GenerateFitness(pose)?GenerateFitness(pose):fitness;
		delete pose;
	}
	return fitness;
}

int Robot::GenerateFitness(Pose* pose){
	return (1-pose->Magnitude()/DETECT_SOURCE_RANGE)*F_MAX;
}

// bool Robot::GotStuckIn(){
// 	if (history_memory_.ScaleSameRecords()==StationaryScaler::STUCK){
// 		return true;
// 	}else{
// 		return false;
// 	}
// }

// bool Robot::SystemCrashed(){
// 	if (history_memory_.ScaleSameRecords()==StationaryScaler::CRASHED){
// 		return true;
// 	}else{
// 		return false;
// 	}
// }

void Robot::SaveCurrentPlace(){
	double x = engine_.GetXPos();
	double y = engine_.GetYPos();
	history_memory_.Save( new Place(x, y, fitness_) );
	if (id_==1){
		Place* place = new Place(x, y, fitness_);
		cout << "Robot " << id_ << "'s current place is " << *place << endl;
	}
}

Place Robot::GetCurrentPlace(){
	Place place(engine_.GetXPos(), engine_.GetYPos(), fitness_);
	return place;
}

Pose Robot::GetCurrentPose(){
	// as the origin of a coordinate system
	Pose pose(0, 0, engine_.GetYaw());
	return pose;
}

void Robot::TestPoseClass(int neighbours_index){
	player_pose3d_t pose0 = robot_detector_.GetFiducialItem(neighbours_index).pose;
	Pose* pose1 = new Pose(pose0);
	Pose* pose2 = new Pose(pose0);
	cout << "player_pose3d_t: " << pose0;
	cout << "pose: " << *pose1;
	cout << "pose sum: " << (*pose1)+(*pose2);
	cout << "pose divide: " << (*pose1)/2 << endl;

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
	center_.Clear();
	center_with_max_fitness_.Clear();

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
	Pose* pose;
	// it is also the coordinate of the fiducial robot
	center_.Clear();
	center_with_max_fitness_.Clear();
	int max_fitness_ = fiducial_robot_->get_fitness();
	int max_fitness_counter = 1;
	for (int i = 0; i < group_size_-1 ; ++i){	// not includes itself
		pose = fiducial_robot_->GetNeighbourPose(i);
		center_ +=  *pose;
		int fitness = fitness_array[fiducial_robot_->GetNeighbourId(i)-1];
		// picks places with max fitness
		if (max_fitness_==fitness){
			center_with_max_fitness_ += *pose;
			max_fitness_counter++;
		}
		if (max_fitness_<fitness){
			max_fitness_ = fitness;
			center_with_max_fitness_ = *pose;
			max_fitness_counter = 1;
		}
		delete pose;
	}
	center_ /= group_size_;
	center_with_max_fitness_ /= max_fitness_counter;
	identical_fitness_=max_fitness_counter==group_size_?true:false;
	// cout << "center_with_max_fitness_ : "<< center_with_max_fitness_<<endl;
}

Pose Group::WeightGroupVelocity(Robot& robot){
	Pose velocity;
	if (identical_fitness_ && group_size_>=2){
		if (robot.id_==1)
			cout << "strategy 1 ";
		velocity = VelocityDepartCenter(robot);
	}else{
		if (group_size_>GROUP_SIZE){
			if (robot.id_==1)
				cout << "strategy 1+2 ";
			velocity = VelocityDepartCenter(robot);
			velocity += VelocityAsFitnessGradient(robot);
		}
		if (group_size_<=GROUP_SIZE && group_size_>=2){
			if (robot.id_==1)
				cout << "strategy 2 ";
			velocity = VelocityAsFitnessGradient(robot);
		}
	}
	// if only one robot in the group, return velocity with (0, 0)
	return velocity;
	// TestTranslateCoordinate();
}

inline Pose Group::VelocityDepartCenter(Robot& robot){
	Pose origin;
	return origin-GetCenter(robot);
}

inline Pose Group::VelocityAsFitnessGradient(Robot& robot){
	// cout << "center :" << GetCenter(robot);
	// cout << "center_with_max_fitness :" << GetCenterWithMaxFitness(robot);
	return GetCenterWithMaxFitness(robot) - GetCenter(robot);
}

Pose Group::GetCenter(Robot& robot){
	// cout << "Robot " << robot.id_ << " : ";
	if (robot.id_==fiducial_robot_id_){
		// cout << center_;
		return center_;
	}else{
		// translate the coordinate system of the center_ to the robot's
		return center_.TranslateToCoordinate(
			*(robot.GetNeighbourPose(*fiducial_robot_)));
	}
}

Pose Group::GetCenterWithMaxFitness(Robot& robot){
	if (robot.id_==fiducial_robot_id_){
		return center_with_max_fitness_;
	}else{
		// translate the coordinate system of the center_ to the robot's
		return center_with_max_fitness_.TranslateToCoordinate(
			*(robot.GetNeighbourPose(*fiducial_robot_)));
	}
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

void Group::TestTranslateCoordinate(){
	Pose current_pose(-1, -1, 0);
	Pose relative_pose(0, 2, PI/4);
	cout << "test tanslated coordinate: " << current_pose.TranslateToCoordinate(relative_pose) << endl;

}