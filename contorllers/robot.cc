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
	  infrared_(client, 2*index+1), 
	  robot_detector_(client, 2*index), 
	  source_detector_(client, 2*index+1){
	// assign fitness a random negative number
	fitness_ = int( - ( rand() % F_MAX ));
}

void Robot::SetSpeed(double x, double y, double angle){
	engine_.SetSpeed(x, y, angle);
}

void Robot::set_fitness(int value){
	fitness_ = value;
}

int Robot::get_fitness(){
	return fitness_;
}

void Robot::PrintInterfaces(){
	cout << engine_ << infrared_ << robot_detector_ << endl;
}

void Robot::Run(){
}

void Robot::VoidObstacle(){

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

void Robot::GetSourceIntensity(){

}

Group::Group(Robot* robot)
	: fiducial_robot_(robot),
	  fiducial_robot_id_(robot->id_){
	ClearBitset();
	ResetPose3dT(center_);
	ResetPose3dT(fittest_center_);
	
	JoinGroup(robot->id_);
	// includes itself
	group_size_ = robot->GetNeighboursCount()+1;
	// set the neighbours' corresponding bit 1
	for (int i = 0; i < group_size_ ; ++i){
		JoinGroup(robot->GetNeighbourId(i));
	}
}

// does the same thing as the constructor but locally
void Group::ReConstruct(Robot* robot){
	fiducial_robot_ = robot;
	fiducial_robot_id_ = robot->id_;
	ClearBitset();
	ResetPose3dT(center_);
	ResetPose3dT(fittest_center_);

	JoinGroup(robot->id_);
	group_size_ = robot->GetNeighboursCount()+1;
	// set the neighbours' corresponding bit 1
	for (int i = 0; i < group_size_ ; ++i){
		JoinGroup(robot->GetNeighbourId(i));
	}
}

void Group::JoinGroup(int robot_id){
	if (robot_id>0 && robot_id<=ROBOTS_COUNT){
		members_bitset_[robot_id-1] = 1;
	}else{
		printf("Group::JoinGroup(%d) - index out of range\n", robot_id);
	}
}

// calculates the center and fittest center
void Group::Update(int* fitness_array){
	player_pose3d_t pose;
	int max_fitness_ = fiducial_robot_->get_fitness();
	int fittest_count = 1;
	for (int i = 0; i < group_size_ ; ++i){
		pose = fiducial_robot_->GetNeighbourPose(i);
		center_ = AddPose(center_, pose);
		
		int fitness = fitness_array[fiducial_robot_->GetNeighbourId(i)-1];
		if (max_fitness_<fitness){
			max_fitness_ = fitness;
			fittest_center_ = pose;
			fittest_count = 1;
		}
		if (max_fitness_==fitness){
			fittest_center_ = AddPose(fittest_center_, pose);
			fittest_count++;
		}
	}
	center_ = AveragePose(center_, group_size_);
	fittest_center_ = AveragePose(fittest_center_, fittest_count);
	cout << "center : " << center_ <<endl;
	cout << "fittest_center_ : "<< fittest_center_<<endl;
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

void Group::ResetPose3dT(player_pose3d_t p){
	p.px = 0;
	p.py = 0;
	p.pz = 0;
	p.proll = 0;
	p.ppitch = 0;
	p.pyaw = 0;
}
