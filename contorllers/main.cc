/*
  main.cpp - Initialize all robots and schedule 
  Author: Yang Xiao
  Date: 13 September
*/

#include "config.h"
#include "robot.h"


class Swarm
{
public:
	Swarm(PlayerClient* client);
	~Swarm();
	void Test();

	void Update();
	void UpdateGroup();
	void UpdateCenter();
	bool MergeSameGroup(int group_index);
	void ShrinkGroups();
	void FlipRobotBit(int robot_index);
	void FlipRobotBit(Group* group);
	void ResetSwarmBitset();
	void SetRobotGroupIndex(int robot_id, int group_index);

private:
	vector<Robot*> robots_vector_;	// to record all robots
	vector<Group*> groups_vector_;	// to record all groups
	bitset<ROBOTS_COUNT> swarm_bitset_;
	unsigned group_actual_size_;

};

Swarm::Swarm(PlayerClient* client) : group_actual_size_(0){
    srand(time(NULL));
    for (int i = 0; i < ROBOTS_COUNT; ++i)
    	robots_vector_.push_back( new Robot(client, i) );
}

Swarm::~Swarm(){}

void Swarm::Test(){
	for (vector<Robot*>::iterator it = robots_vector_.begin(); 
		it != robots_vector_.end(); ++it){
		(*it)->SetSpeed(0.5, 0.5, 10);
		(*it)->PrintInterfaces();
		//cout << (*it)->get_fitness() << endl;
	}
	while(true){
		Update();
  		usleep(100);
  	}
}

void Swarm::Update(){
	UpdateGroup();
	UpdateCenter();
}

void Swarm::ShrinkGroups(){
	// for stakes noe used
	int redundant_stake_num = groups_vector_.size() - group_actual_size_;
	if (redundant_stake_num){
		for (int i = 0; i < redundant_stake_num; ++i){
			groups_vector_.pop_back();
		}
	}
}

void Swarm::FlipRobotBit(int robot_index){
	swarm_bitset_[robot_index] = 1;
}

void Swarm::FlipRobotBit(Group* group){
	swarm_bitset_ |= group->get_members_bitset();
}

void Swarm::ResetSwarmBitset(){
	swarm_bitset_.reset();	// all 0
}

void Swarm::SetRobotGroupIndex(int robot_id, int group_index){
	robots_vector_[robot_id-1]->set_group_index(group_index);
}

bool Swarm::MergeSameGroup(int group_index){
	int fiducial_id = groups_vector_[group_index]->get_fiducial_robot_id();
	bitset<ROBOTS_COUNT> fiducial_bitset = groups_vector_[group_index]->get_members_bitset();
	for (int i = 0; i < group_index; ++i){
		bitset<ROBOTS_COUNT> current_bitset = groups_vector_[i]->get_members_bitset();
		if ((current_bitset^fiducial_bitset).none()){
			SetRobotGroupIndex(fiducial_id, i);
			return true;
		}
	}
	return false;
}

void Swarm::UpdateGroup(){
	if (groups_vector_.empty()){
		// initialize the group with the first robot
		groups_vector_.push_back( new Group(robots_vector_[0]) );
		group_actual_size_ = 1;
	}
	ResetSwarmBitset();	// sets all bits 0
	for (unsigned int i = 0; i < groups_vector_.size(); ++i){
		Group* current_group = groups_vector_[i];
		for (int robot_index = current_group->NextMemberIndex(-1); robot_index != -1;
			 robot_index = current_group->NextMemberIndex(robot_index)){
			// if the robot hasn't been put into a correct group
			if (!swarm_bitset_.test(robot_index)){
				// if groups in the vector are all used
				if (groups_vector_.size()==group_actual_size_){	
					// appends a new stake
					groups_vector_.push_back(new Group(robots_vector_[robot_index]));
				}else{	// for the stakes not used
					// reuses it
					groups_vector_[group_actual_size_]->ReConstruct(robots_vector_[robot_index]);
				}
				int current_group_index = group_actual_size_;
				// doesn't find the group has the exactly same subset
				if (!MergeSameGroup(current_group_index)){
					// then uses the current one
					group_actual_size_++;
					SetRobotGroupIndex(robot_index+1, current_group_index);
				}
			}
		}
		FlipRobotBit(current_group);
	}
	ShrinkGroups();
}

void Swarm::UpdateCenter(){
	int fitness_array[ROBOTS_COUNT];
	for (int i = 0; i < ROBOTS_COUNT; ++i)
		fitness_array[i] = robots_vector_[i]->get_fitness();
	for (vector<Group*>::iterator it = groups_vector_.begin(); 
		 it != groups_vector_.end(); ++it){
		(*it)->Update(fitness_array);
	}
}

int main(int argc, char const *argv[])
{	
	PlayerClient client("localhost", 6665);
	Swarm swarm(&client);
	client.Read();
  	swarm.Test();

	return 0;
}