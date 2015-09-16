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
	// updates the groups whihc hava at least one common element
	void UpdateConnectedGroup();
	void UpdateCenter();
	void AutoConstructGroup(Robot* robot);
	bool MergeSameGroup(int group_index);
	void ShrinkGroups();
	void FlipRobotBit(int robot_index){ swarm_bitset_[robot_index] = 1; }
	void FlipRobotBit(Group* group){ swarm_bitset_ |= group->get_members_bitset(); }
	void ResetSwarmBitset(){ swarm_bitset_.reset(); }
	bool HasRobotUngrouped(){ return !swarm_bitset_.all(); }
	bool IsRobotInGroup(int robot_index){ return swarm_bitset_.test(robot_index); }
	void SetRobotGroupIndex(int robot_id, int group_index);
	int GetUngroupedRobot();
	void PrintGroupsMembers();


private:
	vector<Robot*> robots_vector_;	// to record all robots
	vector<Group*> groups_vector_;	// to record all groups
	// 0 stands for idle, 1 represents being grouped
	bitset<ROBOTS_COUNT> swarm_bitset_;	
	unsigned group_actual_size_;

};

Swarm::Swarm(PlayerClient* client) : group_actual_size_(0){
    srand(time(NULL));
    for (int i = 0; i < ROBOTS_COUNT; ++i)
    	robots_vector_.push_back( new Robot(client, i) );
}

Swarm::~Swarm(){}

void Swarm::PrintGroupsMembers(){
	for (int i = 0; i < groups_vector_.size(); ++i){
		cout << "group-" << i << " : " << groups_vector_[i]->get_members_bitset().to_string() << endl;
	}
}

void Swarm::Test(){
	for (vector<Robot*>::iterator it = robots_vector_.begin(); 
		it != robots_vector_.end(); ++it){
		(*it)->SetSpeed(0.5, 0.5, 10);
		(*it)->PrintInterfaces();
		//cout << (*it)->get_fitness() << endl;
	}
}

int Swarm::GetUngroupedRobot(){
	for (int i = 0; i < ROBOTS_COUNT; ++i){
		if (!IsRobotInGroup(i))
			return i;
	}
	return -1;
}

void Swarm::ShrinkGroups(){
	// for stakes noe used
	int redundant_stake_num = groups_vector_.size() - group_actual_size_;
	if (redundant_stake_num){
		for (int i = 0; i < redundant_stake_num; ++i){
			delete groups_vector_.back();
			groups_vector_.pop_back();
		}
	}
}


void Swarm::SetRobotGroupIndex(int robot_id, int group_index){
	robots_vector_[robot_id-1]->set_group_index(group_index);
}

// groups that have same members are merged, their fiducial robots point to the sole one
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

// constructs groups in different ways according to the size of the group vector
// if there is extra unused slot, reconstructs the group with new fiducial robot
// if not, allocates a new slot for the group and appends it to the vector
void Swarm::AutoConstructGroup(Robot* robot){
	// if groups in the vector are all used
	if (groups_vector_.size()==group_actual_size_){	
		// appends a new stake
		groups_vector_.push_back(new Group(robot));
	}else{	// for the stakes not used
		// reuses it
		groups_vector_[group_actual_size_]->ReConstruct(robot);
	}	
}

void Swarm::Update(){
	// begin to update all robot's neighbours and group again
	ResetSwarmBitset();
	group_actual_size_ = 0;
	// if there exists a robot not in any group
	while(HasRobotUngrouped()){
		AutoConstructGroup(robots_vector_[GetUngroupedRobot()]);
		// as a robot unconnected to anyone in the previous groups
		// it surely can't be merged and the group size shall increase 1
		group_actual_size_++;
		UpdateConnectedGroup();
	}
	ShrinkGroups();
	// UpdateCenter();
}

void Swarm::UpdateConnectedGroup(){
	int current_group_index = group_actual_size_ - 1;
	for (unsigned int i = current_group_index; i < groups_vector_.size(); ++i){
		Group* current_group = groups_vector_[i];
		int fiducial_robot_index = current_group->get_fiducial_robot_id() - 1;
		
		if (IsRobotInGroup(fiducial_robot_index)){
			// the group has already constructed
			// cout << "robot " << current_group->get_fiducial_robot_id() << " already grouped." << endl; // TO CLEAN
			continue;
		}
		for (int robot_index = current_group->NextMemberIndex(-1); robot_index != -1;
			 robot_index = current_group->NextMemberIndex(robot_index)){
			// cout << "robot index: " << robot_index << endl;		// TO CLEAN
			// if the robot hasn't been put into a correct group
			if (!IsRobotInGroup(robot_index)){
				AutoConstructGroup(robots_vector_[robot_index]);
				current_group_index = group_actual_size_;
				// doesn't find the group has the exactly same subset
				if (!MergeSameGroup(current_group_index)){
					// then uses the current one
					group_actual_size_++;
					SetRobotGroupIndex(robot_index+1, current_group_index);
				}
			}
		}
		// cout << "Group " << i << " created." << endl; // TO CLEAN
		FlipRobotBit(current_group);
	}
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
	
  	swarm.Test();
  	while(true){
  		client.Read();
		swarm.Update();
		swarm.PrintGroupsMembers();
  		usleep(UPDATE_INTERVAL);
  	}

	return 0;
}