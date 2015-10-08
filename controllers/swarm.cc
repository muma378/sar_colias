/*
* swarm.cc - 
* Auhtor: Yang Xiao
* Date: 18 September 2015
*/

#include "swarm.h"

ofstream logfile;
ofstream csvfile;
int iteration = 0;


Swarm::Swarm(PlayerClient* client) 
	: group_actual_size_(0), 
	  targets_manager_(client){
    srand(time(NULL));
    for (int i = 0; i < ROBOTS_COUNT; ++i)
    	robots_vector_.push_back(new Robot(client, i));
}

Swarm::~Swarm(){
	// delete targets_manager_;
	for (int i = 0; i < ROBOTS_COUNT; ++i){
		delete robots_vector_[i];
	}
	for (int i = 0; i < group_actual_size_; ++i){
		delete groups_vector_[i];
	}
}

void Swarm::Test(){
	for (vector<Robot*>::iterator it = robots_vector_.begin(); 
		it != robots_vector_.end(); ++it){
		Vector2d vec(0.35, 0);
		(*it)->SetRelativeVelocity(vec);
		// (*it)->PrintInterfaces();
	}
}

void Swarm::PrintGroupsMembers(){
	for (unsigned int i = 0; i < groups_vector_.size(); ++i){
		cout << "group-" << i << " : " << groups_vector_[i]->get_members_bitset().to_string() << endl;
	}
	for (unsigned int i = 0; i < robots_vector_.size(); ++i){
		cout << "robot-" << i << " : " << robots_vector_[i]->get_group_index() << endl;
	}
}

void Swarm::DetectSignals(){
	targets_manager_.Prepare();
	for (vector<Robot*>::iterator it = robots_vector_.begin(); 
		 it != robots_vector_.end(); ++it){
		(*it)->UpdateFitness(targets_manager_);
		logfile << **it << endl;
	}
	return;
}

void Swarm::UpdateVelocities(){
	Group* belonging_group;

	for (vector<Robot*>::iterator it = robots_vector_.begin();
		 it != robots_vector_.end(); ++it){
		// cout << "Robot " << (*it)->id_ << " uses ";
		if ((*it)->IsMoving()){
			belonging_group = groups_vector_[(*it)->get_group_index()];
			if(belonging_group->get_group_size() > 1){
				Pose velocity = belonging_group->WeightGroupVelocity(**it);	
				(*it)->Run(velocity);
			} else {
				(*it)->Run();
			}
		} else {
			(*it)->Continue();
		}
	}
	return;
}

void Swarm::CollectTargets(){
	targets_manager_.Refresh();
	return; 
}


bool Swarm::Complete(){
	return targets_manager_.FoundAll();
}

void Swarm::Grouping(){
	// begin to update all robot's neighbours and group again
	ResetSwarmBitset();
	group_actual_size_ = 0;
	// if there exists a robot not in any group
	while(HasRobotUngrouped()){
		AutoConstructGroup(robots_vector_[GetUngroupedRobot()]);
		// as a robot unconnected to anyone in the previous groups
		// it surely can't be merged and the group size shall increase 1
		group_actual_size_++;
		UpdateConnectedGroups();
	}
	// remove redundant stakes in group_vector_
	ShrinkGroups();
	// PrintGroupsMembers();
	// calculate each group's center and the center of robots with max fitness
	CalculateCenter();
}

int Swarm::GetUngroupedRobot(){
	for (int i = 0; i < ROBOTS_COUNT; ++i){
		if (!IsRobotInGroup(i))
			return i;
	}
	return -1;
}

void Swarm::ShrinkGroups(){
	// for stakes not used
	// cout << "begin to remove : " << groups_vector_.size() << " and " << group_actual_size_ << endl;
	int redundant_stake_num = groups_vector_.size() - group_actual_size_;
	if (redundant_stake_num){
		for (int i = 0; i < redundant_stake_num; ++i){
			delete groups_vector_.back();
			groups_vector_.pop_back();
			// cout << "remove one element : " << groups_vector_.size() << endl;
		}
	}
}

void Swarm::SetRobotGroupIndex(int robot_id, int group_index){
	robots_vector_[robot_id-1]->set_group_index(group_index);
}

// groups that have same members are merged, their fiducial robots point to the sole one
bool Swarm::MergeSameGroups(int group_index){
	int fiducial_id = groups_vector_[group_index]->get_fiducial_robot_id();
	bitset<ROBOTS_COUNT> fiducial_bitset = groups_vector_[group_index]->get_members_bitset();
	for (int i = 0; i < group_index; ++i){
		bitset<ROBOTS_COUNT> current_bitset = groups_vector_[i]->get_members_bitset();
		// cout << current_bitset.to_string() << "^" << fiducial_bitset.to_string() << endl;
		if ((current_bitset^fiducial_bitset).none()){
			// cout << "Merge successful" << endl;
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


void Swarm::UpdateConnectedGroups(){
	int group_index = group_actual_size_ - 1;
	for (unsigned int i = group_index; i < group_actual_size_; ++i){
		Group* current_group = groups_vector_[i];
		for (int robot_index = current_group->NextMemberIndex(-1); robot_index != -1;
			 robot_index = current_group->NextMemberIndex(robot_index)){
			// cout << "robot index: " << robot_index << endl;		// TO CLEAN
			// if the robot hasn't been put into a correct group
			if (!IsRobotInGroup(robot_index)){
				AutoConstructGroup(robots_vector_[robot_index]);
				group_index = group_actual_size_;
				// doesn't find the group has the exactly same subset
				if (!MergeSameGroups(group_index)){
					// then uses the current one
					group_actual_size_++;
					SetRobotGroupIndex(robot_index+1, group_index);
				}
			}
		}
		// cout << "Group " << i << " created." << endl; // TO CLEAN
		FlipRobotBit(current_group);
	}
}

void Swarm::CalculateCenter(){
	int fitness_array[ROBOTS_COUNT];
	for (int i = 0; i < ROBOTS_COUNT; ++i)
		fitness_array[i] = robots_vector_[i]->get_fitness();
	for (vector<Group*>::iterator it = groups_vector_.begin(); 
		 it != groups_vector_.end(); ++it){
		(*it)->Update(fitness_array);
	}
}

void TestTranslateToStandard(){
	Pose forward_velocity(1, 0, 0);
	Pose standard_pose(0, 0, PI/4);
	// should be (0.707, 0.707, PI/4)
	cout << "Translated standard coordinate is" ;
	cout << forward_velocity.TranslateToCoordinate(standard_pose) << endl;
}

void TestTranslateToLocal(){
	Pose standard_pose(1, 1, 0);
	Pose local_pose(0, 0, -PI/4);
	// should be (1.414, 0, 0);
	cout << "Translated local coordinate is" ;
	cout << standard_pose.TranslateToCoordinate(local_pose) << endl;

}

int main(int argc, char const *argv[])
{	
	PlayerClient client("localhost", 6665);
	Swarm swarm(&client);
	logfile.open("robots.log");
	csvfile.open("data.csv", ofstream::out | ofstream::app);
  	do{
  		iteration++;
  		client.Read();
  		swarm.DetectSignals();
		swarm.Grouping();
		swarm.UpdateVelocities();
		swarm.CollectTargets();
  		usleep(UPDATE_INTERVAL);
  	} while (!swarm.Complete());

  	logfile << "Program ends at the iteration " << iteration << endl;
  	logfile.close();
  	csvfile.close();
  	exit(0);
	return 0;
}
