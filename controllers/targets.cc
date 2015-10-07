/*
* targets.cc - managers all targets, wraps the low-level accessing method
* outputs the evaluation data
* Auhtor: Yang Xiao
* Date: 03 October 2015
*/
#include "targets.h"

const string TARGET_CODE = "survivor";
int Target::collected_number = 0;

Target::Target(SimulationProxy& simulation, int target_index)
	: collected_(false),
	  iterations_in_rescue_(0){
	string target_name = TARGET_CODE + to_string(target_index);
	name_ = new char[target_name.length() + 1];
	strcpy(name_, target_name.c_str());
	simulation.GetPose2d(name_, pose_.x_, pose_.y_, pose_.yaw_); 
	// cout << name_ << endl;
	// cout << pose_ << endl;
}

Target::~Target(){
	delete [] name_;
}

// call it at the iteration beginning to reset parameters
void Target::Setup(){
	detected_ = false;
	robots_around_ = 0;
}

// call it at the iteration ending to reset parameters
void Target::Teardown(){
	if (!collected_){
		// logfile << "Target " << name_ << " is surrounded by " << robots_around_ << "robots";
		// logfile << " and has been found for " << iterations_in_rescue_ << " iterations." << endl;
		if (!detected_){	// no one found the target in this iteration
			iterations_in_rescue_ = 0;	// the rescue is intermitted and restart
		}
	}
}

// couts the continuous iterations under rescue
// restart if no one is nearby in the iteration
void Target::Detected(){
	if (!detected_){	// the first time to be detected
		detected_ = true;
		iterations_in_rescue_++;	// increase the iterations under rescue
	}
	robots_around_++;	// more than 1 robot are nearby
}

// remove the target from the map
void Target::Collected(SimulationProxy& simulation){
	simulation.SetPose2d(name_, -100, -100, 0);
	collected_ = true;
	logfile << "Target " << name_ << " was collected at iteration " << iteration << endl;
	csvfile << ++Target::collected_number << ", " << iteration << endl;
}



TargetsManager::TargetsManager(PlayerClient* client)
	: simulation_(client, 0),
	  target_size_(0){
    for (int i = 0; i < TARGETS_COUNT; ++i)
    	targets_.push_back(new Target(simulation_, ++target_size_));
    
}

TargetsManager::~TargetsManager(){
	for (int i = 0; i < TARGETS_COUNT; ++i){
		delete targets_[i];
	}
}

Target*& TargetsManager::operator[](const int i){
	// cout << i << " and " << targets_.size() << endl;
	if (i<targets_.size())
		return targets_[i];
	throw out_of_range("index out of range.");
}

void TargetsManager::Prepare(){
	for (int i = 0; i < TARGETS_COUNT; ++i){
		targets_[i]->Setup();
		// cout << *targets_[i] << endl;
	}
}

void TargetsManager::Refresh(){
	for (int i = 0; i < TARGETS_COUNT; ++i){
		Target* target = targets_[i];
		if (target->IsAlive() && target->TimeOver()){
			target->Collected(simulation_);
			target_size_--;
		}
		target->Teardown();
	}
}