/*
* targets.h - to manager all targets, wraps the low-level accessing method
* Auhtor: Yang Xiao
* Date: 03 October 2015
*/

#ifndef __SAR_COLIAS_TARGETS__
#define __SAR_COLIAS_TARGETS__

#include "config.h"
#include "utils.h"

class Target
{
public:
	Target(SimulationProxy& simulation, int targets_size);
	~Target();
	
	friend ostream& operator<<(ostream& out, const Target& t){
	    out << "name: " << t.name_ << " pose: " << t.pose_;
	    out << "\niterations in rescue: " << t.iterations_in_rescue_;
	    out << "\ncollected: " << t.collected_ << " detected " << t.detected_;
	    return out;
		}

	void Setup();
	void Teardown();
	void Detected();
	void Collected(SimulationProxy& simulation);
	bool IsCollected(){ return collected_; }
	bool IsAlive(){ return !collected_; }
	bool TimeOver(){ return iterations_in_rescue_>RESCUE_TIME; }
	static int collected_number;

private:
	char* name_;
	Pose pose_;
	int robots_around_;
	bool detected_;
	bool collected_;
	int iterations_in_rescue_;
};


// sigleton class to be accessed anywhere
class TargetsManager
{
public:
	TargetsManager(PlayerClient* client);
	~TargetsManager();
	SimulationProxy simulation_;
	vector<Target*> targets_;
	Target*& operator[](const int i);
	// call it at each iteration beginning
	void Prepare();
	void Refresh();
	bool FoundAll(){ return target_size_==0; };

private:
	int target_size_;
};



#endif