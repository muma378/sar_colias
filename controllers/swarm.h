/*
* swarm.h - the class to control whole procedure
* Auhtor: Yang Xiao
* Date: 18 September 2015
*/

#ifndef __SAR_COLIAS_SWARM__
#define __SAR_COLIAS_SWARM__

#include "config.h"
#include "robot.h"


class Swarm
{
public:
	Swarm(PlayerClient* client);
	~Swarm();
	void Test();

	void DetectSignals();
	// puts robots can be detected into a group
	void Grouping();
	void UpdateVelocities();
	void CollectTargets();

	// updates groups which hava at least one common element
	void UpdateConnectedGroups();
	void CalculateCenter();
	void AutoConstructGroup(Robot* robot);
	bool MergeSameGroups(int group_index);
	void ShrinkGroups();
	void SetRobotGroupIndex(int robot_id, int group_index);
	int GetUngroupedRobot();
	void FlipRobotBit(int robot_index){ swarm_bitset_[robot_index] = 1; }
	void FlipRobotBit(Group* group){ swarm_bitset_ |= group->get_members_bitset(); }
	void ResetSwarmBitset(){ swarm_bitset_.reset(); }
	bool HasRobotUngrouped(){ return !swarm_bitset_.all(); }
	bool IsRobotInGroup(int robot_index){ return swarm_bitset_.test(robot_index); }
	void PrintGroupsMembers();


private:
	vector<Robot*> robots_vector_;	// to record all robots
	vector<Group*> groups_vector_;	// to record all groups
	// 0 stands for idle, 1 represents being grouped
	bitset<ROBOTS_COUNT> swarm_bitset_;	
	unsigned group_actual_size_;

};

#endif