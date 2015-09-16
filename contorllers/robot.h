/*
* robot.hpp - the argments and headers used in the robot.cpp
* Auhtor: Yang Xiao
* Date: 13 September 2015
*/

#ifndef __SAR_COLIAS_ROBOT__
#define __SAR_COLIAS_ROBOT__

#include <bitset>
#include "config.h"

class Robot
{
public:
    Robot(PlayerClient* client, int index);
    ~Robot();
    const int id_;	// corresponds its fiducial return 
    void SetSpeed(double x, double y, double angle);
    void set_fitness(int value);
    int get_fitness();
    void Run();
    void VoidObstacle();

    int GetNeighboursCount();
    // neighbours_index counted from 0 to GetNeighboursCount()-1
    int GetNeighbourId(int neighbours_index);
    player_pose3d_t GetNeighbourPose(int neighbours_index);
    player_pose3d_t GetNeighbourPoseError(int neighbours_index);
    player_fiducial_item_t GetNeighbour(int neighbours_index);

    void set_group_index(int group_index){ group_index_=group_index; }
    int get_group_index(){ return group_index_; }
    void GetSourceIntensity();    
    void PrintInterfaces();

private:
    int fitness_;
    int group_index_;
    Position2dProxy engine_;
    RangerProxy infrared_;
    FiducialProxy robot_detector_;
    FiducialProxy source_detector_;
};

// Robots with its neighbours form a group
class Group
{
public:
	// constructs the group with the robot and its neighbours
	Group(Robot* robot);
	~Group();
	// bool updated_;
	// construct the group locally
	void ReConstruct(Robot* robot);
	// sets the corresponding bit 1
	void JoinGroup(int robot_id);
	// calculates the center and fittest center
	void Update(int* fitness_array);
	void ResetPose3dT(player_pose3d_t p);
	// sets all bits 1
	void FillBitset();
	// sets all bits 0
	void ClearBitset();
	// returns the robot next to the robot_id in the group
	int NextMemberIndex(int robot_index);
	bitset<ROBOTS_COUNT> get_members_bitset(){
		return members_bitset_;
	}
	// hides the process of calculation, returns the pose relative to current one
	player_pose3d_t GetCenter(Robot* robot);
	player_pose3d_t GetFittestCenter(Robot* robot);
	player_pose3d_t AddPose(player_pose3d_t p1, player_pose3d_t p2){
		p1.px += p2.px; p1.py += p2.py; p1.pyaw += p2.pyaw; return p1; }
	player_pose3d_t AveragePose(player_pose3d_t p, int count){
		p.px /= count; p.py /= count; p.pyaw /= 3.1415926; return p;
	}
	int get_fiducial_robot_id(){ return fiducial_robot_id_; }
private:
	// each bit stands for a corresponding robot whose index is (i+1)
	bitset<ROBOTS_COUNT> members_bitset_;
	// the group's center is relative to the fiducial point
	player_pose3d_t center_;
	// center of the robots with maximum fitness in the group
	player_pose3d_t fittest_center_;
	// fiducial point is the place where the first robot at
	// player_pose3d_t fiducial_point_;
	Robot* fiducial_robot_;
	int fiducial_robot_id_;
	int group_size_;
};

#endif

