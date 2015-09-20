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
    const int id_;	// corresponds its fiducial return 
    Robot(PlayerClient* client, int index);
    ~Robot();
    void Run(double& forward_speed, double& turn_speed);
    void VoidObstacles();
    void UpdateFitness();
    int GetSourceIntensity();
    int DetectFitness(player_pose3d_t pose); 
    void SetVelocity(double x, double y);
    void SetVelocity(double x, double y, double normal);
    double GetObstacleBearing(double& x, double& y);
    void Normalization(double& x, double& y);
	bool InBumperRange();
	bool GotStuckIn();

	double GetIRRangerIntensity(int ranger_index);
 	int GetIRRangerCount();

    int GetNeighboursCount();
    // neighbours_index counted from 0 to GetNeighboursCount()-1
    int GetNeighbourId(int neighbours_index);
    player_pose3d_t GetNeighbourPose(int neighbours_index);
    player_pose3d_t GetNeighbourPoseError(int neighbours_index);
    player_fiducial_item_t GetNeighbour(int neighbours_index);

    void SetSpeed(double x, double y, double angle);
    void SetSpeed(double x, double angle);
    void set_fitness(int value);
    int get_fitness();
    void set_group_index(int group_index){ group_index_=group_index; }
    int get_group_index(){ return group_index_; }

    void PrintInterfaces();

private:
    int fitness_;
    int group_index_;
    Position2dProxy engine_;
    RangerProxy bumper_;
    RangerProxy infrared_;
    FiducialProxy robot_detector_;
    FiducialProxy source_detector_;

    double velocity_x_;
    double velocity_y_;
    const double radians_between_irs_ [6] = { PI/3, 0, -PI/3, -PI*2/3, PI, PI*2/3 };
    const double sin_radians_between_irs_ [6] = { 0.866, 0, -0.866, -0.866, 0, 0.866 };
    const double cos_radians_between_irs_ [6] = { 0.5, 1, 0.5, -0.5, -1, -0.5 };
};

// Robots with its neighbours form a group
class Group
{
public:
	// constructs the group with the robot and its neighbours
	Group(Robot* robot);
	~Group();
	// construct the group locally
	void ReConstruct(Robot* robot);
	void Initialize(Robot* robot);
	// sets the corresponding bit 1
	void JoinGroup(const int robot_id);
	// calculates the center and fittest center
	void Update(int* fitness_array);
	void WeightGroupSpeed(double& forward_speed, double& turn_speed);
	void ResetPose3dT(player_pose3d_t& p);
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
		p1.px += p2.px; p1.py += p2.py; return p1; }
	player_pose3d_t AveragePose(player_pose3d_t p, int count){
		p.px /= count; p.py /= count; return p;
	} // TODO: get the right average yaw
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

