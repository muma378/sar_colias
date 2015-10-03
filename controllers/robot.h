/*
* robot.hpp - the argments and headers used in the robot.cpp
* Auhtor: Yang Xiao
* Date: 13 September 2015
*/

#ifndef __SAR_COLIAS_ROBOT__
#define __SAR_COLIAS_ROBOT__

#include <bitset>
#include "config.h"
#include "utils.h"

enum Status { ROTATE, MOVING };

class Robot
{
public:
    const int id_;	// corresponds its fiducial return 
    Robot(PlayerClient* client, int index);
    ~Robot();
    void Run();
    void Run(Pose& velocity);
    Vector2d RandomUnitVelocity();
    void SetAbsoluteVelocity(const Vector2d& absolute_velocity);
	void SetRelativeVelocity(const Vector2d& relative_velocity);
    void VoidObstacles();
    void Continue();
    bool IsMoving(){ return status==Status::MOVING; };

    int SeekTarget();
    void UpdateFitness();
    int Gating(int actual);
    int GetSourceIntensity();
    int GenerateFitness(Pose* pose); 
    Vector2d GetObstacleBearing();
	bool InBumperRange();
	// bool GotStuckIn();
	// bool SystemCrashed();
	void SaveCurrentPlace();
	double GetIRRangerIntensity(int ranger_index);
 	int GetIRRangerCount();
    int GetNeighboursCount();
    // neighbours_index counted from 0 to GetNeighboursCount()-1
    int GetNeighbourId(int neighbours_index);
    Pose* GetNeighbourPose(int neighbours_index);
    Pose* GetNeighbourPose(Robot& robot);
    Pose* GetNeighbourPoseError(int neighbours_index);
    player_fiducial_item_t GetNeighbour(int neighbours_index);
    Pose* GetSourcePose(int source_index);
    int GetSourceId(int source_index);
	Place GetCurrentPlace();
	Pose GetCurrentPose();

    void SetSpeed(double x, double y, double angle);
    void SetSpeed(double x, double angle);
    void set_fitness(int value);
    int get_fitness();
    void set_group_index(int group_index){ group_index_=group_index; }
    int get_group_index(){ return group_index_; }

    void PrintInterfaces();
    void TestPoseClass(int neighbours_index);


private:
    int fitness_;
    int group_index_;
    int found_target_id_;
    Position2dProxy engine_;
    RangerProxy bumper_;
    RangerProxy infrared_;
    FiducialProxy robot_detector_;
    FiducialProxy source_detector_;

    Vector2d last_velocity_;
    HistoryMemory history_memory_;
    const double radians_between_irs_ [6] = { PI/3, 0, -PI/3, -PI*2/3, PI, PI*2/3 };
    const double sin_radians_between_irs_ [6] = { 0.866, 0, -0.866, -0.866, 0, 0.866 };
    const double cos_radians_between_irs_ [6] = { 0.5, 1, 0.5, -0.5, -1, -0.5 };
    Status status;
    int moving_counter_ = 0;
    void SetVelocity(const Vector2d& velocity);

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
	Pose WeightGroupVelocity(Robot& robot);
	Pose VelocityDepartCenter(Robot& robot);
	Pose VelocityAsFitnessGradient(Robot& robot);
	void FillBitset();		// sets all bits 1
	void ClearBitset();		// sets all bits 0
	// returns the robot next to the robot_id in the group
	int NextMemberIndex(int robot_index);
	bitset<ROBOTS_COUNT> get_members_bitset(){
		return members_bitset_;
	}
	// hides the process of calculation, returns the pose relative to current one
	Pose GetCenter(Robot& robot);
	Pose GetCenterWithMaxFitness(Robot& robot);
	
	int get_group_size(){ return group_size_; };
	int get_fiducial_robot_id(){ return fiducial_robot_id_; }
    void TestTranslateCoordinate();

private:
	// each bit stands for a corresponding robot whose index is (i+1)
	bitset<ROBOTS_COUNT> members_bitset_;
	// the group's center is relative to the fiducial point
	Pose center_;
	// center of the robots with maximum fitness in the group
	Pose center_with_max_fitness_;
	// fiducial point is the place where the first robot at
	Robot* fiducial_robot_;
	int fiducial_robot_id_;
	int group_size_;
	bool identical_fitness_;

};


class Target
{
public:
	Target();
	~Target();

	static int target_size_;
	static SimulationProxy* simulation_;
	bool collected_;
	bool detected_;
	int robots_around_;

	void Setup();
	void Teardown();
	void Detected();
	void Collected();
	int get_iterations(){ return iterations_in_rescue_; }

private:
	char* name_;
	Pose pose_;
	int iterations_in_rescue_;
};

#endif

