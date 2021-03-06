/*
* utils.h - tools for other classes
* Auhtor: Yang Xiao
* Date: 21 September 2015
*/

#ifndef __SAR_COLIAS_UTILS__
#define __SAR_COLIAS_UTILS__

#include <deque>
#include "config.h"

class Vector2d
{
public:
	Vector2d() : x_(0), y_(0){};
	Vector2d(double x, double y) : x_(x), y_(y){};
	Vector2d(const Vector2d& obj){ x_=obj.x_; y_=obj.y_; }
	double x_;
	double y_;

	Vector2d operator+(const Vector2d& vec2d) const;
	Vector2d operator/(const int divisor) const;
	Vector2d operator*(const int multiplier) const;
	Vector2d& operator+=(const Vector2d& vec2d);
	double& operator[](int i);

	friend ostream& operator<<(ostream& out, const Vector2d& vec){
	    out << "(" << vec.x_ << ", " << vec.y_ << ")  ";
	    return out;
	}
	double Magnitude() const{ return sqrt(x_*x_+y_*y_); }
	Vector2d Normalize() const{ return *this/this->Magnitude(); }
	double Radian() const;
	bool IsAtOrigin() const{ return x_==0&&y_==0; };
	void Random() { x_=rand()%100+1; y_=rand()%100+1; };
};

class Place: public Vector2d
{
public:
	Place() : Vector2d(), fitness_(0){};
	Place(double x, double y) : Vector2d(x, y), fitness_(0){};
	Place(double x, double y, int f) : Vector2d(x, y), fitness_(f){};
	Place(const Place& obj);

	Place operator+(const Place& place);
	Place operator-(const Place& place);
	Place operator/(const int divisor);
	Place& operator+=(const Place& place);
	Place& operator-=(const Place& place);
	Place& operator/=(const int divisor);
	void operator=(const Place& place);
	bool operator==(const Place& place) const;

	friend ostream& operator<<(ostream& out, const Place& place){
	    out << "(" << place.x_ << ", " << place.y_ << ")  ";
	    out << "Fitness: " << place.fitness_;
	    return out;
	}
	int get_fitness(){ return fitness_; }
	using Vector2d::Magnitude;
	using Vector2d::Radian;

private:
	int fitness_;
};

class Pose: public Vector2d
{
public:
	Pose() : Vector2d(), yaw_(0){};
	Pose(const Vector2d& vec) : Vector2d(vec), yaw_(0){}
	Pose(const Vector2d& vec, double yaw) : Vector2d(vec), yaw_(yaw){}
	Pose(double x, double y) : Vector2d(x, y), yaw_(0){};
	Pose(double x, double y, double yaw) : Vector2d(x, y), yaw_(yaw){};
	Pose(player_pose3d_t player_pose)
		: Vector2d(player_pose.px, player_pose.py),
		  yaw_(player_pose.pyaw){}
	double yaw_;
	
	Pose operator+(const Pose& pose);
	Pose operator/(const int divisor);
	Pose operator-(const Pose& pose);
	Pose& operator+=(const Pose& pose);	
	Pose& operator/=(const int divisor);
	Pose& operator-=(const Pose& pose);
	void operator=(const Pose& pose);
	friend ostream& operator<<(ostream& out, const Pose& pose){
	    out << "(" << pose.x_ << ", " << pose.y_ <<  ", "  << pose.yaw_ << ") ";
	    return out;
	}

	void Clear(){ x_ = y_ = yaw_ = 0; }
	Pose TranslateToCoordinate(Pose& relative_pose);
	
	using Vector2d::Magnitude;
	using Vector2d::Radian;
	using Vector2d::IsAtOrigin;

};


enum class StationaryScaler{
	SLIGHT = 0,
	STUCK = 1,
	CRASHED = 2,
};

class HistoryMemory
{
public:
	HistoryMemory();
	HistoryMemory(int memory_size);
	~HistoryMemory();

	Place*& operator[](const int i);
	Place*& back(){ return places_history_deque_.back(); };
	void Save(Place* place);
	void RenewCenter();
	int EstimateFitness();
	Place* GetPlaceWithMaxFitness();
	int GetMaxFitness();
	void CountStationaryTime(const Place* new_place);
	StationaryScaler ScaleSameRecords();


private:
	deque<Place*> places_history_deque_;
	Place* place_with_max_fitness_;
	int memory_size_;
	int stationarity_counter_;
};


#endif
