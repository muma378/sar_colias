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
	Vector2d();
	Vector2d(double x, double y);
	~Vector2d();
	void SetValue(double a);
	void SetValue(double x, double y);
	Vector2d operator+(const Vector2d& vec2d);
	Vector2d operator/(const int divisor);
	
	inline double &operator[](int i){
		if (i==0)
			return this->x_;
		if (i==1)
			return this->y_;
		throw out_of_range("Can not refer index beyonds 1 ");
	}

protected:
	double x_;
	double y_;
};


class Place: public Vector2d
{
public:
	Place() : Vector2d(), fitness_(0){};
	Place(double x, double y);
	Place(double x, double y, int fitness);
	Place(const Place& obj);
	~Place(){}

	Place operator+(const Place& place);
	Place operator/(const int divisor);
	Place& operator+=(const Place& place);
	Place& operator/=(const int divisor);
	void operator=(const Place& place);
	friend ostream& operator<<(ostream& out, const Place& place){
	    out << "(" << place.x_ << ", " << place.y_ << ")  ";
	    out << "Fitness: " << place.fitness_;
	    return out;
	}

	int get_fitness(){ return fitness_; }

private:
	int fitness_;
};


class HistoryMemory
{
public:
	HistoryMemory();
	HistoryMemory(int memory_size);
	~HistoryMemory();

	void Save(Place* place);
	void RenewCenter();
	int EstimateFitness();
	Place* GetPlaceWithMaxFitness();


private:
	deque<Place*> places_history_deque_;
	Place* place_with_max_fitness_;
	int memory_size_;
};


#endif
