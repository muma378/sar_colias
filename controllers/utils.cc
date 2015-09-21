/*
* utils.cc - tools for other classes
* Auhtor: Yang Xiao
* Date: 21 September 2015
*/

#include "utils.h"

Vector2d::Vector2d() : x_(0), y_(0){}

Vector2d::Vector2d(double x, double y) : x_(x), y_(y){}

Vector2d::~Vector2d(){}

Vector2d Vector2d::operator+(const Vector2d& vec2d){
	Vector2d vec2d_sum;
	vec2d_sum.x_ = this->x_ + vec2d.x_;
	vec2d_sum.y_ = this->y_ + vec2d.y_;
	return vec2d_sum;
}	

Vector2d Vector2d::operator/(const int divisor){
	Vector2d vec2d_avg;
	vec2d_avg.x_ = this->x_/divisor;
	vec2d_avg.y_ = this->y_/divisor;
	return vec2d_avg;
}


Place::Place(double x, double y) : Vector2d(x, y), fitness_(0){
}

Place::Place(double x, double y, int fitness)
	: Vector2d(x, y),
	  fitness_(fitness){
}

Place::Place(const Place& obj){
	x_ = obj.x_;
	y_ = obj.y_;
	fitness_ = obj.fitness_;
}


Place Place::operator+(const Place& place){
	Place place_sum(*this);
	place_sum.x_ += place.x_;
	place_sum.y_ += place.y_;
	return place_sum;
}

Place Place::operator/(const int divisor){
	Place place_center(*this);
	place_center.x_ /= divisor;
	place_center.y_ /= divisor;
	return place_center;
}

Place& Place::operator+=(const Place& place){
	*this = *this + place;
	return *this;
}

Place& Place::operator/=(const int divisor){
	*this = *this / divisor;
	return *this;
}

void Place::operator=(const Place& place){
	x_ = place.x_;
	y_ = place.y_;
	fitness_ = place.fitness_;
}


HistoryMemory::HistoryMemory()
	: memory_size_(DEFAULT_MEMORY_SIZE){
	place_with_max_fitness_ = new Place();
}

HistoryMemory::HistoryMemory(int memory_size)
	: memory_size_(memory_size){
	place_with_max_fitness_ = new Place();
}

HistoryMemory::~HistoryMemory(){
	while(!places_history_deque_.empty()){
		delete places_history_deque_.front();
		places_history_deque_.pop_front();
	}
}

void HistoryMemory::Save(Place* place){
	places_history_deque_.push_back(place);
	if (places_history_deque_.size()>memory_size_){
		delete places_history_deque_.front();
		places_history_deque_.pop_front();
	}
	RenewCenter();
}

void HistoryMemory::RenewCenter(){
	int count = 0;
	Place* place_with_max_fitness = new Place();
	Place* place;
	for (int i = 0; i < places_history_deque_.size(); ++i){
		place = places_history_deque_[i];
		if (place_with_max_fitness->get_fitness() < place->get_fitness()){
			*place_with_max_fitness = *place;
			count = 1;
		}
		if (place_with_max_fitness->get_fitness()==place->get_fitness()){
			*place_with_max_fitness += *place;
			count++;
		}
	}
	*place_with_max_fitness /= count;
	delete place_with_max_fitness_;
	place_with_max_fitness_ = place_with_max_fitness;
	return;	
}
	
int HistoryMemory::EstimateFitness(){
	int size = places_history_deque_.size();
	int estimate;
	if (size==0){
		estimate = -1;
	}else{
		estimate = places_history_deque_.back()->get_fitness();
	}
	if (size>1){
		estimate = estimate + (estimate - places_history_deque_[size-2]->get_fitness());
		estimate = estimate>place_with_max_fitness_->get_fitness()?\
				   place_with_max_fitness_->get_fitness():estimate;
		estimate = estimate<F_MIN?F_MIN:estimate;
	}
	return estimate;
}

Place* HistoryMemory::GetPlaceWithMaxFitness(){
	return place_with_max_fitness_;
}
