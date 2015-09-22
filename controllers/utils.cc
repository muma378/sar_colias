/*
* utils.cc - tools for other classes
* Auhtor: Yang Xiao
* Date: 21 September 2015
*/

#include "utils.h"

/*
 * Vector2d Class
 * Base class.
*/
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

double& Vector2d::operator[](int i){
	if (i==0)
		return this->x_;
	if (i==1)
		return this->y_;
	throw out_of_range("Index beyonds 1 ");
}

// the radians to postive x-axis
double Vector2d::Radian() const{
	double radian = atan(y_/x_);
	if (x_<0&&y_>0)
		radian += PI;
	if (x_<0&&y_<0)
		radian -= PI;
	return radian;
}

/*
 * Place Class
 * Saves coordinate and fitness.
*/
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
Place Place::operator-(const Place& place){
	Place place_diff(*this);
	place_diff.x_ = this->x_ - place.x_;
	place_diff.y_ = this->y_ - place.y_;
	return place_diff;
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

Place& Place::operator-=(const Place& place){
	*this = *this - place;
	return *this;
}

Place& Place::operator/=(const int divisor){
	*this = *this / divisor;
	return *this;
}

void Place::operator=(const Place& place){
	this->x_ = place.x_;
	this->y_ = place.y_;
	this->fitness_ = place.fitness_;
}

bool Place::operator==(const Place& place) const{
	return ( abs(x_-place.x_)<DISTANCE_ERROR
			 && abs(y_-place.y_)<DISTANCE_ERROR );
}

/*
 * Pose Class
 * Saves coordinate and yaw.
*/

Pose Pose::operator+(const Pose& pose){
	Pose pose_sum(*this);
	pose_sum.x_ = this->x_ + pose.x_;
	pose_sum.y_ = this->y_ + pose.y_;
	return pose_sum;
}

Pose Pose::operator-(const Pose& pose){
	Pose pose_diff(*this);
	pose_diff.x_ = this->x_ - pose.x_;
	pose_diff.y_ = this->y_ - pose.y_;
	return pose_diff;
}

Pose Pose::operator/(const int divisor){
	Pose pose_avg(*this);
	pose_avg.x_ = this->x_/divisor;
	pose_avg.y_ = this->y_/divisor;
	return pose_avg;
}

Pose& Pose::operator+=(const Pose& pose){
	*this = *this + pose;
	return *this;
}

Pose& Pose::operator/=(const int divisor){
	*this = *this / divisor;
	return *this;
}

Pose& Pose::operator-=(const Pose& pose){
	*this = *this - pose;
	return *this;
}

void Pose::operator=(const Pose& pose){
	this->x_ = pose.x_;
	this->y_ = pose.y_;
	this->yaw_ = pose.yaw_;
}

// translate current coordinate to the corrdinate system that relative_pose within
Pose Pose::TranslateCoordinate(Pose& relative_pose){
	if (this->IsAtOrigin()){
		return relative_pose;
	} else {
		// the radians between the vector and x-axis 
		double radian = this->Radian();
		double magnitude = this->Magnitude();
		// relative_pose.yaw is radians of difference between the two coordinate system
		Pose relative_center(magnitude*cos(radian+relative_pose.yaw_), 
							 magnitude*sin(radian+relative_pose.yaw_));
		// shift
		return relative_center+relative_pose;
	}
}

/*
 * HistoryMemory Class
 * Saves last memory_size data.
*/
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

Place*& HistoryMemory::operator[](const int i){
	if (i<places_history_deque_.size())
		return places_history_deque_[i];
	throw out_of_range("Can not refer index beyonds 1 ");
}

void HistoryMemory::Save(Place* place){
	// CountStationaryTime(place);
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
	// cout << "place with max fitness: " << *place_with_max_fitness_;
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
		estimate = estimate>GetMaxFitness()?\
				   GetMaxFitness():estimate;
		estimate = estimate<F_MIN?F_MIN:estimate;
	}
	return estimate;
}

Place* HistoryMemory::GetPlaceWithMaxFitness(){
	return place_with_max_fitness_;
}

int HistoryMemory::GetMaxFitness(){
	return place_with_max_fitness_->get_fitness();
}

void HistoryMemory::CountStationaryTime(const Place* new_place){
	if (places_history_deque_.size()){
		if (*new_place==*(places_history_deque_.back())){
			stationarity_counter_++;
			return;
		}
	}
	stationarity_counter_ = 0;
	return;
}

StationaryScaler HistoryMemory::ScaleSameRecords(){
	if (stationarity_counter_<MIN_STATIONARY_TIME){
		return StationaryScaler::SLIGHT;
	}
	if (stationarity_counter_>MAX_STATIONARY_TIME){
		stationarity_counter_ = 0;
		return StationaryScaler::CRASHED;
	}
	stationarity_counter_ -= 10;
	return StationaryScaler::STUCK;
}
