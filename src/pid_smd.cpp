#include <../include/pid_smd.h>

namespace pid_smd
{

pidSMDSystem::pidSMDSystem(){
	std::cout << "[pidSMDSystem constructor] Class object created with default values" << std::endl;
}

pidSMDSystem::pidSMDSystem(double kp_, double ki_, double kd_)
							:kp(kp_), ki(ki_), kd(kd_){
	this->tau = 0.0;
	this->prev_error = 0.0;
	this->prev_measurement = 0.0;
	this->max_int_limit = 1.0;
	this->min_int_limit = 0.0;
	this->prev_integrator = 0.0;
	this->prev_differentiator = 0.0;
	this->sampling_time = 0.01; //in seconds
	this->system_max = 1.0;
	this->system_min = 0.0;
	printf("[pidSMDSystem constructor] Class object created gain values kp: %lf ki: %f kd: %lf \n"
																, this->kp, this->ki, this->kd);
}

void pidSMDSystem::setGainValues(double new_kp, double new_ki, double new_kd){
	this->kp = new_kp;
	this->ki = new_ki;
	this->kd = new_kd;
}

void pidSMDSystem::getGainValues(double &kp_, double &ki_, double &kd_){
	kp_ = this->kp;
	ki_ = this->ki;
	kd_ = this->kd;
}

void pidSMDSystem::setCutoffFrequency(double freq){
	this->tau = 1/freq;
}

double pidSMDSystem::getCutoffFrequency(){
	return 1/tau;
}

void pidSMDSystem::setSamplingTime(double time_){
	this->sampling_time = time_;
}

double pidSMDSystem::getSamplingTime(){
	return this->sampling_time;
}

void pidSMDSystem::setTimeConstant(double new_time_const){
	this->tau = new_time_const;
}

double pidSMDSystem::getTimeConstant(){
	return this->tau;
}

void pidSMDSystem::setIntegratorLimit(double min_val, double max_val){
	this->min_int_limit = min_val;
	this->max_int_limit = max_val;
}

void pidSMDSystem::getIntegratorLimit(double &min_, double &max_){
	max_ = this->max_int_limit;
	min_ = this->min_int_limit;
}

void pidSMDSystem::setSystemLimits(double min_, double max_){
	this->system_max = max_;
	this->system_min = min_;
}

void pidSMDSystem::getSystemLimits(double &min_, double &max_){
	min_ = this->system_min;
	max_ = this->system_max;
}

double pidSMDSystem::pidUpdate(double set_pt, double measurement){

	double controlled_out; //variable to return the controlle output

	double error = set_pt - measurement; //error value

	double proportional = this->kp * error; //proportinal part

	//Integral
    this->prev_integrator = this->prev_integrator + (this->ki * this->sampling_time * (error + this->prev_error))/2;

	// Anti-wind-up via integrator clamping
    if (this->prev_integrator > this->max_int_limit){

        this->prev_integrator = this->max_int_limit;        
    }
    else if (this->prev_integrator < this->min_int_limit){

        this->prev_integrator = this->min_int_limit;
    }

	//Derivative
    this->prev_differentiator = -(2.0f * this->kd * (measurement - this->prev_measurement)	
                        + (2.0f * this->tau - this->sampling_time) * this->prev_differentiator)/ (2.0f * this->tau + this->sampling_time);

    controlled_out = proportional + this->prev_integrator + this->prev_differentiator;

    //clamping output vairable
    if (controlled_out > this->system_max) {

        controlled_out = this->system_max;

    } else if (controlled_out < this->system_min) {

        controlled_out = this->system_min;

    }

	// Store error and measurement for later use
    this->prev_error = error;
    this->prev_measurement = measurement;

	// Return controller output
	return controlled_out;
}

} //end of namespace