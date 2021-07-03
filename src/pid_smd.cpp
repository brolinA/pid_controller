#include <../include/pid_smd.h>

namespace pid_smd
{

pidSMDSystem::pidSMDSystem()
{
	std::cout << "contructor called" << std::endl;
}

pidSMDSystem::pidSMDSystem(double kp_, double ki_, double kd_)
							:kp(kp_), ki(ki_), kd(kd_)
{
	this->tau = 0.0;
	this->prev_error = 0.0;
	this->prev_measurement = 0.0;
}

void pidSMDSystem::setGainValues(double new_kp, double new_ki, double new_kd)
{
	this->kp = new_kp;
	this->ki = new_ki;
	this->kd = new_kd;
}

void pidSMDSystem::getGainValues(double &kp_, double &ki_, double &kd_)
{
	kp_ = this->kp;
	ki_ = this->ki;
	kd_ = this->kd;
}

void pidSMDSystem::setCutoffFrequency(double freq)
{
	this->tau = 1/freq;
}

double pidSMDSystem::getCutoffFrequency()
{
	return 1/tau;
}

void pidSMDSystem::setTimeConstant(double new_time_const)
{
	this->tau = new_time_const;
}

double pidSMDSystem::getTimeConstant()
{
	return this->tau;
}

double pidSMDSystem::pidUpdate(double set_pt, double measurement)
{
	double controlled_out; //variable to return the controlle output


	return controlled_out;
}


} //end of namespace