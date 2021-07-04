/*********************************
 *
 * Software License Agreement 
 *
 *  Copyright (c) 2021. All rights reserved.
 *
 * Author: Brolin A 
 *
 *********************************/

#ifndef PID_SMD_H
#define PID_SMD_H

#include <iostream>

namespace pid_smd
{
	class pidSMDSystem
	{
		public:

			//constructor
			pidSMDSystem();

			pidSMDSystem(double kp_, double ki_, double kd_);

			//destructor
			~pidSMDSystem() {}

			void setGainValues(double new_kp, double new_ki, double new_kd);

			void getGainValues(double &kp_, double &ki_, double &kd_);

			void setCutoffFrequency(double freq);

			double getCutoffFrequency();

			void setSamplingTime(double time_);

			double getSamplingTime();

			void setTimeConstant(double new_time_const);

			double getTimeConstant();

			void setIntegratorLimit(double min_val, double max_val);

			void getIntegratorLimit(double &min_, double &max_);

			void setSystemLimits(double min_, double max_);

			void getSystemLimits(double &min_, double &max_);

			double pidUpdate(double set_pt, double measurement);

		private:

			double kp, ki, kd; //gain values

			double tau; //time constant for low pass filter

			double sampling_time;

			double prev_error, prev_measurement, prev_integrator, prev_differentiator;

			double max_int_limit, min_int_limit;

			double system_max, system_min;
	};
}

#endif //PID_SMD_H