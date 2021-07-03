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
			~pidSMDSystem(){}

			void setGainValues(double new_kp, double new_ki, double new_kd);

			void getGainValues(double &kp_, double &ki_, double &kd_);

			void setCutoffFrequency(double freq);

			double getCutoffFrequency();

			void setTimeConstant(double new_time_const);

			double getTimeConstant();

			double pidUpdate(double set_pt, double measurement);

		private:

			double kp, ki, kd; //gain values

			double tau; //time constant for low pass filter

			double prev_error, prev_measurement;
	};
}

#endif //PID_SMD_H