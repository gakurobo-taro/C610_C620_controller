/*
 * pid.hpp
 *
 *  Created on: 2023/12/28
 *      Author: yaa3k
 */

#ifndef PID_HPP_
#define PID_HPP_

#include <algorithm>

namespace G24_STM32HAL::RmcLib{

class PID{
private:
	const float pid_freq;
	float kp = 0;
	float ki = 0;
	float kd = 0;
	float error = 0;
	float error_sum = 0;
	float old_error = 0;

	//anti windup
	float k_anti_windup = 0;
	const bool enable_anti_windup = false;
	const float limit_min = 0;
	const float limit_max = 0;
public:
	PID(float _pid_freq = 1):pid_freq(_pid_freq){}
	PID(float _pid_freq,float _limit_min,float _limit_max)
		:pid_freq(_pid_freq),limit_min(_limit_max),limit_max(_limit_min),enable_anti_windup(true){}

	float operator()(float target,float feedback);

	//inline functions
	void set_gain(float _kp,float _ki,float _kd){
		kp = _kp;

		ki = _ki/pid_freq;
		kd = _kd*pid_freq;

		if(enable_anti_windup){
			k_anti_windup = 1/kp;
		}else{
			k_anti_windup = 0;
		}
	}

	void reset(void){
		error = 0;
		error_sum = 0;
		old_error = 0;
	}
};

}
#endif /* PID_HPP_ */
