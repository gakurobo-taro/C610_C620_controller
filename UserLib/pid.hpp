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
	float limit_min;
	float limit_max;
public:
	PID(float _pid_freq = 1.0f,float _kp = 0.0f,float _ki = 0.0f,float _kd = 0.0f, bool anti_windup = false,float _limit_min = 0.0f,float _limit_max = 0.0f):
		pid_freq(_pid_freq),kp(_kp),ki(_ki),kd(_kd),
		enable_anti_windup(anti_windup),limit_min(_limit_min),limit_max(_limit_max),
		k_anti_windup(1/kp){}

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
	void set_limit(float _limit_min,float _limit_max){
		limit_min = _limit_min;
		limit_max = _limit_max;
	}

	void reset(void){
		error = 0;
		error_sum = 0;
		old_error = 0;
	}
};

class PIDBuilder{
public:
	float freq = 1.0f;
	float kp = 0.0f;
	float ki = 0.0f;
	float kd = 0.0f;
	float limit_max = 0.0f;
	float limit_min = 0.0f;
	bool enable_anti_windup= false;

	PIDBuilder(float _freq = 1.0f):freq(_freq){}
	PIDBuilder& set_gain(float _kp,float _ki, float _kd){
		kp = _kp;
		ki = _ki;
		kd = _kd;
		return *this;
	}
	PIDBuilder& set_limit(float _limit_min,float _limit_max){
		limit_max = _limit_max;
		limit_min = _limit_min;
		enable_anti_windup = true;
		return *this;
	}
	PID build()const{
		return PID{freq,kp,ki,kd,enable_anti_windup,limit_min,limit_max};
	}
};

}
#endif /* PID_HPP_ */
