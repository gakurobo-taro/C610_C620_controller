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

struct PIDGain{
	float kp;
	float ki;
	float kd;
};

class PID{
private:
	const float pid_freq;
	PIDGain gain;
	float error = 0;
	float error_sum = 0;
	float old_error = 0;

	//anti windup
	float k_anti_windup;
	const bool enable_anti_windup = false;
	float limit_min;
	float limit_max;
public:
	PID(float _pid_freq,PIDGain _gain={0.0f,0.0f,0.0f}, bool anti_windup = false,float _limit_min = 0.0f,float _limit_max = 0.0f):
		pid_freq(_pid_freq),
		gain(_gain),
		enable_anti_windup(anti_windup),
		limit_min(_limit_min),
		limit_max(_limit_max),
		k_anti_windup((gain.kp != 0.0f) ? 1/gain.kp : 0){}

	float operator()(float target,float feedback);

	//inline functions
	void set_gain(const PIDGain &_gain){
		gain.kp = _gain.kp;

		gain.ki= _gain.ki/pid_freq;
		gain.kd = _gain.kd*pid_freq;

		if(enable_anti_windup){
			k_anti_windup = 1/gain.kp;
		}else{
			k_anti_windup = 0;
		}
	}
	PIDGain get_gain(void)const{
		PIDGain _gain;
		_gain = gain;
		_gain.ki = gain.ki*pid_freq;
		_gain.kd = gain.kd/pid_freq;
		return _gain;
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
	PIDGain gain = {0,0,0};
	float limit_max = 0.0f;
	float limit_min = 0.0f;
	bool enable_anti_windup= false;

	PIDBuilder(float _freq = 1.0f):freq(_freq){}
	PIDBuilder& set_gain(PIDGain _gain){
		gain = _gain;
		return *this;
	}
	PIDBuilder& set_gain(float kp,float ki,float kd){
		gain.kp = kp;
		gain.ki = ki*freq;
		gain.kd = kd/freq;
		return *this;
	}
	PIDBuilder& set_limit(float _limit_min,float _limit_max){
		limit_max = _limit_max;
		limit_min = _limit_min;
		enable_anti_windup = true;
		return *this;
	}
	PID build()const{
		return PID{freq,gain,enable_anti_windup,limit_min,limit_max};
	}
};

}
#endif /* PID_HPP_ */
