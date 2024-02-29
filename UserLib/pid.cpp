/*
 * pid.cpp
 *
 *  Created on: 2023/12/28
 *      Author: yaa3k
 */


#include "pid.hpp"


namespace G24_STM32HAL::RmcLib{


	float PID::operator() (float target,float feedback){
		error = target - feedback;
		float p = error * gain.kp;

		error_sum += error;
		float i = error_sum * gain.ki;

		float d = (error - old_error) * gain.kd;
		old_error = error;

		if(enable_anti_windup){
			float pid_result = p+i+d;

			float pid_result_clamped = std::clamp<float>(pid_result, limit_min, limit_max);

			error_sum -= (pid_result - pid_result_clamped)*k_anti_windup;

			return pid_result_clamped;
		}else{
			return p+i+d;
		}
	}


}
