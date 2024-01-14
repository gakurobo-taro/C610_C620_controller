/*
 * encoder.hpp
 *
 *  Created on: 2024/01/04
 *      Author: yaa3k
 */

#ifndef ENCODER_HPP_
#define ENCODER_HPP_

#include "main.h"

namespace G24_STM32HAL::RmcLib{

	class IEncoder{
	public:
		virtual float get_rad(void) = 0;
	};

	class AngleEncoder : public IEncoder{
	private:
		const size_t resolution_bit;
		const size_t resolution;
		const float angle_to_rad;
		const float rad_to_angle;

		uint16_t raw_angle;
		int turn_count = 0;

	public:
		AngleEncoder(size_t _resolution_bit):
			resolution_bit(_resolution_bit),
			resolution(1<<resolution_bit),
			angle_to_rad(2*M_PI/resolution),
			rad_to_angle(1/angle_to_rad){
		}

		float get_rad(void)override{
			return angle_to_rad*(raw_angle + turn_count*(int)resolution);
		}

		float update_angle(uint16_t angle){
			angle = angle&(resolution-1);
			int angle_top2 = angle>>(resolution_bit-2);
			int old_angle_top2 = raw_angle>>(resolution_bit-2);

			if(old_angle_top2  == 3 && angle_top2 == 0){
				turn_count ++;
			}else if(old_angle_top2  == 0 && angle_top2 == 3){
				turn_count --;
			}

			raw_angle = angle;
			return get_rad();
		}
	};

}



#endif /* ENCODER_HPP_ */
