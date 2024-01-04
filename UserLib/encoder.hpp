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
		//指定したangleを指定したradとして解釈する（現在の角度(angle)を0(rad)としろ的な）
		virtual void set_angle_bias(uint16_t angle,float rad) = 0;
		virtual float get_rad(void) = 0;
		virtual float update_angle(uint16_t angle) = 0;
	};

	class C6x0Encoder : public IEncoder{
	private:
		const float gear_ratio;
		const size_t resolution_bit;
		const size_t resolution;
		const float angle_to_rad;
		const float rad_to_angle;

		uint16_t origin;
		uint16_t raw_angle;
		int turn_count;

	public:
		C6x0Encoder(float _gear_ratio,size_t _resolution_bit):
			gear_ratio(_gear_ratio),
			resolution_bit(_resolution_bit),
			resolution(1<<resolution_bit),
			angle_to_rad(2*M_PI/(resolution*gear_ratio)),
			rad_to_angle(1/angle_to_rad){}


		void set_angle_bias(uint16_t angle,float rad)override{
			origin = angle + rad*rad_to_angle;
		}
		float get_rad(void)override{
			return angle_to_rad*(raw_angle + turn_count*(int)resolution);
		}
		float update_angle(uint16_t angle)override{
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
