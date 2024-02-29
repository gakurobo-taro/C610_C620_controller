/*
 * LED_pattern.hpp
 *
 *  Created on: Feb 29, 2024
 *      Author: gomas
 */

#ifndef LED_PATTERN_HPP_
#define LED_PATTERN_HPP_

#include "LED_control.hpp"

namespace G24_STM32HAL::RmcLib::LEDPattern{

	inline const LEDState pwm_mode[] = {
			{true,100},
			{false,900},

			{false,0},
	};
	inline const LEDState speed_mode[] = {
			{true,100},
			{false,100},
			{true,100},
			{false,700},

			{false,0},
	};
	inline const LEDState position_mode[] = {
			{true,100},
			{false,100},
			{true,100},
			{false,100},
			{true,100},
			{false,500},

			{false,0},
	};
	inline const auto led_mode = std::array<LEDState *,3>{
		const_cast<LEDState*>(pwm_mode),
		const_cast<LEDState*>(speed_mode),
		const_cast<LEDState*>(position_mode),
	};

	inline const LEDState ok[] = {
			{true,100},
			{false,100},
			{false,0},
	};
	inline const LEDState error[]={
			{true,100},
			{false,100},
			{true,700},
			{false,100},
			{false,0},
	};
}



#endif /* LED_PATTERN_HPP_ */
