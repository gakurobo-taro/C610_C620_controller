/*
 * timer_control.hpp
 *
 *  Created on: Mar 5, 2024
 *      Author: gomas
 */

#ifndef TIMER_CONTROL_HPP_
#define TIMER_CONTROL_HPP_

#include "tim.h"

#include <functional>

namespace G24_STM32HAL::RmcLib{
	class IInterruptionTimer{
	public:
		void virtual set_and_start(uint16_t period) = 0; //period==0:stop, period!=0 set period and start
		uint16_t virtual get_state(void)const = 0;       //period==0:stop, period!=0 running
		void virtual set_task(std::function<void(void)> f) = 0;
	};

	class InterruptionTimerHard:public IInterruptionTimer{
	private:
		TIM_HandleTypeDef *tim;
		bool first_interrupt_flag = false;

		std::function<void(void)> task = nullptr;
	public:
		InterruptionTimerHard(TIM_HandleTypeDef *_tim):tim(_tim){}

		void set_and_start(uint16_t period)override{
			if(period == 0){
				HAL_TIM_Base_Stop_IT(tim);
				first_interrupt_flag = false;
			}else{
				__HAL_TIM_SET_AUTORELOAD(tim,period-1);
				__HAL_TIM_SET_COUNTER(tim,0);

				if(HAL_TIM_Base_GetState(tim) == HAL_TIM_STATE_READY){
					HAL_TIM_Base_Start_IT(tim);
				}
			}
		}
		uint16_t get_state(void)const override{
			if(HAL_TIM_Base_GetState(tim) == HAL_TIM_STATE_BUSY){
				return __HAL_TIM_GET_AUTORELOAD(tim)+1;
			}else{
				return 0;
			}
		}

		void set_task(std::function<void(void)> f)override{
			task = f;
		}

		void interrupt_task(void){
			if(!first_interrupt_flag){
				first_interrupt_flag = true;
			}else if(task != nullptr){
				task();
			}
		}

		TIM_HandleTypeDef *get_handler(void)const{return tim;}
	};
}

#endif /* TIMER_CONTROL_HPP_ */
