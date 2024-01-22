/*
 * monitor_management.hpp
 *
 *  Created on: 2024/01/22
 *      Author: yaa3k
 */

#ifndef MONITOR_MANAGEMENT_HPP_
#define MONITOR_MANAGEMENT_HPP_

#include <stdint.h>
#include <array>

namespace G24_STM32HAL::RmcLib{

template<size_t BANK_N>
class MonitorManagement{
private:
	std::array<uint64_t,BANK_N> reg {0};
public:
	void set_register( size_t register_bank,uint64_t val){ reg.at(register_bank) = val; }
	uint64_t get_register( size_t register_bank )const{return reg.at(register_bank); }

	bool is_requested_to_monitor(uint16_t reg_number){
		size_t n = (reg_number >> 6)&0x3;
		size_t bit = reg_number & 0x3F;
		return (bool)((reg[n] >> bit) & 1);
	}
	constexpr size_t get_bank_num(void)const{return BANK_N;}
	constexpr size_t get_monitor_register_num(void)const{return 64*BANK_N;}
};

}


#endif /* MONITOR_MANAGEMENT_HPP_ */
