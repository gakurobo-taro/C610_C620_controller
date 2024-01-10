/*
 * board_task.cpp
 *
 *  Created on: Jan 6, 2024
 *      Author: yaa3k
 */

#include "board_task.hpp"

namespace G24_STM32HAL::RmcBoard{

	uint8_t read_board_id(void){
		uint8_t id = 0;
		for(int i = 0; i<4; i++){
			id |= !(uint8_t)HAL_GPIO_ReadPin(dip_sw.at(i).port,dip_sw.at(i).pin) << i;
		}
		return id;
	}

	//各クラス起動処理
	void init(void){
		can_main.start();
		can_main.set_filter_free(16);
		can_c6x0.start();
		can_c6x0.set_filter_free(0);
		LED_R.start();
		LED_G.start();
		LED_B.start();

		for(auto d:driver){
			d.set_speed_gain({0.2f, 0.002, 0});
			d.set_position_gain({1.0f, 0.001, 0});
			d.set_speed_limit(-1.0,1.0);
		}
	}

	//受信したモーター情報の処理
	void motor_data_process(void){
		if(can_c6x0.rx_available()){
			CommonLib::CanFrame rx_frame;
			can_c6x0.rx(rx_frame);

			if(rx_frame.id & 0x200){
				int id = (rx_frame.id&0xF)-1;
				motor_state.at(id).update(rx_frame);
				driver.at(id).update_operation_val(motor_state.at(id));
				HAL_GPIO_WritePin(LED.at(id).port,LED.at(id).pin,GPIO_PIN_SET);
			}
		}
	}

	//PWM値の送信
	void send_motor_parameters(void){
		CommonLib::CanFrame tx_frame;
		tx_frame.id = 0x200;
		auto writer = tx_frame.writer();

		for(int i = 0; i < 4; i++){
			int16_t duty = (int16_t)(driver.at(i).get_pwm() * 10000.0f);
			writer.write<uint8_t>(duty>>8);
			writer.write<uint8_t>(duty&0xFF);
		}

		can_c6x0.tx(tx_frame);
		for(auto l:LED){
			HAL_GPIO_WritePin(l.port,l.pin,GPIO_PIN_RESET);
		}
	}

	//メインCANの処理（外部との通信）
	void main_comm_prossess(void){
		bool data_received = false;
		int id = read_board_id();
		CommonLib::DataPacket rx_data;

		if(can_main.rx_available()){
			CommonLib::CanFrame rx_frame;
			can_main.rx(rx_frame);
			CommonLib::DataConvert::decode_can_frame(rx_frame, rx_data);
			data_received = true;
		}else if(usb_cdc.rx_avilable()){
			uint8_t rx_bytes[64] = {0};
			usb_cdc.rx(rx_bytes, sizeof(rx_bytes));
			CommonLib::DataConvert::decode_COBS_bytes(rx_bytes, rx_data);
			data_received = true;
		}

		if(data_received && id == rx_data.board_ID && rx_data.data_type == CommonLib::DataType::RMC_DATA){
			LED_R.out_as_gpio_toggle();
			execute_rmc_command(rx_data);
		}else if(data_received && id == rx_data.board_ID && rx_data.data_type == CommonLib::DataType::COMMON_DATA){
			LED_G.out_as_gpio_toggle();
			execute_common_command(rx_data);
		}else if(data_received && rx_data.data_type == CommonLib::DataType::COMMON_DATA){
			LED_G.out_as_gpio_toggle();
			execute_common_command(rx_data);
		}
	}
	void execute_rmc_command(CommonLib::DataPacket &data){
		RmcReg reg_id = (RmcReg)(data.register_ID & 0xFF);
		uint16_t motor_id = data.register_ID >> 8;

		//read
		if(data.is_request){
			CommonLib::DataPacket return_data;
			switch(reg_id){
			case RmcReg::NOP:
				//nop
				break;
			case RmcReg::MOTOR_TYPE:
				//TODO:PWMの最大値が変わるが正直現状誤差なので放置しちゃった
				break;
			case RmcReg::CONTROL_TYPE:

				break;
			case RmcReg::MOTOR_STATE:
				break;
			case RmcReg::PWM:
				break;
			case RmcReg::PWM_TARGET:
				break;
			case RmcReg::SPD:
				break;
			case RmcReg::PWM_LIM:
				break;
			case RmcReg::SPD_GAIN_P:
				break;
			case RmcReg::SPD_GAIN_I:
				break;
			case RmcReg::SPD_GAIN_D:
				break;
			case RmcReg::MONITOR_PERIOD:
				break;
			case RmcReg::MONITOR_REG1:
				break;
			case RmcReg::MONITOR_REG2:
				break;
			case RmcReg::MONITOR_REG3:
				break;
			case RmcReg::MONITOR_REG4:
				break;
			default:
				break;
			}


		//write
		}else{
			CommonLib::ByteReader reader = data.reader();
			std::optional<float> fval;
			std::optional<uint8_t> u8val;
			std::optional<uint16_t> u16val;
			std::optional<uint64_t> u64val;
			RmcLib::PIDGain tmp_gain;

			switch(reg_id){
			case RmcReg::NOP:
				//nop
				break;

			case RmcReg::MOTOR_TYPE:
				break;

			case RmcReg::CONTROL_TYPE:
				u8val = reader.read<uint8_t>();
				if(u8val.has_value()) driver.at(motor_id).set_control_mode((RmcLib::ControlMode)u8val.value());
				break;

			case RmcReg::GEAR_RATIO:
				fval = reader.read<float>();
				if(fval.has_value()) motor_state.at(motor_id).set_gear_ratio(fval.has_value());
				break;

			case RmcReg::MOTOR_STATE:
				break;

			case RmcReg::PWM:
				//nop
				break;

			case RmcReg::PWM_TARGET:
				fval = reader.read<float>();
				if(fval.has_value()) driver.at(motor_id).set_pwm(fval.value());
				break;

			case RmcReg::SPD:
				//nop
				break;

			case RmcReg::PWM_LIM:
				fval = reader.read<float>();
				if(fval.has_value()) driver.at(motor_id).set_pwm_limit(fval.value());
				break;

			case RmcReg::SPD_GAIN_P:
				fval = reader.read<float>();
				if(fval.has_value()){
					tmp_gain = driver.at(motor_id).get_speed_gain();
					tmp_gain.kp = fval.value();
					driver.at(motor_id).set_speed_gain(tmp_gain);
				}
				break;

			case RmcReg::SPD_GAIN_I:
				fval = reader.read<float>();
				if(fval.has_value()){
					tmp_gain = driver.at(motor_id).get_speed_gain();
					tmp_gain.ki = fval.value();
					driver.at(motor_id).set_speed_gain(tmp_gain);
				}
				break;

			case RmcReg::SPD_GAIN_D:
				fval = reader.read<float>();
				if(fval.has_value()){
					tmp_gain = driver.at(motor_id).get_speed_gain();
					tmp_gain.kd = fval.value();
					driver.at(motor_id).set_speed_gain(tmp_gain);
				}
				break;

			case RmcReg::POS:
				//nop
				break;
			case RmcReg::SPD_LIM:
				fval = reader.read<float>();
				if(fval.has_value()) driver.at(motor_id).set_speed_limit(fval.value());
				break;
			case RmcReg::POS_TARGET:
				fval = reader.read<float>();
				if(fval.has_value()) driver.at(motor_id).set_target_position(fval.value());
				break;
			case RmcReg::POS_GAIN_P:
				fval = reader.read<float>();
				if(fval.has_value()){
					tmp_gain = driver.at(motor_id).get_position_gain();
					tmp_gain.kp = fval.value();
					driver.at(motor_id).set_speed_gain(tmp_gain);
				}
				break;
			case RmcReg::POS_GAIN_I:
				fval = reader.read<float>();
				if(fval.has_value()){
					tmp_gain = driver.at(motor_id).get_position_gain();
					tmp_gain.ki = fval.value();
					driver.at(motor_id).set_speed_gain(tmp_gain);
				}
				break;
			case RmcReg::POS_GAIN_D:
				fval = reader.read<float>();
				if(fval.has_value()){
					tmp_gain = driver.at(motor_id).get_position_gain();
					tmp_gain.kd = fval.value();
					driver.at(motor_id).set_speed_gain(tmp_gain);
				}
				break;

			case RmcReg::MONITOR_PERIOD:
				break;
			case RmcReg::MONITOR_REG1:
				break;
			case RmcReg::MONITOR_REG2:
				break;
			case RmcReg::MONITOR_REG3:
				break;
			case RmcReg::MONITOR_REG4:
				break;
			default:
				break;
			}
		}
	}
	void execute_common_command(const CommonLib::DataPacket &data){
		CommonReg reg_id = (CommonReg)data.register_ID;

		if(data.is_request){
			switch(reg_id){
			case CommonReg::ID_REQEST:
				break;
			case CommonReg::ID_RESPONSE:
				break;
			case CommonReg::EMERGENCY_STOP:
				break;
			case CommonReg::RESET_EMERGENCY_STOP:
				break;
			default:
				break;
			}
		}else{
			switch(reg_id){
			case CommonReg::ID_REQEST:
				break;
			case CommonReg::ID_RESPONSE:
				break;
			case CommonReg::EMERGENCY_STOP:
				break;
			case CommonReg::RESET_EMERGENCY_STOP:
				break;
			default:
				break;
			}
		}
	}
	int monitor_task(void){
		int next_period = 0;
		return next_period;
	}
#ifdef MOTOR_DEBUG
	void test(void){
		for(int i = 0; i < 4; i++){
			driver.at(i).set_control_mode(RmcLib::ControlMode::POSITION_MODE);
			driver.at(i).set_target_position(-0.3f);
			driver.at(i).set_speed_limit(-0.2,0.2);
		}
		HAL_Delay(1000);

		for(int i = 0; i < 4; i++){
			driver.at(i).set_control_mode(RmcLib::ControlMode::POSITION_MODE);
			driver.at(i).set_target_position(0.3f);
			driver.at(i).set_speed_limit(-0.1,0.1);
		}
		HAL_Delay(2000);
	}
#endif
}


