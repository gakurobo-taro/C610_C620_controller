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

		LED_R.out_as_gpio(true);

		for(auto &d:driver){
			d.set_speed_gain({0.5f, 0.2f, 0.0f});
			d.set_position_gain({1.0f, 0.5f, 0.0f});
			d.set_speed_limit(-1.0f,1.0f);
		}
		LED_R.out_as_gpio(false);
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
		for(auto &l:LED){
			HAL_GPIO_WritePin(l.port,l.pin,GPIO_PIN_RESET);
		}
	}

	//メインCANの処理（外部との通信）
	void main_comm_prossess(void){
		int id = read_board_id();
		CommonLib::DataPacket rx_data;
		CommPort data_from = CommPort::NO_DATA;

		if(can_main.rx_available()){
			CommonLib::CanFrame rx_frame;
			can_main.rx(rx_frame);
			CommonLib::DataConvert::decode_can_frame(rx_frame, rx_data);
			data_from = CommPort::CAN_MAIN;
		}else if(usb_cdc.rx_available()){
			CommonLib::SerialData rx_serial;
			CommonLib::CanFrame rx_frame;
			usb_cdc.rx(rx_serial);
			CommonLib::DataConvert::slcan_to_can((char*)rx_serial.data, rx_frame);
			CommonLib::DataConvert::decode_can_frame(rx_frame, rx_data);
			data_from = CommPort::CDC;
		}

		if(data_from != CommPort::NO_DATA && id == rx_data.board_ID && rx_data.data_type == CommonLib::DataType::RMC_DATA){
			if(rx_data.is_request){
				LED_R.out_as_gpio(true);
				CommonLib::DataPacket tx_data;
				if(read_rmc_command(rx_data,tx_data)){

					CommonLib::SerialData tx_serial;
					CommonLib::CanFrame tx_frame;
					switch(data_from){
					case CommPort::NO_DATA:
						//nop
						break;
					case CommPort::CAN_MAIN:
						CommonLib::DataConvert::encode_can_frame(tx_data,tx_frame);
						can_main.tx(tx_frame);
						break;
					case CommPort::CDC:
						CommonLib::DataConvert::encode_can_frame(tx_data,tx_frame);
						tx_serial.size = CommonLib::DataConvert::can_to_slcan(tx_frame,(char*)tx_serial.data,tx_serial.max_size);
						usb_cdc.tx(tx_serial);
					}
				}
			}else{
				LED_G.out_as_gpio(true);
				write_rmc_command(rx_data);
			}
		}else if(data_from != CommPort::NO_DATA && id == rx_data.board_ID && rx_data.data_type == CommonLib::DataType::COMMON_DATA){
			execute_common_command(rx_data);
		}else if(data_from != CommPort::NO_DATA && rx_data.data_type == CommonLib::DataType::COMMON_DATA){
			execute_common_command(rx_data);
		}

	}

	bool write_rmc_command(const CommonLib::DataPacket &data){
		RmcReg reg_id = (RmcReg)(data.register_ID & 0xFF);
		uint16_t motor_id = data.register_ID >> 8;

		CommonLib::ByteReader reader = data.reader();
		std::optional<float> fval;
		std::optional<uint8_t> u8val;
		std::optional<uint16_t> u16val;
		std::optional<uint64_t> u64val;
		RmcLib::PIDGain tmp_gain;

		switch(reg_id){
		case RmcReg::NOP:
			//nop
			return false;
			break;

		case RmcReg::MOTOR_TYPE:
			return false;
			break;

		case RmcReg::CONTROL_TYPE:
			u8val = reader.read<uint8_t>();
			if(u8val.has_value()){
				driver.at(motor_id).set_control_mode((RmcLib::ControlMode)u8val.value());
			}else{
				return false;
			}
			break;

		case RmcReg::GEAR_RATIO:
			fval = reader.read<float>();
			if(fval.has_value()){
				motor_state.at(motor_id).set_gear_ratio(fval.has_value());
			}else{
				return false;
			}
			break;

		case RmcReg::MOTOR_STATE:
			return false;
			break;

		case RmcReg::PWM:
			//nop
			return false;
			break;

		case RmcReg::PWM_TARGET:
			fval = reader.read<float>();
			if(fval.has_value()){
				driver.at(motor_id).set_pwm(fval.value());
			}else{
				return false;
			}
			break;

		case RmcReg::SPD:
			//nop
			break;

		case RmcReg::SPD_TARGET:
			fval = reader.read<float>();
			if(fval.has_value()){
				driver.at(motor_id).set_target_speed(fval.value());
			}else{
				return false;
			}
			break;

		case RmcReg::PWM_LIM:
			fval = reader.read<float>();
			if(fval.has_value()){
				driver.at(motor_id).set_pwm_limit(fval.value());
			}else{
				return false;
			}
			break;

		case RmcReg::SPD_GAIN_P:
			fval = reader.read<float>();
			if(fval.has_value()){
				tmp_gain = driver.at(motor_id).get_speed_gain();
				tmp_gain.kp = fval.value();
				driver.at(motor_id).set_speed_gain(tmp_gain);
			}else{

			}
			break;

		case RmcReg::SPD_GAIN_I:
			fval = reader.read<float>();
			if(fval.has_value()){
				tmp_gain = driver.at(motor_id).get_speed_gain();
				tmp_gain.ki = fval.value();
				driver.at(motor_id).set_speed_gain(tmp_gain);
			}else{
				return false;
			}
			break;

		case RmcReg::SPD_GAIN_D:
			fval = reader.read<float>();
			if(fval.has_value()){
				tmp_gain = driver.at(motor_id).get_speed_gain();
				tmp_gain.kd = fval.value();
				driver.at(motor_id).set_speed_gain(tmp_gain);
			}else{
				return false;
			}
			break;

		case RmcReg::POS:
			fval = reader.read<float>();
			if(fval.has_value()){
				float current_angle = driver.at(motor_id).get_current_low_position();
				driver.at(motor_id).set_origin(current_angle - fval.value());
			}else{
				return false;
			}
			break;
		case RmcReg::POS_TARGET:
			fval = reader.read<float>();
			if(fval.has_value()){
				driver.at(motor_id).set_target_position(fval.value());
			}else {
				return false;
			}
			break;
		case RmcReg::SPD_LIM:
			fval = reader.read<float>();
			if(fval.has_value()){
				driver.at(motor_id).set_speed_limit(fval.value());
			}else{
				return false;
			}
			break;
		case RmcReg::POS_GAIN_P:
			fval = reader.read<float>();
			if(fval.has_value()){
				tmp_gain = driver.at(motor_id).get_position_gain();
				tmp_gain.kp = fval.value();
				driver.at(motor_id).set_position_gain(tmp_gain);
			}else{
				return false;
			}
			break;
		case RmcReg::POS_GAIN_I:
			fval = reader.read<float>();
			if(fval.has_value()){
				tmp_gain = driver.at(motor_id).get_position_gain();
				tmp_gain.ki = fval.value();
				driver.at(motor_id).set_position_gain(tmp_gain);
			}else{
				return false;
			}
			break;
		case RmcReg::POS_GAIN_D:
			fval = reader.read<float>();
			if(fval.has_value()){
				tmp_gain = driver.at(motor_id).get_position_gain();
				tmp_gain.kd = fval.value();
				driver.at(motor_id).set_position_gain(tmp_gain);
			}else{
				return false;
			}
			break;

		case RmcReg::MONITOR_PERIOD:
			u16val = reader.read<uint16_t>();
			if(u16val.has_value()){
				if(u16val.value() == 0){
					monitor_enable = false;
				}else{
					monitor_enable = true;

					__HAL_TIM_SET_AUTORELOAD(&htim13,u16val.value()*2);
					__HAL_TIM_SET_COUNTER(&htim13,0);
				}
			}
			break;
		case RmcReg::MONITOR_REG:
			u64val = reader.read<uint64_t>();
			if(u64val.has_value()){
				monitor.at(motor_id) = std::bitset<0x35>(u64val.value());
			}
			break;
		default:
			return false;
			break;
		}
		return true;
	}

	bool read_rmc_command(const CommonLib::DataPacket &data,CommonLib::DataPacket &return_data){
		CommonLib::ByteWriter writer = return_data.writer();
		RmcReg reg_id = (RmcReg)(data.register_ID & 0xFF);
		uint16_t motor_id = data.register_ID >> 8;

		return_data = data;
		return_data.is_request = false;

		switch(reg_id){
		case RmcReg::NOP:
			//nop
			return false;
			break;
		case RmcReg::MOTOR_TYPE:
			//TODO:PWMの最大値が変わるが正直現状誤差なので放置しちゃった
			return false;
			break;
		case RmcReg::CONTROL_TYPE:
			writer.write((uint8_t)driver.at(motor_id).get_control_mode());
			break;
		case RmcReg::GEAR_RATIO:
			writer.write(motor_state.at(motor_id).get_gear_ratio());
			break;
		case RmcReg::MOTOR_STATE:
			//TODO::実装
			return false;
			break;
		case RmcReg::PWM:
			writer.write(driver.at(motor_id).get_pwm());
			break;
		case RmcReg::PWM_TARGET:
			writer.write(driver.at(motor_id).get_pwm());
			break;
		case RmcReg::SPD:
			writer.write(driver.at(motor_id).get_current_speed());
			break;
		case RmcReg::SPD_TARGET:
			writer.write(driver.at(motor_id).get_target_speed());
			break;
		case RmcReg::PWM_LIM:
			//TODO:実装　消してもいいかもしれん
			return false;
			break;
		case RmcReg::SPD_GAIN_P:
			writer.write<float>(driver.at(motor_id).get_speed_gain().kp);
			break;
		case RmcReg::SPD_GAIN_I:
			writer.write<float>(driver.at(motor_id).get_speed_gain().ki);
			break;
		case RmcReg::SPD_GAIN_D:
			writer.write<float>(driver.at(motor_id).get_speed_gain().kd);
			break;
		case RmcReg::POS:
			writer.write(driver.at(motor_id).get_current_position());
			break;
		case RmcReg::POS_TARGET:
			writer.write(driver.at(motor_id).get_target_position());
			break;
		case RmcReg::SPD_LIM:
			//TODO:実装　消してもいいかもしれん
			return false;
			break;
		case RmcReg::POS_GAIN_P:
			writer.write<float>(driver.at(motor_id).get_position_gain().kp);
			break;
		case RmcReg::POS_GAIN_I:
			writer.write<float>(driver.at(motor_id).get_position_gain().ki);
			break;
		case RmcReg::POS_GAIN_D:
			writer.write<float>(driver.at(motor_id).get_position_gain().kd);
			break;
		case RmcReg::MONITOR_PERIOD:
			if(monitor_enable){
				writer.write<uint16_t>(__HAL_TIM_GET_AUTORELOAD(&htim13)/2);
			}else{
				writer.write<uint16_t>(0);
			}
			break;
		case RmcReg::MONITOR_REG:
			writer.write<uint64_t>(monitor.at(motor_id).to_ullong());
			break;
		default:
			return false;
			break;
		}
		return true;
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

	void monitor_task(void){
		for(size_t motor_n = 0; motor_n < MOTOR_N; motor_n++){
			for(size_t reg_n = 0; reg_n < monitor[motor_n].size(); reg_n ++){
				if(monitor[motor_n].test(reg_n)){
					CommonLib::DataPacket tmp_packet;
					CommonLib::DataPacket tx_packet;
					CommonLib::CanFrame tx_frame;

					tmp_packet.board_ID = read_board_id();
					tmp_packet.data_type = CommonLib::DataType::RMC_DATA;
					tmp_packet.register_ID = (motor_n << 8) | reg_n;

					if(read_rmc_command(tmp_packet,tx_packet)){
						CommonLib::DataConvert::encode_can_frame(tx_packet, tx_frame);
						can_main.tx(tx_frame);
					}
				}
			}
		}
	}


#ifdef MOTOR_DEBUG
	void motor_test(void){
		for(auto &d:driver){
			d.set_control_mode(RmcLib::ControlMode::POSITION_MODE);
			d.set_target_position(-0.3f);
			d.set_speed_limit(-0.2,0.2);

//			driver.at(i).set_control_mode(RmcLib::ControlMode::PWM_MODE);
//			driver.at(i).set_pwm(0.1);
		}
		HAL_Delay(1000);

		for(auto &d:driver){
			d.set_control_mode(RmcLib::ControlMode::POSITION_MODE);
			d.set_target_position(0.3f);
			d.set_speed_limit(-0.1,0.1);

//			d.set_control_mode(RmcLib::ControlMode::PWM_MODE);
//			d.set_pwm(0.1);
		}
		HAL_Delay(2000);
	}
#endif
}


