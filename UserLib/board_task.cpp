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

		for(auto &d:driver){
			d.set_speed_gain({0.5f, 0.2f, 0.0f});
			d.set_position_gain({1.0f, 0.5f, 0.0f});
			d.set_speed_limit(-1.0f,1.0f);
		}
	}

	//受信したモーター情報の処理
	void motor_data_process(void){
		if(can_c6x0.rx_available()){
			CommonLib::CanFrame rx_frame;
			can_c6x0.rx(rx_frame);

			if(rx_frame.id & 0x200){
				const size_t id = (rx_frame.id&0xF)-1;
				if(MOTOR_N<=id){
					return;
				}

				motor_state[id].update(rx_frame);
				driver[id].update_operation_val(motor_state[id],abs_enc[id]);

				if(!LED[id].is_playing()){
					LED[id].play(RmcLib::LEDPattern::led_mode.at((uint8_t)driver[id].get_control_mode()));
				}
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
	}

	//メインCANの処理（外部との通信）
	void main_comm_prossess(void){

		const int board_id = read_board_id();
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

		if(data_from != CommPort::NO_DATA && board_id == rx_data.board_ID && rx_data.data_type == CommonLib::DataType::RMC_DATA){
			execute_rmc_command(board_id,rx_data,data_from);
		}else if((data_from != CommPort::NO_DATA && board_id == rx_data.board_ID && rx_data.data_type == CommonLib::DataType::COMMON_DATA)
				||(data_from != CommPort::NO_DATA && rx_data.data_type == CommonLib::DataType::COMMON_DATA_ENFORCE)){
			execute_common_command(board_id,rx_data,data_from);
		}
	}

	void execute_rmc_command(size_t board_id,const CommonLib::DataPacket &rx_data,CommPort data_from){
		const size_t motor_n = (rx_data.register_ID>>8)&0xFF;
		const size_t reg_id = rx_data.register_ID & 0xFF;

		if(MOTOR_N <= motor_n){
			return;
		}

		if(rx_data.is_request){
			CommonLib::DataPacket tx_data;
			auto writer = tx_data.writer();

			if(id_map[motor_n].get(reg_id, writer)){
				CommonLib::SerialData tx_serial;
				CommonLib::CanFrame tx_frame;

				tx_data.board_ID = board_id;
				tx_data.register_ID = rx_data.register_ID;
				tx_data.data_type = CommonLib::DataType::RMC_DATA;
				tx_data.priority = rx_data.priority;

				switch(data_from){
				case CommPort::NO_DATA:
					//nop
					break;
				case CommPort::CAN_MAIN:
					CommonLib::DataConvert::encode_can_frame(tx_data,tx_frame);
					can_main.tx(tx_frame);
					break;
				case CommPort::CAN_SUB:
					//nop
					break;
				case CommPort::CDC:
					CommonLib::DataConvert::encode_can_frame(tx_data,tx_frame);
					tx_serial.size = CommonLib::DataConvert::can_to_slcan(tx_frame,(char*)tx_serial.data,tx_serial.max_size);
					usb_cdc.tx(tx_serial);
				}
			}
		}else{
			auto reader = rx_data.reader();
			id_map[motor_n].set(reg_id, reader);
		}
	}

	void execute_common_command(size_t board_id,const CommonLib::DataPacket &rx_data,CommPort data_from){
		CommonLib::DataPacket tx_data;
		CommonLib::CanFrame tx_frame;
		CommonLib::SerialData tx_serial;

		switch((CommonReg)rx_data.register_ID){
		case CommonReg::NOP:
			break;
		case CommonReg::ID_REQEST:
			if(rx_data.is_request){
				tx_data.board_ID = board_id;
				tx_data.data_type = CommonLib::DataType::COMMON_DATA;
				tx_data.register_ID = (uint16_t)CommonReg::ID_REQEST;
				tx_data.writer().write<uint8_t>((uint8_t)CommonLib::DataType::RMC_DATA);
				tx_data.priority = rx_data.priority;

				switch(data_from){
				case CommPort::NO_DATA:
					//nop
					break;
				case CommPort::CAN_MAIN:
					CommonLib::DataConvert::encode_can_frame(tx_data,tx_frame);
					can_main.tx(tx_frame);
					break;
				case CommPort::CAN_SUB:
					//nop
					break;
				case CommPort::CDC:
					CommonLib::DataConvert::encode_can_frame(tx_data,tx_frame);
					tx_serial.size = CommonLib::DataConvert::can_to_slcan(tx_frame,(char*)tx_serial.data,tx_serial.max_size);
					usb_cdc.tx(tx_serial);
				}
			}
			break;
		case CommonReg::EMERGENCY_STOP:
			emergency_stop_sequence();
			break;
		case CommonReg::RESET_EMERGENCY_STOP:
			emergency_stop_release_sequence();
			break;
		default:
			break;
		}
	}

	void monitor_task(void){
		for(size_t motor_n = 0; motor_n < MOTOR_N; motor_n++){
			for(auto &map_element : id_map[motor_n].accessors_map){
				if(map_element.first < monitor[motor_n].size()){
					if(monitor[motor_n].test(map_element.first)){
						CommonLib::DataPacket tx_packet;
						CommonLib::CanFrame tx_frame;
						tx_packet.register_ID = map_element.first | (motor_n << 8);
						tx_packet.board_ID = read_board_id();
						tx_packet.data_type = CommonLib::DataType::RMC_DATA;

						auto writer = tx_packet.writer();
						if(map_element.second.get(writer)){
							CommonLib::DataConvert::encode_can_frame(tx_packet, tx_frame);
							can_main.tx(tx_frame);
						}
					}
				}
			}
		}
	}

	void emergency_stop_sequence(void){
		for(size_t i = 0; i < MOTOR_N; i++){
			control_mode_tmp.at(i) = driver.at(i).get_control_mode();
			driver.at(i).set_control_mode(RmcLib::ControlMode::PWM_MODE);
		}
		RmcBoard::LED_R.play(RmcLib::LEDPattern::error);
	}
	void emergency_stop_release_sequence(void){
		for(size_t i = 0; i < MOTOR_N; i++){
			driver.at(i).set_control_mode(control_mode_tmp.at(i));
		}
	}


#ifdef MOTOR_DEBUG
	void motor_test(void){
		for(auto &d:driver){
//			d.set_control_mode(RmcLib::ControlMode::POSITION_MODE);
//			d.set_target_position(-0.3f);
//			d.set_speed_limit(-0.2,0.2);

			d.set_control_mode(RmcLib::ControlMode::PWM_MODE);
			d.set_pwm(0.05);
		}
		HAL_Delay(2000);

		for(auto &d:driver){
//			d.set_control_mode(RmcLib::ControlMode::POSITION_MODE);
//			d.set_target_position(0.3f);
//			d.set_speed_limit(-0.1,0.1);

			d.set_control_mode(RmcLib::ControlMode::PWM_MODE);
			d.set_pwm(-0.05);
		}
		HAL_Delay(2000);
	}
#endif
}


