 /*
 * board_task.cpp
 *
 *  Created on: Jan 6, 2024
 *      Author: yaa3k
 */

#include "board_task.hpp"

namespace G24_STM32HAL::RmcBoard{

	uint8_t read_board_id(void){
		struct GPIOParam{
			GPIO_TypeDef * port;
			uint16_t pin;
			GPIOParam(GPIO_TypeDef * _port,uint16_t _pin):port(_port),pin(_pin){}
		};
		auto dip_sw = std::array<GPIOParam,4>{
			GPIOParam{ID0_GPIO_Port,ID0_Pin},
			GPIOParam{ID1_GPIO_Port,ID1_Pin},
			GPIOParam{ID2_GPIO_Port,ID2_Pin},
			GPIOParam{ID3_GPIO_Port,ID3_Pin},
		};

		uint8_t id = 0;
		for(int i = 0; i<4; i++){
			id |= !(uint8_t)HAL_GPIO_ReadPin(dip_sw.at(i).port,dip_sw.at(i).pin) << i;
		}
		return id;
	}

	//各クラス起動処理
	void init(void){
		board_id = read_board_id();

		motor_control_timer.set_task([](){
			//通信系
			usb_cdc.tx_interrupt_task();
			send_motor_parameters_to_c6x0();
			send_motor_parameters_to_vesc();

			//LED
			LED_R.update();
			LED_G.update();
			LED_B.update();
			for(auto &m:motor){
				m.led.update();
			}

			//abs enc reading start
			abs_enc_reading_iter = motor.begin();
			abs_enc_reading_iter->abs_enc.read_start();

			//OK
			LED_G.play(RmcLib::LEDPattern::ok);
		});

		monitor_timer.set_task([](){
			monitor_task();
			LED_R.play(RmcLib::LEDPattern::ok);
		});

		can_timeout_timer.set_task(RmcBoard::emergency_stop_sequence);

		can_main.set_filter_mask(16, 0x00200000|(board_id<<16), 0x00FF0000, CommonLib::FilterMode::STD_AND_EXT, true);
		can_main.set_filter_mask(17, 0x00000000|(board_id<<16), 0x00FF0000, CommonLib::FilterMode::STD_AND_EXT, true);
		can_main.set_filter_mask(18, 0x00F00000,0x00F00000, CommonLib::FilterMode::STD_AND_EXT, true);
		can_main.start();

		can_motor.set_filter_free(0);
		can_motor.start();

		LED_R.start();
		LED_G.start();
		LED_B.start();

		for(auto &m:motor){
			m.driver.set_speed_gain({0.5f, 0.2f, 0.0f});
			m.driver.set_position_gain({6.0f, 3.0f, 0.0f});
			m.driver.set_speed_limit(-6.0f,6.0f);

			m.abs_enc.start();
		}
		motor_control_timer.set_and_start(1000);
	}

	//受信したモーター情報の処理
	void motor_data_process(void){
		if(can_motor.rx_available()){
			CommonLib::CanFrame rx_frame;
			can_motor.rx(rx_frame);

			if(rx_frame.id & 0x200){
				const size_t id = (rx_frame.id&0xF)-1;
				if(MOTOR_N<=id){
					return;
				}

				motor[id].motor_enc.update(rx_frame);
				motor[id].driver.operation(motor[id].motor_enc);

				if(!motor[id].led.is_playing()){
					motor[id].led.play(RmcLib::LEDPattern::led_mode.at((uint8_t)motor[id].driver.get_control_mode()));
				}
			}
		}
	}

	//PWM値の送信
	void send_motor_parameters_to_c6x0(void){
		CommonLib::CanFrame to_c6x0_frame;
		to_c6x0_frame.id = 0x200;
		auto writer = to_c6x0_frame.writer();

		for(auto &m:motor){
			m.driver.abs_operation(m.abs_enc);
			int16_t duty = (int16_t)(m.driver.get_pwm() * 10000.0f);
			writer.write<uint8_t>(duty>>8);
			writer.write<uint8_t>(duty&0xFF);
		}

		can_motor.tx(to_c6x0_frame);
	}
	void send_motor_parameters_to_vesc(void){
		for(size_t i = 0; i < MOTOR_N; i++){
			if(motor[i].motor_type == MotorType::VESC){
				CommonLib::CanFrame to_vesc_frame;
				to_vesc_frame.id = i;
				to_vesc_frame.is_ext_id = true;
				to_vesc_frame.is_remote = false;

				int32_t duty = (int32_t)(motor[i].driver.get_pwm()*100000.0f);
				to_vesc_frame.data[0] = (duty >> 24)&0xFF;
				to_vesc_frame.data[1] = (duty >> 16)&0xFF;
				to_vesc_frame.data[2] = (duty >>  8)&0xFF;
				to_vesc_frame.data[3] = (duty      )&0xFF;
				to_vesc_frame.data_length  = 4;
				can_motor.tx(to_vesc_frame);
			}
		}
	}

	//メインCANの処理（外部との通信）
	void main_comm_prossess(void){
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

		for(auto &m:motor){
			if((m.motor_type == MotorType::VESC) && !m.led.is_playing()){
				m.led.play(RmcLib::LEDPattern::vesc_mode);
			}
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
				if((map_element.first < motor[motor_n].monitor.size()) && motor[motor_n].monitor.test(map_element.first)){
					CommonLib::DataPacket tx_packet;
					CommonLib::CanFrame tx_frame;
					tx_packet.register_ID = map_element.first | (motor_n << 8);
					tx_packet.board_ID = board_id;
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

	void emergency_stop_sequence(void){
		for(auto &m:motor){
			m.mode_tmp = m.driver.get_control_mode();
			m.driver.set_control_mode(RmcLib::ControlMode::PWM_MODE);
		}
		RmcBoard::LED_R.play(RmcLib::LEDPattern::error);
	}
	void emergency_stop_release_sequence(void){
		for(auto &m:motor){
			m.driver.set_control_mode(m.mode_tmp);
		}
	}


#ifdef MOTOR_DEBUG
	void motor_test(void){
//		for(auto &m:motor){
//			m.driver.set_control_mode(RmcLib::ControlMode::POSITION_MODE);
//			m.driver.set_target_position(-3.14f);
//			m.driver.set_speed_limit(-5,5);
//
////			m.driver.set_control_mode(RmcLib::ControlMode::PWM_MODE);
////			m.driver.set_pwm(0.05);
//		}
//		HAL_Delay(2000);
//
//		for(auto &m:motor){
//			m.driver.set_control_mode(RmcLib::ControlMode::POSITION_MODE);
//			m.driver.set_target_position(3.14f);
//			m.driver.set_speed_limit(-10,10);
//
////			m.driver.set_control_mode(RmcLib::ControlMode::PWM_MODE);
////			m.driver.set_pwm(-0.05);
//		}
		motor[0].driver.set_control_mode(RmcLib::ControlMode::PWM_MODE);
		motor[1].driver.set_control_mode(RmcLib::ControlMode::POSITION_MODE);
		HAL_Delay(2000);
	}
#endif
}


