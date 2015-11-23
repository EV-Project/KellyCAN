
#include "KellyCAN.h"


/*
typedef struct CAN_message_t {
  uint32_t id; // can identifier
  uint8_t ext; // identifier is extended
  uint8_t len; // length of data
  uint16_t timeout; // milliseconds, zero will disable waiting
  uint8_t buf[8];
} CAN_message_t;
*/

KellyCAN::KellyCAN(CANcallbacks *kellycanbus, uint16_t kellyrequestID, uint16_t kellyresponseID){
	canbus = kellycanbus;
	requestID = kellyrequestID;
	responseID = kellyresponseID;
	module_name[8]='\0';
	module_ver[3]='\0';
	response_pending = 0;
	request_time = 0;
	timeout = 1000;
    processError = false;

}

void KellyCAN::checktimeout(){
	if(micros() - request_time >= timeout){
		response_pending = 2;	//timed out
	}
}

bool KellyCAN::request(CAN_message_t &message){
	checktimeout();
	if(response_pending != 1){	//the kelly is stateful. don't confuse it.
		if(message.id == requestID){	//messages to other devices should bypass the kelly lib
			if(canbus->transmit(message)){
				memcpy(&outgoing, &message, sizeof(CAN_message_t));
				response_pending = 1;
				request_time = micros();
				return true;
			}
		}
	}
	return false;
}


int KellyCAN::dataReady(){
	return response_pending;
}


void KellyCAN::processMessage(CAN_message_t &message){
	response_pending = 0;  //kelly state closed ready for next message.
	processError = false;
	switch(outgoing.buf[0]){
		case CCP_FLASH_READ:
		switch(outgoing.buf[1]){
			case INFO_MODULE_NAME:
			if(message.len == 8){
				memcpy(module_name, message.buf, 8);
			}else{
				processError = true;
			}
			break;
			case INFO_SOFTWARE_VER:
			if(message.len == 2){
				memcpy(module_ver, message.buf, 2);
			}else{
				processError = true;
			}
			break;
			case CAL_TPS_DEAD_ZONE_LOW:
				throttle_deadzone_low = message.buf[0];
			break;
			case CAL_TPS_DEAD_ZONE_HIGH:
				throttle_deadzone_high = message.buf[0];
			break;
			case CAL_BRAKE_DEAD_ZONE_LOW:
				brake_deadzone_low = message.buf[0];
			break;
			case CAL_BRAKE_DEAD_ZONE_HIGH:
				brake_deadzone_high = message.buf[0];
			break;
			default:
				processError = true;
		}
		break;
		case CCP_A2D_BATCH_READ1:
			if(message.len == 5){
				brake_pot = message.buf[0];
        		throttle_pot = message.buf[1];
        		operation_voltage = message.buf[2];
        		signal_voltage = message.buf[3];
        		battery_voltage = message.buf[4];
			}else{
				processError = true;
			}
		break;
		case CCP_A2D_BATCH_READ2:
			if(message.len == 6){
				current_A = message.buf[0];
				current_B = message.buf[1];
				current_C = message.buf[2];
				voltage_A = message.buf[3];
				voltage_B = message.buf[4];
				voltage_C = message.buf[5];
			}else{
				processError = true;
			}
		break;
		case CCP_MONITOR1:
			if(message.len == 6){
				pwm_output = message.buf[0];
				motor_enable = message.buf[1];
				motor_temp = message.buf[2];
				kelly_temp = message.buf[3];
				hfet_temp = message.buf[4];
				lfet_temp = message.buf[5];
			}else{
				processError = true;
			}
		break;
		case CCP_MONITOR2:
			//if(message.len == 3){
			if(message.len == 5){	//the datasheet lies!
				mech_rpm = (((uint16_t)(message.buf[0]))<<8) + ((uint16_t)(message.buf[1]));
				current_pc = message.buf[2];

			}else{
				processError = true;
			}
		break;
		case COM_SW_ACC:
			throttle_switch = message.buf[0] == 1;
		break;
		case COM_SW_BRK:
			brake_switch = message.buf[0] == 1;
		break;
		case COM_SW_REV:
			reverse_switch = message.buf[0] == 1;
		break;
		default:
			processError = true;
	}
}

bool KellyCAN::get_process_error(){
	return processError;
}		
bool KellyCAN::get_intercepted(){
	return response_pending == 0;
}
bool KellyCAN::get_waiting(){
	checktimeout();
	return response_pending == 1;
}
bool KellyCAN::get_timed_out(){
	checktimeout();
	return response_pending == 2;
}


//ccp_flash_read
char* KellyCAN::get_module_name(){
	return module_name;
}
char* KellyCAN::get_module_ver(){
	return module_ver;
}
uint8_t KellyCAN::get_throttle_deadzone_low(){
	return throttle_deadzone_low;
}
uint8_t KellyCAN::get_throttle_deadzone_high(){
	return throttle_deadzone_high;
}
uint8_t KellyCAN::get_brake_deadzone_low(){
	return brake_deadzone_low;
}
uint8_t KellyCAN::get_brake_deadzone_high(){
	return brake_deadzone_high;
}
//CCP_A2D_BATCH_READ1
uint8_t KellyCAN::get_brake_pot(){
	return brake_pot;
}
uint8_t KellyCAN::get_throttle_pot(){
	return throttle_pot;
}
uint8_t KellyCAN::get_operation_voltage(){
	return operation_voltage;
}
uint8_t	KellyCAN::get_signal_voltage(){
	return signal_voltage;
}
uint8_t KellyCAN::get_battery_voltage(){
	return battery_voltage;
}
//CCP_A2D_BATCH_READ2
uint8_t KellyCAN::get_current_A(){
	return current_A;
}
uint8_t KellyCAN::get_current_B(){
	return current_B;
}
uint8_t KellyCAN::get_current_C(){
	return current_C;
}
uint8_t KellyCAN::get_voltage_A(){
	return voltage_A;
}
uint8_t KellyCAN::get_voltage_B(){
	return voltage_B;
}
uint8_t KellyCAN::get_voltage_C(){
	return voltage_C;
}
//ccp_monitor1
uint8_t KellyCAN::get_pwm_output(){
	return pwm_output;
}
bool KellyCAN::get_motor_enable(){
	return motor_enable;
}
uint8_t KellyCAN::get_motor_temp(){
	return motor_temp;
}
uint8_t KellyCAN::get_kelly_temp(){
	return kelly_temp;
}
uint8_t KellyCAN::get_hfet_temp(){
	return hfet_temp;
}
uint8_t KellyCAN::get_lfet_temp(){
	return lfet_temp;
}
//ccp_monitor2
uint16_t KellyCAN::get_mech_rpm(){
	return mech_rpm;
}
uint8_t KellyCAN::get_current_pc(){
	return current_pc;
}
//com_sw_*
bool KellyCAN::get_throttle_switch(){
	return throttle_switch;
}
bool KellyCAN::get_brake_switch(){
	return brake_switch;
}
bool KellyCAN::get_reverse_switch(){
	return reverse_switch;
}


