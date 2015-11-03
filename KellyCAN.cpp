
#include "KellyCAN.h"


KellyCAN::KellyCAN(CanBus* kellycanbus, uint16_t kellyrequestID, uint16_t kellyresponseID){
	canbus = kellycanbus;
	requestID = kellyrequestID;
	responseID = kellyresponseID;
}


bool KellyCAN::request(tCAN* message){
	if(micros() - request_time >= timeout){
		response_pending = 2;	//timed out
	}
	if(response_pending != 1){	//the kelly is stateful. don't confuse it.
		if(message->id == requestID){	//messages to other devices should bypass the kelly lib
			if(canbus->transmit(message)){
				memcpy(&outgoing, message, sizeof(tCAN));
				response_pending = 1;
				request_time = micros();
				return true;
			}
		}
	}
	return false;
}


bool KellyCAN::receive(tCAN* message){

	bool retval = false;
	if(canbus->receive(message)){
		if(message->id == responseID){	//it's for us, kelly state closed ready for next message.
			response_pending = 0;
			processMessage(message);  //process the message here
		}else{
			if(micros() - request_time >= timeout){
				response_pending = 2;	//timed out
			}
		}
	retval = true;	//return true for any valid message
	}
	return retval;
}


int KellyCAN::dataReady(){
	return response_pending;
}


void KellyCAN::processMessage(tCAN *message){
	processError = false;
	switch(outgoing.data[0]){
		case CCP_FLASH_READ:
		switch(outgoing.data[1]){
			case INFO_MODULE_NAME:
			if(message->header.length == 8){
				memcpy(module_name, message->data, 8);
			}else{
				processError = true;
			}
			break;
			case INFO_SOFTWARE_VER:
			if(message->header.length == 2){
				memcpy(module_ver, message->data, 2);
			}else{
				processError = true;
			}
			break;
			case CAL_TPS_DEAD_ZONE_LOW:
				throttle_deadzone_low = message->data[0];
			break;
			case CAL_TPS_DEAD_ZONE_HIGH:
				throttle_deadzone_high = message->data[0];
			break;
			case CAL_BRAKE_DEAD_ZONE_LOW:
				brake_deadzone_low = message->data[0];
			break;
			case CAL_BRAKE_DEAD_ZONE_HIGH:
				brake_deadzone_high = message->data[0];
			break;
			default:
				processError = true;
		}
		break;
		case CCP_A2D_BATCH_READ1:
			if(message->header.length == 5){
				brake_pot = message->data[0];
        		throttle_pot = message->data[1];
        		operation_voltage = message->data[2];
        		signal_voltage = message->data[3];
        		battery_voltage = message->data[4];
			}else{
				processError = true;
			}
		break;
		case CCP_A2D_BATCH_READ2:
			if(message->header.length == 6){
				current_A = message->data[0];
				current_B = message->data[1];
				current_C = message->data[2];
				voltage_A = message->data[3];
				voltage_B = message->data[4];
				voltage_C = message->data[5];
			}else{
				processError = true;
			}
		break;
		case CCP_MONITOR1:
			if(message->header.length == 6){
				pwm_output = message->data[0];
				motor_enable = message->data[1];
				motor_temp = message->data[2];
				kelly_temp = message->data[3];
				hfet_temp = message->data[4];
				lfet_temp = message->data[5];
			}else{
				processError = true;
			}
		break;
		case CCP_MONITOR2:
			if(message->header.length == 3){
				mech_rpm = ((uint16_t)message->data[0])<<8 + message->data[1];
				current_pc = message->data[2];
			}else{
				processError = true;
			}
		break;
		case COM_SW_ACC:
			throttle_switch = message->data[0] == 1;
		break;
		case COM_SW_BRK:
			brake_switch = message->data[0] == 1;
		break;
		case COM_SW_REV:
			reverse_switch = message->data[0] == 1;
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
	return response_pending == 1;
}
bool KellyCAN::get_timed_out(){
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






char CanBus::init(){
    return mcp2515_init(CANSPEED_1000);
    n_callbacks = 0;
}
bool CanBus::transmit (tCAN* message){
    mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
    if (mcp2515_send_message(message)) {
    	return true;
	}
	return false;
}
bool CanBus::receive (tCAN* message){
	if (mcp2515_check_message()) 
    {
        if (mcp2515_get_message(message)) 
        {
        	//distribute messages here.
        	for(int i=0; i<n_callbacks; i++){
        		if(message.id == callbackID[i]){
        			(*callback[i])(message);
        		}
        	}
        	return true;
        }
    }
    return false;
}

bool CanBus::set_callback(uint16_t messageID, bool (*new_callback)(tCAN* message)) {
	if(n_callbacks < MAX_CAN_CALLBACKS){
		(*callback)(tCAN* message)[n_callbacks++] = new_callback;
	}

}