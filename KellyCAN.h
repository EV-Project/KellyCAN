#ifndef KELLYCAN_H
#define KELLYCAN_H
#include <inttypes.h>
#include <Arduino.h> // Arduino 1.0

#include <FlexCAN.h>
#include <CANcallbacks.h>

#define DEF_REQUEST_ID 107
#define DEF_RESPONSE_ID 115
//main command headings
#define CCP_FLASH_READ 0xF2
#define CCP_A2D_BATCH_READ1 0x1b
#define CCP_A2D_BATCH_READ2 0x1a
#define CCP_MONITOR1 0x33
#define CCP_MONITOR2 0x37
#define COM_SW_ACC 0x42
#define COM_SW_BRK 0x43
#define COM_SW_REV 0x44

//command subheadings
	//ccp_flash_read appears to just be an offset into memory.
#define INFO_MODULE_NAME 64
#define INFO_SOFTWARE_VER 83
#define CAL_TPS_DEAD_ZONE_LOW 4
#define CAL_TPS_DEAD_ZONE_HIGH 5
#define CAL_BRAKE_DEAD_ZONE_LOW 38
#define CAL_BRAKE_DEAD_ZONE_HIGH 39
    //com_sw_*
#define COM_READING 0 //not used




class KellyCAN{
    public:
        KellyCAN(CANcallbacks *kellycanbus, uint16_t kellyrequestID, uint16_t kellyresponseID);

        bool request(CAN_message_t &message);
        //bool receive(CAN_message_t &message); //wrapper that intercepts response packets
        void processMessage(CAN_message_t &message);
        int dataReady();

        //ccp_flash_read
        char* get_module_name();
        char* get_module_ver();
        uint8_t get_throttle_deadzone_low();
        uint8_t get_throttle_deadzone_high();
        uint8_t get_brake_deadzone_low();
        uint8_t get_brake_deadzone_high();

        //CCP_A2D_BATCH_READ1
        uint8_t get_brake_pot();
        uint8_t get_throttle_pot();
        uint8_t get_operation_voltage();
        uint8_t get_signal_voltage();
        uint8_t get_battery_voltage();
        
        //CCP_A2D_BATCH_READ2
        uint8_t get_current_A();
        uint8_t get_current_B();
        uint8_t get_current_C();
        uint8_t get_voltage_A();
        uint8_t get_voltage_B();
        uint8_t get_voltage_C();

        //ccp_monitor1
        uint8_t get_pwm_output();
        bool get_motor_enable();
        uint8_t get_motor_temp();
        uint8_t get_kelly_temp();
        uint8_t get_hfet_temp();
        uint8_t get_lfet_temp();

        //ccp_monitor2
        uint16_t get_mech_rpm();
        uint8_t get_current_pc();

        //com_sw_*
        bool get_throttle_switch();
        bool get_brake_switch();
        bool get_reverse_switch();


        bool get_process_error();
        bool get_intercepted();
        bool get_timed_out();
        bool get_waiting();

    private:
        CANcallbacks *canbus;
        uint16_t requestID;// = DEF_REQUEST_ID; //sane defaults
        uint16_t responseID;// = DEF_RESPONSE_ID;

        int response_pending;// = 0;
        uint32_t request_time;// = 0;
        uint32_t timeout;// = 10000;
        CAN_message_t outgoing;// = {DEF_REQUEST_ID,{0,3},0xF2,64,8,0,0,0,0,0}; //the message that went out last.
        void checktimeout();

        bool processError;// = false;


		//ccp_flash_read
        char module_name[9];
        char module_ver[3];
		uint8_t throttle_deadzone_low;
        uint8_t throttle_deadzone_high;
		uint8_t brake_deadzone_low;
        uint8_t brake_deadzone_high;

		//CCP_A2D_BATCH_READ1
        uint8_t brake_pot;
        uint8_t throttle_pot;
        uint8_t operation_voltage;
        uint8_t	signal_voltage;
        uint8_t battery_voltage;        
        
		//CCP_A2D_BATCH_READ2
        uint8_t current_A;
        uint8_t current_B;
        uint8_t current_C;
        uint8_t voltage_A;
		uint8_t voltage_B;
		uint8_t voltage_C;

        //ccp_monitor1
        uint8_t pwm_output;
        bool motor_enable;
        uint8_t motor_temp;
        uint8_t kelly_temp;
        uint8_t hfet_temp;
        uint8_t lfet_temp;

        //ccp_monitor2
        uint16_t mech_rpm;
        uint8_t current_pc;

        //com_sw_*
        bool throttle_switch;
        bool brake_switch;
        bool reverse_switch;

};


#endif