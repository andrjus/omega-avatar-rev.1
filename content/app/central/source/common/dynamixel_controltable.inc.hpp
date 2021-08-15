#include "core/robosd_frontend.hpp"


#define RCTR(name,type, address, len) { #name, type , address, len}

#define RCTR_U(name,address,length) RCTR(name, RCTR_U_##length##_s, address, length)

namespace robo{
	namespace backend{
		namespace dynamixel{
			robo::cstr none_converter = RT("");

			robo::cstr RCTR_U_1_s = frontend::type_names.u8;
			robo::cstr RCTR_U_2_s = frontend::type_names.u16;
			robo::cstr RCTR_U_4_s = frontend::type_names.u32;
			
			
			const frontend::contrltable::record  records_xm[] = {
									
				RCTR_U( MODEL_NUMBER,							0,	2)
				,RCTR_U( MODEL_INFORMATION,				2,	4)
				,RCTR_U( FIRMWARE_VERSION,					6,	1)
				,RCTR_U( ID,												7,	1)
				,RCTR_U( BAUD_RATE,								8,	1)
				,RCTR_U( RETURN_DELAY_TIME,				9,	1)
				,RCTR_U( DRIVE_MODE,								10,	1)
				,RCTR_U( OPERATING_MODE,						11,	1)
				,RCTR_U( SECONDARY_ID,							12,	1)
				,RCTR_U( PROTOCOL_VERSION,					13,	1)
				,RCTR_U( HOMING_OFFSET,						20,	4)
				,RCTR_U( MOVING_THRESHOLD,				24,	4)
				,RCTR_U( TEMPERATURE_LIMIT,				31,	1)
				,RCTR_U( MAX_VOLTAGE_LIMIT,				32,	2)
				,RCTR_U( MIN_VOLTAGE_LIMIT,				34,	2)
				,RCTR_U( PWM_LIMIT,								36,	2)
				,RCTR_U( CURRENT_LIMIT,						38,	2)
				,RCTR_U( VELOCITY_LIMIT,					44,	4)
				,RCTR_U( MAX_POSITION_LIMIT,			48,	4)
				,RCTR_U( MIN_POSITION_LIMIT,			52,	4)
				,RCTR_U( SHUTDOWN,								63,	1)
				,RCTR_U( TORQUE_ENABLE,						64,	1)

				,RCTR_U( LED,                     65, 1)
				,RCTR_U( STATUS_RETURN_LEVEL,     68, 1)
				,RCTR_U( REGISTERED_INSTRUCTION,  69, 1)
				,RCTR_U( HARDWARE_ERROR_STATUS,   70, 1)
				,RCTR_U( VELOCITY_I_GAIN,         76, 2)
				,RCTR_U( VELOCITY_P_GAIN,         78, 2)
				,RCTR_U( POSITION_D_GAIN,         80, 2)
				,RCTR_U( POSITION_I_GAIN,         82, 2)
				,RCTR_U( POSITION_P_GAIN,         84, 2)
				,RCTR_U( FEEDFORWARD_2ND_GAIN,    88, 2)
				,RCTR_U( FEEDFORWARD_1ST_GAIN,    90, 2)  
				,RCTR_U( BUS_WATCHDOG,            98, 2)
				,RCTR_U( GOAL_PWM,               100, 2)
				,RCTR_U( GOAL_VELOCITY,          104, 4)
				,RCTR_U( PROFILE_ACCELERATION,   108, 4)
				,RCTR_U( PROFILE_VELOCITY,       112, 4)
				,RCTR_U( GOAL_POSITION,          116, 4)
				,RCTR_U( REALTIME_TICK,          120, 2)
				,RCTR_U( MOVING,                 122, 1)    
				,RCTR_U( MOVING_STATUS,          123, 1)    
				,RCTR_U( PRESENT_PWM,            124, 2)
				,RCTR_U( PRESENT_CURRENT,        126, 2)
				,RCTR_U( PRESENT_VELOCITY,       128, 4)
				,RCTR_U( PRESENT_POSITION,       132, 4)
				,RCTR_U( VELOCITY_TRAJECTORY,    136, 4)
				,RCTR_U( POSITION_TRAJECTORY,    140, 4)  
				,RCTR_U( PRESENT_INPUT_VOLTAGE,  144, 2)
				,RCTR_U( PRESENT_TEMPERATURE,    146, 1)
				,{ RT("PRESENT"), RT("snapshot") , 124, 12}
			};
			
			const frontend::contrltable::record  records_XL320[] = {
				RCTR_U( MODEL_NUMBER,           0, 2)
				,RCTR_U( FIRMWARE_VERSION,       2, 1)
				,RCTR_U( ID,                     3, 1)
				,RCTR_U( BAUD_RATE,              4, 1)
				,RCTR_U( RETURN_DELAY_TIME,      5, 1)
				,RCTR_U( CW_ANGLE_LIMIT,         6, 2)
				,RCTR_U( CCW_ANGLE_LIMIT,        8, 2)
				,RCTR_U( CONTROL_MODE,          11, 1)
				,RCTR_U( TEMPERATURE_LIMIT,     12, 1)
				,RCTR_U( MIN_VOLTAGE_LIMIT,     13, 1)
				,RCTR_U( MAX_VOLTAGE_LIMIT,     14, 1)
				,RCTR_U( MAX_TORQUE,            15, 2)
				,RCTR_U( STATUS_RETURN_LEVEL,   17, 1)
				,RCTR_U( SHUTDOWN,              18, 1)

				,RCTR_U( TORQUE_ENABLE,         24, 1)
				,RCTR_U( LED,                   25, 1)
				,RCTR_U( D_GAIN,                27, 1)
				,RCTR_U( I_GAIN,                28, 1)
				,RCTR_U( P_GAIN,                29, 1)
				,RCTR_U( GOAL_POSITION,         30, 2)
				,RCTR_U( MOVING_SPEED,          32, 2)
				,RCTR_U( TORQUE_LIMIT,          35, 2)
				,RCTR_U( PRESENT_POSITION,      37, 2)  
				,RCTR_U( PRESENT_SPEED,         39, 2)
				,RCTR_U( PRESENT_LOAD,          41, 2)
				,RCTR_U( PRESENT_VOLTAGE,       45, 1)
				,RCTR_U( PRESENT_TEMPERATURE,   46, 1)
				,RCTR_U( REGISTERED,            47, 1)
				,RCTR_U( MOVING,                49, 1)    
				,RCTR_U( HARDWARE_ERROR_STATUS, 50, 1)
				,RCTR_U( PUNCH,                 51, 2)
				,{ RT("PRESENT"), RT("snapshot") , 37, 8}
			};			
		}
	}
}
