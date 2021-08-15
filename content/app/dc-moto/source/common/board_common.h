#ifndef __board_common_h
#define  __board_common_h
#include "__robosd_head_begin.h"

#define BOARD_KIND_YAW_1 0x1
#define BOARD_KIND_PITCH_2 0x2
#define BOARD_KIND_PITCH_3 0x3
#define BOARD_KIND_ROLL_4 0x4
#define BOARD_KIND_PITCH_5 0x5


#define BOARD_MOTOR_TYPE_NONE 0
#define BOARD_MOTOR_TYPE_DCMOTO 1


#define BOARD_MOTOR_CURRENT_SENCE_TYPE_NONE 0
#define BOARD_MOTOR_CURRENT_SENCE_TYPE_ADC 1

#define BOARD_MOTOR_POS_SENCE_TYPE_NONE 0
#define BOARD_MOTOR_POS_SENCE_TYPE_INCREMENT 2


#define BOARD_ACTUATOR_POS_SENCE_TYPE_NONE 0
#define BOARD_ACTUATOR_POS_SENCE_TYPE_ABS_SSI_16 1

#define BOARD_FREE_MASTER_DRIVER_TYPE_NONE 0
#define BOARD_FREE_MASTER_DRIVER_TYPE_PROTO_ABONENT 1
#define BOARD_FREE_MASTER_DRIVER_TYPE_SERIAL_DIRRECT 2
#define BOARD_FREE_MASTER_DRIVER_TYPE_DIRRECT_STREAM0 3

//Конфигурация и состав привода - его назначение,  подключённые датчик и т.д.
//Для каждой версии привода- свои
#include "board_options.h"

//До определяем  пропущенные опции

#ifndef BOARD_MOTOR_TYPE
#define BOARD_MOTOR_TYPE BOARD_MOTOR_TYPE_NONE
#endif

#ifndef BOARD_MOTOR_CURRENT_SENCE_TYPE
#define BOARD_MOTOR_CURRENT_SENCE_TYPE BOARD_MOTOR_CURRENT_SENCE_TYPE_NONE
#endif 

#ifndef BOARD_MOTOR_POS_SENCE_TYPE 
#define BOARD_MOTOR_POS_SENCE_TYPE MOTOR_POS_SENCE_TYPE_NONE
#endif

#ifndef BOARD_ACTUATOR_POS_SENCE_TYPE 
#define BOARD_ACTUATOR_POS_SENCE_TYPE ACTUATOR_POS_SENCE_TYPE_NONE
#endif

#ifndef BOARD_MEXO_NET_FLOW_ENABLED
#define BOARD_MEXO_NET_FLOW_ENABLED 0
#endif

#ifndef BOARD_TERMINAL_ENABLED
#define BOARD_TERMINAL_ENABLED 0
#endif

#ifndef BOARD_NET_PROTO_SWITCH_ENABLED
#define BOARD_NET_PROTO_SWITCH_ENABLED 0
#endif

#ifndef BOARD_FREE_MASTER_DRIVER_TYPE
#define BOARD_FREE_MASTER_DRIVER_TYPE BOARD_FREE_MASTER_DRIVER_TYPE_NONE
#endif 

#ifndef  BOARD_MOTOR_POS_SENCE_TYPE
#defien BOARD_MOTOR_POS_SENCE_TYPE BOARD_MOTOR_POS_SENCE_TYPE_NONE
#endif


#if BOARD_FREE_MASTER_DRIVER_TYPE == BOARD_FREE_MASTER_DRIVER_TYPE_NONE
#define BOARD_FREEMASTER_ENABLED 0
#else
#define BOARD_FREEMASTER_ENABLED 1
#endif


#if BOARD_MOTOR_TYPE == BOARD_MOTOR_TYPE_NONE
#define BOARD_MOTOR_ENABLED 0
#define BOARD_MOTOR_CURRENT_SENCE_ENABLED 0
#define BOARD_MOTOR_POS_SENCE_ENABLED 0
#define BOARD_ACTUATOR_POS_SENCE_ENABLED 0
#else

	#define BOARD_MOTOR_ENABLED 1

	#if BOARD_MOTOR_CURRENT_SENCE_TYPE == BOARD_MOTOR_CURRENT_SENCE_TYPE_NONE
	#define BOARD_MOTOR_CURRENT_SENCE_ENABLED 0
	#else
	#define BOARD_MOTOR_CURRENT_SENCE_ENABLED 1
	#endif 

	#if BOARD_MOTOR_CURRENT_SENCE_TYPE == BOARD_MOTOR_CURRENT_SENCE_TYPE_NONE
	#define BOARD_MOTOR_POS_SENCE_ENABLED 0
	#else
	#define BOARD_MOTOR_POS_SENCE_ENABLED 1
	#endif
	
	#if BOARD_MOTOR_POS_SENCE_ENABLED == 1
	#ifndef BOARD_MOTOR_POS_SENCE_INCREMENT_DIR
	#define BOARD_MOTOR_POS_SENCE_INCREMENT_DIR 0
	#endif
	#endif 

#endif 





#ifndef BOARD_STARTUP_PAUSE_US
#define BOARD_STARTUP_PAUSE_US 100000
#endif


#ifndef  BOARD_EXTERN_FLOW_PROTO
#define BOARD_EXTERN_FLOW_PROTO 0
#endif

#ifndef BOARD_SERIAL_RESET_TIMEOUT_US
#define BOARD_SERIAL_RESET_TIMEOUT_US 500000
#endif

#define BOARD_PP_TO_ANGLE_SEC(x) (x)
#define BOARD_PP_TO_ANGLE_MIN_PER_SEC(x) (x)
#define BOARD_PP_TO_ANGLE_AMP(x) (x)
#define BOARD_ANGLE_SEC_TO_PP(x) (x)
#define BOARD_ANGLE_MINT_PER_SEC_TO_PP(x) (x)
#define BOARD_CURRENT_AMP_TO_PP(x) (x)

#define  BOARD_ANGLE_SEC_MAX  65535*250
#define  BOARD_ANGLE_MINT_PER_SEC_MAX 500

#include "__robosd_head_end.h"
#endif