/*
здесь переопределяются параметры MEXO_..., 
управляющие поведением 
всех компонент mexo 
*/


#if (!defined(__mexo_option)) && defined(__mexo_common_h)
#define __mexo_option
#else
#error error of using mexo_option.h
#endif

#define MEXO_SYSCLOCK_ENABLE 0

//mexo working in acp interrupt
#define MEXO_LOCK_ENABLE 0 

#define CMD_MEXO_STREAM0_RESET_TIMEOUT_US 500000
#define MEXO_WATCH_DOG_ENABLE 1
#include "board_common.h"
#define MEXO_TICK_PERIOD_US BOARD_APP_TICK_PERIOD_US
#ifdef IMITATION_MODEL

	#define MEXO_SETTINGS_STORE_ENABLE 0
	#define MEXO_DEBUG_SIGNAL_ENABLE 0
	#define MEXO_GUARD_ENABLE 0
	#define MEXO_VARTEBLE_LEVEL MEXO_VARTEBLE_LEVEL_FULL

#else

//mexo can  can kill message queue and buffer data
#define MEXO_GUARD_ENABLE 1

	#define MEXO_DEBUG_SIGNAL_ENABLE 1
	#define MEXO_WATCH_DOG_ENABLE 1
	#define MEXO_VARTEBLE_LEVEL MEXO_VARTEBLE_LEVEL_TUNING

#endif


#define MEXO_SLOT_COUNT 8
#define FLOW_MSG_PORT0_ENABLED 1
#define FLOW_MSG0_SUBA_COUNT 16
#define FLOW_MSG0_CHANNEL_INDEX 1
#define FLOW_MSG0_DATA_SIZE 8
#define CMD_MEXO_STREAM0_ENABLED 1
#define MEXO_STREAM0_PORT_ID 101

#define MEXO_EXTERNAL_INIT 1
#define MEXO_DELAY_US 1
