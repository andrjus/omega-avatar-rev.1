#if (!defined(__robo_app_tunning_h)) && defined(__robosd_common_h)
#define __robo_app_tunning_h
#else
#error error of using robo_app_tunning.h
#endif

#define MEXO_SIDE_ENABLED 1

#define ROBO_MEMORY_HEAP_ENABLED 1
#define ROBO_DEBUG_LOG_ENABLED 1
#define ROBO_OS_INI_WIN_ENABLED 1
#define ROBO_OS_SYSTEM_WIN_ENABLED 1

#define APP_TICK_PERIOD_US BOARD_APP_TICK_PERIOD_US
#define TERMO_STREAMER_OUT_BUF_SIZE_BITS 9

#include "board_common.h"

#if BOARD_FREEMASTER_ENABLED == 1
#define APP_FREEMASTER_ROBO_SERIAL_ENABLED 1
#else
#define APP_FREEMASTER_ROBO_SERIAL_ENABLED 0
#endif



#if BOARD_TERMINAL_ENABLED == 1
#define APP_TERMINAL_ENABLED 1
#define APP_TERMINAL_VT_ENABLED 1
#endif

#if BOARD_NET_PROTO_SWITCH_ENABLED == 1
#define APP_PROTO_SWITCH_ENABLED 1
#endif