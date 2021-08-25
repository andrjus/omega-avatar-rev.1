#if (!defined(__robosd_app_tunning_hpp)) && defined(__robosd_common_hpp)
#define __robosd_app_tunning_hpp
#else
#error error of using robosd_app_tunning.hpp
#endif

#define ROBO_UNICODE_ENABLED 0

#define ROBO_APP_SYSTEM_ENABLED  1
#define ROBO_APP_DEBUG_LOG_ENABLED 1
#define ROBO_TERMINAL_PRINT_ENABLED 1
#define ROBO_APP_MODULE_ENABLED 1

#define ROBO_APP_ENV_TYPE ROBO_APP_TYPE_KEIL
#define ROBO_APP_INI_TYPE ROBO_APP_TYPE_NATIVE
#define ROBO_APP_LIB_TYPE ROBO_APP_TYPE_NATIVE
//#define ROBO_APP_ALLOC_TYPE ROBO_APP_TYPE_KEIL


#define ROBO_APP_TYPE_KEIL 101

/*
*/

#include <string.h>
//#define robo_errlog(f,...) 
