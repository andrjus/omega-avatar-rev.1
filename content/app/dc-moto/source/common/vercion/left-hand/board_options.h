/* определять опции так
#define YOUR_OPTION  YOUR_OPTION_VALUE
*/

#if (!defined(__board_option)) && defined(__board_common_h)
#define __board_option
#else
#error error of using board_option.h
#endif


#define BOARD_MOTOR_TYPE BOARD_MOTOR_TYPE_DCMOTO

#define BOARD_APP_TICK_PERIOD_US 50

#define BOARD_TERMINAL_ENABLED 1

#define BOARD_NET_PROTO_SWITCH_ENABLED 1

#define BOARD_FREE_MASTER_DRIVER_TYPE BOARD_FREE_MASTER_DRIVER_TYPE_PROTO_ABONENT

#define BOARD_MEXO_NET_FLOW_ENABLED 1

#define BOARD_CURRENT_MAX 1700


#define BOARD_VARTABLE_ADDRESS_OFFSET 0


#if BOARD_KIND == BOARD_KIND_Z_AXIS
#define BOARD_MEXO_FLOW_DEFAULT_ADDRESS 0x1
#define BOARD_MOTOR_ENCO_DIR 
#endif



#define BOARD_MOTOR_CURRENT_SENCE_TYPE BOARD_MOTOR_CURRENT_SENCE_TYPE_ADC
#define BOARD_MOTOR_POS_SENCE_TYPE BOARD_MOTOR_POS_SENCE_TYPE_INCREMENT
#define BOARD_MOTOR_POS_SENCE_INCREMENT_DIR 1


