#if (!defined(__dcmoto_config)) && defined(__board_joint_h)
#define __dcmoto_config
#else
#error error of using dcmoto_config.h
#endif


// силовой преобразователь привода поддерживает управление по напряжению
#define actuator_ps_POWER_SUPPLY_VOLTAGE_ENABLED 1


#if BOARD_MOTOR_CURRENT_SENCE_ENABLED == 1
#define actuator_ps_POWER_SUPPLY_CURRENT_MEASSURY_ENABLED 1
#define actuator_ps_POWER_SUPPLY_CURRENT_FILTER 1
#define actuator_ps_POWER_SUPPLY_CURRENT_DIFF 0
#define actuator_ps_PPOWER_SUPPLY_CURRENT_DIFF_FILTER 0
#define actuator_ps_POWER_SUPPLY_CURRENT_REGULATOR_ENABLED 1
#define actuator_ps_POWER_SUPPLY_CURRENT_ENABLED 1
#define actuator_ps_POWER_SUPPLY_VOLTAGE_CL_ENABLED 1
#else
#define actuator_ps_POWER_SUPPLY_CURRENT_MEASSURY_ENABLED 0
#endif

#define actuator_ACTUATOR_BREAK_ENABLED 0 




#if BOARD_MOTOR_POS_SENCE_ENABLED == 1 
#define actuator_MOTOR_POSITION_INFINITE_ENABLED BOARD_MOTOR_MULTYROTATE
#define actuator_MOTOR_POSITION_ENABLED 1
#define actuator_MOTOR_POSITION_SPEED_FLT_ENABLED 1
#endif

#if BOARD_ACTUATOR_POS_SENCE_ENABLED == 1 
#define actuator_ACTUATOTR_POSITION_ENABLED 1
#endif


#define actuator_MODE_VOLTAGE_ENABLED 1
#define actuator_MODE_MOTOR_SPEED_OVER_VOLTAGE_ENABLED 0
#define actuator_MODE_MOTOR_POSITION_OVER_VOLTAGE_ENABLED 0

#if actuator_ps_POWER_SUPPLY_CURRENT_MEASSURY_ENABLED == 1

#define actuator_MODE_CURRENT_ENABLED 1
#define actuator_MODE_VOLTAGE_CL_ENABLED 1

#define actuator_MODE_MOTOR_SPEED_OVER_CURRENT_ENABLED 0
#define actuator_MODE_MOTOR_SPEED_OVER_VOLTAGE_CL_ENABLED 1

#define actuator_MODE_MOTOR_POSITION_OVER_CURRENT_ENABLED 0
#define actuator_MODE_MOTOR_POSITION_OVER_VOLTAGE_CL_ENABLED 1

#define actuator_MODE_ALIGN_OVER_VOLTAGE_CL_ENABLED 0
#endif 



