/*переопределять конкретные параметры так
#undef YOUR_PARAMETR
#define YOUR_PARAMETR  PARAMETR_VALUE
*/
#if (!defined(__joint_settings_tunning)) && defined(__board_joint_h)
#define __joint_settings_tunning
#else
#error error of using oint_settings_tunning.h
#endif


