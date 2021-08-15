#if ( !defined(__free_master_cfg_tuning_h)) && defined(__FREEMASTER_CFG_H) 
#define __free_master_cfg_tuning_h
#else
#error error of using free_master_cfg_tuning.h
#endif

#define FMSTR_PLATFORM_DEFAULT 1
#define STREAMER_OUT_RING_BUFFER_ENABLED 1
#define MEXO_FREEMASTER_ENABLED 1

#include "board_common.h"
#include "core/robosd_common.h"
#undef PLATFORM_INLINE
#define PLATFORM_INLINE ROBO_PLATFORM_INLINE
#define FMSTR_ADDRESS_OFFSET BOARD_VARTABLE_ADDRESS_OFFSET
#ifdef IMITATION_MODEL
#define FMSTR_ADDRESS_OFFSET_TYPE uintptr_t
#else
#define FMSTR_ADDRESS_OFFSET_TYPE uintptr_t
#endif
#undef FMSTR_REC_TIMEBASE
#define FMSTR_REC_TIMEBASE  (int)(32768+APP_TICK_PERIOD_US)

#ifdef IMITATION_MODEL
#undef FMSTR_REC_BUFF_SIZE
#define FMSTR_REC_BUFF_SIZE 4096
#endif
