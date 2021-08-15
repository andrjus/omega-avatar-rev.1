#ifndef __dc_moto_flow_proto_h
#define __dc_moto_flow_proto_h

#include "mexo/machine/actuator/actuator.proto.h"


#ifndef MEXO_SIDE_ENABLED
#define  MEXO_SIDE_ENABLED 0
#endif
#ifndef ROBOSD_SIDE_ENABLED
#define  ROBOSD_SIDE_ENABLED 0
#endif

#if MEXO_SIDE_ENABLED == 1
#include "mexo/mexo.h"
#endif

typedef struct dc_moto_snapshot_s{
	dev_snapshot_t dev; //pack 1 //1
	proto_signal_t position; //3
	proto_signal_t speed; //5
	proto_signal_t current; //7
} dc_moto_snapshot_t;
typedef dc_moto_snapshot_t * dc_moto_snapshot_p;
#define	  dc_moto_SNAPSHOT_DATA_SIZE 7

typedef actuator_control_data_t dc_moto_control_data_t;
typedef dc_moto_control_data_t * dc_moto_control_data_p;


#include "__robosd_head_begin.h"
#if MEXO_SIDE_ENABLED
robo_result_t dc_moto_snapshot_encode(dev_snapshot_p _snapshot, proto_stream_p _stream);
#define  EXTERN_JOINT_SNAPSHOT_UPDATE_ENABLED 1
robo_result_t dc_moto_snapshot_update(mexo_dev_p _dev);
#endif
#if ROBOSD_SIDE_ENABLED
ROBO_EXPORT robo_result_t ROBO_DECL  dc_moto_snapshot_decode(dc_moto_snapshot_p _snapshot, proto_stream_p _stream);
#endif
#include "__robosd_head_end.h"


#endif
