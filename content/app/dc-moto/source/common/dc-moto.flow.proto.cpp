#include "dc-moto.flow.proto.h"
#include "robosd_target_api.h"
#include "core/robosd_log.h"

#if MEXO_SIDE_ENABLED == 1
#include "mexo/mexo_signal.h"
#include "mexo/mexo_signal.h"
#include "mexo/board/dcmoto/dcmoto_joint.hpp"
robo_result_t dc_moto_snapshot_update(mexo_dev_p _dev){
	dc_moto_snapshot_p  snapshot = (dc_moto_snapshot_p)(_dev->proto.snapshot.ref);
	
	snapshot->dev.mode = 0;
	snapshot->dev.flags = 0;
	snapshot->dev.mode_ix = _dev->mode;
	snapshot->dev.err = (_dev->error !=0);
#if BOARD_MOTOR_ENABLED == 1
	#if BOARD_MOTOR_POS_SENCE_ENABLED == 1
	snapshot->position = (mexo_signal_t)joint.motor_position.raw;
	snapshot->speed = joint.motor_position.speed.value;
	#else
	snapshot->position = 0;
	snapshot->speed = 0;
	#endif
	snapshot->current = joint_ps.current.value;
#else
	snapshot->position = 0;
	snapshot->speed = 0;
	snapshot->current = 0;
#endif
	return ROBO_SUCCESS;
}


robo_result_t dc_moto_snapshot_encode(dev_snapshot_p _snapshot, proto_stream_p _stream){
	proto_data_t tmp;
	dc_moto_snapshot_p snapshot = (dc_moto_snapshot_p)_snapshot;
	tmp = (_snapshot->mode & 0x7F) + (_snapshot->err<<7);
	ROBO_CHECKRET(proto_stream_put(_stream, tmp)); //1
	ROBO_CHECKRET(proto_stream_s_encode(_stream, snapshot->current)); //3
	ROBO_CHECKRET(proto_stream_s_encode(_stream, snapshot->speed)); //5
	ROBO_CHECKRET(proto_stream_s_encode(_stream, snapshot->position)); //7
	return ROBO_SUCCESS;
}

#endif

#if ROBOSD_SIDE_ENABLED == 1
robo_result_t ROBO_DECL  dc_moto_snapshot_decode(dc_moto_snapshot_p _snapshot, proto_stream_p _stream){
	proto_data_t tmp;
	dc_moto_snapshot_p snapshot = (dc_moto_snapshot_p)_snapshot;
	ROBO_CHECKRET(proto_stream_get(_stream, &tmp)); //1
	_snapshot->dev.mode=0;
	_snapshot->dev.flags = 0;
	_snapshot->dev.err = (tmp >> 7) & 0x1;
	_snapshot->dev.mode = tmp & 0x7F;
	ROBO_CHECKRET(proto_stream_s_decode(_stream, &(((dc_moto_snapshot_p)snapshot)->current))); //3
	ROBO_CHECKRET(proto_stream_s_decode(_stream, &(((dc_moto_snapshot_p)snapshot)->speed))); //5
	ROBO_CHECKRET(proto_stream_s_decode(_stream, &(((dc_moto_snapshot_p)snapshot)->position))); //7
	return ROBO_SUCCESS;
}
#endif
