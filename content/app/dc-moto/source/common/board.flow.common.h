#ifndef __board__flow_common_h
#define __board__flow_common_h
#include "mexo/net/flow/flow_route_record.h"
#include "mexo/dev.proto.h"
#include "mexo/net/flow/flow_can_msg_id.h"


#define FLOW_LOGIC_CHANNEL_COUNT 1
#define FLOW_LOGIC_SUBA_COUNT 16

#define FLOW_CAN0_ID 0
#define FLOW_CAN0_DATA_SIZE 8


#define CMD_JOINT_MACHINE_ID 8
#define CMD_JOINT_MACHINE_ENABLED 1

#define joint_FLOW_ID 1

#define FLOW_LOGIC_CHANNEL_COUNT 1
#define FLOW_LOGIC_CAN1_SUBA_COUNT 16


#if CMD_MEXO_STREAM0_ENABLED == 1
#define joint_FLOW_CMD_MEXO_STREAM0 CMD_MEXO_STREAM0_ID
#define joint_FLOW_CAN0_MEXO_STREAM0_SUBA 0xF
#define joint_FLOW_CAN0_MEXO_STREAM0_SUBA_ANSW 0xF
#endif

#if CMD_MEXO_STREAM1_ENABLED == 1
#define joint_FLOW_CMD_MEXO_STREAM1 CMD_MEXO_STREAM1_ENABLED
#define joint_FLOW_CAN0_MEXO_STREAM1_SUBA 0xE
#define joint_FLOW_CAN0_MEXO_STREAM1_SUBA_ANSW 0xE
#endif


#define joint_FLOW_CMD_DEV_SNAPSHOT_QUERY CMD_DEV_SNAPSHOT_QUERY_ID
#define joint_FLOW_CAN0_DEV_SNAPSHOT_QUERY_SUBA 0x1
#define joint_FLOW_CAN0_DEV_SNAPSHOT_QUERY_SUBA_ANSW 0x1

#define joint_FLOW_CMD_DEV_QUERY_VAR CMD_DEV_QUERY_VAR_ID
#define joint_FLOW_CAN0_DEV_QUERY_VAR_SUBA 0xB
#define joint_FLOW_CAN0_DEV_QUERY_VAR_SUBA_ANSW 0xB

#define joint_FLOW_CMD_DEV_SET_WORK_MODE CMD_DEV_SET_WORK_MODE_ID
#define joint_FLOW_CAN0_DEV_SET_WORK_MODE_SUBA 0x2
#define joint_FLOW_CAN0_DEV_SET_WORK_MODE_SUBA_ANSW 0x1

#define joint_FLOW_CMD_DEV_EXTENDED_QUERY CMD_DEV_EXTENDED_QUERY_ID
#define joint_FLOW_CAN0_DEV_EXTENDED_QUERY_SUBA 0x3
#define joint_FLOW_CAN0_DEV_EXTENDED_QUERY_SUBA_ANSW 0x3


#define  joint_FLOW_CMD_DEV_QUERY_VAR CMD_DEV_QUERY_VAR_ID
#define  joint_FLOW_CAN0_DEV_QUERY_VAR_SUBA 0xB
#define  joint_FLOW_CAN0_DEV_QUERY_VAR_SUBA_ANSW 0xB

#define  joint_FLOW_CMD_JOINT_MACHINE CMD_JOINT_MACHINE_ID
#define  joint_FLOW_CAN0_JOINT_MACHINE_SUBA 0xD
#define  joint_FLOW_CAN0_JOINT_MACHINE_SUBA_ANSW 0xD

#define BOARD_ROUTE_RECORDS \
	FLOW_ROUTE_RECORD(joint, JOINT_MACHINE, CAN0)

#define joint_snapshot_t dc_moto_snapshot_t
#define joint_control_data_t dc_moto_control_data_t
#define joint_snapshot_update dc_moto_snapshot_update
#define joint_snapshot_encode dc_moto_snapshot_encode
#define joint_SNAPSHOT_DATA_SIZE dc_moto_SNAPSHOT_DATA_SIZE

#include "board.flow.tuning.h"

#include "dc-moto.flow.proto.h"

#endif
