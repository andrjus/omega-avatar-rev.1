#include "dc-moto.hpp"

#include "mexo/mexo.h"
#include "mexo/mexo_signal.h"
#include "mexo/mexo_events.h"
#include "core/robosd_log.h"
#include "core/robosd_vartable.hpp"

robo_result_t dc_moto::prioritet_loop_(){
	dc_moto_.perefery.prioritet_loop_();
	dc_moto_.core.prioritet_loop_();
	return ROBO_SUCCESS;
}
robo_result_t dc_moto::permanent_loop_(){
	dc_moto_.perefery.permanent_loop_();
	dc_moto_.core.permanent_loop_();
	return ROBO_SUCCESS;
}
robo_result_t dc_moto::background_loop_(){
	dc_moto_.perefery.background_loop_();
	dc_moto_.core.background_loop_();
	return ROBO_SUCCESS;
}

dc_moto dc_moto_;
dc_moto_machine::settings bmz_test_machine_settings_;
dc_moto_machine dc_moto_machine_(bmz_test_machine_settings_);
			

#if BOARD_FREEMASTER_ENABLED == 1
#include "freemaster/robosd_fm.hpp"
#endif


#if BOARD_MOTOR_TYPE == BOARD_MOTOR_TYPE_DCMOTO
#include "mexo/board/dcmoto/dcmoto_joint.hpp"
#endif

#if BOARD_TERMINAL_ENABLED == 1
#include "terminal/robosd_termo.hpp"
#endif

#if BOARD_NET_PROTO_SWITCH_ENABLED == 1
#include "net/switch/robosd_proto_switch.hpp"
#endif

#include "mexo/board/board+flow+uart+can-rev3.hpp"


#if MEXO_SETTINGS_STORE_ENABLE == 1
void  mexo_ev_settings_save_default_(void);
#endif

#if BOARD_MOTOR_ENABLED == 1
void machines_init_(void);
#endif 
#if BOARD_MOTOR_CURRENT_SENCE_ENABLED == 1
void current_sence(void);
#endif



robo_result_t mexo_run_B_(void);

robo::vartable joint_vtw;

typedef struct dc_moto_settings_s{
	int ver;
	#if BOARD_MOTOR_POS_SENCE_ENABLED == 1
	#if BOARD_MOTOR_POS_SENCE_TYPE == BOARD_MOTOR_POS_SENCE_TYPE_INCREMENT
	struct {
		int dir;
	} inc;
	#endif
	#endif

	#if BOARD_MEXO_NET_FLOW_ENABLED == 1
	uint8_t mexo_flow_address;
	#endif
	
	struct{
		#if BOARD_MOTOR_CURRENT_SENCE_ENABLED == 1
		struct{
			uint32_t period;
			mexo_signal_t level;
		} overcurrent;
		#endif		
	} fault;	
	robo_time_us_t adc_startupPause_us;
}dc_moto_settings_t;

dc_moto_settings_t dc_moto_settings = {
	0
	#if BOARD_MOTOR_POS_SENCE_ENABLED == 1
	#if BOARD_MOTOR_POS_SENCE_TYPE == BOARD_MOTOR_POS_SENCE_TYPE_INCREMENT
	,{
		BOARD_MOTOR_POS_SENCE_INCREMENT_DIR
	}
	#endif
	#endif
	
	#if BOARD_MEXO_NET_FLOW_ENABLED == 1
	, BOARD_MEXO_FLOW_DEFAULT_ADDRESS
	#endif
	
	,{
		#if BOARD_MOTOR_CURRENT_SENCE_ENABLED == 1
		{
			0
			,0
		}
		#endif
	}
	,BOARD_STARTUP_PAUSE_US
};


#if BOARD_MOTOR_POS_SENCE_ENABLED == 1

#if BOARD_MOTOR_POS_SENCE_TYPE == BOARD_MOTOR_POS_SENCE_TYPE_INCREMENT

void motor_enco_update(void){
	mexo_signal_t inc_pos_dif;
	static int prev = 0;
	int now = dc_moto_.perefery.motor_position_query() * dc_moto_settings.inc.dir;
	inc_pos_dif = now - prev;
	prev = now;
	actuator_motor_position_inc(&joint, inc_pos_dif);
}
#endif

#endif

#if BOARD_MOTOR_ENABLED == 1

//==========  ќЌ‘»√”–ј÷»я —»Ћќ¬ќ√ќ ѕ–≈ќЅ–ј«ќ¬ј“≈Ћя ============


#define PWM_BRIDGE_BOOT() dc_moto_.perefery.pwm_power_boot()
#define PWM_BRIDGE_ON() dc_moto_.perefery.pwm_power_on()
#define PWM_BRIDGE_OFF()dc_moto_.perefery.pwm_power_off()
#define PWM_BRIDGE_SHUTDOWN() dc_moto_.perefery.pwm_power_shutdown()
#define PWM_BRIDGE_NAME joint_pwm_bridge
#define PWM_BRIDGE_SET_PWM dc_moto_.perefery.pwm_set
#define PWM_BRIDGE_PWM_MAX dc_moto_.perefery.pwm_max
#define PWM_BRIDGE_PWM_SOURCE joint_ps.voltage
#include "mexo\machine\power\pwm_dc_brifge.impl.inc.h"

#include "board.flow.common.h"
//==========  ќЌ‘»√”–ј÷»я ћќ“ќ–ј ============
#define VAR_BUF_COUNT 250
void machines_init_(void){
	{
		static robo_varreg_p var_table[VAR_BUF_COUNT] = { PLATFORM_STATIC_ZEROS_STRUCT };

		{
			joint_config_t config = {
					joint_pwm_bridge_run//mexo_powerRun_f powerRun;
					, {//struct{
						var_table//robo_varreg_p * memo;
						, VAR_BUF_COUNT//robo_size_t size;
					}//} vartable;
					//, &(break_inst.itf)//mexo_break_p break;
			};
			#if BOARD_MEXO_NET_FLOW_ENABLED == 1
			joint_configure(joint_FLOW_ID, &config);
			#else
			joint_configure(0, &config);
			#endif
		}
	}
}
#endif

robo_result_t mexo_run_B_(void){
#if BOARD_MOTOR_ENABLED == 1
	motor_enco_update();
	dc_moto_machine_.run();
	return mexo_dev_runB_p((mexo_dev_p)(&joint));
#else
	return ROBO_SUCCESS;
#endif
}

void dc_moto::icore::var_attach_(void){

	if (vartable != 0){// &(joint.actuator.ps_dev.mexo_dev.vartable);

		ROBO_VAR_PUSH(vartable, "dc_moto");
		ROBO_VAR_PUSH(vartable, "fault");

#if BOARD_MOTOR_CURRENT_SENCE_ENABLED==1
		ROBO_VAR_PUSH(vartable, "ovc");
		ROBO_VAR_REG(vartable, dc_moto_settings.fault.overcurrent.period, "@us", ROBO_VAR_TYPE_UNSIGNED);
		ROBO_VAR_REG(vartable, dc_moto_settings.fault.overcurrent.level, "@c", ROBO_VAR_TYPE_UNSIGNED);
		ROBO_VAR_POP(vartable);
#endif

#if MEXO_WATCH_DOG_ENABLE == 1
		ROBO_VAR_REG(vartable, mexo_instance.watch_dog_timeout_us, "wd_us", ROBO_VAR_TYPE_UNSIGNED);
#endif
		ROBO_VAR_POP(vartable);

#if BOARD_MEXO_NET_FLOW_ENABLED == 1
		ROBO_VAR_REG(vartable, dc_moto_settings.mexo_flow_address, "addr", ROBO_VAR_TYPE_CONST_UNSIGNED);
#endif

		ROBO_VAR_REG(vartable, tick_us_, "tick_@us", ROBO_VAR_TYPE_UNSIGNED);
		ROBO_VAR_POP(vartable);

	}
}

#if BOARD_MOTOR_CURRENT_SENCE_ENABLED == 1

#define CURRENT_DC_NAME current
#define CURRENT_DC_SOURCE dc_moto_.perefery.adc_data
#define CURRENT_DC_PS joint_ps
#define CURRENT_DC_STARTUP_PAUSE_TICK dc_moto_.core.adc_startupPause
#define CURRENT_DC_SET actuator_ps_set_current
#include "mexo/machine/sence/current_dc.impl.inc.h"

#endif

#if MEXO_SETTINGS_STORE_ENABLE == 1
#include "robosd_target_api.h"

typedef union dc_moto_settings_buffer_s{
	struct {	
#if BOARD_MOTOR_ENABLED == 1
		joint_settings_t joint_settings;
		joint_ps_settings_t joint_ps_settings;
#endif
		dc_moto_settings_t dc_moto_settings;
	};
	uint32_t memo[ (sizeof(joint_settings_t) +sizeof(joint_ps_settings_t)+ sizeof(dc_moto_settings_t))/sizeof(uint32_t) + 3 ];
} dc_moto_settings_buffer_t;

dc_moto_settings_buffer_t dc_moto_settngs_buffer = { PLATFORM_STATIC_ZEROS_STRUCT };
dc_moto_settings_buffer_t dc_moto_settngs_default;

robo_result_t  mexo_ev_settings_save(void){	
	
#if BOARD_MOTOR_ENABLED == 1
		ROBO_STD_MEM_COPY_TO(&joint_settings, &dc_moto_settngs_buffer.joint_settings, sizeof(joint_settings));
		ROBO_STD_MEM_COPY_TO(&joint_ps_settings, &dc_moto_settngs_buffer.joint_ps_settings, sizeof(joint_ps_settings));
#endif
		ROBO_STD_MEM_COPY_TO(&dc_moto_settings, &dc_moto_settngs_buffer.dc_moto_settings, sizeof(dc_moto_settings_t));
			
		ROBO_RETEX(dc_moto_save(RDK_SETTINGS_STORE, (uint8_t *)&dc_moto_settngs_buffer, sizeof(dc_moto_settngs_buffer)));
}

robo_result_t  mexo_ev_settings_load(void){
		ROBO_CHECKRET( dc_moto_load(RDK_SETTINGS_STORE, (uint8_t *)&dc_moto_settngs_buffer, sizeof(dc_moto_settngs_buffer)));
#if BOARD_MOTOR_ENABLED == 1
		ROBO_STD_MEM_COPY_TO(&dc_moto_settngs_buffer.joint_settings, &joint_settings, sizeof(joint_settings_t));
		ROBO_STD_MEM_COPY_TO(&dc_moto_settngs_buffer.joint_ps_settings, &joint_ps_settings, sizeof(joint_ps_settings_t));
#endif
		ROBO_STD_MEM_COPY_TO(&dc_moto_settngs_buffer.dc_moto_settings, &dc_moto_settings, sizeof(dc_moto_settings_t));
		return ROBO_SUCCESS;
}

void  mexo_ev_settings_save_default_(void){
#if BOARD_MOTOR_ENABLED == 1
		ROBO_STD_MEM_COPY_TO(&joint_settings, &dc_moto_settngs_default.joint_settings, sizeof(joint_settings));
		ROBO_STD_MEM_COPY_TO(&joint_ps_settings, &dc_moto_settngs_default.joint_ps_settings, sizeof(joint_ps_settings));
#endif
		ROBO_STD_MEM_COPY_TO(&dc_moto_settings, &dc_moto_settngs_default.dc_moto_settings, sizeof(dc_moto_settings_t));	
}

robo_result_t  mexo_ev_settings_reset(void) {
#if BOARD_MOTOR_ENABLED == 1
		ROBO_STD_MEM_COPY_TO(&dc_moto_settngs_default.joint_settings, &joint_settings, sizeof(joint_settings_t));
		ROBO_STD_MEM_COPY_TO(&dc_moto_settngs_default.joint_ps_settings, &joint_ps_settings, sizeof(joint_ps_settings_t));
#endif
		ROBO_STD_MEM_COPY_TO(&dc_moto_settngs_default.dc_moto_settings, &dc_moto_settings, sizeof(dc_moto_settings_t));
		ROBO_RETEX(dc_moto_store_reset(RDK_SETTINGS_STORE));
	
}
#endif
#if BOARD_MEXO_NET_FLOW_ENABLED == 1
mexo_performer_result_t joint_machine_exchange(mexo_performer_p _performer){
	dc_moto_machine_.applay_data(_performer->in_msg->data);
	return MEXO_PFR_SUCCESS;
}

mexo_performer_t  joint_machine_exchange_performer = {
	joint_FLOW_CMD_JOINT_MACHINE
	, MEXO_PROTO_REQUEST_EXCHANGE
	, joint_machine_exchange	/**непосредственный обработчик*/
	, MEXO_PROTO_REQUEST_IDLE //ответим сами
	, 0
	, 0
	, 0
	, 0
	, 0
	, 0
};



namespace board{
	robo_result_t  mexo_net_flow_set_addr(uint8_t _addr){
		ROBO_CHECKRET(dc_moto_.perefery.flow_set_addr(_addr));
		dc_moto_settings.mexo_flow_address = _addr;
		return ROBO_SUCCESS;
	}

	uint8_t  mexo_net_flow_get_addr(void){
		return (uint8_t)dc_moto_settings.mexo_flow_address;
	}
}
#endif 

#if BOARD_EXTERN_FLOW_PROTO == 1
#include "dc_moto.flow.proto.h"
#endif
#if MEXO_SETTINGS_STORE_ENABLE == 1
mexo_ev_settings_save_default_();
mexo_settings_load();
#endif
#if MEXO_WATCH_DOG_ENABLE == 1
void mexo_ev_watch_dog_raise(void){
	joint.ps_dev.mexo_dev.error ='s';
}
#endif

//конфигураци€ €дра прошивки 

void dc_moto::icore::begin_(){
	adc_startupPause = dc_moto_settings.adc_startupPause_us / BOARD_APP_TICK_PERIOD_US;
#if BOARD_MEXO_NET_FLOW_ENABLED == 1
	board::pool_init();
#endif
#if BOARD_MEXO_NET_FLOW_ENABLED == 1
	board::mexo_net_flow_init(dc_moto_settings.mexo_flow_address);
#endif
#if BOARD_MOTOR_ENABLED == 1
	machines_init_();
	joint.ps_dev.mexo_dev.error = 'n';
	//todo
	vartable = &(joint.ps_dev.mexo_dev.vartable);
#if BOARD_MEXO_NET_FLOW_ENABLED == 1
	joint_vtw.setup(joint_FLOW_ID, vartable, BOARD_VARTABLE_ADDRESS_OFFSET);
#else
	joint_vtw.setup(0, vartable, BOARD_VARTABLE_ADDRESS_OFFSET);
#endif
#else
#define VAR_BUF_COUNT 20
	static robo_varreg_p var_table[VAR_BUF_COUNT] = { PLATFORM_STATIC_ZEROS_STRUCT };
	static robo_vartable_t vartable_ = { 0, VAR_BUF_COUNT, var_table };
	vartable = &vartable_;
	joint_vtw.setup(0, vartable, BOARD_VARTABLE_ADDRESS_OFFSET);
#endif

#if BOARD_MEXO_NET_FLOW_ENABLED == 1
	ROBO_LOGERR(mexo_performer_reg_p((mexo_dev_p)(&joint), &joint_machine_exchange_performer));

#if CMD_MEXO_STREAM0_ENABLED == 1
	board::init((mexo_dev_p)&joint);
#endif
	board::mexo_net_flow_set_addr(dc_moto_settings.mexo_flow_address);
#endif

	DELEGAT_PRIORITY_RUN(dc_moto::prioritet_loop_);
	DELEGAT_PERMANENT_RUN(dc_moto::permanent_loop_);
	DELEGAT_BACKGRAUND_RUN(dc_moto::background_loop_);
	DELEGAT_SLOT(0, mexo_run_B_);
	DELEGAT_SLOT(1, mexo_proto_execute);
	DELEGAT_SLOT(2, mexo_proto_execute);
	DELEGAT_SLOT(3, mexo_proto_execute);

#if MEXO_DEBUG_SIGNAL_ENABLE == 1
	mexo_instance.debugVerb = 55;//ACP_TEST_SHOW;
#endif


#if  BOARD_MOTOR_POS_SENCE_TYPE == BOARD_MOTOR_POS_SENCE_TYPE_ABS_INTERACT_16
	//motor_enco_begin();
#endif 
	dc_moto_control_data_t * cd = (dc_moto_control_data_t *)joint.ps_dev.mexo_dev.proto.control;
	cd->max.current = 2048;
	cd->max.voltage = 32767;
	mexo_instance.watch_dog_timeout_us = 10000000;
	dc_moto_settings.fault.overcurrent.period = 2000000  / BOARD_APP_TICK_PERIOD_US;
	dc_moto_settings.fault.overcurrent.level = BOARD_CURRENT_MAX-100;
	
}

//дискретные автоматы
void dc_moto::icore::tick(void){
#if BOARD_MOTOR_CURRENT_SENCE_ENABLED == 1
	if (dc_moto_settings.fault.overcurrent.level != 0){
		if (
			joint_ps.current_raw > dc_moto_settings.fault.overcurrent.level
			|| joint_ps.current_raw < -dc_moto_settings.fault.overcurrent.level
			){
			overcurrent_tick_++;
			if (overcurrent_tick_> dc_moto_settings.fault.overcurrent.period){
				joint.ps_dev.mexo_dev.error = 'c';
				overcurrent_tick_ = 0;
			}
		}
		else{
			if (overcurrent_tick_>0){
				overcurrent_tick_--;
			}
		}
	}
#endif
#if BOARD_MOTOR_ENABLED == 1
	if (joint.ps_dev.mexo_dev.error){
		joint.ps_dev.mexo_dev.mode = MEXO_MODE_NEUTRAL;
		joint.ps_dev.mexo_dev.proto.control->mode = MEXO_MODE_NEUTRAL;
	}
#endif
	tick_flag_ = true;
	tick_us_ += BOARD_APP_TICK_PERIOD_US;
	tick_raw_++;
}

void dc_moto::icore::prioritet_loop_(void){
#if BOARD_MOTOR_ENABLED == 1
#if BOARD_MOTOR_CURRENT_SENCE_ENABLED == 1
	current_sence();
#endif
#endif
	dc_moto_.core.tick();
#if BOARD_FREEMASTER_ENABLED == 1
	robo::freemaster::recorder();
#endif
}

void dc_moto::icore::permanent_loop_(void){
#if  BOARD_MOTOR_POS_SENCE_TYPE == BOARD_MOTOR_POS_SENCE_TYPE_ABS_INTERACT_16
	//motor_enco_machine_step();
#endif 


#if BOARD_FREEMASTER_ENABLED == 1 
	robo::freemaster::poll();
#endif
}

void  dc_moto::icore::background_loop_(void){
	dc_moto_machine_.pool();
	if (dc_moto_.core.tick_flag_){
		dc_moto_.core.tick_flag_ = 0;
		static robo_time_us_t tick_prev_us = 0;
#if BOARD_NET_PROTO_SWITCH_ENABLED == 1
		robo::net::proto::switcher::core::poll(tick_us_ - tick_prev_us);
#endif
		tick_prev_us = tick_us_;
	}
#if BOARD_TERMINAL_ENABLED == 1
	robo::termo::itf::poll();
	robo::termo::itf::set_prompt(">");
#endif 	
#if BOARD_MOTOR_POS_SENCE_ENABLED == 1
#if BOARD_MOTOR_POS_SENCE_TYPE == BOARD_MOTOR_POS_SENCE_TYPE_ABS_INTERACT_16_MULTYROTATE || \
	BOARD_MOTOR_POS_SENCE_TYPE == BOARD_MOTOR_POS_SENCE_TYPE_ABS_INTERACT_16
	motor_enco_run();
#endif
#endif	
}

mexo_data_p  mexo_ev_pool_get(mexo_size_t _size){
	return board::pool_get(_size);
}
void  mexo_ev_pool_release(mexo_data_p _buf){
	board::pool_release(_buf);
}
#if MEXO_EXTERNAL_INIT == 1
void mexo_ev_init(void){
	dc_moto_.begin();
}
#endif

#if MEXO_DELAY_US == 1
void mexo_ev_delay_us(robo_time_us_t _us){
	dc_moto::iperefery::delay_us(_us);
}
#endif 



void dc_moto_machine::release_task::onStartup(void) {
#if BOARD_BRAKE_ENABLED==1
    bmzmoto.core.brake_req_state = MEXO_BRAKE_REQ_RELEASE;
#endif
};
void dc_moto_machine::release_task::onShutdown(void) {
#if BOARD_BRAKE_ENABLED==1
    bmzmoto.core.brake_req_state = MEXO_BRAKE_REQ_FIXED;
#endif
}

mexo::machine::task::result dc_moto_machine::release_task::doExecute(void) {
    return CONTINUE;
}

mexo::machine::task::result dc_moto_machine::release_task::doShutdown(void) {
#if BOARD_BRAKE_ENABLED==1
    return (brake_inst.itf.state == MEXO_BRAKE_FIXED ? SUCCESS : CONTINUE);
#else
    return SUCCESS;
#endif
}

void  dc_moto_machine::move_task::onBegin(void) {
    tick_ = settings_.startup_pause;
}

mexo::machine::task::result dc_moto_machine::move_task::doBegin(void) {
    if (tick_ == 0) {
        return SUCCESS;
    }
    else {
        tick_--;
        return CONTINUE;
    }
}
void dc_moto_machine::move_task::onReset(void) {
    tick_ = settings_.shutdown_pause;
}

mexo::machine::task::result dc_moto_machine::move_task::doReset(void) {
    if (tick_ == 0) {
        return SUCCESS;
    }
    else {
        tick_--;
        return CONTINUE;
    }
}

void dc_moto_machine::move_task::onFinish(void) {
    control_data()->dev.mode = MEXO_MODE_NEUTRAL;
}

void dc_moto_machine::posicioner::onBegin(void) {
    control_data()->position = joint.motor_position.raw;
    control_data()->speed = 0;            
    control_data()->voltage = actuator_ps_VOLTAGE_MAX_LIM;
    control_data()->current = current_;
    control_data()->dev.mode = MEXO_MODE_MOTOR_POSITION_OVER_VOLTAGE_CL_ID;
    mexo_dev()->control_updated = 1;
    auto_stop_counter_ = settings_.auotostop_pause;
    move_task::onBegin();
}

mexo::machine::task::result dc_moto_machine::posicioner::doExecute(void) {
    if (update_need_) {
        control_data()->speed = speed_;
        control_data()->position = position_;
        joint.ps_dev.mexo_dev.control_updated = 1;
        update_need_ = false;
    }
    else {
        if (autobrake_ && joint.speed_req == 0) {
            if (auto_stop_counter_ > 0) {
                auto_stop_counter_--;
            }
            else {
                stop();
            }
        }
    }
    return CONTINUE;
}
void dc_moto_machine::posicioner::onShutdown(void) {
    control_data()->speed = 0;
    control_data()->position = position_;
    mexo_dev()->control_updated = 1;
    move_task::onShutdown();
}

void dc_moto_machine::motion::onBegin(void) {
    control_data()->position = joint.motor_position.raw;
    control_data()->speed = 0;
    control_data()->voltage = actuator_ps_VOLTAGE_MAX_LIM;
    control_data()->current = current_;
    control_data()->dev.mode = MEXO_MODE_MOTOR_SPEED_OVER_VOLTAGE_CL_ID;
    mexo_dev()->control_updated = 1;
    //move_task::onBegin();
}

mexo::machine::task::result  dc_moto_machine::motion::doBegin() {
    return SUCCESS;
}

mexo::machine::task::result   dc_moto_machine::motion::doExecute() {
    if (update_need_) {
        control_data()->speed = speed_;
        mexo_dev()->control_updated = 1;
        update_need_ = false;
    }
    return CONTINUE;
}
void  dc_moto_machine::motion::onShutdown(void) {
    control_data()->speed = 0;
    joint.ps_dev.mexo_dev.control_updated = 1;
    move_task::onShutdown();
}


void dc_moto_machine::calibrate::onBegin(void) {
    control_data()->speed = 0;
    control_data()->voltage = actuator_ps_VOLTAGE_MAX_LIM;
    control_data()->current = current_;
    control_data()->dev.mode = MEXO_MODE_MOTOR_SPEED_OVER_VOLTAGE_CL_ID;
    mexo_dev()->control_updated = 1;
    operation = ioperation::relax;
    release_task::onBegin();
}

void dc_moto_machine::calibrate::onExecute() {
    release_task::onExecute();
    control_data()->dev.mode = MEXO_MODE_MOTOR_SPEED_OVER_VOLTAGE_CL_ID;
}

void dc_moto_machine::calibrate::onShutdown() {
    release_task::onShutdown();
    control_data()->dev.mode = MEXO_MODE_NEUTRAL;
}


mexo::machine::task::result dc_moto_machine::calibrate::doExecute(void) {      
	if( dir_ > 0) {
    switch (operation) {
    case ioperation::relax:
        control_data()->speed = speed_;
        control_data()->dev.mode = MEXO_MODE_MOTOR_SPEED_OVER_VOLTAGE_CL_ID;
        mexo_dev()->control_updated = 1;
        tick_ = 0;
        operation = ioperation::forward;
        break;
    case ioperation::forward:
        if ( (current_*7)>>3 <= joint_ps.current_raw) {
            tick_++;
            if (tick_ > period_tick_) {
                tick_ = 0;
                control_data()->speed = -speed_;
                mexo_dev()->control_updated = 1;
                position_max_ = joint.motor_position.raw;
                operation = ioperation::backward;
            }
        }
        else {
            if (tick_ > 0) {
                tick_--;
            }
        }
				break;
    case ioperation::backward:
        if ( ((-current_ * 7) >> 3) >= joint_ps.current_raw) {
            tick_++;
            if (tick_ > period_tick_) {
                tick_ = 0;
                control_data()->speed = speed_*5;
                mexo_dev()->control_updated = 1;
								position_max_ -= joint.motor_position.raw;
                position_min_ = joint.motor_position.raw=0;
                operation = ioperation::go_forward;
            }
        }
        else {
            if (tick_ > 0) {
                tick_--;
            }
        }
				break;
    case ioperation::go_forward:
        if ( move_position_ <= joint.motor_position.raw) {
					/*
            control_data()->speed = -speed_*5;
            mexo_dev()->control_updated = 1;
            operation = ioperation::go_backward;
						*/
            control_data()->speed = 0;
            mexo_dev()->control_updated = 1;
            operation = ioperation::stopped;
						complete_ = true;
        }
        break;
    case ioperation::go_backward:
				control_data()->speed = speed_*5;
				mexo_dev()->control_updated = 1;
				operation = ioperation::stopped;
        break;
    }
    return result::CONTINUE;
	} else {
		switch (operation) {
    case ioperation::relax:
				complete_ = false;
        control_data()->speed =  - speed_;
        control_data()->dev.mode = MEXO_MODE_MOTOR_SPEED_OVER_VOLTAGE_CL_ID;
        mexo_dev()->control_updated = 1;
        tick_ = 0;
        operation = ioperation::forward;
        break;
    case ioperation::forward:
        if ( (-current_*7)>>3 >= joint_ps.current_raw) {
            tick_++;
            if (tick_ > period_tick_) {
                tick_ = 0;
                control_data()->speed = speed_;
                mexo_dev()->control_updated = 1;
                position_min_ = joint.motor_position.raw;
                operation = ioperation::backward;
            }
        }
        else {
            if (tick_ > 0) {
                tick_--;
            }
        }
				break;
    case ioperation::backward:
        if ( ((current_ * 7) >> 3) <= joint_ps.current_raw) {
            tick_++;
            if (tick_ > period_tick_) {
                tick_ = 0;
                control_data()->speed = -speed_*5;
                mexo_dev()->control_updated = 1;
								position_max_ =  joint.motor_position.raw = joint.motor_position.raw - position_min_;
                position_min_ = 0;
                operation = ioperation::go_backward;
            }
        }
        else {
            if (tick_ > 0) {
                tick_--;
            }
        }
				break;
    case ioperation::go_forward:
				control_data()->speed = 0;
				mexo_dev()->control_updated = 1;
				operation = ioperation::stopped;
        break;
    case ioperation::go_backward:
        if (  move_position_  >= joint.motor_position.raw) {
            /*control_data()->speed = speed_*5;
            mexo_dev()->control_updated = 1;
            operation = ioperation::go_forward;*/
            control_data()->speed = 0;
            mexo_dev()->control_updated = 1;
            operation = ioperation::stopped;
						complete_ = true;
        }
        break;
    }
    return result::CONTINUE;
	}
}


void dc_moto_machine::doTerminate(void) {
        joint.ps_dev.mexo_dev.mode = MEXO_MODE_NEUTRAL;
        joint.ps_dev.mexo_dev.proto.control->mode = MEXO_MODE_NEUTRAL;
#if BOARD_BRAKE_ENABLED == 1
        bmzmoto.core.brake_req_state = MEXO_BRAKE_REQ_FIXED;
#endif
 }


void dc_moto_machine::update_status_(void) {
    mexo_debSignalOn(55);
    mexo_long_signal_t position;
    mexo_signal_t speed;
    mexo_signal_t current;
    {
        mexo::guard g__;
        //todo 
#if bmz_moto_MOTOR_POSITION_FLT_ENABLED == 1
        position = joint.actuator.motor_position.flt.value;
#else
        position = joint.motor_position.raw;
#endif
#if actuator_MOTOR_POSITION_SPEED_FLT_ENABLED == 1
        speed = joint.motor_position.speed.value;
#else
        speed = joint.motor_position.diff;
#endif

#if  actuator_ps_POWER_SUPPLY_CURRENT_FILTER == 1
        current = joint_ps.current.value;
#else
        current = joint_ps.current_raw;
#endif
    }

    actual_angle_sec_ = BOARD_PP_TO_ANGLE_SEC(position);
    actual_speed_minpersec_ = BOARD_PP_TO_ANGLE_MIN_PER_SEC(speed);

    actual_current_amp_ = BOARD_PP_TO_ANGLE_AMP(current);

    //status.speed_minPerSec = BOARD_PP_TO_ANGLE_MIN_PER_SEC(speed);
    //status.error = (Error)joint.actuator.ps_dev.mexo_dev.error;
    //status.breakState = spotter_machine_.brake_state();
    //status.isCalibrating = spotter_machine_.calibrate_active();
}

void dc_moto_machine::varreg(robo_vartable_p _vt) {
        ROBO_VAR_PUSH(_vt, "test");


        ROBO_VAR_REG(_vt, enable_, "enable", ROBO_VAR_TYPE_UNSIGNED);

        ROBO_VAR_REG(_vt, command_, "command", ROBO_VAR_TYPE_UNSIGNED);

        ROBO_VAR_PUSH(_vt, "req");
        ROBO_VAR_REG(_vt, req_current_amp_, "current", ROBO_VAR_TYPE_SIGNED);
        ROBO_VAR_REG(_vt, req_speed_minpersec_, "speed", ROBO_VAR_TYPE_SIGNED);
        ROBO_VAR_REG(_vt, req_position_sec_, "angle", ROBO_VAR_TYPE_SIGNED);
        ROBO_VAR_POP(_vt);

        ROBO_VAR_PUSH(_vt, "actual");
        ROBO_VAR_REG(_vt, actual_current_amp_, "current", ROBO_VAR_TYPE_SIGNED);
        ROBO_VAR_REG(_vt, actual_speed_minpersec_, "speed", ROBO_VAR_TYPE_SIGNED);
        ROBO_VAR_REG(_vt, actual_angle_sec_, "angle", ROBO_VAR_TYPE_SIGNED);
        ROBO_VAR_POP(_vt);

        ROBO_VAR_POP(_vt);

}

dc_moto_machine::dc_moto_machine(settings& _settings)
    : mexo::machine::process()
    , release_task_(_settings)
    , posicioner_(_settings)
    , motion_(_settings)
    , calibrate_(_settings)
{
}

bool dc_moto_machine::position_mode(int32_t _angle_millysecond, int32_t _speed_minpersec_max, int32_t  _current_amp, bool _autobrake) {
    if (joint.ps_dev.mexo_dev.error == 0) {
        mexo_long_signal_t actuator_position;
        mexo_long_signal_t req_actuator_position;
        //1. ограничиваем заданные значениІ, чтобы невлезть в переполнениІ
        if (_angle_millysecond > BOARD_ANGLE_SEC_MAX) _angle_millysecond = BOARD_ANGLE_SEC_MAX;
        if (_angle_millysecond < 0) _angle_millysecond = 0;
        if (_speed_minpersec_max > BOARD_ANGLE_MINT_PER_SEC_MAX) _speed_minpersec_max = BOARD_ANGLE_MINT_PER_SEC_MAX;
        if (_speed_minpersec_max < 0) _speed_minpersec_max = 0;

        {
            mexo::guard g__;
            //todo 
#if MOTOR_POSITION_FLT_ENABLED == 1
            actuator_position = joint.actuator.motor_position.flt.value;
#else
            actuator_position = joint.motor_position.raw;
#endif
        }
        //2. преобразуем текущее положение в формат от 0:359.9999
        //actuator_position = (mexo_long_signal_t)(((uint32_t)actuator_position) & BOARD_ENCODER_ACTUAL_MASK);
        req_actuator_position = BOARD_ANGLE_SEC_TO_PP(_angle_millysecond);
        //3. выбираем сторону, в которую крутитсІ
        
        mexo_signal_t speed = BOARD_ANGLE_MINT_PER_SEC_TO_PP(_speed_minpersec_max);

        //запускаем 
        posicioner_.position_ = req_actuator_position;
        posicioner_.speed_ = speed;
        posicioner_.update_need_ = true;
        posicioner_.current_ = BOARD_CURRENT_AMP_TO_PP(_current_amp);
        posicioner_.autobrake_ = _autobrake;
        switchto(&posicioner_);
        return true;
    }
    else {
        return false;
    }
}
bool dc_moto_machine::speed_mode(int32_t _speed_minpersec, int32_t  _current_amp) {
    if (joint.ps_dev.mexo_dev.error == 0) {
        if (_speed_minpersec > BOARD_ANGLE_MINT_PER_SEC_MAX) _speed_minpersec = BOARD_ANGLE_MINT_PER_SEC_MAX;
        if (_speed_minpersec < -BOARD_ANGLE_MINT_PER_SEC_MAX) _speed_minpersec = -BOARD_ANGLE_MINT_PER_SEC_MAX;

        motion_.current_ = BOARD_CURRENT_AMP_TO_PP(_current_amp);
        motion_.speed_ = BOARD_ANGLE_MINT_PER_SEC_TO_PP(_speed_minpersec);
        motion_.update_need_ = true;
        switchto(&motion_);
        return true;
    }
    else {
        return false;
    }
}


bool dc_moto_machine::calibrate_mode(int32_t _angle_millysecond, int32_t _speed_minpersec, int32_t  _current_amp, int _dir) {
    if (joint.ps_dev.mexo_dev.error == 0) {
        if (_speed_minpersec > BOARD_ANGLE_MINT_PER_SEC_MAX) _speed_minpersec = BOARD_ANGLE_MINT_PER_SEC_MAX;
        if (_speed_minpersec < -BOARD_ANGLE_MINT_PER_SEC_MAX) _speed_minpersec = -BOARD_ANGLE_MINT_PER_SEC_MAX;
				calibrate_.dir_ = _dir;
        calibrate_.current_ = BOARD_CURRENT_AMP_TO_PP(_current_amp);
        calibrate_.speed_ = BOARD_ANGLE_MINT_PER_SEC_TO_PP(_speed_minpersec);
        calibrate_.update_need_ = true;
				calibrate_.move_position_ = BOARD_ANGLE_SEC_TO_PP(_angle_millysecond);
        switchto(&calibrate_);
        return true;
    }
    else {
        return false;
    }
}


bool dc_moto_machine::brake_set(void) {
    dc_moto_machine::stop();
    return true;
}
bool dc_moto_machine::brake_release(void) {
    if (joint.ps_dev.mexo_dev.error == 0) {
        switchto(&release_task_);
        return true;
    }
    else {
        return false;
    }
}



 

void dc_moto_machine::send_answer_data(void){
	exchange_data  data;
	data.command =  (uint8_t)command_;
	data.actuator_error	=  joint.ps_dev.mexo_dev.error?1:0;
	data.actuator_active = joint.ps_dev.mexo_dev.mode?1:0;
	data.calibrated	=  calibrate_complerte_()?1:0;
	data.position_2 = joint.motor_position.raw >> 1;
	data.speed_4 = joint.motor_position.speed.value >> 2;
	data.current_16 = joint_ps.current.value >> 4;
	data.voltage_256 = joint_ps.voltage >> 8;
	dc_moto::iperefery::async_send(joint_FLOW_CAN0_JOINT_MACHINE_SUBA_ANSW, data.memo, 8 );
}

void dc_moto_machine::applay_data( uint8_t * _in){
	exchange_data * data =(exchange_data *)_in;
	dc_moto_control_data_t * cd = (dc_moto_control_data_t *)joint.ps_dev.mexo_dev.proto.control;
	switch(data->command){
		case 1:
			cd->voltage = data->voltage_256 << 8;
			req_current_amp_ = data->current_16 << 4;
			req_speed_minpersec_ = data->speed_4 << 2;
			joint.ps_dev.mexo_dev.error = 0;
			command_ = 0;
			break;
		case 3:
			cd->voltage = data->voltage_256 << 8;
			req_current_amp_ = data->current_16 << 4;
			req_speed_minpersec_ = data->speed_4 << 2;
			command_ = 3;
			break;
		case 4:
			cd->voltage = (data->voltage_256 << 8);
			req_current_amp_ = (data->current_16 << 4);
			req_speed_minpersec_ = (data->speed_4 << 2);
			req_position_sec_ = (data->position_2 << 1);
			command_ = 4;
			break;
		case 5:
			cd->voltage = (data->voltage_256 << 8);
			req_current_amp_ = (data->current_16 << 4);
			req_speed_minpersec_ = (data->speed_4 << 2);
			req_position_sec_ = (data->position_2 << 1);
			dir_ = 1;
			command_ = 5;
			break;
		case 6:
			cd->voltage = (data->voltage_256 << 8);
			req_current_amp_ = (data->current_16 << 4);
			req_speed_minpersec_ = (data->speed_4 << 2);
			req_position_sec_ = (data->position_2 << 1);
			dir_ = -1;
			command_ = 5;
		break;
		case 7:
			if( ! calibrate_complerte_()){
				calibrate_set_complerte_();
				joint.motor_position.raw = data->position_2<<1;
			}
		break;
			
	}
	send_answer_data();
}


void  dc_moto_machine::pool(void) {
    update_status_();

    if (enable_) {
        switch (command_) {
        case 0:
            brake_set();
            break;
        case 1:
            brake_release();
            break;
        case 3:
            if ( req_speed_minpersec_ != req_speed_minpersec_prev_ || command_prev_ != command_ || req_current_amp_ != req_current_amp_prev_) {
                speed_mode(req_speed_minpersec_, req_current_amp_);
                req_speed_minpersec_prev_ = req_speed_minpersec_;
                req_current_amp_prev_ = req_current_amp_;
            }
            break;

        case 4:
            if (req_speed_minpersec_ != req_speed_minpersec_prev_
                || req_position_sec_prev_ != req_position_sec_
                || command_prev_ != command_
                || req_current_amp_ != req_current_amp_prev_
                ) {
                req_speed_minpersec_prev_ = req_speed_minpersec_;
                req_position_sec_prev_ = req_position_sec_;
                req_current_amp_prev_ = req_current_amp_;
                position_mode(req_position_sec_, req_speed_minpersec_, req_current_amp_, autpblock_);
            }
            break;
        case 5:
            if (req_speed_minpersec_ != req_speed_minpersec_prev_ || command_prev_ != command_ || req_current_amp_ != req_current_amp_prev_) {
                req_speed_minpersec_prev_ = req_speed_minpersec_;
                req_current_amp_prev_ = req_current_amp_;
                calibrate_mode(req_position_sec_,req_speed_minpersec_, req_current_amp_, dir_);
            }
        }
        command_prev_ = command_;
    }
}

