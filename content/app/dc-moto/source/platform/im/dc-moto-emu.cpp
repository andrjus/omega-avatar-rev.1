#include "im/models/dcmachine.hpp"
#define NOMINMAX
#include "im/edev/edev.h"
#include "core/robosd_ini.h"
#include "core/robosd_string.h"
#include "core/robosd_log.h"
#include "mexo/mexo.h"
#include "mexo/mexo_events.h"
#include "net/can/platform/emulator/emu_can_common.h"
#include "net/serial/platform/win/win_com.hpp"
#include "net/switch/robosd_proto_switch.hpp"
#include "board_common.h"
#if BOARD_MEXO_NET_FLOW_ENABLED == 1
#include "mexo/net/adapters/flow_msg_port.h"
#endif
#define PIF 3.14159265359f

#include "dc_moto.hpp"
#include "mexo/board/board+flow+uart+can-rev3.hpp"

#include <math.h>
#include "im/edev/joint_gearbox.hpp"

struct meXo_mod_t{

	robo::im::dcmachine::ideal2 motor;
	robo::im::joint::gearbox::elastic::friction gearbox;

	robo::net::win_com uart0;
	robo::net::win_com uart1;
	int controlSystemPrescale;
	int controlSystemTick;
	emu_can_port_t can_port_0;
	emu_dev_agent_p agent;
	int address0;
	robo_result_t load(const robo_string_t _sec, int _actuator_index);
	float voltageSupply;
	double  busyTime;
	float pwmGain;
	float motorPositionNoizeMag; //rad амплитуда шума на датчике
	int motorDriverPositionResolutionBit; //bit Разрядность  которую предоставляет драйвер датчика положения мотора
	int motorDriverPositionResolution; // pp Разрешение датчика положения , 
	int motorDriverNoizePositionBit; //bit  Шум датчика относительно драйвера
	float motorPositionOffset; // rad смещение датчика относительно ротора в рад
	double motorPositionGain; // pp /rad коэффициент между попугаями драйвера и углом поворота ротора
	uint16_t motorPosition; // pp
	int currentDriverNoizeBit; //Шум датчика тока относительно драйвера
	int currentDriverMax; //максимальный ток относительно АЦП
	int currentDriverZero; // ноль ток относительно АЦП
	float currentNoizeMag;

	float currentSenceMax;
	int currentDriverResolutionBit;
	float currentGain;
	float model_period_sec;
	unsigned int pwmModulo;
	unsigned int pwmMax;
	void pwm_set(int _duty){
		 motor.voltage = voltageSupply * _duty / pwmModulo;
	};
	void pwm_on(){
	}
	void pwm_off(){
		motor.powerOn = false;
// это лишнее - модель сама разберется с напряжением
//		motor.voltage = 0.f;
	}
	void pwm_boot(){
		motor.powerOn = true;
	}
	void pwm_shutdown(){
	}

	void run();
	void motor_pos_query();
	uint32_t sence_current(float _value);
};

meXo_mod_t  mexo_mod;
//joint_t join;



robo::net::proto::switcher::port uart0(
	robo::net::proto::switcher::port::type::SERIAL
	, mexo_mod.uart0
	, BOARD_SERIAL_RESET_TIMEOUT_US
	);
robo::net::proto::switcher::port uart1(
	robo::net::proto::switcher::port::type::SERIAL
	, mexo_mod.uart1
	, BOARD_SERIAL_RESET_TIMEOUT_US
	);


float _saturatef(float a, float b, float c){
	if (a > b) return b;
	if (a <c) return c;
	return a;
}

#define  _saturate(a, b) (((a) > (b))?(b):(a))

void meXo_mod_t::run(){
	dc_moto::instance.perefery.adc_data[0] = sence_current(-motor.current);
	dc_moto::instance.perefery.adc_data[1] = sence_current(motor.current);

	motor.run();
	gearbox.run(); 
}


void meXo_mod_t::motor_pos_query(){
	//1. Фактическое положение датчика в радианах		
	float armadillo_motorPosition = (float)gearbox.supply.position;

	//2. Датчик получает смещенные данные вместе с шумом. Данные дискретизируются (что особенно важно для измерения скорости)
	//как раз с эффективным разрешением датчика
	//
	float motorPositionNoize = motorPositionNoizeMag* (float)(rand() % 1000 - 500) / 500.f;

	double tmp = motorPositionGain * fmodf(armadillo_motorPosition + motorPositionOffset + motorPositionNoize, 2.0 * PIF);

	if (tmp > 0)  tmp += 0.5;
	//else if (tmp < 0)	tmp -= 0.5; // округление данных датчика
	motorPosition = (uint16_t)(tmp);
	motorPosition = motorPosition & ((1 << motorDriverPositionResolutionBit) - 1);
	//Данные датчика положения уже в попугаях
	
}


uint32_t meXo_mod_t::sence_current(float _value){
	float noize = currentNoizeMag*(float)(rand() % 1000 - 500) / 500.f;
	float tmp = (_value + noize)*currentGain;
	if (tmp > 0)  tmp += 0.5; else if (tmp < 0)	tmp -= 0.5; // округление данных датчика

	int32_t ret = currentDriverZero + (int32_t)tmp;
	if (ret < 0) ret = 0;
	if (ret > currentDriverMax) ret = currentDriverMax;
	return (uint32_t)ret;
}





extern "C"  void  mexo_drv_run_(double period, double time){
	emu_can_port_poll(&(mexo_mod.can_port_0));
	if (time >= mexo_mod.busyTime){
		mexo_mod.busyTime = time + mexo_mod.model_period_sec;
		mexo_mod.run();
		// модель можно считать чаще, чем работает mexo
		if (++mexo_mod.controlSystemTick == mexo_mod.controlSystemPrescale){
			mexo_priorityRun();
			mexo_run_step();
			mexo_mod.controlSystemTick = 0;
		}
		mexo_backgroundRun();
	}

}

extern "C"  void  mexo_drv_finish_(void){
	robo_can_close(&(mexo_mod.can_port_0.can));
}

void can0_on_receive_(robo_can_p _can, robo_can_msg_id_t _id, robo_byte_p _buf, robo_size_t _len){
	//printf("--> 0x%x%x%x\n", (unsigned int)((_id & 0xF00) >> 8), (unsigned int)((_id & 0xF0) >> 4), (unsigned int)(_id & 0xF));
#if CMD_MEXO_STREAM0_ENABLED == 1
	if ((((_id & 0x4F0)) >> 4 == mexo_mod.address0) || ((_id & 0xF0) == 0)){
		flow_msg0_on_receive(_id, _buf, _len);
	}
#endif
}
void can0_on_event_(robo_can_p _can, robo_can_event_t _ev){

}

extern "C"	IMAGE_DOS_HEADER __ImageBase;
extern "C"	uintptr_t MODULE_ADDRESS = (uintptr_t)&__ImageBase;

//это единственная экспортируемая функция
extern "C" ROBO_EXPORT int ROBO_EXPORT_RUNTIME_DECL dev_startup(emu_dev_agent_p _agent){
	char sec[ROBO_INI_SECTION_MAX_LEN + 1];
	char comm[50 + 1];
	int tmp;

	//Агент- это наш представитель на стороне эмулятора. Агент может существовать и без реализации. Например, в момент загрузки из ini
	_agent->instance = &mexo_mod;
	_agent->run = &mexo_drv_run_;
	_agent->finish = &mexo_drv_finish_;
	mexo_mod.agent = _agent;


	robo_sprintf(sec, ROBO_INI_SECTION_MAX_LEN, "DEVICE_%d", _agent->index);

	robo_ini_load_int(sec, "CAN0_ID", -1, &tmp);
	mexo_mod.can_port_0.can.channel = tmp;
	int addr;
	robo_ini_load_int(sec, "CAN0_ADDRESS", 0xF, &addr);


	robo_ini_load_int(sec, "CAN0_REPEAT_MAX_COUNT", -1, &tmp);
	mexo_mod.can_port_0.repeat_max_count = tmp;

	mexo_mod.can_port_0.can.on_receive = can0_on_receive_;
	mexo_mod.can_port_0.can.on_event = can0_on_event_;

	comm[0] = 0;
	robo_ini_load_str(sec, "UART1", "", comm, 50);
	if (comm[0] != 0)
		mexo_mod.uart0.connect(comm);
	comm[0] = 0;
	robo_ini_load_str(sec, "UART2", "", comm, 50);
	if (comm[0] != 0)
		mexo_mod.uart1.connect(comm);
	ROBO_CHECKRET(robo_can_open(&mexo_mod.can_port_0.can));


	ROBO_CHECK_LOAD_INT(sec, "ACTUATOR_INDEX", &tmp);

	robo_sprintf(sec, ROBO_INI_SECTION_MAX_LEN, "DEVICE_TYPE_%s", mexo_mod.agent->type);


	ROBO_CHECKRET(mexo_mod.load(sec, tmp));

	dc_moto::instance.perefery.pwm_max = mexo_mod.pwmMax;
	dc_moto::instance.begin();
#if BOARD_MEXO_NET_FLOW_ENABLED == 1	
	board::mexo_net_flow_set_addr(addr);
#endif
	return ROBO_SUCCESS;
}
void dc_moto::iperefery::var_attach_(robo_vartable_p _vartable){
	ROBO_VAR_PUSH(_vartable, "im");
	
	ROBO_VAR_REG(_vartable, mexo_mod.busyTime, "tm", ROBO_VAR_TYPE_CONST_UNSIGNED);
	ROBO_VAR_REG(_vartable, mexo_mod.motor.voltage, "voltage", ROBO_VAR_TYPE_CONST_UNSIGNED);
	ROBO_VAR_REG(_vartable, mexo_mod.motor.current, "current", ROBO_VAR_TYPE_CONST_FLOAT);
	ROBO_VAR_REG(_vartable, mexo_mod.motor.actuator.speed, "speed", ROBO_VAR_TYPE_CONST_FLOAT);
	ROBO_VAR_REG(_vartable, mexo_mod.motor.actuator.position, "position", ROBO_VAR_TYPE_CONST_FLOAT);
	ROBO_VAR_REG(_vartable, mexo_mod.motor.actuator.contr_torque, "contr_torque", ROBO_VAR_TYPE_CONST_FLOAT);
	ROBO_VAR_PUSH(_vartable, "gear");
	ROBO_VAR_REG(_vartable, mexo_mod.gearbox.driver.tension, "tension", ROBO_VAR_TYPE_CONST_FLOAT);
	ROBO_VAR_REG(_vartable, mexo_mod.gearbox.driver.tension_diff, "tension_diff", ROBO_VAR_TYPE_CONST_FLOAT);
	ROBO_VAR_REG(_vartable, mexo_mod.gearbox.torque.friction, "torque.friction", ROBO_VAR_TYPE_CONST_FLOAT);
	ROBO_VAR_REG(_vartable, mexo_mod.gearbox.torque.total, "torque.total", ROBO_VAR_TYPE_CONST_FLOAT);
	ROBO_VAR_POP(_vartable);
	ROBO_VAR_POP(_vartable);
}
#define LOAD_PARAM_I(Z) ROBO_CHECK_LOAD_INT( _sec, #Z ,&(Z) )
#define LOAD_PARAM_U(Z) ROBO_CHECK_LOAD_INT( _sec, #Z ,&tmpi ); Z = (unsigned)tmpi; 
#define LOAD_PARAM_F(Z) ROBO_CHECK_LOAD_FLOAT( _sec, #Z ,&(Z) )

#define LOAD_MOTOR_PARAM_F(Z) ROBO_CHECK_LOAD_FLOAT( _sec, #Z ,&(motor_config.Z) )



robo_result_t meXo_mod_t::load(const robo_string_t _sec, int _actuator_index){

	int tmpi;
	LOAD_PARAM_I(controlSystemPrescale);
	LOAD_PARAM_U(pwmModulo);
	LOAD_PARAM_U(pwmMax);

	LOAD_PARAM_F(voltageSupply);
	pwmGain = pwmModulo / voltageSupply;
	LOAD_PARAM_I(motorDriverPositionResolutionBit);
	LOAD_PARAM_I(motorDriverNoizePositionBit);
	mexo_mod.motorPositionNoizeMag = (2.0f*PIF) / (1 << (mexo_mod.motorDriverPositionResolutionBit - mexo_mod.motorDriverNoizePositionBit));
	LOAD_PARAM_F(motorPositionOffset);

	motorDriverPositionResolution = (1 << motorDriverPositionResolutionBit);
	mexo_mod.motorPositionGain = (float)(motorDriverPositionResolution) / (2.0f*PIF);

	LOAD_PARAM_F(currentSenceMax);
	LOAD_PARAM_F(model_period_sec);
	LOAD_PARAM_I(currentDriverResolutionBit);
	LOAD_PARAM_I(currentDriverNoizeBit);
	currentDriverMax = (1 << currentDriverResolutionBit) - 1;
	currentDriverZero = currentDriverMax >> 1;
	currentGain = (float)currentDriverMax / (2.0f*currentSenceMax);
	currentNoizeMag = (2.0f*currentSenceMax) / (1 << (currentDriverResolutionBit - currentDriverNoizeBit));

	if ( !motor.configure(_sec, model_period_sec) ) return ROBO_ERROR;
	if (!gearbox.configure(_actuator_index, _sec, model_period_sec)) return ROBO_ERROR;
	gearbox.connect_to_actuator(&motor.actuator);
	
	return ROBO_SUCCESS;
}


void idc_moto_test_pin_on(void){

}
void dc_moto_test_pin_off(void){

}
void dc_moto::iperefery::pwm_power_boot(void){
	mexo_mod.pwm_boot();
}
void dc_moto::iperefery::pwm_power_on(void){
	mexo_mod.pwm_on();
}
void dc_moto::iperefery::pwm_power_off(void){
	mexo_mod.pwm_off();
}
void dc_moto::iperefery::pwm_power_shutdown(void){
	mexo_mod.pwm_shutdown();
}
void dc_moto::iperefery::pwm_set(int  _duty){
	mexo_mod.pwm_set(_duty);
}
void dc_moto::iperefery::begin_(void){
	static robo::net::iserial *  serials_[2];
	pwm_max = mexo_mod.pwmMax;	
}
void dc_moto::iperefery::start_(void){
}
void dc_moto::iperefery::background_loop_(void){
}
void dc_moto::iperefery::permanent_loop_(void){
}
void dc_moto::iperefery::prioritet_loop_(void){
}



#if BOARD_MOTOR_POS_SENCE_TYPE == BOARD_MOTOR_POS_SENCE_TYPE_ABS_INTERACT_16

void dc_moto::iperefery::motor_enco_transmit(uint8_t * _x, uint8_t * _y, uint8_t _n){
	mexo_mod . 	motor_pos_query();
	_y[0] = (mexo_mod.motorPosition>>8) & 0xFF;
	_y[1] = mexo_mod.motorPosition & 0xFF;
}
#endif

void dc_moto::iperefery::delay_us(uint32_t _us){
}

#if BOARD_MEXO_NET_FLOW_ENABLED == 1	
#include "mexo/board/board+flow+uart+can-rev3.hpp"
#include "mexo/net/adapters/flow_msg_port.h"
robo_result_t  dc_moto::iperefery::flow_set_addr(uint8_t _addr){
	if (_addr > 0 && _addr < 16){
		mexo_mod.address0 = _addr;
		return ROBO_SUCCESS;
	}
	else {
		return ROBO_ERROR;
	}
}

#else
robo_result_t board_flow_msg0_send(uint16_t _id, mexo_data_p _data, mexo_data_t _len);
#endif

robo_result_t board_flow_msg0_send(uint16_t _id, mexo_data_p _data, mexo_data_t _len){
	ROBO_RETEX(robo_can_send(&(mexo_mod.can_port_0.can), _id, _data, _len));
}





