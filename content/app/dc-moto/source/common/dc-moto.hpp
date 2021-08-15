#ifndef __dc_moto_h
#define __dc_moto_h
#include "core/robosd_common.h"
#include "core/robosd_vartable.h"
#include "mexo/mexo.h"
#include "board_common.h"
#include "net/serial/robosd_serial.hpp"
#include "mexo/mexo.hpp"
#include "mexo/board/dcmoto/dcmoto_joint.hpp"

#define CURRENT_ACP_CH_COUNT 2

class dc_moto {
public:
	class icore{
		friend class dc_moto;
		robo_time_us_t overcurrent_tick_ = 0;
		robo_time_us_t tick_us_ = 0;
		uint32_t tick_raw_ = 0;
		bool tick_flag_ = false;
		void begin_(void);
		/*
		1 - вызывать в отдельном прерывании, если другие realtime  задачи.
		например, сетевой протокол манчестер
		2 - вызывать в прерывании от ADC, чтобы не плодить сущности
		*/
		void prioritet_loop_(void);
		void permanent_loop_(void);
		void background_loop_(void);

		void var_attach_(void);

		icore(void){};
		~icore(void){};
	public:
		robo_vartable_p vartable = 0;
		uint32_t adc_startupPause;
		void tick(void);
		robo_vartable_p find_vt(int id);
	} core;

	class iperefery{
		friend class dc_moto;
		iperefery(void){};
		~iperefery(void){};
		void var_attach_(robo_vartable_p);
		void begin_(void);
		void start_(void);
		void prioritet_loop_(void);
		void permanent_loop_(void);
		void background_loop_(void);
	public:
		unsigned int  pwm_max = 0; //максимальная величина ШИМ (с учетом всех ограничений)
		uint32_t adc_data[CURRENT_ACP_CH_COUNT];

		//Функции управления инвертором. Всеми режимами и их последовательностью  инвертора управляет mexo , для чего написан специальный автомат */
		/*вызывается, когда инвертор в рабочем состоянии. В массиве _pwm требуемые значения ШИМ*/
		void pwm_set(int  _duty);

		/*включаются драйверы, величина стартового ШИМ зависит от конкретных драйверов.
		скорее всего понадобится подтянуть обмотки либо к земле, либо к питанию, но скорее всего к земле
		*/
		void pwm_power_boot(void);

		/*силовой преобразователь переходит в режим динамического торможения - ШИМ, соответствующий 0
		(верхнии и нижние ключи всех фаз синхронно работают с 50% шим)
		*/
		void pwm_power_on(void);

		/*
		надо сделать так, чтобы все ключи выключились сразу, потому что возможно это происходит в аварийном режиме
		После выключения силовой преобразователь некоторое заданное время находится в покое, его нельзя включить сразу
		*/
		void pwm_power_off(void);

		/*
		в большей степени это ритуальная функция
		сигнал о том, что силовой преобразователь выключен полностью. Теперь можно его включать
		*/
		void pwm_power_shutdown(void);

		static void async_send(uint8_t _suba, uint8_t * _data, uint8_t _size);

#if BOARD_MOTOR_POS_SENCE_TYPE ==  BOARD_MOTOR_POS_SENCE_TYPE_INCREMENT
		uint16_t motor_position_query(void);
#endif





#if MEXO_SETTINGS_STORE_ENABLE == 1
#define RDK_SETTINGS_STORE 0
		robo_result_t save(uint8_t _bank, uint8_t * _data, robo_size_t _size);
		robo_result_t load(uint8_t _bank, uint8_t * _data, robo_size_t _size);
		robo_result_t store_reset(uint8_t _bank);
#endif

#if BOARD_MEXO_NET_FLOW_ENABLED == 1
		robo_result_t  flow_set_addr(uint8_t _addr);
#endif

#if BOARD_MOTOR_POS_SENCE_TYPE == BOARD_MOTOR_POS_SENCE_TYPE_ABS_INTERACT_16
		void motor_enco_transmit(uint8_t * _x, uint8_t * _y, uint8_t _n);
#endif

		static void delay_us(uint32_t _us);
	} perefery;


	void begin(void){
		perefery.begin_();
		core.begin_();
		core.var_attach_();
		perefery.var_attach_(core.vartable);
		robo_vartable_create_index(core.vartable);
		perefery.start_();
	}
	dc_moto(void) :core(), perefery(){}
private:
	static robo_result_t prioritet_loop_();
	static robo_result_t permanent_loop_();
	static robo_result_t background_loop_();	
};
extern dc_moto dc_moto_;


class dc_moto_machine : public mexo::machine::process {
public:
    struct  settings {
        //todo  в настройки
        unsigned int startup_pause = 1000;
        unsigned int shutdown_pause = 3000;
        unsigned int auotostop_pause = 100;
        unsigned int calibrate_timeout = 6000000 / BOARD_APP_TICK_PERIOD_US / 4;
    };
private:

    class release_task : public mexo::machine::task {
    protected:
        settings& settings_;
        actuator_control_data_p   control_data(void) {
            return (actuator_control_data_p)(joint.ps_dev.mexo_dev.proto.control);
        }
        mexo_dev_p mexo_dev(void) {
            return (mexo_dev_p) & (joint.ps_dev.mexo_dev);
        }

        virtual void onStartup(void);
        virtual void onShutdown(void);
        virtual mexo::machine::task::result doExecute();
        virtual mexo::machine::task::result doShutdown();
    public:
        release_task(settings& _settings) : mexo::machine::task(), settings_(_settings) {}
        virtual ~release_task(void) {};
    };

    class move_task : public release_task {
        int tick_ = 0;
    protected:
        bool update_need_ = false;
        virtual void onBegin(void);
        virtual result doBegin(void);
        virtual void onReset(void);
        virtual result doReset(void);
        virtual void onFinish(void);
    public:
        move_task(settings& _settings) : release_task(_settings) { }
        virtual ~move_task(void) {};
    };

    class posicioner : public move_task {
        friend class dc_moto_machine;
        mexo_signal_t speed_ = 0;
        mexo_long_signal_t position_ = 0;
        mexo_signal_t current_ = 0;
        bool autobrake_ = true;
        int auto_stop_counter_ = 0;
        virtual void onBegin(void);
        virtual result doExecute(void);
        virtual void onShutdown(void);
        posicioner(settings& _settings) : move_task(_settings) {}
    };

    class motion : public move_task {
        friend class dc_moto_machine;
        mexo_signal_t speed_ = 0;
        mexo_signal_t current_ = 0;
        motion(settings& _settings) : move_task(_settings) {}
        virtual void onBegin(void);
        virtual result doBegin(void);
        virtual result doExecute(void);
        virtual void onShutdown(void);
    };


    friend class calibrate;
    class calibrate : public move_task {
        friend class dc_moto_machine;
        enum class ioperation { stopped = 0, relax, forward, backward, go_forward, go_backward} operation;
        int dir_ = 1;
        mexo_signal_t speed_ = 0;
        mexo_signal_t current_ = 0;
        mexo_long_signal_t position_max_ = 0;
        mexo_long_signal_t position_min_ = 0;
				mexo_long_signal_t  move_position_=0;
        int tick_ = 0;
        int period_tick_ = 5000;
        robo_time_us_t begin_pause_us_ = 100000;
        calibrate(settings& _settings) : move_task(_settings) { }
        virtual ~calibrate(void) {};

        virtual void onBegin(void);

        virtual void onExecute(void);
        virtual void onShutdown(void);
        virtual result doExecute(void);
				bool complete_ = false;
				bool complete(void){return complete_;};
				void set_complete(void) {complete_ = true;}
    };
    

    release_task release_task_;
    posicioner posicioner_;
    motion motion_;
    calibrate calibrate_;
protected:
    virtual void doTerminate(void);
		int dir_ = 1;
    uint32_t enable_ = 1;

    int32_t command_ = 0;
    int32_t command_prev_ = 0;


    int32_t req_current_amp_ = BOARD_CURRENT_MAX;
    int32_t req_current_amp_prev_ = 0;

    int32_t req_speed_minpersec_ = 16;
    int32_t req_speed_minpersec_prev_ = 0;

    int32_t req_position_sec_ = 0;
    int32_t req_position_sec_prev_ = 0;
    int32_t actual_angle_sec_ = 0;
    int32_t actual_speed_minpersec_ = 0;
    int32_t actual_current_amp_ = 0;

    int32_t test_position_min_ = 0;
    int32_t test_position_max_ = 0;

    bool autpblock_ = true;

    void update_status_(void);

public:




    bool position_mode(int32_t _angle_millysecond, int32_t _speed_minpersec_max, int32_t  _current_amp, bool _autobrake);
    bool speed_mode(int32_t _speed_minpersec, int32_t  _current_amp);
    bool calibrate_mode(int32_t _angle_millysecond, int32_t _speed_minpersec, int32_t  _current_amp, int _dir);
    bool brake_set(void);
    bool brake_release(void);
    //bool calibrate_active(void) { return calibrate_.active_; }
    mexo_brake_state_t brake_state(void);


    void pool(void);

    void varreg(robo_vartable_p _vt);

    dc_moto_machine(settings& _settings);
    virtual ~dc_moto_machine(void) {}
		
		union exchange_data{
			struct {
				uint16_t position_2; //2
				int8_t speed_4;	
				int8_t current_16;
				int8_t voltage_256;
				union {
					struct{
						uint8_t command :3;
						uint8_t actuator_active : 1;
						uint8_t actuator_error : 1;
						uint8_t calibrated : 1;
					};
					uint8_t modes;
				};
			};
			uint8_t memo[8];
		};
		void send_answer_data(void);
		void applay_data( uint8_t * _in);
		bool calibrate_complerte_(void){ return calibrate_.complete(); };
		void calibrate_set_complerte_(){ calibrate_.set_complete(); }
};
#endif


