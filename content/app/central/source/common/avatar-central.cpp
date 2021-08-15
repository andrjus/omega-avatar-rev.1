#include <math.h>
#include <string.h>
#include <algorithm>
#define MODULE_NAME_STR RT("avatar")

#include "avatar-central.hpp"

#include "net/robosd_net_master.hpp"

#include "avatar-central-front.hpp"

#define ENABLE_ACTUATOR_XM_540 1
#include "dynamixel_controltable.inc.hpp"

#include "core/robosd_ring_buf.hpp"

#include "core/robosd_delegat.hpp"
#include "core/robosd_ini.hpp"
#include <math.h>
namespace avatar{
		robo::cstr  dynamixel_alias( int _id){
			if( _id == 1130){
				return RT("XM540_W150");
			} else if( _id == 1120){
				return RT("XM540_W270");
			} else if( _id == 350){
				return RT("XL320");
			} else if( _id == 1060){
				return RT("XL430_W250");
			} else return RT("MONAME");
		}

		typedef ::robo::backend::contrltable::var<uint8_t> uint8;
		typedef ::robo::backend::contrltable::var<uint16_t> uint16;
		typedef ::robo::backend::contrltable::var<uint32_t> uint32;
		typedef ::robo::backend::contrltable::fvar<float> real;
		class idev{
			public:
			virtual void query_snapshot(void) = 0;
			virtual void update_control(void) = 0;
			//virtual robo::result do_exchange(void) = 0;

			virtual void start_configure(void) = 0;
			virtual robo::result do_configure(void) = 0;

			virtual void start_calibrate_prepare(void) = 0;
			virtual robo::result do_calibrate_prepare(void) = 0;

			virtual void start_calibrate(void) = 0;
			virtual robo::result do_calibrate(void) = 0;

			virtual void start_tracking(void) = 0;
			virtual robo::result do_tracking(void) = 0;

			virtual void power_off(void) = 0;
			virtual robo::result do_shutdown(void) = 0;
		};
		class actuator : public ::robo::backend::idevagent, public idev {
		protected:
			float speed_max = 0.f;
			float  position_dead_zone = 0.f;
		public:
			struct joint_data{
				float voltage = 0;
				float current = 0;
				float speed = 0;
				float position = 0;
			};
			struct{
				joint_data feedback;
				joint_data action;
			} actual;
			
			float calibrate_position = 0.f;
			uint32_t homing_ofset = 0.f;
			struct{
				robo::converter * position;
			} converters;
			actuator(robo::cstr _name, ::robo::backend::boardagent & _owner) 
				: ::robo::backend::idevagent (_name, _owner){
			}			
			virtual bool do_load(void){
				ROBO_LBREAKN( ::robo::backend::idevagent::do_load() );
				ROBO_LBREAKN( robo::ini::load(name(),RT("speed_max"),speed_max));
				ROBO_LBREAKN( robo::ini::load(name(),RT("calibrate_position"),calibrate_position));
				ROBO_LBREAKN( robo::ini::load(name(),RT("position_dead_zone"),position_dead_zone));
				ROBO_LBREAKN( robo::ini::load(name(),RT("homing_ofset"),homing_ofset));
				return true;
			}
			
			virtual bool do_node_start(void){
				ROBO_LBREAKN( ::robo::backend::idevagent::do_node_start() );
				robo::string cvn;
				ROBO_LBREAKN(cvn.load(name(),RT("position_converter")));
				converters.position = robo::converter::find(cvn);
				ROBO_LBREAKN(converters.position!=nullptr);
				return true;
			}

		};
		
		class Dynamixel: public actuator {			
			::robo::delegat::smember<Dynamixel, void, ::robo::frontend::contrltable::ivar& , bool > startup_confirm_;
			::robo::delegat::smember<Dynamixel, void, ::robo::frontend::contrltable::ivar& , bool> snapshot_update_;
			::robo::delegat::smember<Dynamixel, void, ::robo::frontend::contrltable::ivar& , bool> configure_complete_;
			::robo::delegat::smember<Dynamixel, void, ::robo::frontend::contrltable::ivar& , bool> power_boot_;
			uint32_t start_position_ = 0;
			struct snapshot_s{
				union {
					uint32_t memo32[3];
					uint8_t memo[12];
					struct{
						union{
							uint32_t tmp;
							struct {
								uint16_t pwm;
								uint16_t current;
							};
						};
						uint32_t velocity;
						uint32_t position;
					};
				};
			};
			
			::robo::backend::contrltable  contrltable_;
			::robo::backend::contrltable::var<snapshot_s> snapshot_;
			uint16 model_;
			uint8  hardware_error_status_;
			uint8  power_;
			uint8  id_;

			uint32  speed_max_;
			uint16  speed_prop_gain_;
			uint16  speed_int_gain_;
			
			real  actual_position_;
			real  required_position_;
			real  position_max_;
			real  position_min_;
			uint32  home_offset_;
			
			void startup_confirm__( ::robo::frontend::contrltable::ivar& _var, bool _result){
				if(_result){
					if( 
						contrltable_.ready()
					){

						const int new_addr = 2;

						if( id_.value() == 1){
							id_.post(new_addr,startup_confirm_);
							return;
						}
						if( id_.value()==new_addr ){
							idevagent::dev_set_id(new_addr);
						}

						//if ( power_.try_post(1, startup_confirm_)  != robo::result::complete ) return ;
						//if ( required_position_.try_post(2000/*, startup_confirm_*/) != robo::result::complete ) return ;

						if ( power_.try_post(0, startup_confirm_)  != robo::result::complete ) return ;
						
						if( home_offset_.try_post(homing_ofset,startup_confirm_) != robo::result::complete  )return ;
						
						if( position_min_.conv() && position_min_.conv()->scale() >0 ){
							if ( position_min_.try_post_min(startup_confirm_)  != robo::result::complete ) return ;
							if ( position_max_.try_post_max(startup_confirm_)  != robo::result::complete ) return ;
						} else {
							if ( position_min_.try_post_max(startup_confirm_)  != robo::result::complete ) return ;
							if ( position_max_.try_post_min(startup_confirm_)  != robo::result::complete ) return ;
						}
						
						if ( speed_max_.try_post(actuator::speed_max, startup_confirm_)  != robo::result::complete ) return ;
						if ( speed_prop_gain_.try_post(100, startup_confirm_)  != robo::result::complete ) return ;

						configure_complete_(_var,_result);

					}
				}				
			}

			void power_boot__( ::robo::frontend::contrltable::ivar& _var, bool _result){
				/*if( required_position_.try_post(actual.action.position, power_boot_) == robo::result::resume ){
					return;
				}*/
				if(_result){
					if( power_.try_post(1, power_boot_) == robo::result::resume ){
						return;
					}
				}
			}
			
			void snapshot_update__( ::robo::frontend::contrltable::ivar& _var, bool _result){
				if(_result){
					actual.feedback.voltage = snapshot_.value().pwm;
					actual.feedback.current = snapshot_.value().current;
					actual.feedback.speed = snapshot_.value().velocity;
					actual.feedback.position = converters.position->to_float(snapshot_.value().position);
				}	
			}
			
			void configure_complete__(::robo::frontend::contrltable::ivar& _var, bool _result){
				if(_result){
					configure_complete();
						robo_infolog("'%s' startup info:\n\r\tmodel: '%s'(%d)\n\r\thardware_error_status: %d\n\r\tposition: %d"
							, alias()
							, dynamixel_alias((int)model_.value())
							,(int)model_.value()
							,(int)hardware_error_status_.value()
							,(int)actual_position_.value()						
						);
				}
			}
		public:				

		Dynamixel(::robo::backend::boardagent & _owner,robo::cstr _name) 
				: actuator (_name, _owner)
				, contrltable_(
					*this
					, ::robo::backend::contrltable::priority::hi
					, 1
					, ::robo::backend::dynamixel::records_xm
					, (sizeof( ::robo::backend::dynamixel::records_xm ) / sizeof(::robo::frontend::contrltable::record))  
				)
				, startup_confirm_(this, &Dynamixel::startup_confirm__ )
				, snapshot_update_(this, &Dynamixel::snapshot_update__)
				, configure_complete_(this, &Dynamixel::configure_complete__)
				, power_boot_(this, &Dynamixel::power_boot__)					
				, snapshot_(contrltable_,RT("PRESENT"))
				,	model_( contrltable_,RT("MODEL_NUMBER") )
				,	power_( contrltable_,RT("TORQUE_ENABLE") )
				, hardware_error_status_( contrltable_,RT("HARDWARE_ERROR_STATUS") )
				, actual_position_( contrltable_,RT("PRESENT_POSITION") )
				, required_position_(contrltable_,RT("GOAL_POSITION"))
				, id_(contrltable_,"ID")
				, position_min_(contrltable_,"MIN_POSITION_LIMIT")
				, position_max_(contrltable_,"MAX_POSITION_LIMIT")
				, speed_max_(contrltable_,"PROFILE_VELOCITY")
				, speed_prop_gain_(contrltable_,"POSITION_D_GAIN")
				, speed_int_gain_(contrltable_,"VELOCITY_I_GAIN")
				, home_offset_(contrltable_,"HOMING_OFFSET")
			{
			}
			
		protected:			
			virtual bool do_load(void){
				ROBO_LBREAKN(  actuator::do_load() ) ;
				return true;
			}
			
			virtual bool do_node_start(void){
				ROBO_LBREAKN( actuator::do_node_start() ) ;								
				ROBO_LBREAKN(actual_position_.set_converter(converters.position));
				ROBO_LBREAKN(required_position_.set_converter(converters.position));
				ROBO_LBREAKN(position_max_.set_converter(converters.position));
				ROBO_LBREAKN(position_min_.set_converter(converters.position));
				return true;
			}
		public:
			virtual void query_snapshot(void){
	//				ROBO_ALARMN(required_position_.post(actual.action.position) )
				ROBO_ALARMN(snapshot_.query(snapshot_update_));
			}
			virtual void update_control(void){
					ROBO_ALARMN(required_position_.post(actual.action.position) )
			}
	/*			virtual robo::result do_exchange(void){
				return contrltable_.ready() ?  robo::result::complete : robo::result::resume ;
			}*/

			virtual void start_configure(void){
				ROBO_ALARMN(contrltable_.query(startup_confirm_));
			}
			virtual robo::result do_configure(void){
				return local_state() == state::ilocal::ready ? robo::result::complete : robo::result::resume ;
			}

			virtual void start_calibrate_prepare(void){
				required_position_.post(actual.action.position, power_boot_);
			}

			virtual robo::result do_calibrate_prepare(void){
				return fabs( (float)(actual.feedback.position - actual.action.position) ) < position_dead_zone ? robo::result::complete : robo::result::resume ;
			}

			virtual void start_calibrate(void){
			}
			virtual robo::result do_calibrate(void){
				return robo::result::complete;
			}
			
			virtual void start_tracking(void){
				required_position_.post(actual.action.position, power_boot_);
			}
			
			virtual robo::result do_tracking(void){
				return  robo::result::resume;
			}

			virtual void power_off(void){
				power_.post(0);
			}
			virtual robo::result do_shutdown(void){
				return  power_.value() == 0? robo::result::complete : robo::result::resume;
			}
		};

		class Z : public actuator {			
		
			::robo::delegat::smember<Z, void, bool > startup_confirm_;
			::robo::delegat::smember<Z, void, bool > configure_complete_;
			::robo::delegat::smember<Z, void, bool > update_complete_;
			friend class module;

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
			exchange_data in_msg_;			
			exchange_data out_msg_;			
			friend class stream;
			
			class stream : public idevagent:: stream{
				Z & z_;
				bool request_ = false;
				::robo::delegat::base<void, bool > * delegat_ = nullptr;
				void delegat__(bool _res){ 
					if(delegat_) {
						(*delegat_)(_res); 
					}
				}
			public:
				virtual query_result query(robo_tran_p _tran){
					_tran->size_actual = 8;
					_tran->header.command = 0xD;
					_tran->request=	ROBO_TRAN_EXCANGE;
					std::copy_n(z_.out_msg_.memo,8,_tran->data);
					return query_result::success;
				}
				virtual void confirm(robo_tran_p _tran){
					if(_tran->status == ROBO_TRAN_COMPLETE){
						std::copy_n(_tran->data, 8,  z_.in_msg_.memo );
						request_ = false;
						delegat__(true);
					} else {
						delegat__(false);
					}
				}
				virtual bool exchange_need() { return request_; }
				stream(Z & _z): idevagent:: stream(_z, priority::hi ), z_(_z){}
				void query(::robo::delegat::base<void, bool > * _delegat ){ request_ = true; delegat_=_delegat;}
				void query( void ){ request_ = true; }
			} stream_;

			
			uint16_t start_position_ = 0;
			
			void startup_confirm__(bool _result){
				if(_result){
	/*						start_position_ = 	in_msg_.position_2 << 1;											
						actual.action.position = start_position_;*/
						out_msg_.command = 1;
						stream_.query(&configure_complete_);						
						configure_complete_(_result);
				}				
			}

			
			void snapshot_update__(  bool _result){
				if(_result){					
					actual.feedback.voltage = in_msg_.voltage_256<<8;
					actual.feedback.current = in_msg_.current_16<<4;
					actual.feedback.speed = in_msg_.speed_4 << 2;
					actual.feedback.position = converters.position->to_float( ((int32_t)(in_msg_.position_2)) << 1 );
				}				
			}
			
			void configure_complete__(bool _result){
				if(_result){
					configure_complete();
					robo_infolog("'%s' startup info:\n\r\tmodel: '%s'(%d)\n\r\thardware_error_status: %d\n\r\tposition: %d"
						, alias()
						, RT("Z")
						, 0
						, (int)in_msg_.actuator_error
						, (int)(start_position_)						
					);
				}
			}
			
		public:				

			Z(::robo::backend::boardagent & _owner,robo::cstr _name) 
			: actuator (_name, _owner)
				, stream_(*this)
				, startup_confirm_(this, &Z::startup_confirm__)
				, configure_complete_(this, &Z::configure_complete__)
				, update_complete_(this, &Z::snapshot_update__)
			{
			}
			
		protected:			
			virtual bool do_load(void){
				ROBO_LBREAKN(  actuator::do_load() ) ;
				return true;
			}
			
			virtual bool do_node_start(void){
				ROBO_LBREAKN(  actuator::do_node_start() ) ;								
				return true;
			}
		public:
			
		
			virtual void query_snapshot(void){
				stream_.query(&update_complete_);
			}
			/*virtual robo::result do_exchange(void){
				return robo::result::complete ;
			}*/
			virtual void update_control(void){
				out_msg_.position_2 = (uint16_t)(converters.position->to_u32(actual.action.position) >> 1);
				stream_.query(&update_complete_);
			}

			virtual void start_configure(void){
				stream_.query(&startup_confirm_);
			}
			virtual robo::result do_configure(void){
				return local_state() == state::ilocal::ready ? robo::result::complete : robo::result::resume ;
			}

			virtual void start_calibrate_prepare(void){
			}
			
			virtual robo::result do_calibrate_prepare(void){
				return robo::result::complete;
			}

			virtual void start_calibrate(void){
				
		#ifdef LEFT_HAND
				out_msg_.command = 5; //6 для правой!
		#endif
		#ifdef RIGHT_HAND
				out_msg_.command = 6;
		#endif
				
			out_msg_.position_2 = (uint16_t)(converters.position->to_u32(actual.action.position) >> 1);
			out_msg_.current_16 = ( 1800 >> 4); 
			out_msg_.voltage_256 = 127;
			out_msg_.speed_4 = ( (uint16_t)(speed_max/5) >> 2);// 1/5 от скорости 
			stream_.query(&update_complete_);
		}
			
		virtual void start_restore(uint16_t _position){
			out_msg_.command = 7;
			out_msg_.position_2 = _position;
			stream_.query(&update_complete_);
		}
		
		virtual robo::result do_restore(void){
				if( in_msg_.calibrated == 1){
					return robo::result::complete;
				} else {					 
					return robo::result::resume; 
				}
		}

		virtual robo::result do_calibrate(void){
				if( in_msg_.calibrated == 1){
					return robo::result::complete;
				} else {					 
					return robo::result::resume; 
				}
		}

		virtual void start_tracking(void){
			out_msg_.command = 4;
			actual.action.position = actual.feedback.position;
			out_msg_.position_2 = ((uint16_t)( converters.position->to_u32(actual.action.position) >> 1));
			out_msg_.current_16 = ( 1800 >> 4);  
			out_msg_.voltage_256 = 127;
			out_msg_.speed_4 = ((uint16_t)speed_max)>>2;
			stream_.query(&update_complete_); 
		}
		
		virtual robo::result do_tracking(void){
			float speed = abs( ((float)in_msg_.position_2 - out_msg_.position_2))/500*32;
			if (speed >speed_max){
				speed = speed_max;
			}
			out_msg_.position_2 = ((uint16_t)( converters.position->to_u32(actual.action.position) >> 1));
			out_msg_.speed_4 = ((uint16_t)speed)>>2;
			stream_.query(&update_complete_);
			return  robo::result::resume;
		}

		
		virtual void power_off(void){
			out_msg_.command = 0;
			stream_.query(&update_complete_);
		}
		
		virtual robo::result do_shutdown(void){
			return  in_msg_.actuator_active == 0? robo::result::complete :robo::result::resume;
		}
		
	};				
		
	class hand: public ::robo::backend::boardagent{ 
	public:
		hand(::robo::app::module & _owner) : ::robo::backend::boardagent(RT("hand"), _owner) {};
	};	
	
	class  grab: public ::robo::backend::boardagent{
	public:
		grab(::robo::app::module & _owner) : ::robo::backend::boardagent(RT("grab"), _owner) {};
		
		class fingers : public ::robo::backend::idevagent , public idev {
			friend class stream;
			friend class module;

			union exchange_data{
				struct{
					uint8_t status;
					uint8_t position[5];
				};
				uint8_t memo[6];
			};
			exchange_data in_msg_;			
			exchange_data out_msg_;
			
			class snapshot_stream : public idevagent:: stream{
				fingers & fingers_;
				bool request_ = false;
				::robo::delegat::base<void, bool > * delegat_ = nullptr;
				void delegat__(bool _res){ 
					if(delegat_) {
						(*delegat_)(_res); 
					}
				}
			public:
				virtual query_result query(robo_tran_p _tran){
					_tran->size_actual = 8;
					_tran->header.command = 0xA;
					_tran->request=	ROBO_TRAN_REQUEST_GET;							
					std::copy_n(fingers_.out_msg_.memo,8,_tran->data);
					return query_result::success;
				}
				virtual void confirm(robo_tran_p _tran){
					if(_tran->status == ROBO_TRAN_COMPLETE){
						std::copy_n(_tran->data, 6,  fingers_.in_msg_.memo );
						request_ = false;						
					} else {
					}
				}

			virtual bool exchange_need() { return request_; }
			
			snapshot_stream(fingers & _fingers): idevagent:: stream(_fingers, priority::hi ), fingers_(_fingers){}
				void query(::robo::delegat::base<void, bool > * _delegat ){ request_ = true; delegat_=_delegat;}
				void query( void ){ request_ = true; }
			} snapshot_stream_;
			
			fingers(grab & _owner,robo::cstr _name) 
			: ::robo::backend::idevagent (_name, _owner)
				, snapshot_stream_(*this)
			{
				configure_complete();
			}

			virtual void query_snapshot(void){
					snapshot_stream_.query();
			}
			virtual void update_control(void){
			}
			//virtual robo::result do_exchange(void) = 0;

			virtual void start_configure(void){
				snapshot_stream_.query();
			}
			virtual robo::result do_configure(void){
				return robo::result::complete; 
			}

			virtual void start_calibrate_prepare(void){
			}
			virtual robo::result do_calibrate_prepare(void){
				return robo::result::complete; 
			}

			virtual void start_calibrate(void){
			}
			virtual robo::result do_calibrate(void){
				return robo::result::complete; 
			}

			virtual void start_tracking(void){
			}
			virtual robo::result do_tracking(void){
				return robo::result::complete; 
			}

			virtual void power_off(void){
			}
			virtual robo::result do_shutdown(void){
				return robo::result::complete; 
			}
			
			virtual void apply_action(void){
			}
			virtual void uppdate_feedback(void){
			}	
		};			
	};
}

uint32_t dinamyxel_p2_encode(
	uint8_t _id 
	, uint8_t _instruction 
	, uint8_t * _param
	, uint16_t _length
	, uint8_t * _msg
);


uint32_t dinamyxel_p2_encode_param(
	uint8_t _id 
	, uint16_t  _address
	, uint8_t * _data
	, uint16_t _length
	, uint8_t * _msg
);

	
namespace avatar {

	typedef robo::net::master_t<dynamixel_rs485_channel> joint_channel_t;
	joint_channel_t joint_channel;
	typedef robo::net::master_t<dynamixel_TTL_channel> grab_channel_t;
	grab_channel_t grab_channel;
	typedef robo::net::master_t<CAN_channel> can_channel_t;
	can_channel_t can_channel;
	

	class dynamixel_bus: public robo::backend::bus {
		uint8_t in_buffer[64];
		uint8_t out_buffer[64];
		robo::ring_t<3, msg *> messages_;
		robo::net::master & channel_;
	protected:
		virtual bool do_load(void){
			ROBO_LBREAKN( ::robo::backend::bus::do_load()) ;
			static uint8_t tran_data[1][234];
			for (int i = 0; i < 1; i++){
				msg * tmp = new msg;
				tmp->tran.size_max = 234;
				tmp->tran.data = tran_data[i];
				messages_.put(tmp);
			}
			return true;
		}
		
		::robo::delegat::smember<dynamixel_bus, void, bool > confirm_delegat;
		
		msg * posted_ =nullptr;
		uint16_t wait_len_ = 0;
		void exchange_confirm( bool _result){
			if(posted_ != nullptr &&  _result) {
				uint8_t * in = in_buffer;
				uint8_t * tmp = in;
				if( *in!=0xFD) {
					if( *in==0xFF) in++;
					if( *in==0xFF) in++;
				}
				do{
					if( *in++ != 0xFD) break;
					if( *in++ != 0) break;
					if(*in++ != posted_->address && (posted_->tran.header.command != 0x7) ) break;
					uint16_t len = * ((uint16_t *)in );
					if(len != wait_len_ ) break;

					in +=2;
					if( *in++ !=0x55) break;
					if(*in++ !=0x00) break;
					if(len >4){
						len -=4;
						std::copy_n(in,len,posted_->tran.data);
					}
					bus::confirm(ROBO_TRAN_COMPLETE);
					return;
				}
				while(false);
			}
			bus::confirm(ROBO_TRAN_REFUSE);
			posted_ = nullptr;
		}
		
		
		virtual bool post(msg* _msg){
				posted_ = _msg;			
			if( _msg->tran.request == ROBO_TRAN_REQUEST_PUT ){
				uint32_t sz = dinamyxel_p2_encode_param(
					_msg->address
					, _msg->suba
					, _msg->tran.data
					, _msg->tran.size_actual
					, out_buffer
				);
				in_buffer[7] = 0;
				in_buffer[8] = 0xff;
				wait_len_ = 4;
				channel_.exchange(out_buffer,sz, &(in_buffer[0]),11,&confirm_delegat);
			} else {
				union{
					struct{
						uint16_t addr;
						uint16_t len;
					};
					uint8_t memo[4];
				};
				addr = _msg->suba;
				len = _msg->tran.size_actual;
				
				uint32_t sz = dinamyxel_p2_encode(
					_msg->address
					, 0x02
					, memo
					, 4
					, out_buffer
				);
	
				in_buffer[7] = 0;
				in_buffer[8] = 0xff;
				wait_len_ = 4+len;
				channel_.exchange(out_buffer,sz, &(in_buffer[0]),11+len,&confirm_delegat);
			}
			return true;
		}				
		
		virtual void cancel(void){
				channel_.cancel();					
				posted_ = nullptr;
			}
			virtual bool ready(void){
				return channel_.ready();
			}
			//msg
			virtual msg* get_msg(void){
				if (messages_.available()){
					return messages_.get();
				}
				else {
					return nullptr;
				}
			}
			virtual void  release_msg(msg* _msg){
				if (!messages_.full() && _msg){
					return messages_.put((msg *)_msg);
				}
			}
	public:
		dynamixel_bus(  robo::cstr _name, robo::net::master & _channel  );
	};
	
	class can8_bus: public robo::backend::bus {
		uint8_t in_buffer_[10];
		uint8_t out_buffer_[10];
		robo::ring_t<3, msg *> messages_;
		robo::net::master & channel_;
	protected:
		virtual bool do_load(void){
			ROBO_LBREAKN( ::robo::backend::bus::do_load()) ;
			static uint8_t tran_data[1][8];
			for (int i = 0; i < 1; i++){
				msg * tmp = new msg;
				tmp->tran.size_max = 8;
				tmp->tran.data = tran_data[i];
				messages_.put(tmp);
			}
			return true;
		}
		
		::robo::delegat::smember<can8_bus, void, bool > confirm_delegat;
		
		msg * posted_ =nullptr;
		uint16_t wait_id_ = 0xFF;
		void exchange_confirm( bool _result){
			if(posted_ != nullptr &&  _result) {				
				std::copy_n(in_buffer_+2, 8, posted_->tran.data);
				bus::confirm(ROBO_TRAN_COMPLETE);
			} else {
				bus::confirm(ROBO_TRAN_REFUSE);
				posted_ = nullptr;
			}
		}
		
		
		virtual bool post(msg* _msg){
			posted_ = _msg;			
				uint16_t id=((_msg->address &0xF)<<4)+((_msg->suba)&0xF);
				*((uint16_t *)in_buffer_) = wait_id_ = 0x400+id;
				if( posted_->tran.request == ROBO_TRAN_EXCANGE) {
					*((uint16_t *)out_buffer_) = 0x200+id; 
					std::copy_n(posted_->tran.data,posted_->tran.size_actual, out_buffer_+2);
					channel_.exchange(out_buffer_, posted_->tran.size_actual+2, in_buffer_,posted_->tran.size_actual+2,&confirm_delegat);
				} else if( posted_->tran.request == ROBO_TRAN_REQUEST_PUT){
					*((uint16_t *)out_buffer_) = 0x200+id; 
					std::copy_n( posted_->tran.data,posted_->tran.size_actual, out_buffer_+2);
					channel_.exchange(out_buffer_, posted_->tran.size_actual+2, in_buffer_,0,&confirm_delegat);
				} else if( posted_->tran.request == ROBO_TRAN_REQUEST_GET){
					*((uint16_t *)out_buffer_) = 0x00+id; 
					out_buffer_[2] = 6;
					channel_.exchange(out_buffer_, 3 , in_buffer_,posted_->tran.size_actual+2,&confirm_delegat);
				}
				return true;
		}				
		
		virtual void cancel(void){
				channel_.cancel();					
				posted_ = nullptr;
			}
			virtual bool ready(void){
				return channel_.ready();
			}
			//msg
			virtual msg* get_msg(void){
				if (messages_.available()){
					return messages_.get();
				}
				else {
					return nullptr;
				}
			}
			virtual void  release_msg(msg* _msg){
				if (!messages_.full() && _msg){
					return messages_.put((msg *)_msg);
				}
			}
	public:
		can8_bus(  robo::cstr _name );
	};
	
	
	void dynamixel_rs485_channel::confirm(void){
		joint_channel.confirm();
		robo::system::env::wakeup();
	}
	void dynamixel_TTL_channel::confirm(void){
		grab_channel.confirm();
		robo::system::env::wakeup();
	}
	void CAN_channel::confirm(void){
		can_channel.confirm();
		robo::system::env::wakeup();
	}
	void dynamixel_rs485_channel::refuse(void){
		joint_channel.refuse();
		robo::system::env::wakeup();
	}
	void dynamixel_TTL_channel::refuse(void){
		grab_channel.refuse();
		robo::system::env::wakeup();
	}
	void CAN_channel::refuse(void){
		can_channel.refuse();
		robo::system::env::wakeup();
	}
	
	class module : public robo::app::module {		
		::robo::delegat::member<::robo::backend::repeater, module, void> control_loop_machine_;
		hand hand_;
		grab grab_;
		Z z_1_;
		Dynamixel yaw_2_;
		Dynamixel yaw_3_;
		Dynamixel roll_4_;
		Dynamixel pitch_5_;
		Dynamixel roll_6_;
		Dynamixel grab_7_;
		Dynamixel grab_8_;
		grab::fingers fingers_;
		int converter_count_=0;
		robo::converter  * * converters_;
		enum { count = 8 };
		actuator * actuators[count];
		
		void printf(void){
			float ap[count];
			{
				robo::system::guard g__;
				get_actual_position();
				std::copy_n(actuator_actual_,count, ap);
			}
			robo::system::printf(RT("\033[0;0H"));
			for(int i=0;i<count; ++i){
				robo::system::printf(RT("                                                   \r"));
				robo::system::printf(RT("\t%s\t%2.2f\n\r"), actuators[i]->alias(), ap[i] );
			}
		/*	
			if( mode_ == mode::tracking){
				for(int i=0;i<count; i++){
					robo::system::printf(RT("\t%2.2f"), actuator_req_[i]);
				}
			} else {
				human_to_actuator(human_actual_,test_actuator);
				actuator_to_human(test_actuator, test_human);
				for(int i=0;i<count; i++){
					test_human[i]-= human_actual_[i];
					test_actuator[i]-= actuator_actual_[i];
				}
				
				for(int i=0;i<count; i++){
					robo::system::printf(RT("\t%2.2f"), test_actuator[i]);
				}
			}
			robo::system::printf(RT("\t"), 0);
			if( mode_ == mode::tracking){
				for(int i=0;i<count; i++){
					robo::system::printf(RT("\t%2.2f"), human_req_[i]);
				}	 
			}else {
				for(int i=0;i<count; i++){
					robo::system::printf(RT("\t%2.2f"), test_human[i]);
				}
			}
			
			robo::system::printf( RT("\n\r") );
			
			for(int i=0;i<count; i++){
				robo::system::printf(RT("\t%2.2f"), actuator_actual_[i]);
			}

			robo::system::printf(RT("\t\t"), 0);
			for(int i=0;i<count; i++){
					robo::system::printf(RT("\t%2.2f"), human_actual_[i]);
			}
			
			robo::system::printf( RT("\n\r\n\r") );
			*/
		}
		void for_all( void (actuator::* _f )(void) ) {
			actuator * * p=actuators;
			for(int i=0;i<count; i++, p++){
				if(*p){
				((*p)->*_f)();
				}
			}
		}
		
		robo::result do_all( robo::result (actuator::* _f )(void) ) {
			robo::result r = robo::result::complete;
			actuator * * p=actuators;
			for(int i=0;i<count; i++, p++){
				if(*p){
					switch( ((*p)->*_f)()){
						case robo::result::resume:
							r = robo::result::resume;
							break;
						case robo::result::complete:
							break;
						case robo::result::panic:
							return robo::result::panic;
					}
				}
			}
			return r;
		}
		
		void query_snapshot(void){
			//todo не здесь!
			get_actual_position();
			for_all(&actuator::query_snapshot);
		}
		void update_control(void){
//			get_actual_position();
			for_all(&actuator::update_control);
		}
		
		/*robo::result do_exchange(void){
			return do_all(&actuator::do_exchange);
		}*/


		enum class mode{ booting, configure, idle,restore ,not_calibrated, prepare_calibrate, calibrate, ready,  tracking, panic, power_off,shutdown, off} mode_ = mode::booting;
		enum class command{ none, calibrate,tracking, ready } command_ =
		//command::calibrate;
		command::none;
		//command::tracking;

		void start_configure(void){
			for_all(&actuator::start_configure);
			mode_ = mode::configure;
		}
		robo::result do_configure(void){
			return do_all(&actuator::do_configure);
		}
		
		static void actuator_to_human(float * _actuator, float * _human);
		static float human_to_actuator(float * _human, float * _actuator);

		void set_calibrate_position(void){
			actuator ** p = actuators;			
			for(int i=0;i<count; i++, p++){
				if(*p) {
					actuator_req_[i] = (*p)->actual.action.position =  (*p)->calibrate_position;
				} else {
					actuator_req_[i] = (*p)->actual.action.position =  0.f;
				}
			}
			actuator_to_human( &actuator_req_[0], &human_req_[0]);
		}
		void start_calibrate_prepare(void){
			set_calibrate_position();
			for_all(&actuator::start_calibrate_prepare);
			mode_ = mode::prepare_calibrate;
		}

		robo::result do_calibrate_prepare(void){
			update_control();
			return do_all(&actuator::do_calibrate_prepare);
		}

		void start_calibrate(void){
			for_all(&actuator::start_calibrate);
			mode_ = mode::calibrate;
			command_= command::calibrate;
		}
		robo::result do_calibrate(void){
			update_control();
			return do_all(&actuator::do_calibrate);
		}
		
		float actuator_actual_[count];
		float actuator_req_[count];
		float human_actual_[count];
		float human_req_[count];

		float human_actual_prev_[count];
		float human_actual_speed_[count];
		float human_deseired_[count];

		void get_actual_position(void){
			actuator ** p = actuators;			
			for(int i=0;i<count; i++, p++){
				if(*p) {
					actuator_actual_[i] = (*p)->actual.feedback.position;
				} else {
					actuator_actual_[i] =   0.f;
				}
			}
			actuator_to_human( &actuator_actual_[0], &human_actual_[0]);
		}
		
		void hold_position(void){
			actuator ** p = actuators;			
			for(int i=0;i<count; i++, p++){
				if(*p) {
					actuator_req_[i] = (*p)->actual.action.position =  actuator_actual_[i];
				} else {
					actuator_req_[i] = (*p)->actual.action.position =  0.f;
				}
			}
			actuator_to_human( &actuator_req_[0], &human_req_[0]);
			for(int i=0;i<count; i++, p++){
				human_deseired_[i] = human_req_[i];
			}
			for(int i=0;i<count; i++, p++){
				human_actual_prev_[i] = human_req_[i];
			}
		}
		
		void set_req_position(void){
			actuator ** p = actuators;			
			for(int i=0;i<count; i++, p++){
				if(*p) {
					(*p)->actual.action.position  =  actuator_req_[i];
				} 
			}
		}

		void start_ready(void){
			hold_position();
			//todo
			for_all(&actuator::start_tracking);
			mode_ = mode::ready;
			command_= command::none;
		}
		
		void start_tracking(void){
			human_to_actuator(&human_deseired_[0], &actuator_req_[0]);
			set_req_position();
			for_all(&actuator::start_tracking);
			mode_ = mode::tracking;
			command_= command::tracking;
		}

		robo::result do_tracking(void){
			static robo::time_ms_t last_ms = 0;
			robo::time_ms_t ms = robo::system::env::time_ms();
			if( ms - last_ms>=100){				
				const float T= 0.1f;
				last_ms = ms;
				
				/*float rK[3] = { human_req_[0], human_req_[1], human_req_[2]};
				float r0[3] = { human_actual_[0], human_actual_[1], human_actual_[2]};
				for(int i=0;i<count; i++){
					human_actual_speed_[i] = (human_actual_[i] - human_actual_prev_[i]) / T;
 					human_actual_prev_[i] = human_actual_[i];
				}*/
				float ro[count];
				float mro = 0;
				for(int i=0; i<count;i++){
					ro[i] = human_req_[i] - human_deseired_[i];
					mro +=  (ro[i]*ro[i]);
				}
				mro = sqrt(mro);
				float step = 0.f;
				if(mro>1.f){
					float norm[count];
					//float V0 = 0;
					for(int i=0; i<count;i++){
						norm[i] =  ro[i]/mro;
						//V0 += human_actual_speed_[i]*norm[i];
					}		
					const float Vmax = 200.f;
					float dt = mro/Vmax;
					if( dt > T ){
						dt=T;
					}
					step = Vmax*dt;
					
					/*float Vmax;
					float D = Amax*mro-V0/2;
					float step;
					if(D>0){
						Vmax = sqrt( D);
						float dt = (Vmax - V0)/Amax;
						if( dt > T ){
							dt=T;
						}
						step = V0*dt + Amax*dt*dt/2;
						if(step>mro){
							step = mro;
						}
					} else{
						float dt = V0/Amax;
						if( dt > T ){
							dt=T;
						}
						step = V0*dt - Amax*dt*dt/2;						
					}*/
					for(int i=0;i<6;i++){
						human_deseired_[i] = human_deseired_[i] + step*norm[i];
					}
					/*for(int i=0;i<count;i++){
						human_deseired_[i] = human_req_[i];
					}*/
					human_deseired_[6] = human_req_[6];
					human_deseired_[7] = human_req_[7];
				}
				human_to_actuator(&human_deseired_[0], &actuator_req_[0]);
				set_req_position();
				update_control();
				
			}
			return do_all(&actuator::do_tracking);
		}
		
		void power_off(void){
			for_all(&actuator::power_off);
			mode_ = mode::shutdown;
		}
		robo::result do_shutdown(void){
			return do_all(&actuator::do_shutdown);
		}
		

		void panic(void){			
			power_off();
			mode_ = mode::panic;
		}
		
		::robo::time_ms_t start_time_=0;
		
		void control_loop_machine__(void){
			switch (mode_){
			case mode::booting:
				start_configure();
				break;
			case mode::configure:
				switch(do_configure()){
				case robo::result::resume:						
				break;
				case robo::result::complete:
					uint32_t old_position;
					if(load_state(old_position)){
						z_1_.start_restore(old_position);
						mode_ =mode::restore;
					} else {
						start_time_ = robo::system::env::time_ms();
						mode_ =mode::idle;
					}						
				break;
				case robo::result::panic:
					panic();
				break;
				};
				break;
			case mode::restore:
				if(z_1_.do_restore() == robo::result::complete){
					start_time_ = robo::system::env::time_ms();
					mode_ =mode::idle;
				}
			break;
			case mode::idle:
				if(robo::system::env::time_ms() - start_time_ > 2000){
					if (z_1_.in_msg_.calibrated){//?
						start_ready();
					}  else {
							mode_ = mode::not_calibrated;
							//command_ = command::calibrate;
					}					
				}
			break;
			case mode::not_calibrated:
				switch(command_){
				case command::calibrate:
					start_calibrate_prepare();
					break;
				default:
					break;
				};				
				break;
				
			case mode::prepare_calibrate:
				if(command_!= command::calibrate){
					power_off();
				} else {
					switch(do_calibrate_prepare()){
					case robo::result::resume:						
					break;
					case robo::result::complete:
						start_calibrate();
					break;
					case robo::result::panic:
						panic();
					}
				}
				break;

			case mode::calibrate:
				if(command_!= command::calibrate){
					power_off();
				} else {
					switch(do_calibrate()){
					case robo::result::resume:						
					break;
					case robo::result::complete:
						start_ready();
					break;
					case robo::result::panic:
						panic();
					}
				}
				break;

			case mode::tracking:
				switch(command_){
					case command::tracking:
						switch(do_tracking()){
							case robo::result::resume:						
							break;
							case robo::result::complete:
								start_ready();
							break;
							case robo::result::panic:
								panic();							
						}
					break;
					case command::calibrate:
						start_calibrate_prepare();
					break;
					case command::ready:
						start_ready();
					break;
					default:
					break;
				}
				break;

			case mode::shutdown:
				switch(do_shutdown()){
				case robo::result::resume:						
				break;
				case robo::result::complete:
					//todo save
					if(z_1_.in_msg_.calibrated){
						if( save_state(z_1_.in_msg_.position_2) ){
							mode_ = mode::off;
						} else {
							panic();
						}
					}else {
							mode_ = mode::off;
					}
				break;
				case robo::result::panic:
					panic();
				}
				break;

			case mode::panic:
				break;
			case mode::off:
				break;
			case mode::power_off:
				power_off();
				break;
			case mode::ready:
				update_control();
				switch(command_){
					case command::tracking:
						start_tracking();
					break;
					case command::calibrate:
						start_calibrate();
					break;
					default:
						break;
				}
				break;
			}
		//	if(mode_ > mode::configure){
				query_snapshot();	
		//	}
		}
	protected:
		virtual void frontend_loop(void) {
				static robo::time_us_t tm =0;
				if( robo::system::env::time_ms()-tm>100){
					printf();
					tm = robo::system::env::time_ms();
				}
		};
		virtual void backend_loop(void) {
			joint_channel.poll();
			grab_channel.poll();
			can_channel.poll();
			::robo::backend::bus::perform();
			::robo::backend::task::machine::execute();
		};
		float test_human[count];
		float test_actuator[count];

		virtual bool 	do_load(void){		
			ROBO_LBREAKN(robo::app::module::do_load() );
			ROBO_LBREAKN(robo::ini::load(name(),RT("converter_count"), converter_count_) );
			if(converter_count_>0){
				converters_ = new robo::converter  *  [converter_count_];
				for( int i=0;i<converter_count_;++i){
					converters_[i] = nullptr;
				}
				robo::string key;
				robo::string cvn;
				for( int i=0;i<converter_count_;++i){
					key.format(RT("converter_%d"),i+1);
					cvn.load(name(),key);
					robo::converter * cv = new robo::converter(cvn, this);					
					converters_[i] = cv;
					ROBO_LBREAKN( cv->load() );
				}
			}
			
			joint_channel.begin();
			grab_channel.begin();
			can_channel.begin();
			return true;
		}
		virtual void 	do_clean(void){		
			robo::app::module::do_clean();
			if(converters_){
				for( int i=0;i<converter_count_;++i) if(converters_[i]) delete converters_[i];
				delete []converters_;
			}
		}
		virtual bool 	do_node_start(void){		
			ROBO_LBREAKN(robo::app::module::do_node_start() );
			joint_channel.start();
			grab_channel.start();
			can_channel.start();
			control_loop_machine_.start(100000);
			/*content c;
			c.run(nullptr);*/
			return true;
		}
		struct{
			float L1 = 400;
			float L2 = 300;
			float L3 = 80;
			float L5 = 50;
		}gm;
		module(void) 
			: robo::app::module(MODULE_NAME_STR )
			, control_loop_machine_(this, &module::control_loop_machine__) 
			, hand_(*this)
			, grab_(*this)
			, z_1_(hand_,RT("z-1"))
			, yaw_2_(hand_,RT("yaw-2"))
			, yaw_3_(hand_,RT("yaw-3"))
			, roll_4_(hand_,RT("roll-4"))
			, pitch_5_(hand_,RT("pitch-5"))
			, roll_6_(hand_,RT("roll-6"))
			, grab_7_(hand_,RT("grab-7"))
			, grab_8_(hand_,RT("grab-8"))
			, fingers_(grab_,RT("fingers"))
		{
			actuators[0] = &z_1_;
			actuators[1] = &yaw_2_;
			actuators[2] = &yaw_3_;
			actuators[3] = &roll_4_;
			actuators[4] = &pitch_5_;
			actuators[5] = &roll_6_;
			actuators[6] = &grab_7_;
			actuators[7] = &grab_8_;
		}
		void move_to_(const point & _point){
			if(mode_ != mode::off){
				robo::system::guard g__;
				std::copy_n(_point.values,7,human_req_);
				command_ = command::tracking;
			}
		}
		
		void start_calibrate_(void){
			command_ = command::calibrate;			
		}
		void stop_(void){
			if(mode_ != mode::off){
				command_ = command::ready;			
			}
		}
		
		void query_status_(status & _status){
			mode tmp;
			{
				robo::system::guard g__;
				tmp = mode_;
				std::copy_n(human_actual_,7,_status.pt.values);
			}
			switch(tmp){
				case mode::booting: 
				case mode::configure:
				case mode::idle:
				case mode::restore:
					_status.state = status::states::BOOTING;
				break;
				case mode::not_calibrated:
					_status.state = status::states::NOT_CALIBRATED;
				break;
				case mode::prepare_calibrate:
				case mode::calibrate:
					_status.state = status::states::CALIBRATING;
				break;
				case mode::ready:
					_status.state = status::states::READY;
				break;
				case mode::tracking:
					_status.state = status::states::MOVING;
				break;
				case mode::panic:
					_status.state = status::states::FAIL;
				break;
				case mode::power_off:
				case mode::shutdown:
					_status.state = status::states::SHUTTING_DOWN;
				break;
				case mode::off:
					_status.state = status::states::OFF;
				break;
			}
		}
		void reset_(void){
//			power_off();
			if(mode_ != mode::off){
				mode_ = mode::booting;
				command_ = command::none;
			}
		}
		void shutdown_(void){
//			power_off();
			if(mode_ != mode::off){
				mode_ = mode::power_off;
				command_ = command::none;
			}
		}

	public:
		static void calibrate(void){
			instance().start_calibrate_();
		}

		static void stop(void){
			instance().stop_();
		}

		static void move_to(const point & _point){
			instance().move_to_(_point);
		}

		static void shutdown(void){
			instance().shutdown_();
		}
		static void reset(void){
			instance().reset_();
		}

		static module& instance(void) {
			static module instance_;
			return instance_;
		}
		
		static void query_status(status & _status){
			instance().query_status_(_status);
		}

	};
	
	dynamixel_bus::dynamixel_bus( robo::cstr _name, robo::net::master & _channel  )
			: robo::backend::bus( _name, &module::instance())
			, confirm_delegat(this, &dynamixel_bus::exchange_confirm)
			, channel_(_channel){
	}
	can8_bus::can8_bus( robo::cstr _name)
			: robo::backend::bus( _name, &module::instance())
			, confirm_delegat(this, &can8_bus::exchange_confirm)
			, channel_(can_channel){
				
	}
	dynamixel_bus  dynamixel_rs485_bus(RT("dynamixel_rs485_bus"), joint_channel);
	dynamixel_bus  dynamixel_TTL_bus(RT("dynamixel_TTL_bus"), grab_channel);
	can8_bus  can8_bus_(RT("can8_bus"));
				//dynamixel_rs485_bus
	::robo::backend::router   fake_rs485_router_("fake_router", module::instance());

};

#define MODULE_NAME  avatar
#include "core/robosd_system_module_reg.hpp"


	
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    static const unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}

/*
	Name  : CRC-8
  Poly  : 0x31    x^8 + x^5 + x^4 + 1
  Init  : 0xFF
  Revert: false
  XorOut: 0x00
  Check : 0xF7 ("123456789")
  MaxLen: 15 байт (127 бит) - обнаружение
    одинарных, двойных, тройных и всех нечетных ошибок
*/



uint8_t robo_crc8_by_table( uint8_t * _pdata, uint8_t _len)
{
	static const uint8_t crc8_table[256] = {
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97,
    0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
    0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4,
    0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
    0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11,
    0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
    0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52,
    0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
    0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
    0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
    0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9,
    0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
    0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C,
    0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
    0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F,
    0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
    0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED,
    0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
    0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE,
    0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
    0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B,
    0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
    0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28,
    0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
    0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0,
    0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
    0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
    0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A,
    0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56,
    0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
    0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15,
    0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC
};
    uint8_t crc = 0xFF;

    while (_len--)
        crc = crc8_table[crc ^ *_pdata++];

    return crc;
}


uint32_t dinamyxel_p2_encode(
	uint8_t _id 
	, uint8_t _instruction 
	, uint8_t * _param
	, uint16_t _length
	, uint8_t * _msg
){
	uint16_t packet_length =  _length + 3;
	uint8_t * _ptr = _msg;
	*_ptr++ = 0xFF;
	*_ptr++ = 0xFF;
	*_ptr++ = 0xFD;
	*_ptr++ = 0x00;
	*_ptr++ = _id;
	*_ptr++ = (uint8_t)(packet_length & 0xFF);
	*_ptr++ = (uint8_t)((packet_length >> 8 ) & 0xFF);
	*_ptr++ = _instruction;
	while(_length--){
		*_ptr++ = *_param++;
	}
	uint16_t crc= update_crc(0, _msg, packet_length+5);
	*_ptr++ = (uint8_t)(crc &0xff);
	*_ptr++ = (uint8_t)((crc>>8) &0xff);
	return (uint32_t)packet_length + 7;
}

uint32_t dinamyxel_p2_encode_param(
	uint8_t _id 
	, uint16_t  _address
	, uint8_t * _data
	, uint16_t _length
	, uint8_t * _msg
){
	uint16_t  payload_length=2+_length;
	uint8_t payload[payload_length];
	uint8_t * _ptr = payload;
	*_ptr++  = (uint8_t)(_address & 0xFF);
	*_ptr++  = (uint8_t)( (_address>>8) & 0xFF);
	while(_length--){
		*_ptr++ = *_data++;
	}
	return dinamyxel_p2_encode(
		_id
		, 0x03
		, payload
	  , payload_length
		, _msg
	);
}
void avatar::move_to(const avatar::point & _point){
	module::move_to(_point);
}
void avatar::calibrate(void){
	module::calibrate();
}
void avatar::stop(void){
	module::stop();
}
void avatar::query_status(avatar::status & _status){
	module::query_status(_status);
}
void avatar::reset(void){
	module::reset();
}
void avatar::shutdown(void){
	module::shutdown();
}


void avatar::module::actuator_to_human(float * _actuator, float * _human){
	const float gain =robo::pi<float>/180.f;
	
	#ifdef RIGHT_HAND
	float q0 = (-_actuator[0])*gain;
	#endif
	#ifdef LEFT_HAND
	float q0 = (-_actuator[0]+180)*gain;
	#endif

	float q1 = -_actuator[1]*gain;
	
	#ifdef RIGHT_HAND
	float q2 = -_actuator[2]*gain;
	#endif
	#ifdef LEFT_HAND
	float q2 = _actuator[2]*gain;
	#endif
	
	float q3 = _actuator[3]*gain;

	#ifdef RIGHT_HAND
	float q4 = _actuator[4]*gain;
	#endif
	#ifdef LEFT_HAND
	float q4 = -_actuator[4]*gain;
	#endif

	float q5 = _actuator[5];
	
	float f1 = q1;		
	float f2=f1 + q2;
	float f3=f2 + q3;
	float L1= instance().gm.L1;
	float L2= instance().gm.L2;
	float L3= instance().gm.L3;
	float L5= instance().gm.L5;
	float x = L1 * ( cos( f1 ) ) + 
						L2 * ( cos( f2 ) );

		float y = L1 * ( sin( f1 ) ) + 
							L2 * ( sin( f2 ) );

		float z = q0;
	
/*		
% A(Q4,Q5) =  
% [          cos(q5),       0,          sin(q5)]
% [  sin(q4)*sin(q5), cos(q4), -cos(q5)*sin(q4)]
% [ -cos(q4)*sin(q5), sin(q4),  cos(q4)*cos(q5)]
% A(Y,P,R) = 
% [ cos(P)*cos(Y), - cos(R)*sin(Y) - cos(Y)*sin(P)*sin(R),   sin(R)*sin(Y) - cos(R)*cos(Y)*sin(P)]
% [ cos(P)*sin(Y),   cos(R)*cos(Y) - sin(P)*sin(R)*sin(Y), - cos(Y)*sin(R) - cos(R)*sin(P)*sin(Y)]
% [        sin(P),                          cos(P)*sin(R),                          cos(P)*cos(R)]
*/
		
    float P = asin(-cos(q4)*sin(q5));
    float R = atan2( sin(q4), cos(q4)*cos(q5));
    float Y = atan2( sin(q4)*sin(q5), cos(q5) ) - robo::pi<float>/2 + f3;
		
		_human[0] = x;
		_human[1] = y;
		_human[2] = z;
		_human[3] = Y/gain;
		_human[4] = P/gain;
		_human[5] = R/gain;
		_human[6] = _actuator[6]; 
		_human[7] = _actuator[7]; 

	}


float avatar::module::human_to_actuator(float * _human, float * _actuator){
	const float gain =180.f/robo::pi<float>;
	float xg =_human[0];
	float yg =_human[1];
	float zg =_human[2];
	float T =_human[3]/gain;
	float P =_human[4]/gain;
	float R =_human[5]/gain;
	float L1= instance().gm.L1;
	float L2= instance().gm.L2;
	float L3= instance().gm.L3;
	float L5= instance().gm.L5;
	//function [q1,q2,q3,q4,q5,q6] = align(T, Y, P, R)
//global L1 L2 L3 L5 xg yg zg

	float Y; 
	if( abs(cos(P))<0.001 ){
		Y = 0;		
	} else {
		Y = atan2(- sin(R) * sin(P), cos(R) );;
	}
	union{
		struct{
			float q0;
			float q1;
			float q2;
			float q3;
			float q4;
			float q5;
		};
		float Q[6];
	};
	q5 =atan2(sin(R)*sin(Y) - cos(R)*cos(Y)*sin(P), cos(P)*cos(Y) );
	q4 = atan2( cos(P)*sin(R), cos(R)*cos(Y) - sin(P)*sin(R)*sin(Y) );

	float f3 = T + robo::pi<float>/2 - Y;

	q0 = zg;
	float ro2=xg*xg+yg*yg;

	float ro = sqrt(ro2);
	float alfa;
	if(ro>=L1+L2){
		alfa=robo::pi<float>;
		ro = L1+L2;
	}else{
		#ifdef RIGHT_HAND
		alfa= acos( (L1*L1 + L2*L2 - ro2) / (2*L1*L2) );
		#endif
		#ifdef LEFT_HAND
		alfa= -acos( (L1*L1 + L2*L2 - ro2) / (2*L1*L2) );
		#endif
	}
	q2 = robo::pi<float> - alfa;
	float beta = asin( sin(alfa)/ro*L2);
	#ifdef POINT_IN_THE_GRAB
	float gama= atan2(yr,xr);
	#endif
	#ifdef POINT_IN_THE_WIRST
	float gama= atan2(yg,xg);
	#endif
	q1 = gama-beta;
	q3 = f3- q1 - q2;

	float dx4t = L5*cos(q5)+L3;
	float dy4t = L5*sin(q4)*sin(q5);
	float dzt = -L5*cos(q4)*sin(q5);

	float f1t = q1;
	float f2t = f1t + q2;
	float f3t = f1t + q2 + q3;

	float dx3t = dx4t * cos(f3) - dy4t *sin(f3);
	float dy3t = dx4t * sin(f3) + dy4t *cos(f3);

	float xt = L1 * cos(f1t) +L2 * cos(f2t)  + dx3t;
	float yt = L1 * sin(f1t) +L2 * sin(f2t)  + dy3t;
	float zt = q0 + dzt;

	float Pt = asin(-cos(q4)*sin(q5));
	float Rt = atan2( sin(q4), cos(q4)*cos(q5));
	float Yt = atan2( sin(q4)*sin(q5), cos(q5) );
	float Y2 = atan2(- sin(R) * sin(P), cos(R) );
	float Tt = Y + f3 - robo::pi<float>/2;
	float err = (abs(xt-xg)+abs(yt-yg)+abs(zt-zg))/700 + (abs(Pt-P)+abs(Rt-R)+abs(Tt-T))/3.14;

	for(int i=0;i<5;++i){
		float tmp = Q[i];
		if(tmp>robo::pi<float> ) Q[i] = tmp -2*robo::pi<float>;
		if(tmp<-robo::pi<float> ) Q[i] = tmp +2*robo::pi<float>;
	}
	
	#ifdef RIGHT_HAND
	_actuator[1] = -q1*gain;
	#endif
	#ifdef LEFT_HAND
	float tmp = -q1*gain-180;
	if(tmp>180) tmp = tmp-360;
	if(tmp<-180) tmp = tmp+360;
	_actuator[1] = tmp;
	#endif
	
	_actuator[2] = -q2*gain;
	#ifdef RIGHT_HAND
	_actuator[3] = -q3*gain;
	#endif
	#ifdef LEFT_HAND
	_actuator[4] = q3*gain;
	#endif
	
	_actuator[4] = q4*gain;

	#ifdef RIGHT_HAND
	_actuator[5] = q5*gain;
	#endif
	#ifdef LEFT_HAND
	_actuator[5] = -q5*gain;
	#endif

	_actuator[6] = _human[6];
	_actuator[7] = _human[7]; 

	return err;
	
}


	