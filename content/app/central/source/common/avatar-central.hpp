#ifndef avatar_central_hpp
#define avatar_central_hpp
#include "core/robosd_backend.hpp"
namespace avatar{
	
	class dynamixel_rs485_channel{	
		protected:
		typedef ::robo::system::guard guard;

			void send_cancel(void);
			void receive_cancel(void);
			void send(const uint8_t * _data, unsigned int _count);			
			void receive(uint8_t * _data, unsigned int _count);
			bool panic(void){ return false;	}
			unsigned int time_ms( void ) { return robo::system::env::time_ms(); }			
		public:
			static void confirm(void);
			static void refuse(void);
	};
	
	class dynamixel_TTL_channel{	
		protected:
		typedef ::robo::system::guard guard;

			void send_cancel(void);
			void receive_cancel(void);
			void send(const uint8_t * _data, unsigned int _count);			
			void receive(uint8_t * _data, unsigned int _count);
			bool panic(void){ return false;	}
			unsigned int time_ms( void ) { return robo::system::env::time_ms(); }			
		public:
			static void confirm(void);
			static void refuse(void);
	};

		class CAN_channel{	
		protected:
		typedef ::robo::system::guard guard;

			void send_cancel(void){};
			void receive_cancel(void){};
			void send(const uint8_t * _data, unsigned int _count);			
			void receive(uint8_t * _data, unsigned int _count);
			bool panic(void){ return false;	}
			unsigned int time_ms( void ) { return robo::system::env::time_ms(); }			
		public:
			static void confirm(void);
			static void refuse(void);
	};
	struct point{
		union{
			struct{
				struct {
					float x;
					float y;
					float z;
				} pos;
				struct {
					float y;
					float p;
					float r;
				} rot;
				float fingers[5];
			};
			float values[11];
		};
	};
	
	struct status{
		point pt;
		
			enum  class states { 
				BOOTING = 0  // � ����������� ����������������
				, READY = 1 // � ����������� ����� � ������
				, NOT_CALIBRATED = 2// � ����������� �� ������������
				, CALIBRATING = 3 // � ����������� �����������
				,	SHUTTING_DOWN = 4// � ����������� ����������� (����� ������� SHUTDOWN)
				, OFF = 5// � ����������� ����� � ���������� (����� ������� SHUTDOWN)
				, MOVING = 6// � ����������� ��������� ��������
				, FAIL = 7//� �� ������������ ���������� ���������� ������ (��������� ������� ������� RESET)
				, ERROR = 8// � �� ������������ ���������� ������������ ������. ��������� ������������� ����������� (����� ������� ����������� ������� REBOOT 
				, POWER_OFF = 9// � � ������������ ��������� ������� ��������
				, POWER_ON = 10// � � ������������ ��������� ������� ��������
				, COUNT = POWER_ON+1
			};
		
		states state;		
	};
	
	void calibrate(void);
	void stop(void);
	void power_off(void);
	void power_on(void);
	void move_to(const point &_point);
	void query_status(status & _status);
	void shutdown(void);
	void reset(void);
	bool save_state(uint32_t _res);
	bool load_state(uint32_t & _dst);
	void led_on(void);
	void led_off(void);
}
#endif