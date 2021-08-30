extern "C"{
#include "avatar-central-common.h"
}
#include "jsonsl.h"
#include "avatar-central.hpp"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "stm32f4xx_hal.h"
#include "ff.h"
#include "fatfs.h"
#include "fsmc.h"
#include "tim.h"
#include "usbd_cdc_if.h"
#include "lwip.h"
#include "stm32_bsp_nand.h"
#include "lwip/apps/tftp_server.h"
#include "net/robosd_net_master.hpp"
#include "core/robosd_system.hpp"
#include "core/robosd_log.hpp"
#include "core/robosd_ini_parser.h"
#include "core/robosd_app.hpp"
#include "core/robosd_backend.hpp"
#include "core/robosd_ring_buf.hpp"
#ifdef RIGHT_HAND
#define AVATAR_INI_FILE "right_hand.ini"
#endif
#ifdef LEFT_HAND
#define AVATAR_INI_FILE "left_hand.ini"
#endif
bool nand_file_system_mount(void);
class dynamixel_UART4 ;
class dynamixel_UART5 ;
bool nand_file_system_build_(void);
void command_poll(void);
void send_config(void);
enum {transport_buffers_count_shift =7, transport_buffers_count=1<<transport_buffers_count_shift};
struct transport_buffer{
	uint8_t memo[32];
	int  space=0;
} transport_buffers[transport_buffers_count];

robo::ring_t< transport_buffers_count_shift,transport_buffer *> transport_buffers_pool_;
robo::ring_t< transport_buffers_count_shift,transport_buffer *> transport_buffers_queue_;
enum{ UART3_BUFFER_SIZE=1048};
uint8_t UART3_BUFFER[UART3_BUFFER_SIZE];
uint8_t UART3_TX_BUFFER[UART3_BUFFER_SIZE];
uint8_t * pUART3_BUFFER = UART3_BUFFER;
uint8_t  tmp_uart;
size_t  UART3_BUFFER_COUNT = 0;
size_t UART3_ACTUAL_SIZE=0;
bool UART3_ABORT = false;
void transport_poll(void);
transport_buffer * transport_buffer_query(void){
	if( robo::system::env::is_frontend() ){
		int cnt = 0;
		do{
			{
				robo::system::guard g__;
				cnt = transport_buffers_pool_.count();
			}
			if  ( __get_PRIMASK() && NVIC_GetPendingIRQ (OTG_FS_IRQn) ){
				HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
			}
			transport_poll();
		} while(cnt<1);
	}
	robo::system::guard g__;
	if(transport_buffers_pool_.count()>0){
		transport_buffer * ret = transport_buffers_pool_.get();
		if(ret){
			ret->space=32;
		}
		return ret;
	} else {
		
		return nullptr;
	}
}

void transport_buffer_release(transport_buffer * _buffer){
	robo::system::guard g__;
	if(transport_buffers_pool_.space()>0){
		transport_buffers_pool_.put(_buffer);
	}
}
transport_buffer * transport_buffer_pop(void){
	robo::system::guard g__;
	if(transport_buffers_queue_.count()>0){
		return transport_buffers_queue_.get();
	} else {
		return nullptr;
	}
}
void transport_buffer_push(transport_buffer * _buffer){
	robo::system::guard g__;
	if(transport_buffers_queue_.space()>0){
		transport_buffers_queue_.put(_buffer);
	}
}
void transport_poll(void){
			static transport_buffer * current_packet = nullptr;
		if(current_packet == nullptr){
			current_packet = transport_buffer_pop();	
		}
		if(current_packet){
			if( CDC_Transmit_FS( current_packet->memo, 32-current_packet->space) == 0){
				transport_buffer_release(current_packet);
				current_packet = nullptr;
			}
		}
}
#if ROBO_UNICODE_ENABLED == 1
#include <wchar.h>
#endif
#define PERSISTENT_INI 1

#if  PERSISTENT_INI == 0
bool build_default_ini_(void);
#endif

bool tftp_init_(void);
void CAN1_configure_filter(void);

struct ican_wait {
	uint16_t id = 0xff;
	uint16_t count = 0;
	uint8_t dummy[8];
	uint8_t * data = dummy;	
	void reset( void ){
		id = 0xFF;
		count = 0;
		data = dummy;
	}
} can_wait;


void avatar::CAN_channel::send(const uint8_t * _data, unsigned int _count){
	if(_count>=2){
		can_wait.reset();
		CAN_TxHeaderTypeDef header;
		header.DLC =_count-2;
		header.ExtId = 0;
		header.IDE = CAN_ID_STD;
		header.RTR = CAN_RTR_DATA;
		header.StdId = *((uint16_t *)_data);
		if( HAL_CAN_AddTxMessage(&hcan1, &header, const_cast<uint8_t *>(_data + 2 ), (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK ) {
			avatar::CAN_channel::refuse();			
		}
	}
}



void avatar::CAN_channel::receive(uint8_t * _data, unsigned int _count){
	if( _count>=2){
		can_wait.id = *((uint16_t *)_data);
		can_wait.count = _count-2;
		can_wait.data = _data+2;
	}
}


void avatar::dynamixel_rs485_channel::send_cancel(void){
	HAL_UART_DMAStop(&huart5);
}

void avatar::dynamixel_rs485_channel::receive_cancel(void){
	HAL_UART_DMAStop(&huart5);
}
		

void avatar::dynamixel_rs485_channel::send(const uint8_t * _data, unsigned int _count){
	HAL_HalfDuplex_EnableTransmitter(&huart5);
	HAL_GPIO_WritePin(RE_DE_485_GPIO_Port,RE_DE_485_Pin,GPIO_PIN_SET);
	HAL_UART_Transmit_DMA(&huart5,const_cast<uint8_t *>(_data),_count) ;
}
		
void avatar::dynamixel_rs485_channel::receive(uint8_t * _data, unsigned int _count){
	HAL_UART_Receive_DMA(&huart5,_data,_count);
	HAL_GPIO_WritePin(RE_DE_485_GPIO_Port,RE_DE_485_Pin,GPIO_PIN_RESET);
	HAL_HalfDuplex_EnableReceiver(&huart5);
}


void avatar::dynamixel_TTL_channel::send_cancel(void){
	HAL_UART_DMAStop(&huart4);
}

void avatar::dynamixel_TTL_channel::receive_cancel(void){
	HAL_UART_DMAStop(&huart4);
}
		

void avatar::dynamixel_TTL_channel::send(const uint8_t * _data, unsigned int _count){
		HAL_GPIO_WritePin(TX_EN_DYNAMIXEL_GPIO_Port,TX_EN_DYNAMIXEL_Pin,GPIO_PIN_SET);
		HAL_UART_Transmit_DMA(&huart4,const_cast<uint8_t *>(_data),_count) ;
}
		
void avatar::dynamixel_TTL_channel::receive(uint8_t * _data, unsigned int _count){
		HAL_GPIO_WritePin(TX_EN_DYNAMIXEL_GPIO_Port,TX_EN_DYNAMIXEL_Pin,GPIO_PIN_RESET);
		HAL_UART_Receive_DMA(&huart4,_data,_count);
}


namespace robo {
	bool system::env::begin(void) {
		//todo только первый раз
		//nand_file_system_build_();

		ROBO_LBREAKN(nand_file_system_mount());

		ROBO_LBREAKN(tftp_init_());
		
		#if  PERSISTENT_INI == 0
		ROBO_LBREAKN(build_default_ini_() );
		#endif
		
		return true;
	}
		
	void system::env::finish(void) {
	}

	#if ROBO_APP_ENV_TYPE == ROBO_APP_TYPE_KEIL

	time_us_t period_us_ = 100;
	time_us_t us_ = 0;
	time_ms_t ms_ = 0;
	time_us_t us_acc_ = 0;
	void tick_(void){
		us_+=period_us_;
		us_acc_+=period_us_;
		if(us_acc_>=1000){
			us_acc_-=1000;
			ms_++;
		}
	}

	void  system::env::abort(void) {
		::abort();
	}
	
	bool system::env::start(void) {
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY );
		CAN1_configure_filter();
		HAL_CAN_Start(&hcan1);

		HAL_TIM_Base_Start_IT(&htim7);
		
		//HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_BUFFER, UART3_BUFFER_SIZE);
		UART_Start_Receive_IT(&huart3,&tmp_uart,1);
		return true;
	}

	void system::env::stop(void) {
	}

	result system::env::startup(void) {
		return result::complete;
	}
	
	result system::env::shutdown(void) {
		return result::complete;
	}

	void system::env::frontend_loop(void) {
		MX_LWIP_Process();
		command_poll();
	}
	
	void system::env::backend_loop(void) {
	}

	void * system::env::critical_enter(void) {
		return nullptr;
	}
	
	void system::env::critical_leave(void * /*context_*/) {
	}
	
		system::context current_context_ = system::context::frontend;
	int locked_ = 0;
	

	bool system::env::is_frontend(void) {
		return current_context_==context::frontend;
	}

	bool system::env::is_backend(void) {
		return current_context_==context::backend;
	}
	

	void* system::env::enter(void) {
		uint32_t prim = __get_PRIMASK();
		__disable_irq();
		return (void *) prim;
		
	}
	
	void system::env::leave(void* _context) {
		int32_t prim = (uint32_t)_context;
		if(!prim) 
			__enable_irq();
	}
	

	void system::env::lock(void) {
		__disable_irq();
		ROBO_APP_ASSERT(locked_ == false);			
		locked_ = true;
	}
	void system::env::unlock(void) {
		ROBO_APP_ASSERT(locked_ == true);			
		locked_ = false;
		__enable_irq();
	}
	

	void system::env::fall(void) {
		ROBO_APP_ASSERT(current_context_==context::frontend);
		current_context_=context::backend;
	}
	
	void system::env::comeback(void) {
		ROBO_APP_ASSERT(current_context_==context::backend);
		current_context_=context::frontend;
	}
	
	time_us_t system::env::time_us(void) {
		return us_;
	}
	
	time_us_t system::env::realtime_us(void) {
		return us_;
	}
	
	time_ms_t system::env::time_ms(void) {			
		return ms_;
	}
	
	random_t system::env::rand(random_t _max) {
		return SysTick->VAL % _max;
	}
	
	void system::env::wakeup(void) {
		NVIC_SetPendingIRQ(TIM7_IRQn);
	}
	
	time_us_t system::env::period_us(void) {
		return period_us_;
	}
	
	void system::env::sleep(void) {
	}
	
	
	void system::env::print( cstr  _s){
		
		transport_buffer * packet = transport_buffer_query();
		while (packet)	{
			uint8_t * ptr = packet->memo;
			while(*_s && packet->space > 0){
				*ptr++ = (uint8_t)*_s++;
				packet->space--;
			}
			if( packet->space == 0){
				transport_buffer_push(packet);
				if(*_s==0){
					return;
				}
			}
			else{
				if(*_s==0){
					transport_buffer_push(packet);
					return;
				}
			}
			packet = transport_buffer_query();						
		}	
		
		/*while(*_s){
			huart3.Instance->DR = *_s++;
			while ( (huart3.Instance->SR & (UART_FLAG_TC)) != (UART_FLAG_TC) );
		}*/
		
	}
	
#if ROBO_APP_ALLOC_ENABLED == 1
	void* system::env::mem_alloc(size_t _size) {
		return malloc(_size);
	}
	void system::env::mem_free(void * _memo) {
		free(_memo);
	}
#endif

	size_t system::env::sprintf(char_t* _dst, size_t _max_sz, cstr _format, va_list _args) {

	#if ROBO_UNICODE_ENABLED == 1
			//size_t sz = vswprintf(_dst, _max_sz, _format, _args);
		size_t sz = vswprintf(_dst, _max_sz, _format, _args);
	#else
			size_t sz = vsprintf(_dst, _format, _args);
	#endif
			if (sz < _max_sz-1) {
				_dst[sz] = 0;
			}
			return sz;

		}
#endif
	#if ROBO_APP_INI_TYPE == ROBO_APP_TYPE_NATIVE	
	cstr g_robo_ini_fn = nullptr;
	bool system::ini::begin(cstr _ini) {
		g_robo_ini_fn = _ini;
		FIL testFile;
		size_t bytesread;
		char * buf = new char [4000];
		f_open(&testFile, g_robo_ini_fn, FA_READ);
		f_read (&testFile,buf, 4000,&bytesread);
		robo_ipa_init(ROBO_IPA_NORMAL);
		robo_ipa_applay(buf, bytesread);
		delete [] buf;
		f_close(&testFile);
		return true;
	}
	
	void system::ini::finish(void) {
		g_robo_ini_fn = nullptr;
	}
	
	bool system::ini::load_str(char_t* _dst, size_t _max_sz, cstr _section, cstr _key) {
		ROBO_LBREAKN_F(g_robo_ini_fn!=nullptr, "ini is't initialized")
		robo::robo_ipa_string_get(_section, _key, "", _dst, _max_sz);

		return true;
	}

#endif
}

void  CAN1_configure_filter( void ){
	CAN_FilterTypeDef canFilterConfig={0};
		for(int i=0;i<13;i++){
			canFilterConfig.FilterActivation = CAN_FILTER_DISABLE;
			canFilterConfig.FilterBank = i;
			HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);
		}
		canFilterConfig.FilterMode = 	CAN_FILTERMODE_IDMASK;
		canFilterConfig.FilterScale =  CAN_FILTERSCALE_32BIT;

		canFilterConfig.FilterMaskIdHigh = 0;
		canFilterConfig.FilterMaskIdLow = 0;

		canFilterConfig.FilterIdHigh = 0;
		canFilterConfig.FilterIdLow =  0;
		canFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
		canFilterConfig.FilterBank = 14;

		
		HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);




}

//RFTP
FIL tftp_file_;
bool tftp_file_busy_ =false;
/**
   * Open file for read/write.
   * @param fname Filename
   * @param mode Mode string from TFTP RFC 1350 (netascii, octet, mail)
   * @param write Flag indicating read (0) or write (!= 0) access
   * @returns File handle supplied to other functions
   */
void* tftp_open_(const char* fname, const char* mode, u8_t write){
	UNUSED(mode);
	UNUSED(write);
	if(!tftp_file_busy_){
		if ( f_open(&tftp_file_, fname, write? (FA_WRITE | FA_CREATE_ALWAYS) :FA_READ) == FR_OK){			
			tftp_file_busy_ = true;
			return &tftp_file_;
		}
	}
	return 0;
}

void tftp_close_(void* handle){
	UNUSED(handle);
	if( tftp_file_busy_){
		f_close(&tftp_file_);
		tftp_file_busy_ = false;
	}
}

/**
 * Read from file 
 * @param handle File handle returned by open()
 * @param buf Target buffer to copy read data to
 * @param bytes Number of bytes to copy to buf
 * @returns &gt;= 0: Success; &lt; 0: Error
 */
int tftp_read_(void* handle, void* buf, int bytes){
	UNUSED(handle);
	UINT n;
	f_read(&tftp_file_, buf,(UINT)bytes, &n );
	return n;
}
 
/**
 * Write to file
 * @param handle File handle returned by open()
 * @param pbuf PBUF adjusted such that payload pointer points
 *             to the beginning of write data. In other words,
 *             TFTP headers are stripped off.
 * @returns &gt;= 0: Success; &lt; 0: Error
 */
int tftp_write_(void* handle, struct pbuf* p){
	int cnt = 0;
	while(p){
		UINT wrt = 0;
		if( f_write(&tftp_file_, p->payload, p->len, &wrt ) != FR_OK ){
			return -1;			
		}	
		cnt += wrt;		
		p=p->next;
	}
	return cnt;
}

bool tftp_init_(void){
	static	tftp_context tftp_context_ = {
		tftp_open_
		, tftp_close_
		, tftp_read_
		, tftp_write_
	};
	tftp_init( &tftp_context_ );
	return true;
}




//конфигурция файловой системы для K9F1G08 
#define NAND_PAGE_SIZE             ((uint16_t)0x0800) /* 2 * 1024 bytes per page w/o Spare Area */
#define NAND_BLOCK_SIZE            ((uint16_t)0x0040) /* 64 pages per block */
#define NAND_ZONE_SIZE             ((uint16_t)0x0400) /* 1024 Block per zone */
#define NAND_SPARE_AREA_SIZE       ((uint16_t)0x0040) /* last 64 bytes as spare area */
#define NAND_MAX_ZONE              ((uint16_t)0x0001) /* 1 zones of 1024 block */

static bool nand_file_test_(void){
	FIL testFile;
	f_open(&testFile, "test.txt", FA_READ);
	char buf[12];
	uint32_t rd;
	f_read(&testFile, buf,12,&rd);
	buf[rd] = 0;
	bool ret = (std::strcmp(buf,"Hello world")==0);
	f_close(&testFile);
	return ret;
}


bool nand_file_system_build_(void){
static 	BYTE work[_MAX_SS];
	f_mkfs (
		"A"	//-- Logical drive number 
		, FM_FAT //BYTE opt,			-- Format option 
		, NAND_PAGE_SIZE//		-- Size of allocation unit (cluster) [byte] 
		, work			//-- Pointer to working buffer 
		, _MAX_SS	//		-- Size of working buffer 
	);
	uint8_t nandPath[5] = "NAND";
	FATFS fileSystem;
	ROBO_LBREAKN_F( (f_mount(&fileSystem, (char *)nandPath, 0) == FR_OK), "file system  mount error" );
	FIL testFile;
	uint32_t rd;
	f_open(&testFile, "test.txt", FA_WRITE | FA_CREATE_ALWAYS);
	f_write(&testFile,"Hello world",11,&rd);
	f_close(&testFile);
	ROBO_LRET_F(nand_file_test_(), "file system run fault" );
}



bool nand_file_system_mount(void){
	static FATFS fileSystem;
	static uint8_t nandPath[5] = "NAND";
	ROBO_LBREAKN_F( (f_mount(&fileSystem, (char *)nandPath, 0) == FR_OK), "file system  mount error" );
	ROBO_LRET_F(nand_file_test_(), "file system run fault" );
}

int avatar_central_begin(void){

	transport_buffer * tmp = transport_buffers;
	for(int i=0;i<transport_buffers_count;++i,++tmp){
		transport_buffer_release(tmp);
	}
//	ROBO_LBREAKN( robo::app::machine::begin(AVATAR_INI_FILE));
	ROBO_BREAKN( robo::app::machine::begin(AVATAR_INI_FILE, robo::termial::print),1 );
	//ROBO_BREAKN( robo::app::machine::begin("default-servo-2.ini"),1 );
	robo::app::machine::start();	
	return 0;
}
void avatar_central_finish(void){
	robo::app::machine::finish();	
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart4){
		avatar::dynamixel_TTL_channel::confirm();
	} else if(huart==&huart5){
		avatar::dynamixel_rs485_channel::confirm();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart==&huart4){
		avatar::dynamixel_TTL_channel::confirm();
	} else if(huart==&huart5){
		avatar::dynamixel_rs485_channel::confirm();
	}
	else if(huart==&huart3){
		if(UART3_ABORT){
			if(tmp_uart == '\n'){
				UART3_ABORT = false;
			}
		} else {
		if( UART3_BUFFER_COUNT<UART3_BUFFER_SIZE ){
			UART3_BUFFER_COUNT++;
			*pUART3_BUFFER++ = tmp_uart;
			if(tmp_uart == '\n'){
				UART3_BUFFER[UART3_BUFFER_COUNT]=0;
				pUART3_BUFFER = UART3_BUFFER;
				UART3_ACTUAL_SIZE = UART3_BUFFER_COUNT;
				UART3_BUFFER_COUNT = 0;
				//HAL_UART_Transmit_DMA(&huart3, UART3_BUFFER,UART3_ACTUAL_SIZE);
			}
		} else {
			pUART3_BUFFER = UART3_BUFFER;
			UART3_BUFFER_COUNT = 0;
		}
	}
		UART_Start_Receive_IT(&huart3,&tmp_uart,1);
		//UART3_BUFFER[20]=0;
		//robo::system::printf(RT("%s"),UART3_BUFFER);
		//HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_BUFFER, UART3_BUFFER_SIZE);
		//HAL_UART_Transmit_DMA(&huart3, UART3_BUFFER, UART3_BUFFER_SIZE);
	}
}


/*
class uart0 :  public robo::net::iserial{
public:
			virtual robo_size_t available(void){
				return (huart3.Instance->SR & UART_FLAG_RXNE) == UART_FLAG_RXNE ? 1:0;
			}
			virtual robo_size_t space(void){
				return (huart3.Instance->SR & UART_FLAG_TC) == UART_FLAG_TC ? 1 : 0;
			}
			virtual robo_size_t get(robo_byte_p _data, robo_size_t _max_size){
				if(_max_size>0){
					*_data = (uint8_t) huart3.Instance->DR;
					return 1;
				}else{
					return 0;
				}
			
			virtual robo_result_t put(robo_byte_p _data, robo_size_t _size){
				if(_size == 1){
					huart3.Instance->DR = *_data ;
					return ROBO_SUCCESS;
				} else {
					return ROBO_ERROR;
				}
			}
			
			virtual uint8_t get(void){
					return (uint8_t) huart3.Instance->DR;
			}
			virtual robo_result_t  put(uint8_t _data){
				huart3.Instance->DR = _data;
				return ROBO_SUCCESS;
			}
			virtual void reset(void){
			}

} uart0_;
*/	

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	if(huart==&huart4){
		avatar::dynamixel_TTL_channel::refuse();
	} else if(huart==&huart5){
		avatar::dynamixel_rs485_channel::refuse();
	} else if(huart==&huart3){
			pUART3_BUFFER = UART3_BUFFER;
			UART3_BUFFER_COUNT = 0;
			UART3_ACTUAL_SIZE = 0;
			UART3_ABORT = true;
			UART_Start_Receive_IT(&huart3,&tmp_uart,1);
		
	}
}

void avatar_central_loop(void){
	if(! robo::app::machine::terminated() ) robo::app::machine::frontend_loop();
	transport_poll();
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
	avatar::CAN_channel::confirm();
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){
	avatar::CAN_channel::refuse();
	can_wait.reset();
}

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan){
	avatar::CAN_channel::refuse();
	can_wait.reset();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
/*	static int trig = 0;
	static uint8_t RxData[8];
	board_flow_msg0_send(RxHeader.StdId|0x100, RxData, RxHeader.DLC);*/
	static CAN_RxHeaderTypeDef RxHeader;
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, can_wait.data );
	if(RxHeader.StdId == can_wait.id && RxHeader.DLC == can_wait.count ){
		avatar::CAN_channel::confirm();
		can_wait.reset();
	}		
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	HAL_GPIO_WritePin(TEST_1_GPIO_Port,TEST_1_Pin,GPIO_PIN_SET);
	robo::tick_();
	HAL_TIM_TriggerCallback(htim);
	if(! robo::app::machine::terminated() ) robo::app::machine::backend_loop();
	HAL_GPIO_WritePin(TEST_1_GPIO_Port,TEST_1_Pin,GPIO_PIN_RESET);

}

void avatar::led_on(void){
	HAL_GPIO_WritePin(TEST_2_GPIO_Port,TEST_2_Pin,GPIO_PIN_SET);
}
void avatar::led_off(void){
	HAL_GPIO_WritePin(TEST_2_GPIO_Port,TEST_2_Pin,GPIO_PIN_RESET);
}

void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim){
	//HAL_GPIO_TogglePin(TEST_2_GPIO_Port,TEST_2_Pin);
}


//заглушки
#ifndef SIO_DEFINED 
#include "sio.h"

sio_fd_t sio_open(u8_t devnum)
{
  return 0;
}

void sio_send(u8_t c, sio_fd_t fd){
	ROBO_UNUSED(c);
	ROBO_UNUSED(fd);
}

u32_t sio_read(sio_fd_t fd, u8_t *data, u32_t len)
{
	ROBO_UNUSED(fd);
	ROBO_UNUSED(data);
	ROBO_UNUSED(len);
  return 0;
}

u32_t sio_tryread(sio_fd_t fd, u8_t *data, u32_t len)
{
	ROBO_UNUSED(fd);
	ROBO_UNUSED(data);
	ROBO_UNUSED(len);  return 0;
}
#endif

//#include "net/robosd_net_master.hpp"

/*static int test_void2(void * _instance) {
		return *(int *)_instance + 1;
	}
	int instance;
	robo::delegat::suni<int> test_void_2_(&instance, test_void2);
*/

#if PERSISTENT_INI== 0

bool build_default_ini_(void){
	#ifdef LEFT_HAND
	static const robo::char_t buf[] =
		"[SETTINGS]\n"
		"DEBUG_VERB=6\n"
		"DEBUG_MASK_BITS=\"0 1 2 3\"\n"					
		"[MODULES]\n"
		"COUNT=1\n"
		"M_1=\"avatar\"\n"

		"[avatar]\n"
		"converter_count=8\n"
		"converter_1=z-1_position_co\n"
		"converter_2=yaw-2_position_co\n"
		"converter_3=yaw-3_position_co\n"
		"converter_4=roll-4_position_co\n"
		"converter_5=pitch-5_position_co\n"
		"converter_6=roll-6_position_co\n"
		"converter_7=grab-7_position_co\n"
		"converter_8=grab-8_position_co\n"
	
		"[z-1_position_co]\n"
		"min=10\n"
		"max=390\n"
		"offset=0\n"
		"scale=256\n"

		"[yaw-2_position_co]\n"
		"min=-108\n"
		"max=98\n"
		"offset=-206,1\n"
		"scale=11.375\n"

		"[yaw-3_position_co]\n"
		"min=-170\n"
		"max=0\n"
		"offset=-316,4\n"
		"scale=11.375\n"
		
		"[roll-4_position_co]\n"
		"min=-87\n"
		"max=91\n"
		"offset=-87.94\n"
		"scale=22.766\n"

		"[pitch-5_position_co]\n"
		"min=-95\n"
		"max=80\n"
		"offset=-97.2\n"
		"scale=22.766\n"

		"[roll-6_position_co]\n"
		"min=-180\n"
		"max=130\n"
		"offset=-222.9\n"
		"scale=11.375\n"

		"[grab-7_position_co]\n"
		"min=0\n"
		"max=255\n"
		"offset=613,9\n"
		"scale=-3.725\n"

		"[grab-8_position_co]\n"
		"min=0\n"
		"max=255\n"
		"offset=585,2\n"
		"scale=-3.725\n"

		"[dynamixel_rs485_bus]\n"
		"BUS_ID=1\n"
		"DEFAULT_TIMEOUT_US=50000\n"
		
		"[dynamixel_TTL_bus]\n"
		"BUS_ID=2\n"
		"DEFAULT_TIMEOUT_US=5000\n"

		"[can8_bus]\n"
		"BUS_ID=3\n"
		"DEFAULT_TIMEOUT_US=5000\n"

		"[hand]\n"
		"REQUEST_PAUSE_US=500 #rx485 proto unswering after delay\n"

		"[grab]\n"
		"REQUEST_PAUSE_US=0 #TTL proto unswering after delay\n"

		"[fake_router]\n"
		"ROUT_TABLE_SIZE=0\n"
		"#bus_id dev_id command request_suba abswer_suba\n"
		"RT_1=\"1 1 0 0 0\"\n"

		"[yaw-2]\n"
		"BOARD_DEV_ID=1\n"
		"ALIAS=\"yaw-2\"\n"
		"BUS_NAME=\"avatar/dynamixel_rs485_bus\"\n"
		"ROUTER_NAME=\"avatar/fake_router\"\n"
		"BOARD_DEV_ID=1\n"
		"BOARD_ADDRESS=0x02\n"		
		"ENABLED=1\n"		
		"calibrate_position=82.35\n"
		"position_dead_zone=2\n"
		"speed_max=200\n"
		"position_converter=\"avatar/yaw-2_position_co\"\n"
		"homing_ofset=0\n"		
		"pwm_limit=885\n"		

		"[yaw-3]\n"
		"BOARD_DEV_ID=1\n"
		"ALIAS=\"yaw-3\"\n"
		"BUS_NAME=\"avatar/dynamixel_rs485_bus\"\n"
		"ROUTER_NAME=\"avatar/fake_router\"\n"
		"BOARD_DEV_ID=1\n"
		"BOARD_ADDRESS=0x03\n"		
		"ENABLED=1\n"		
		"calibrate_position=-165\n"
		"position_dead_zone=2\n"
		"speed_max=200\n"
		"position_converter=\"avatar/yaw-3_position_co\"\n"
		"homing_ofset=0\n"		
		"pwm_limit=885\n"		

		
		"[roll-4]\n"
		"BOARD_DEV_ID=1\n"
		"ALIAS=\"roll-4\"\n"
		"BUS_NAME=\"avatar/dynamixel_TTL_bus\"\n"
		"ROUTER_NAME=\"avatar/fake_router\"\n"
		"BOARD_DEV_ID=1\n"
		"BOARD_ADDRESS=0x02\n"		
		"ENABLED=1\n"		
		"calibrate_position=0\n"
		"position_dead_zone=2\n"
		"speed_max=120\n"
		"position_converter=\"avatar/roll-4_position_co\"\n"
		"homing_ofset=0\n"		
		"pwm_limit=885\n"		

		"[pitch-5]\n"
		"BOARD_DEV_ID=1\n"
		"ALIAS=\"pitch-5\"\n"
		"BUS_NAME=\"avatar/dynamixel_TTL_bus\"\n"
		"ROUTER_NAME=\"avatar/fake_router\"\n"
		"BOARD_DEV_ID=1\n"
		"BOARD_ADDRESS=0x03\n"		
		"ENABLED=1\n"		
		"calibrate_position=0\n"
		"position_dead_zone=2\n"
		"speed_max=120\n"
		"position_converter=\"avatar/pitch-5_position_co\"\n"
		"homing_ofset=0\n"		
		"pwm_limit=885\n"		

		"[roll-6]\n"
		"BOARD_DEV_ID=1\n"
		"ALIAS=\"roll-6\"\n"
		"BUS_NAME=\"avatar/dynamixel_TTL_bus\"\n"
		"ROUTER_NAME=\"avatar/fake_router\"\n"
		"BOARD_DEV_ID=1\n"
		"BOARD_ADDRESS=0x04\n"		
		"ENABLED=1\n"		
		"calibrate_position=0\n"
		"position_dead_zone=2\n"
		"speed_max=120\n"
		"position_converter=\"avatar/roll-6_position_co\"\n"
		"homing_ofset=0\n"		
		"pwm_limit=885\n"		

		"[grab-7]\n"
		"BOARD_DEV_ID=1\n"
		"ALIAS=\"grab-7\"\n"
		"BUS_NAME=\"avatar/dynamixel_TTL_bus\"\n"
		"ROUTER_NAME=\"avatar/fake_router\"\n"
		"BOARD_DEV_ID=1\n"
		"BOARD_ADDRESS=0x05\n"		
		"ENABLED=1\n"		
		"calibrate_position=30\n"
		"position_dead_zone=2\n"
		"speed_max=60\n"
		"position_converter=\"avatar/grab-7_position_co\"\n"
		"homing_ofset=0\n"		
		"pwm_limit=160\n"		

		"[grab-8]\n"
		"BOARD_DEV_ID=1\n"
		"ALIAS=\"grab-8\"\n"
		"BUS_NAME=\"avatar/dynamixel_TTL_bus\"\n"
		"ROUTER_NAME=\"avatar/fake_router\"\n"
		"BOARD_DEV_ID=1\n"
		"BOARD_ADDRESS=0x06\n"		
		"ENABLED=1\n"		
		"calibrate_position=30\n"
		"position_dead_zone=2\n"
		"speed_max=60\n"
		"position_converter=\"avatar/grab-8_position_co\"\n"
		"homing_ofset=0\n"		
		"pwm_limit=885\n"		

		"[z-1]\n"
		"BOARD_DEV_ID=1\n"
		"ALIAS=\"Z\"\n"
		"BUS_NAME=\"avatar/can8_bus\"\n"
		"ROUTER_NAME=\"avatar/fake_router\"\n"
		"BOARD_DEV_ID=1\n"
		"BOARD_ADDRESS=0x01\n"		
		"ENABLED=1\n"		
		"calibrate_position=250\n"
		"position_dead_zone=0.1\n"
		"speed_max=350\n"
		"position_converter=\"avatar/z-1_position_co\"\n"	
		"homing_ofset=0\n"		
		"pwm_limit=0\n"		
	
		"[fingers]\n"
		"BOARD_DEV_ID=1\n"
		"ALIAS=\"FINGERS\"\n"
		"BUS_NAME=\"avatar/can8_bus\"\n"
		"ROUTER_NAME=\"avatar/fake_router\"\n"
		"BOARD_DEV_ID=1\n"
		"BOARD_ADDRESS=0x0E\n"		
		"ENABLED=0\n"
;
#endif

	FIL fini;
	f_open(&fini, AVATAR_INI_FILE, FA_WRITE | FA_CREATE_ALWAYS);
	uint32_t byteswritten;
	uint32_t bytes = strlen(buf);
	f_write (&fini,buf, strlen(buf),&byteswritten);
	f_close(&fini);
	ROBO_LRET_F(byteswritten == bytes,"build ini error");
}

void  encode( const char * _str);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart==&huart3){
		//HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_BUFFER, UART3_BUFFER_SIZE);
		//UART3_BUFFER[Size]=0;
		//data_size_ = Size;
		//robo::system::printf(RT("%s"),UART3_BUFFER);
		//HAL_UART_Transmit_DMA(&huart3, UART3_BUFFER,Size);
		
	}
	
}
#endif

	struct content {		
		enum{key_sz = 20};
		struct {
			bool error = false;
			char key[key_sz];
			bool is_rot = false;
			bool is_pos = false;
			int finger_counter = 0;
			int speed_limit_counter = 0;;
			float speed_limit[6];
		} parse;

		char timestamp[key_sz];
		char cmd[key_sz];
		int arm =1;
		avatar::point point_;
		void do_encode_callback(
			struct jsonsl_state_st* state,
			const char* buf
		)
		{
			if (
				((state->special_flags & JSONSL_SPECIALf_UNSIGNED) == JSONSL_SPECIALf_UNSIGNED)
			||
				((state->special_flags & JSONSL_SPECIALf_UNSIGNED) == JSONSL_SPECIALf_UNSIGNED)
			||
				((state->special_flags & JSONSL_SPECIALf_FLOAT) == JSONSL_SPECIALf_FLOAT)
			||
				((state->special_flags & JSONSL_SPECIALf_EXPONENT) == JSONSL_SPECIALf_EXPONENT)
				)
			{
				const char* s = buf - (state->pos_cur - state->pos_begin);
					if (strcmp(parse.key, "arm") == 0) {
						arm = atoi(s);
					parse.key[0] = 0;
					} else
				if (strcmp(parse.key, "x") == 0) {
					if (parse.is_pos) {
						point_.pos.x = atof(s);
					}
					else {
						parse.error = true;
					}
					parse.key[0] = 0;

				} else 
				if (strcmp(parse.key, "y") == 0) {
					if (parse.is_pos) {
						point_.pos.y = atof(s);
					}
					else
						if (parse.is_rot) {
							point_.rot.y = atof(s);
						}
						else {
							parse.error = true;
						}
						parse.key[0] = 0;
				} else 
				if (strcmp(parse.key, "z") == 0) {
					if (parse.is_pos) {
						point_.pos.z = atof(s);
					}
					else {
						parse.error = true;
					}
					parse.key[0] = 0;
				} else
				if (strcmp(parse.key, "r") == 0) {
					if (parse.is_rot) {
						point_.rot.r = atof(s);
					}
					else {
						parse.error = true;
					}
					parse.key[0] = 0;
				} else 
				if (strcmp(parse.key, "p") == 0) {
					if (parse.is_rot) {
						point_.rot.p = atof(s);
					}
					else {
						parse.error = true;
					}
					parse.key[0] = 0;

				} else
				if (strcmp(parse.key, "fingers") == 0) {
					if (parse.finger_counter >= 5) {
						parse.key[0] = 0;
					}
					point_.fingers[parse.finger_counter++] = atof(s);
				} else
				if (strcmp(parse.key, "sl") == 0) {
					if ( parse.speed_limit_counter >=8 ) {
						parse.key[0] = 0;
					}
					parse.speed_limit[parse.speed_limit_counter++] = atof(s);
				}
			}

			switch (state->type) {
			case JSONSL_T_HKEY:
			case JSONSL_T_LIST:
			{
				const char* s = buf - (state->pos_cur - state->pos_begin) + 1;
				char* d = parse.key;
				for (int i = state->pos_begin + 1; i < state->pos_cur; ++i, ++s, ++d)	*d = *s;	*d = 0;
			}
			if (strcmp(parse.key, "pos") == 0) {
				parse.is_pos = true;
				parse.is_rot = false;
			}
			if (strcmp(parse.key, "rot") == 0) {
				parse.is_rot = true;
				parse.is_pos = false;
			}
			break;
			case JSONSL_T_STRING:
			{
				const char* s = buf - (state->pos_cur - state->pos_begin) + 1;
				char* d = nullptr;
				if (strcmp(parse.key, "timestamp") == 0) {
					 d = timestamp;
				}
				else if ( strcmp(parse.key, "command") ==0  ){
					d = cmd;
				} 
				if (d) {
					for (int i = state->pos_begin + 1; i < state->pos_cur; ++i, ++s, ++d) *d = *s;		*d = 0;
				}

				parse.key[0] = 0;

			}
				break;
			}
		}
		jsonsl_t parser;
		int error_callback_(
      jsonsl_error_t error,
      struct jsonsl_state_st* state,
      jsonsl_char_t *at){
				parse.error = true;
				return 0;
			}
		static void encode_callback(
			jsonsl_t jsn,
			jsonsl_action_t action,
			struct jsonsl_state_st* state,
			const char* buf);
		static int error_callback(jsonsl_t jsn,
        jsonsl_error_t error,
        struct jsonsl_state_st* state,
        jsonsl_char_t *at);
		content(void){
			parser =  jsonsl_new(100);
			parser->action_callback_POP = encode_callback;
			parser->error_callback = error_callback;
			jsonsl_enable_all_callbacks(parser);
		};
		~content(void){
			jsonsl_destroy(parser);
		}

			
	bool encode(const char* _buf, size_t _size) {
		parse.error = false;
		parse.key[0] = 0;
		parse.is_rot = false;
		parse.is_pos = false;
		parse.finger_counter = 0;
		parse.speed_limit_counter = 0;
		jsonsl_reset(parser);
		jsonsl_feed(parser, _buf, _size);
		
		return true;
	}
};
content content_;
void content::encode_callback(
			jsonsl_t jsn,
			jsonsl_action_t action,
			struct jsonsl_state_st* state,
			const char* buf){
	content_.do_encode_callback(state,buf);
}
int content::error_callback(jsonsl_t jsn,
        jsonsl_error_t error,
        struct jsonsl_state_st* state,
        jsonsl_char_t *at){
	return content_.error_callback_(error,state,at);
}
static robo::cstr  status_names[ 10 ] = {
	RT("BOOTING")
	, RT("READY")
	, RT("NOT_CALIBRATED")
	, RT("CALIBRATING")
	,	RT("SHUTTING_DOWN")
	, RT("OFF = 5")
	, RT("MOVING")
	, RT("FAIL")
	, RT("ERROR")
	, RT("POWER OFF")
};
void send_status(void){
	avatar::status status;
	avatar::query_status(status);
	static robo::char_t answer[] = {
			"{"
				"\"command\": \"STATUS\","
				"\"timestamp\": \"%s\","
				"\"data\": {"
						"\"status\": \"%s\","
						"\"arm\": {"
							 "\"arm\": %d,"
							 "\"pos\": {"
										"\"x\": %2.2f,"
										 "\"y\": %2.2f,"
										 "\"z\": %2.2f"
							 "},"
							 "\"rot\": {"
										"\"r\": %2.2f,"
										"\"p\": %2.2f,"
										"\"y\": %2.2f"
							 "},"
								"\"fingers,\": [%2.2f, %2.2f, %2.2f, %2.2f, %2.2f]"
								"\"actuators\": [%2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f]"
						"}"
				"}"
		"}\n"
	};

	int no = (int )status.state;
	size_t sz = robo::system::sprintf(
		(robo::char_t*)UART3_TX_BUFFER
		,UART3_BUFFER_SIZE
		,answer
		, content_.timestamp
		, status_names[no]
		, 0 // arm
		, status.pt.pos.x
		, status.pt.pos.y
		, status.pt.pos.z
		, status.pt.rot.r
		, status.pt.rot.p
		, status.pt.rot.y
		, status.pt.fingers[0]
		, status.pt.fingers[1]
		, status.pt.fingers[2]
		, status.pt.fingers[3]
		, status.pt.fingers[4]
		, status.pt.actuator[0]
		, status.pt.actuator[1]
		, status.pt.actuator[2]
		, status.pt.actuator[3]
		, status.pt.actuator[4]
		, status.pt.actuator[5]
		, status.pt.actuator[6]
		, status.pt.actuator[7]
	);
	HAL_UART_Transmit_DMA(&huart3, UART3_TX_BUFFER,sz);	
}

void send_config(void){
	avatar::coordDescs coordDescs;
	avatar::query_config(coordDescs);

	static robo::char_t coord_s[] = 
	"{"
		"\"caption\":\"%s\","
		"\"range\":{"
			"\"actual\":{"
				"\"min\":%2.2f,"
				"\"max\":%2.2f"
			"},"
			"\"required\":{"
				"\"min\":%2.2f,"
				"\"max\":%2.2f"
				 "}"
		"},"
		"\"actual\":%2.2f"
	"}";
	static robo::char_t config_s[] = 
		"{\"command\":\"CONFIG\""
		",\"status\":\"%s\""
		",\"coords\":[%s, %s, %s, %s, %s, %s, %s ]}\n";
	
	
	size_t sz = robo::system::sprintf(
		(robo::char_t*)UART3_TX_BUFFER
		,UART3_BUFFER_SIZE
		,config_s
		,status_names[(int)coordDescs.state]
		, robo::string  (
				coord_s
				, coordDescs.descs[0].caption.c_str()
				,	coordDescs.descs[0].range.actual.min
				,	coordDescs.descs[0].range.actual.max
				,	coordDescs.descs[0].range.required.min
				,	coordDescs.descs[0].range.required.max
				,	coordDescs.descs[0].actual
			).c_str()
		, robo::string  (
				coord_s
				, coordDescs.descs[1].caption.c_str()
				,	coordDescs.descs[1].range.actual.min
				,	coordDescs.descs[1].range.actual.max
				,	coordDescs.descs[1].range.required.min
				,	coordDescs.descs[1].range.required.max
				,	coordDescs.descs[1].actual
			).c_str()
		, robo::string  (
				coord_s
				, coordDescs.descs[2].caption.c_str()
				,	coordDescs.descs[2].range.actual.min
				,	coordDescs.descs[2].range.actual.max
				,	coordDescs.descs[2].range.required.min
				,	coordDescs.descs[2].range.required.max
				,	coordDescs.descs[2].actual
			).c_str()
			, robo::string  (
				coord_s
				, coordDescs.descs[3].caption.c_str()
				,	coordDescs.descs[3].range.actual.min
				,	coordDescs.descs[3].range.actual.max
				,	coordDescs.descs[3].range.required.min
				,	coordDescs.descs[3].range.required.max
				,	coordDescs.descs[3].actual
			).c_str()
		, robo::string  (
				coord_s
				, coordDescs.descs[4].caption.c_str()
				,	coordDescs.descs[4].range.actual.min
				,	coordDescs.descs[4].range.actual.max
				,	coordDescs.descs[4].range.required.min
				,	coordDescs.descs[4].range.required.max
				,	coordDescs.descs[4].actual
			).c_str()
		, robo::string  (
				coord_s
				, coordDescs.descs[5].caption.c_str()
				,	coordDescs.descs[5].range.actual.min
				,	coordDescs.descs[5].range.actual.max
				,	coordDescs.descs[5].range.required.min
				,	coordDescs.descs[5].range.required.max
				,	coordDescs.descs[5].actual
			).c_str()
		, robo::string  (
				coord_s
				, coordDescs.descs[6].caption.c_str()
				,	coordDescs.descs[6].range.actual.min
				,	coordDescs.descs[6].range.actual.max
				,	coordDescs.descs[6].range.required.min
				,	coordDescs.descs[6].range.required.max
				,	coordDescs.descs[6].actual
			).c_str()
		);
	
	/*robo::string answer(
		config_s
		, robo::string( 
				coord_s
				,coordDescs.descs[0].range).c_str()
		)
	
	);*/
	
	

	/*int no = (int )status.state;
	size_t sz = robo::system::sprintf(
		(robo::char_t*)UART3_TX_BUFFER
		,UART3_BUFFER_SIZE
		,answer
		, content_.timestamp
		, status_names[no]
		,	RT("We're moving! Dhat's funny!")
		, 0 // arm
		, status.pt.pos.x
		, status.pt.pos.y
		, status.pt.pos.z
		, status.pt.rot.r
		, status.pt.rot.p
		, status.pt.rot.y
		, status.pt.fingers[0]
		, status.pt.fingers[1]
		, status.pt.fingers[2]
		, status.pt.fingers[3]
		, status.pt.fingers[4]
	);*/
	HAL_UART_Transmit_DMA(&huart3, UART3_TX_BUFFER,sz);	
}

void command_poll(void){
	size_t size;
	{
		robo::system::guard g__;
		size =  UART3_ACTUAL_SIZE;
		UART3_ACTUAL_SIZE = 0;
	}
	if(size>0){
		//за такое убивать на месте
		content_.encode((const char *)UART3_BUFFER,size);
		//robo_detaillog(5,0,"%s",(const char *)UART3_BUFFER);
		if(!content_.parse.error){
			if( strcmp(content_.cmd,"STATUS") == 0 ){
				send_status();
			}else			if( strcmp(content_.cmd,"ARM") == 0 ){
				avatar::move_to(content_.point_);
				send_status();
			} else if( strcmp(content_.cmd,"CALIBRATE") == 0 ){				
				avatar::calibrate();
				send_status();
			} else if( strcmp(content_.cmd,"STOP") == 0 ){				
				avatar::stop();
				send_status();
			} else    if( strcmp(content_.cmd,"SHUTDOWN") == 0 ){	
				avatar::shutdown();					
				send_status();
			} else  if( strcmp(content_.cmd,"RESET") == 0 ){	
				avatar::reset();					
				send_status();
			} else if( strcmp(content_.cmd,"POWER_OFF") == 0 ){	
				avatar::power_off();					
				send_status();
			} else if( strcmp(content_.cmd,"CONFIG") == 0 ){	
				send_config();					
			} else if( strcmp(content_.cmd,"POWER_ON") == 0 ){	
				avatar::power_on();					
				send_status();
			} else  if( strcmp(content_.cmd,"REBOOT") == 0 ){	
					IWDG->KR = 0xCCCC; /* (1) */
					IWDG->KR = 0x5555; /* (2) */
					IWDG->PR = IWDG_PR_PR_1; /* (3) */
					IWDG->RLR = 0xFFF; /* (4) */
					while(IWDG->SR); /* (5) */
			} else  if( strcmp(content_.cmd,"SET_SERVOS_LIMITS ") == 0 ){	
				avatar::set_speed_limit(content_.parse.speed_limit);					
				send_status();
			}
		}
	}
	else{
		static robo::time_ms_t prev = 0;
		robo::time_ms_t ms = robo::system::env::time_ms();
		if( ms - prev > 1000 ){
			prev = ms;
			//send_status();
		}
	}
}


bool avatar::save_state(uint32_t _res){
	FIL settingsFile;
	if( f_open(&settingsFile, "lastst.bin", FA_WRITE | FA_CREATE_ALWAYS) ==FR_OK){
		size_t byteswrite;
			struct {
			uint32_t flags;
			uint32_t data;
		} tmp;
		tmp.flags = 0x777;
		tmp.data = _res;
		if(f_write (&settingsFile,&tmp, 8,&byteswrite)==FR_OK){
			if(byteswrite==8){
				f_close(&settingsFile);
				return true;
			}
		}
		f_close(&settingsFile);
	}
	return false;
}
bool avatar::load_state(uint32_t & _dst){
	FIL settingsFile;
	if( f_open(&settingsFile, "lastst.bin", FA_READ) ==FR_OK){
		size_t bytesread;
		struct {
			uint32_t flags;
			uint32_t data;
		} tmp;
		if(f_read (&settingsFile,&tmp, 8,&bytesread)==FR_OK){
			if(bytesread==8){
				if(tmp.flags==0x777){
					_dst= tmp.data;
					f_close(&settingsFile);
					f_open(&settingsFile, "lastst.bin", FA_WRITE | FA_CREATE_ALWAYS);
					tmp.flags=0xFFEE;
					tmp.data=0x0000;
					f_write (&settingsFile,&tmp, 8,&bytesread);
					f_close(&settingsFile);
					return true;
				}
			}
		}
		f_close(&settingsFile);
	}
	return false;
}

			


//void command_poll(void){
//}