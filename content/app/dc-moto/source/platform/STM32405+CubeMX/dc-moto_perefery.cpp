#include "main.h"
#include "adc.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "dc-moto.hpp"
#include "freemaster/robosd_fm.hpp"
#include "net/serial/robosd_serial.hpp"
#include "mexo/mexo_events.h"
#include "mexo/net/flow/flow_can_msg_id.h"
uint8_t uart_recv_data = 0;
volatile unsigned int *DWT_CYCCNT = (volatile unsigned int *)0xE0001004; //address of the register
volatile unsigned int *DWT_CONTROL = (volatile unsigned int *)0xE0001000; //address of the register
volatile unsigned int *SCB_DEMCR = (volatile unsigned int *)0xE000EDFC; //address of the register 

#if MEXO_DEBUG_SIGNAL_ENABLE == 1
void mexo_ev_debugSignalOn(void){
	//HAL_GPIO_WritePin(TP1_GPIO_Port,TP1_Pin,GPIO_PIN_SET);
}
void mexo_ev_debugSignalOff(void){
//	HAL_GPIO_WritePin(TP1_GPIO_Port,TP1_Pin,GPIO_PIN_RESET);
}
#endif

#if MEXO_GUARD_ENABLE == 1
/** приостанавливаем работу mexo*/
void * mexo_ev_guard_enter(void){
	uint32_t prim = __get_PRIMASK();
	__disable_irq();
	return (void *) prim;
}
/** продолжаем работу mexo*/
void mexo_ev_guard_leave(void * _context){
	int32_t prim = (uint32_t)_context;
	if(!prim) 
		__enable_irq();
}
#endif

class uart0 :  public robo::net::iserial{
public:
			virtual robo_size_t available(void){
				return (huart1.Instance->SR & UART_FLAG_RXNE) == UART_FLAG_RXNE ? 1:0;
			}
			virtual robo_size_t space(void){
				return (huart1.Instance->SR & UART_FLAG_TC) == UART_FLAG_TC ? 1 : 0;
			}
			virtual robo_size_t get(robo_byte_p _data, robo_size_t _max_size){
				if(_max_size>0){
					*_data = (uint8_t) huart1.Instance->DR;
					return 1;
				}else{
					return 0;
				}
			}
			virtual robo_result_t put(robo_byte_p _data, robo_size_t _size){
				if(_size == 1){
					huart1.Instance->DR = *_data ;
					return ROBO_SUCCESS;
				} else {
					return ROBO_ERROR;
				}
			}
			
			virtual uint8_t get(void){
					return (uint8_t) huart1.Instance->DR;
			}
			virtual robo_result_t  put(uint8_t _data){
				huart1.Instance->DR = _data;
				return ROBO_SUCCESS;
			}
			virtual void reset(void){
			}

} uart0_;


void dc_moto::iperefery::var_attach_(robo_vartable_p _vt){
}

void dc_moto::iperefery::begin_(void){
//Bootloader's interrupts disable
	HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
	//HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
	HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
	HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
	
	pwm_max = PWM_PERIOD*0.98;
	uart0_.reg(RS("UART0"));
	*SCB_DEMCR = *SCB_DEMCR | 0x01000000; 
	*DWT_CYCCNT = 0; // reset the counter 
	*DWT_CONTROL = *DWT_CONTROL | 1 ; // enable the counter	
	#if BOARD_FREE_MASTER_DRIVER_TYPE == BOARD_FREE_MASTER_DRIVER_TYPE_SERIAL_DIRRECT
	robo::freemaster::connect(robo::net::iserial::query(RS("UART0")));
	#endif
	
	HAL_GPIO_WritePin(CS_AS5040_GPIO_Port,CS_AS5040_Pin, GPIO_PIN_RESET);

	
}

void dc_moto::iperefery::start_(void){
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);	
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(&hcan1);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	
}
void dc_moto::iperefery::prioritet_loop_(void){
	adc_data[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1); // читаем полученное значение в переменную adc	
	adc_data[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2); // читаем полученное значение в переменную adc
}
void dc_moto::iperefery::permanent_loop_(void){
}


void  dc_moto::iperefery::pwm_power_boot(void){
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_GPIO_WritePin(nSD_GPIO_Port,nSD_Pin,GPIO_PIN_SET);
}

void dc_moto::iperefery::pwm_power_on(void){
}

void dc_moto::iperefery::pwm_power_off(void){
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	HAL_GPIO_WritePin(nSD_GPIO_Port,nSD_Pin,GPIO_PIN_RESET);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
}
void dc_moto::iperefery::pwm_power_shutdown(void){
}

void dc_moto::iperefery::pwm_set( int  _duty){
	if(_duty>0){
		TIM1->CCR1 = (uint32_t)_duty;
		TIM1->CCR2 = 0;
	} else {
		TIM1->CCR1 = 0;
		TIM1->CCR2 = (uint32_t)(-_duty);
	}
}

void dc_moto::iperefery::delay_us(uint32_t _us){
	unsigned int start, current; 
	start = *DWT_CYCCNT;
	_us *=168;
	do{
		current = *DWT_CYCCNT;
	} while((current - start) < _us);
}
uint16_t dc_moto::iperefery::motor_position_query(void){
	return htim3.Instance->CNT;
}


#if BOARD_MEXO_NET_FLOW_ENABLED == 1	
#include "mexo/board/board+flow+uart+can-rev3.hpp"
#include "mexo/net/adapters/flow_msg_port.h"

uint8_t g_dc_moto_mexo_addr = 0x0;
robo_result_t  dc_moto::iperefery::flow_set_addr( uint8_t _addr){
	//volatile uint32_t tmp = rdk_store_array[0];
	if (_addr > 0 && _addr < 16){
		CAN_FilterTypeDef canFilterConfig={0};
		for(int i=0;i<13;i++){
			canFilterConfig.FilterActivation = CAN_FILTER_DISABLE;
			canFilterConfig.FilterBank = i;
			HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);
		}
		canFilterConfig.FilterMode = 	CAN_FILTERMODE_IDMASK;
		canFilterConfig.FilterScale =  CAN_FILTERSCALE_32BIT;

		canFilterConfig.FilterMaskIdHigh = 0xDF0<<5;
		canFilterConfig.FilterMaskIdLow = 0;

		canFilterConfig.FilterIdHigh = _addr<<4<<5;
		canFilterConfig.FilterIdLow =  0;
		canFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
		canFilterConfig.FilterBank = 14;

		
		HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);


		canFilterConfig.FilterMode = 	CAN_FILTERMODE_IDMASK;
		canFilterConfig.FilterScale =  CAN_FILTERSCALE_32BIT;

		canFilterConfig.FilterMaskIdHigh = 0xDF0<<5;
		canFilterConfig.FilterMaskIdLow = 0;

		canFilterConfig.FilterIdHigh = 0x0<<4<<5;
		canFilterConfig.FilterIdLow =  0;
		canFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
		canFilterConfig.FilterBank = 15;
		HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);
		g_dc_moto_mexo_addr = _addr;
		return ROBO_SUCCESS;
	} else{
		return ROBO_ERROR;
	}
}
#else
robo_result_t board_flow_msg0_send(uint16_t _id, mexo_data_p _data, mexo_data_t _len);
#endif

#if FLOW_MSG_PORT0_ENABLED == 1
robo_result_t board_flow_msg0_send(uint16_t _id, mexo_data_p _data, mexo_data_t _len){
	if(_len>0){
		//HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox)
		CAN_TxHeaderTypeDef header;
		header.DLC =_len;
		header.ExtId = 0;
		header.IDE = CAN_ID_STD;
		header.RTR = CAN_RTR_DATA;
		header.StdId = _id;
		if (HAL_CAN_AddTxMessage(&hcan1, &header, _data, (uint32_t *)CAN_TX_MAILBOX0) == HAL_OK ){
			return ROBO_SUCCESS;
		}
	}
	return ROBO_ERROR;
}

void dc_moto::iperefery::async_send(uint8_t _suba, uint8_t * _data, uint8_t _size){
	flow_msg_id_t id;
	id.addr = g_dc_moto_mexo_addr;
	id.request = 0;
	id.slave = 1;
	id.suba = _suba;
	board_flow_msg0_send( id.value, _data, _size);
}
#endif

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	static int trig = 0;
	static CAN_RxHeaderTypeDef RxHeader;
	static uint8_t RxData[8];
	if(trig){
		mexo_debSignalOff(101);
		trig =0;
	} else{
		trig =1;
		mexo_debSignalOn(101);
	}
  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
#if FLOW_MSG_PORT0_ENABLED == 1
	flow_msg0_on_receive(
		RxHeader.StdId | ( g_dc_moto_mexo_addr  <<4)
		, RxData
			,RxHeader.DLC
		);
#else
	board_flow_msg0_send(RxHeader.StdId|0x100, RxData, RxHeader.DLC);
#endif
}


extern "C"{

void __aeabi_assert (const char *expr, const char *file, int line) {
  for (;;);
}

}


int16_t enco_counter = 0;
void dc_moto::iperefery::background_loop_(void){
	enco_counter = htim3.Instance->CNT;
static int n = 0;
if(n++ == 1000){
n = 0;
//as5048a2.getRawRotation();
//uart0_.put('A');
//board_flow_msg0_send(111, (uint8_t *)&n, 4);
	static int step = 0;
	switch(step++){
		case 0:
			HAL_GPIO_WritePin(TP1_GPIO_Port,TP1_Pin,GPIO_PIN_SET);
			break;
		case 1:
			HAL_GPIO_WritePin(TP2_GPIO_Port,TP2_Pin,GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(TP1_GPIO_Port,TP1_Pin,GPIO_PIN_RESET);
			break;
		case 3:
			step = 0;
			HAL_GPIO_WritePin(TP2_GPIO_Port,TP2_Pin,GPIO_PIN_RESET);
			break;
			
	}
	
	HAL_GPIO_TogglePin(TP1_GPIO_Port,TP1_Pin);

}
}
