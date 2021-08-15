#include "spi.h"
#include "mexo/mexo_events.h"
#define SPI_BEGIN_TRANSACTION()
#define SPI_END_TRANSACTION()
#define SPI_CS_SET() HAL_GPIO_WritePin(CSE2_GPIO_Port,CSE2_Pin,GPIO_PIN_SET)
#define SPI_CS_RESET() HAL_GPIO_WritePin(CSE2_GPIO_Port,CSE2_Pin,GPIO_PIN_RESET)

uint16_t  SPI_TRANSFER16( uint16_t _x) {
	uint16_t res;
	uint8_t x[2];
	uint8_t y[2];
	
	x[1]= _x & 0xFF;
	x[0]= _x & ( _x >> 8 ) & 0xFF;

	res = ((uint16_t)(y[0]))<<8;
	res += y[1];
	
	return  res;
}

#define DELAY() mexo_ev_delay_us(10)
