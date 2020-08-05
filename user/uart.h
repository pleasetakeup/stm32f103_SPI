#ifndef		_UART_H
#define		_UART_H



#include "stm8s_uart1.h"



#include "stdio.h"




extern void uart_init( uint32_t baudrate);

extern void uart_txdata(uint8_t *p, uint8_t dataLen);








#endif