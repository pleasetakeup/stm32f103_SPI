#ifndef		_I2C_H
#define		_I2C_H


#include "stm8s_i2c.h"
#include "spi.h"


#define			I2C_SLAVE_ADDRES			(0x10)




extern uint8_t MessageBegin ,transmitflag, buf_t[4];


extern void i2c_init(void);
extern void I2C_Slave_check_event_ISR(void);


#endif