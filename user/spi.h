#ifndef		_SPI_H
#define		_SPI_H



#include "stm8s_spi.h"
#include "stm8s_gpio.h"



//#define			CS_PORT		GPIOC
//#define			CS_PIN		GPIO_PIN_4
//
//#define		    MISO_PORT	GPIOC
//#define			MISO_PIN	GPIO_PIN_7
//
//#define			SCK_PORT	GPIOC
//#define			SCK_PIN		GPIO_PIN_5
//
//#define			CS_LOW()		GPIO_WriteLow(CS_PORT, CS_PIN)
//#define			CS_HIGH()		GPIO_WriteHigh(CS_PORT, CS_PIN)


#define SPI_CS                    GPIOC
#define SPI_Pin_CS                GPIO_PIN_3

#define SPI_DC                    GPIOC
#define SPI_Pin_DC                GPIO_PIN_4

#define SPI_SCK                   GPIOC
#define SPI_Pin_SCK               GPIO_PIN_5

#define SPI_MOSI                  GPIOC
#define SPI_Pin_MOSI              GPIO_PIN_6




extern void spi_config(void);
extern void spi_readbytes(uint8_t num, uint8_t *pt);

extern void DelayMS(unsigned int ms);


#endif





