#include "uart.h"



void uart_init( uint32_t baudrate)
{
	//Initalize the PD6, it's must to do this
	GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_FL_NO_IT );
	GPIO_Init(GPIOD,GPIO_PIN_5,GPIO_MODE_IN_PU_NO_IT);
	
	//RX_EN();
	//Reset the UART1
	UART2_DeInit();
	
	//Initalize the UART1
	UART2_Init((uint32_t)(baudrate),                    \
			  UART2_WORDLENGTH_8D,                      \
			  UART2_STOPBITS_1,                         \
			  UART2_PARITY_NO ,                         \
			  UART2_SYNCMODE_CLOCK_DISABLE ,            \
			  UART2_MODE_TXRX_ENABLE);

	//Open the IT.
	
	UART2_ClearITPendingBit(UART2_IT_RXNE);
	UART2_ITConfig( UART2_IT_RXNE_OR, DISABLE );

	//Open UART1
	UART2_Cmd( ENABLE );
	

	//Enable interrupt of all.
	//__enable_interrupt(); 
}


int fputc(int ch ,FILE *f)
{

	UART2_SendData8((unsigned char)ch);
	/* Loop until the end of transmission */
	while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET);
	return (ch);

}


void uart_txdata(uint8_t *p, uint8_t dataLen)
{

	while(dataLen){
	
		UART2_SendData8(*p);
		p++;
		dataLen--;
		/* Loop until the end of transmission */
		while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET);
		
	}
	
	while (UART2_GetFlagStatus( UART2_FLAG_TC) == RESET);

}


















