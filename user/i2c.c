#include "i2c.h"



#define		RXBUFFER_LEN		(10)




uint8_t MessageBegin = FALSE, transmitflag = FALSE, buf_t[4] = {0,1,2,3};

uint8_t regaddr = 0;

void i2c_init(void)
{
	//I2C_DeInit();

	GPIO_Init(GPIOB , GPIO_PIN_4 , GPIO_MODE_OUT_OD_HIZ_FAST);//I2C_SCL
	GPIO_Init(GPIOB , GPIO_PIN_5 , GPIO_MODE_OUT_OD_HIZ_FAST);//I2C_SDA

	I2C_Init((uint32_t)(400000), (I2C_SLAVE_ADDRES << 1), 
			 I2C_DUTYCYCLE_2, I2C_ACK_CURR , 
			 I2C_ADDMODE_7BIT, 16 );

	I2C_ITConfig((I2C_IT_TypeDef)(I2C_IT_ERR | I2C_IT_EVT | I2C_IT_BUF), ENABLE);
}



// ********************** Data link function ****************************
// * These functions must be modified according to your application neeeds *
// * See AN document for more precision
// **********************************************************************
//void I2C_transaction_begin(void)
//{
//	spi_readbytes(4, buf_t);
//}



void I2C_transaction_end(void)
{

}



void I2C_byte_received(u8 u8_RxData)
{
	
	regaddr = u8_RxData;
	
}



u8 I2C_byte_write(void)
{	
	
	uint8_t data_t;
	
	if(regaddr > 3)
		return 0xff;
	
	data_t = buf_t[regaddr];
	regaddr++;
	
	return data_t;
	
}


// ********************** Data link interrupt handler *******************

void I2C_Slave_check_event_ISR(void) {
	
	__IO static uint8_t sr1;					
	__IO static uint8_t sr2;
	__IO static uint8_t sr3;
	
	// save the I2C registers configuration 
	sr1 = I2C->SR1; 
	sr2 = I2C->SR2;
	sr3 = I2C->SR3;
	
	/* Communication error? */
	if (sr2 & (I2C_SR2_WUFH | I2C_SR2_OVR |I2C_SR2_ARLO |I2C_SR2_BERR))
	{		
		I2C->CR2|= I2C_CR2_STOP;  // stop communication - release the lines
		I2C->SR2= 0;					    // clear all error flags
	}
	/* More bytes received ? */
	if ((sr1 & (I2C_SR1_RXNE | I2C_SR1_BTF)) == (I2C_SR1_RXNE | I2C_SR1_BTF))
	{
		//I2C_byte_received(I2C->DR);
	}
	/* Byte received ? */
	if (sr1 & I2C_SR1_RXNE)
	{
		I2C_byte_received(I2C->DR);
	}
	/* NAK? (=end of slave transmit comm) */
	if (sr2 & I2C_SR2_AF)
	{	
		I2C->SR2 &= ~I2C_SR2_AF;	  // clear AF
		I2C_transaction_end();
	}
	/* Stop bit from Master  (= end of slave receive comm) */
	if (sr1 & I2C_SR1_STOPF) 
	{
		I2C->CR2 |= I2C_CR2_ACK;	  // CR2 write to clear STOPF
		I2C_transaction_end();
	}
	/* Slave address matched (= Start Comm) */
	if (sr1 & I2C_SR1_ADDR)
	{	 
		//I2C_transaction_begin();
	}
	/* More bytes to transmit ? */
	if ((sr1 & (I2C_SR1_TXE | I2C_SR1_BTF)) == (I2C_SR1_TXE | I2C_SR1_BTF))
	{
		//I2C->DR = I2C_byte_write();
	}
	/* Byte to transmit ? */
	if (sr1 & I2C_SR1_TXE)
	{
		I2C->DR = I2C_byte_write();
	}	
	//GPIOD->ODR^=1;
}









