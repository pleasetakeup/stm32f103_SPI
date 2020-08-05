#include "spi.h"




#define		MISO_DATA()			GPIO_ReadInputPin(MISO_PORT, MISO_PIN)

#define		SCK_LOW()			GPIO_WriteLow(SCK_PORT, SCK_PIN)
#define		SCK_HIGH()			GPIO_WriteHigh(SCK_PORT, SCK_PIN)

void DelayMS(unsigned int ms)
{
	unsigned int i;
	while(ms != 0)
	{
		for(i=0;i<1150;i++)
		{
		}
		ms--;  
	}     
}

#if 1
void spi_config(void)
{
//	SPI_DeInit();
//	
//	GPIO_Init(MISO_PORT,MISO_PIN, GPIO_MODE_IN_PU_NO_IT);
//	
//	GPIO_Init(SCK_PORT, SCK_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
//	
//    
//	
//	//SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_16, SPI_MODE_MASTER,\
//			 SPI_CLOCKPOLARITY_LOW, SPI_CLOCKPHASE_2EDGE,SPI_DATADIRECTION_1LINE_RX, SPI_NSS_SOFT, 0x07);
//	
//	SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_64, SPI_MODE_MASTER, SPI_CLOCKPOLARITY_LOW, \
//			 SPI_CLOCKPHASE_1EDGE,  SPI_DATADIRECTION_2LINES_FULLDUPLEX, SPI_NSS_SOFT, 0x07);
//	SPI_Cmd(ENABLE);
//	GPIO_Init(CS_PORT, CS_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);
        GPIO_Init(SPI_CS , SPI_Pin_CS, GPIO_MODE_OUT_PP_HIGH_FAST);
        GPIO_Init(SPI_DC , SPI_Pin_DC, GPIO_MODE_OUT_PP_HIGH_FAST);
        SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_2, SPI_MODE_MASTER,\
            SPI_CLOCKPOLARITY_HIGH, SPI_CLOCKPHASE_2EDGE, \
            SPI_DATADIRECTION_2LINES_FULLDUPLEX, SPI_NSS_SOFT, 0x07); 
        SPI_Cmd(ENABLE);  
   
}





void spi_readbytes(uint8_t num, uint8_t *pt)
{
	
	//SPI_ReceiveData();
	
	SPI_ClearFlag(SPI_FLAG_RXNE);
	
	CS_LOW();
	//SPI_Cmd(ENABLE);
#if 1
	//for(i = 0; i < num; i++)
	//{
		
		while (SPI_GetFlagStatus( SPI_FLAG_TXE) == RESET);
  SPI_SendData(0X00);//--------------------------发送命令
  while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET);
  pt[0]=SPI_ReceiveData();
		
		while (SPI_GetFlagStatus( SPI_FLAG_TXE) == RESET);
  SPI_SendData(0X00);//--------------------------发送命令
  while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET);
  pt[1]=SPI_ReceiveData();
		
		while (SPI_GetFlagStatus( SPI_FLAG_TXE) == RESET);
  SPI_SendData(0X00);//--------------------------发送命令
  while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET);
  pt[2]=SPI_ReceiveData();
		
		while (SPI_GetFlagStatus( SPI_FLAG_TXE) == RESET);
  SPI_SendData(0X00);//--------------------------发送命令
  while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET);
  pt[3]=SPI_ReceiveData();
		//SPI_ClearFlag(SPI_FLAG_RXNE);
		//pt++;
	//}
	
	
	//while ( SPI_GetFlagStatus(SPI_FLAG_BSY) );


	//SPI_Cmd(DISABLE);
#endif
	CS_HIGH();
	

	
}








#endif

#if 0

void delayUs(uint16_t nCount)
{
  uint16_t i;
  for(i=0;i<nCount;i++)
  {
    nop();nop();nop();nop();nop();nop();nop();nop();
  }
}



void spi_config(void)
{
	GPIO_Init(MISO_PORT,MISO_PIN, GPIO_MODE_IN_PU_NO_IT);
	GPIO_Init(SCK_PORT, SCK_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
	GPIO_Init(CS_PORT, CS_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);
}
/**
  * @brief  发送数据到595芯片.
  * @param  rData : 列扫信号，打开某一列如：0XFFFE，打开第1列.
  * @param  pData : 列扫描数据.
  * @retval void.
  */

void spi_readbytes(uint8_t num, uint8_t *pt)
{
  uint8_t i;
  uint32_t data = 0;
 
  CS_LOW();
  delayUs(20);
  
  
  
  for(i=0;i<32;i++)
  {
	
	SCK_HIGH();
	 delayUs(10);
	
	SCK_LOW();
	if(MISO_DATA())
		data |= 1;
	 delayUs(10);
	data <<= 1;
	
  }
  
  SCK_LOW();
  CS_HIGH();
  
  *pt =  data >> 24;
  pt++;
  *pt =  data >> 16;
  pt++;
  *pt =  data >> 8;
  pt++;
  *pt =  data ;

  
}


#endif












