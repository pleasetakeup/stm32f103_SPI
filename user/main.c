/**
******************************************************************************
* @file    Project/main.c 
* @author  MCD Application Team
* @version V2.2.0
* @date    30-September-2014
* @brief   Main program body
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
*
* Unless required by applicable law or agreed to in writing, software 
* distributed under the License is distributed on an "AS IS" BASIS, 
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
******************************************************************************
*/ 
/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "stm8s_clk.h"
#include "stm8s_i2c.h"
#include "stm8s_spi.h"
#include "stm8s_it.h"
#include "stm8s_gpio.h"
#include "stm8s_uart1.h"
#include "stm8s_exti.h"
//#include "stm8s_tim1.h"
//#include "spi.h"
//#include "i2c.h"

/* Private defines -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#define ASR_MIC 0x0b
#define ASR_MONO 0x23


#define ASR_CLK_IN   	          24///频率
#define ASR_PLL_11			(uint8_t)((ASR_CLK_IN/2.0)-1)
#define ASR_PLL_ASR_19 		(uint8_t)(ASR_CLK_IN*32.0/(ASR_PLL_11+1) - 0.51)
#define ASR_PLL_ASR_1B 		0x48
#define ASR_PLL_ASR_1D 		0x1f

#define ASR_MIC_VOL 0x55//ADC增益初始值
#define ASR_SPEECH_ENDPOINT 0x10//语音端点检测初始值
#define ASR_SPEECH_START_TIME 0x08//语音端点检测开始时间初始值
#define ASR_SPEECH_END_TIME 0x10//语音端点检测结束时间初始值
#define ASR_VOICE_MAX_LENGHT 0xC3//最长语音段时间，默认20秒
#define ASR_NOISE_TIME 0x02//忽略上电噪声时间

#define SERIAL_TX_BUFFER_SIZE 128        //buffer大小
#define SPI_TX_BUFFER_SIZE    128              

typedef uint16_t tx_buffer_index_t;


#define	I2C_SLAVE_ADDRES (0x50)

typedef enum {
  
  BEGIN,
  ADDCOMMAND,
  START,
  LOOP,
  PASSWORD,
  BUTTON,
  IDLE,
}eCondition_t;


eCondition_t condition = IDLE;
eCondition_t _mode = IDLE;


unsigned char a = 0;
unsigned char b = 1;
unsigned char c = 0;
uint8_t regaddr; //在i2c中获得regaddr的值


uint8_t _tx_buffer[SERIAL_TX_BUFFER_SIZE];  //设定存储数据的数组

uint8_t _spi_buffer[SPI_TX_BUFFER_SIZE];  //设定存储数据的数组

uint8_t command[80];  // 命令数组


volatile tx_buffer_index_t _tx_buffer_head = 0 ;  
volatile tx_buffer_index_t _tx_buffer_tail = 0 ;  

volatile tx_buffer_index_t _spi_buffer_head = 0 ;  
volatile tx_buffer_index_t _spi_buffer_tail = 0 ;  

volatile uint8_t ready = 0;
volatile uint8_t commandNum = 0;

volatile uint8_t commandLen = 0;
volatile uint8_t state = 0 ;
uint8_t state1 = 0 ;
uint8_t startCommend = 0 ;
uint8_t gMic;



void Delay(u32 nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
} 
void spi_write(uint8_t data){    
  while(SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET);   
  SPI_SendData(data);    //一定不可省略不写，若不接收对应数据，马上去读数据就会造成读到的数据不对。   
  while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET);   
  u8 data2 = SPI_ReceiveData();
} 
void spi_wr_cmd(u8 cmd)
{
  //GPIO_WriteLow(GPIOA,GPIO_PIN_3);      //CS
  spi_write(cmd);
 /// GPIO_WriteHigh(GPIOA,GPIO_PIN_3);
}



void spi_wr_dat(u8 dat)
{
 //GPIO_WriteLow(GPIOA,GPIO_PIN_3);      //CS

  spi_write(dat);
  //a = SPI_ReceiveData();
//GPIO_WriteHigh(GPIOA,GPIO_PIN_3);
}





uint8_t spi_read(){
  //选择一个无效数据发送(自定义，只是为了给从设备提供时钟)，然后读取到对应数据   
  uint8_t data = 0x00;   
  while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET);    
  SPI_SendData(data);   
  while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET);   
  uint8_t rxdata  = SPI_ReceiveData();
  return rxdata;

} 


void uart_init( uint32_t baudrate)
{
	//Initalize the PD6, it's must to do this
	GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_FL_NO_IT );
	GPIO_Init(GPIOD,GPIO_PIN_5,GPIO_MODE_IN_PU_NO_IT);
	
	//RX_EN();
	//Reset the UART1
	UART1_DeInit();
	
	//Initalize the UART1
	UART1_Init((uint32_t)(baudrate),                    \
			  UART1_WORDLENGTH_8D,                      \
			  UART1_STOPBITS_1,                         \
			  UART1_PARITY_NO ,                         \
			  UART1_SYNCMODE_CLOCK_DISABLE ,            \
			  UART1_MODE_TXRX_ENABLE);

	UART1_ClearITPendingBit(UART1_IT_RXNE);
	//UART1_ITConfig( UART1_IT_RXNE_OR, DISABLE);

	//Open UART1
	UART1_Cmd( ENABLE );
	

	//Enable interrupt of all.
	//__enable_interrupt(); 
}


void uart1_txdata(u8 data){
  
    UART1_SendData8(data);
    while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
}

void uart1_txstr(u8 *p,u8 datalen){
  while(datalen){
    uart1_txdata(*p);
    p++;
    datalen--;
		
  }
  while (UART1_GetFlagStatus( UART1_FLAG_TC) == RESET);
}

void spi_write_buffer(uint8_t data){

  tx_buffer_index_t i = (_spi_buffer_head + 1) % SPI_TX_BUFFER_SIZE;  
  while(i == _spi_buffer_tail)  //说明buffer满了     
  {
    
      I2C->DR = _spi_buffer[_spi_buffer_tail];  
      _spi_buffer_tail = (_spi_buffer_tail + 1) % SPI_TX_BUFFER_SIZE;
      
  }
  _spi_buffer[_spi_buffer_head] = data; //将数据存入了_tx_buffer中，
  _spi_buffer_head = i;  //头指针增加
  
}


void i2c_init(void)
{
	//I2C_DeInit();

	//GPIO_Init(GPIOB , GPIO_PIN_4 , GPIO_MODE_OUT_OD_HIZ_FAST);//I2C_SCL
	//GPIO_Init(GPIOB , GPIO_PIN_5 , GPIO_MODE_OUT_OD_HIZ_FAST);//I2C_SDA

	I2C_Init((uint32_t)(8000000), (I2C_SLAVE_ADDRES << 1), 
			 I2C_DUTYCYCLE_2 , I2C_ACK_CURR , 
			 I2C_ADDMODE_7BIT, 16 );

	I2C_ITConfig((I2C_IT_TypeDef)(I2C_IT_ERR | I2C_IT_EVT | I2C_IT_BUF), ENABLE); //I2C中断
       //I2C_Cmd(ENABLE);
} 


void spi_init(void)
{

  //CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI, ENABLE);
    SPI_DeInit();
   GPIO_Init(GPIOC , GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST);   //RST
   GPIO_Init(GPIOD , GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST);
   SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_2, SPI_MODE_MASTER,\
            SPI_CLOCKPOLARITY_HIGH, SPI_CLOCKPHASE_1EDGE, \
            SPI_DATADIRECTION_2LINES_FULLDUPLEX, SPI_NSS_SOFT, 0x07);   
   SPI_Cmd(ENABLE);     
}

void SPI_Slave_check_event_ISR(void){
  uint8_t  data = 0;
   data = spi_read();
   spi_write_buffer(data);
}

void simulateReg(uint8_t sendData){
  
  if(sendData == 0xA1){
     condition = BEGIN;  
  }
  
  if(sendData == 0xA3){
    ready = 0;
    condition = ADDCOMMAND;
    commandLen = 0;
  }

  if(ready == 1) {
    command[commandLen++] =  sendData;
    if(commandNum == 1)  startCommend = command[0];
  }  

  if(sendData == 0xA2){    
    ready = 1;
    commandNum++;
  }
  
  if(sendData == 0xA4  && ready == 0){
    condition = START;
  }
  
  if(sendData == 0xA5  && ready == 0){
    condition = LOOP;
    _mode = LOOP;
  }
  if(sendData == 0xA6  && ready == 0){
    condition = PASSWORD;
    _mode = PASSWORD;
  }
  if(sendData == 0xA7  && ready == 0){
    condition = BUTTON;
    _mode = BUTTON;
  }
  if(sendData == 0xA8  && ready == 0){
    state1 = 0;
    
      GPIO_WriteHigh(GPIOD,GPIO_PIN_4);  
      GPIO_WriteHigh(GPIOD,GPIO_PIN_5);  
      GPIO_WriteHigh(GPIOD,GPIO_PIN_6);  
  }
  if(sendData == 0xA9  && ready == 0){
     gMic = ASR_MIC;
  }
  if(sendData == 0xAA  && ready == 0){
     gMic = ASR_MONO;
  }  
}


void write_buffer(u8 data)
{

  u8 sendData;
  tx_buffer_index_t i = (_tx_buffer_head + 1) % SERIAL_TX_BUFFER_SIZE;  
  while(i == _tx_buffer_tail)  //说明buffer满了     
  {      
      
      simulateReg(_tx_buffer[_tx_buffer_tail]);
      _tx_buffer_tail = (_tx_buffer_tail + 1) % SERIAL_TX_BUFFER_SIZE;
      
  }
  _tx_buffer[_tx_buffer_head] = data; //将数据存入了_tx_buffer中，
  _tx_buffer_head = i;  //头指针增加
  
}


unsigned char read_buffer()
{  
  u8 sendData;
  if (_tx_buffer_head == _tx_buffer_tail)  //buffer为空，没有数据
  {
   return 0;
  } 
  else
  {
      ///spi_write(_tx_buffer[_tx_buffer_tail]);  
    simulateReg(_tx_buffer[_tx_buffer_tail]);
      _tx_buffer_tail = (_tx_buffer_tail + 1) % SERIAL_TX_BUFFER_SIZE;
  } 
  
}


//IIC接收数据和SPI发送数据都在此函数实现。（中断）
void I2C_byte_received(u8 u8_RxData)
{
    write_buffer(u8_RxData);	
}

void spi_read_buffer(){

  if (_spi_buffer_head == _spi_buffer_tail)  //buffer为空，没有数据
  {
   return ;
  } 
  else
  {
      I2C->DR = _spi_buffer[_spi_buffer_tail] ;  
     _spi_buffer_tail = (_spi_buffer_tail + 1) % SPI_TX_BUFFER_SIZE;
  }
   
}




void I2C_Slave_check_event_ISR(void) {
  

	//STM8内部寄存器的定义在stm8s.h中
	__IO static uint8_t sr1;					
	__IO static uint8_t sr2;
	__IO static uint8_t sr3;
	 //read_buffer();
	// save the I2C registers configuration 
	sr1 = I2C->SR1;   //I2C_SR1 ： I2C status register 1
	sr2 = I2C->SR2;   //I2C_SR2 ： I2C status register 2
	sr3 = I2C->SR3;   //I2C_SR3 ： I2C status register 3
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
	if (sr1 & I2C_SR1_RXNE)  //I2C_SR1_RXNE ： Data Register not Empty (receivers)
	{        
		I2C_byte_received(I2C->DR); //I2C_DR ： I2C data register 
               // uart1_txdata(I2C->DR);
                
                
	}
	/* NAK? (=end of slave transmit comm) */
	if (sr2 & I2C_SR2_AF)
	{	
		I2C->SR2 &= ~I2C_SR2_AF;	  // clear AF
		//I2C_transaction_end();
	}
	/* Stop bit from Master  (= end of slave receive comm) */
	if (sr1 & I2C_SR1_STOPF) 
	{
		I2C->CR2 |= I2C_CR2_ACK;	  // CR2 write to clear STOPF
		//I2C_transaction_end();
	}
	/* Slave address matched (= Start Comm) */
	if (sr1 & I2C_SR1_ADDR)
	{	 
		//I2C_transaction_begin();
	}
	/* More bytes to transmit ? */
	if ((sr1 & (I2C_SR1_TXE | I2C_SR1_BTF)) == (I2C_SR1_TXE | I2C_SR1_BTF))
	{ 
          //spi_read_buffer();
	}
	/* Byte to transmispi_init();t ? */
	if (sr1 & I2C_SR1_TXE)
	{
	  spi_read_buffer();	

	}	

}



void spi_writeReg(uint8_t addr ,uint8_t data){
 GPIO_WriteLow(GPIOC,GPIO_PIN_3); 
 
  spi_write(0x04);
  spi_write(addr);
  spi_write(data);
  GPIO_WriteHigh(GPIOC,GPIO_PIN_3);
}

uint8_t spi_readReg(uint8_t addr){
   uint8_t regData = 0x00;
   GPIO_WriteLow(GPIOC,GPIO_PIN_3); 
   spi_write(0x05);
   spi_write(addr);
   regData = spi_read();
   GPIO_WriteHigh(GPIOC,GPIO_PIN_3);
  
  return regData;
}
void initLd3320(void){
  
  GPIO_WriteHigh(GPIOD,GPIO_PIN_3); 
  Delay(1000);
  GPIO_WriteLow(GPIOD,GPIO_PIN_3); 
  Delay(1000);
  GPIO_WriteHigh(GPIOD,GPIO_PIN_3); 
 
  Delay(1000);
  GPIO_WriteLow(GPIOC,GPIO_PIN_3); 
  Delay(1000);
  GPIO_WriteHigh(GPIOC,GPIO_PIN_3); 
  Delay(1000);
  spi_writeReg(0xb9, 0x00);
  
}


int LDbegin()
{

  state = 0;
   initLd3320();
  spi_readReg(0x06);  
  spi_writeReg(0x17, 0x35);
   Delay(1000);
  spi_readReg(0x06);  
  spi_writeReg(0x89, 0x03);  
   Delay(1000);
  spi_writeReg(0xcf, 0x43);  
   Delay(1000);
  spi_writeReg(0xcb, 0x02);
  spi_writeReg(0x11, ASR_PLL_11);  
  spi_writeReg(0x1e,0x00);
  spi_writeReg(0x19, ASR_PLL_ASR_19); 
  spi_writeReg(0x1b, ASR_PLL_ASR_1B);	
  spi_writeReg(0x1d, ASR_PLL_ASR_1D);
   Delay(1000);;
  spi_writeReg(0xcd, 0x04);
  spi_writeReg(0x17, 0x4c); 
   Delay(1000);
  spi_writeReg(0xb9, 0x00);
  spi_writeReg(0xcf, 0x4f);  
   spi_writeReg(0x6f, 0xff); 
  spi_writeReg(0xbd, 0x00);
  spi_writeReg(0x17, 0x48);
   Delay(1000);
  spi_writeReg(0x3c, 0x80);  
  spi_writeReg(0x3e, 0x07);
  spi_writeReg(0x38, 0xff);  
  spi_writeReg(0x3a, 0x07);
  spi_writeReg(0x40, 0);   
  spi_writeReg(0x42, 8);
  spi_writeReg(0x44, 0); 
  spi_writeReg(0x46, 8); 
  Delay(100);
      GPIO_WriteHigh(GPIOD,GPIO_PIN_4);  
      GPIO_WriteHigh(GPIOD,GPIO_PIN_5);  
    GPIO_WriteHigh(GPIOD,GPIO_PIN_6); 
  condition = IDLE;
 
  return 1;
}


u32 numberBlink = 0;

bool blinkFlag = 0;
uint8_t firstFlag = 0;
void gpioInit()
{
    //GPD->PIN3 设置为输入模式 带上拉电阻输入 使能外部中断
  GPIO_Init(GPIOA , GPIO_PIN_3 , GPIO_MODE_IN_PU_IT); 
  
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOA , EXTI_SENSITIVITY_RISE_FALL); //PD端口为下降沿触发中断

}

void blink(){
  
  if(firstFlag ==   0){
    if(_mode == LOOP){
       GPIO_WriteLow(GPIOD,GPIO_PIN_4);
      //goto out;
       // EXTI_DeInit();
        firstFlag++;
    }
    else if(_mode == BUTTON){
       //GPIO_WriteLow(GPIOD,GPIO_PIN_6);
         disableInterrupts();  //关闭系统总中断
         gpioInit();
         __enable_interrupt();
       firstFlag++;
    }
    else if(_mode == PASSWORD && state1 == 1){
       //GPIO_WriteLow(GPIOD,GPIO_PIN_4);
      // GPIO_WriteLow(GPIOD,GPIO_PIN_5);
     //  GPIO_WriteLow(GPIOD,GPIO_PIN_6);
    //  EXTI_DeInit();  
      firstFlag++;
    }
    else{
    
    }
   
        
    }
    
 
  if(blinkFlag == 1){
     numberBlink++;
  
  } 
  else{
    return;
  }
  if(numberBlink == 5000){
    
    if(_mode == LOOP){
    GPIO_WriteHigh(GPIOD,GPIO_PIN_4);
    }
    else if(_mode == BUTTON){
    GPIO_WriteHigh(GPIOD,GPIO_PIN_6);
    }
    else if(_mode == PASSWORD){
      GPIO_WriteHigh(GPIOD,GPIO_PIN_4);  
      GPIO_WriteHigh(GPIOD,GPIO_PIN_5);  
    GPIO_WriteHigh(GPIOD,GPIO_PIN_6);  
      
    }
  }
  if(numberBlink == 10000){
    if(_mode == LOOP){
    GPIO_WriteLow(GPIOD,GPIO_PIN_4);
    }
    else if(_mode == BUTTON){
    GPIO_WriteLow(GPIOD,GPIO_PIN_6);
    }
    else if(_mode == PASSWORD){
      GPIO_WriteLow(GPIOD,GPIO_PIN_4);  
      GPIO_WriteLow(GPIOD,GPIO_PIN_5);  
    GPIO_WriteLow(GPIOD,GPIO_PIN_6);  
      
    }
  }
  if(numberBlink == 15000)
    if(_mode == LOOP){
    GPIO_WriteHigh(GPIOD,GPIO_PIN_4);
    }
    else if(_mode == BUTTON){
    GPIO_WriteHigh(GPIOD,GPIO_PIN_6);
    }
    else if(_mode == PASSWORD){
      GPIO_WriteHigh(GPIOD,GPIO_PIN_4);  
      GPIO_WriteHigh(GPIOD,GPIO_PIN_5);  
    GPIO_WriteHigh(GPIOD,GPIO_PIN_6);  
      
    }
  
  if(numberBlink >= 20000){
    if(_mode == LOOP){
    GPIO_WriteLow(GPIOD,GPIO_PIN_4);
    }
    else if(_mode == BUTTON){
    GPIO_WriteLow(GPIOD,GPIO_PIN_6);
    }
    else if(_mode == PASSWORD){
      GPIO_WriteLow(GPIOD,GPIO_PIN_4);  
      GPIO_WriteLow(GPIOD,GPIO_PIN_5);  
    GPIO_WriteLow(GPIOD,GPIO_PIN_6);  
      
    }
    blinkFlag = 0;
    numberBlink = 0;
    
    _mode =  IDLE;
  }
  
}

int read(){
   uint8_t Asr_Count=0;
   uint8_t  readnum = 0xFF;
  //  GPIO_WriteLow(GPIOD,GPIO_PIN_4);
    if((spi_readReg(0x2b) & 0x10) && spi_readReg(0xb2)==0x21 && spi_readReg(0xbf) == 0x35)//如果有语音识别中断、DSP闲、ASR正常结束
    {
      spi_writeReg(0x29,0) ;///////////关中断
      spi_writeReg(0x02,0) ;/////////////关FIFO中断
      Asr_Count = spi_readReg(0xba);//读中断辅助信息
      if(Asr_Count>=1 && Asr_Count<4) //////如果有识别结果
      {
          readnum=spi_readReg(0xc5);
          spi_write_buffer(readnum);
          blinkFlag = 1;
      }
      else{
          spi_write_buffer(readnum);
      
      }
      spi_writeReg(0x2b,0);//////清楚中断编号
      spi_writeReg(0x1C,0);////////貌似关麦克风啊~~为毛
      spi_writeReg(0x37, 0x06);//开始识别
      spi_writeReg(0x1c, gMic);//选择麦克风
      spi_writeReg(0x29, 0x10);//开同步中断
      spi_writeReg(0xbd, 0x00);///启动为语音识别
    }
    else{
      spi_write_buffer(readnum);
    
    }
   return readnum;
}


int checkBus()
{ 
  int j;
  for (j=0; j<10; j++)
	{
	  if (spi_readReg(0xb2) == 0x21)
		{
			return 1;
		}
	  Delay(100);		
	}
  return 0;
}
void RGB_init()
{
  GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(GPIOD, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_OUT_PP_LOW_FAST);

  GPIO_WriteHigh(GPIOD,GPIO_PIN_6);
  GPIO_WriteHigh(GPIOD,GPIO_PIN_4);
  GPIO_WriteHigh(GPIOD,GPIO_PIN_5);

}

uint8_t start(){

  spi_writeReg(0x35, ASR_MIC_VOL);////adc增益；会影响识别范围即噪声
  spi_writeReg(0xb3, ASR_SPEECH_ENDPOINT);//语音端点检测控制
  spi_writeReg(0xb4, ASR_SPEECH_START_TIME);//语音端点起始时间
  spi_writeReg(0xb5, ASR_SPEECH_END_TIME);//语音结束时间
  spi_writeReg(0xb6, ASR_VOICE_MAX_LENGHT);//语音结束时间
  spi_writeReg(0xb7, ASR_NOISE_TIME);//噪声时间
  spi_writeReg(0x1c, 0x09);////////麦克风设置保留
  spi_writeReg(0xbd, 0x20);/////////保留设置
  spi_writeReg(0x08, 0x01);///////////清除FIFO_DATA
  Delay(1000);
  spi_writeReg(0x08, 0x00);////////////清除指定FIFO后再写入一次00H
  Delay(1000);
  if(checkBus() == 0)////////读取0xB2寄存器函数如果DSP没在闲状态则RETURN 0
  {
     return 0;
  }
  spi_writeReg(0xb2, 0xff);////////给0xB2写FF
  
  spi_writeReg(0x37, 0x06);////////开始识别
  Delay(5);
  spi_writeReg(0x1c, gMic);////////选择麦克风
  spi_writeReg(0x29, 0x10);////////开同步中断
  spi_writeReg(0xbd, 0x00);/////////启动为语音识别
  
  
  condition = IDLE;
  firstFlag = 0;
  
  
  
  return 1;////返回1
}


void addCommand()
{

    int i;
    spi_writeReg(0xc1, command[0]);
    spi_writeReg(0xc3, 0x00);
    spi_writeReg(0x08, 0x04);
    Delay(1000);
    spi_writeReg(0x08, 0x00);
    Delay(1000);
    for(i=2;i<command[1] + 2;i++)
    {
        spi_writeReg(0x5, command[i]);

    }
    spi_writeReg(0xb9, command[1]);
    spi_writeReg(0xb2, 0xff); 
    spi_writeReg(0x37, 0x04);
    condition = IDLE;
}

int loopMode(){
  read();
  condition = IDLE;

}

int commandMode(){
   uint8_t Asr_Count=0;
   
   uint8_t  readnum = 0xFF;
   
    if((spi_readReg(0x2b) & 0x10) && spi_readReg(0xb2)==0x21 && spi_readReg(0xbf) == 0x35)//如果有语音识别中断、DSP闲、ASR正常结束
    {
      spi_writeReg(0x29,0) ;///////////关中断
      spi_writeReg(0x02,0) ;/////////////关FIFO中断
      Asr_Count = spi_readReg(0xba);//读中断辅助信息
      if(Asr_Count>=1 && Asr_Count<4) //////如果有识别结果
      {
          readnum=spi_readReg(0xc5);
          if(readnum == startCommend){
            state1 = 1;
             blinkFlag = 1;
            spi_write_buffer(readnum);
            
          }
          else if(state1 == 1){
            spi_write_buffer(readnum); 
            blinkFlag = 1;
          }
          else{
            spi_write_buffer(0xff);
          
          }
      }
      else{
          spi_write_buffer(readnum);
      
      }
      spi_writeReg(0x2b,0);//////清楚中断编号
      spi_writeReg(0x1C,0);////////貌似关麦克风啊~~为毛
      spi_writeReg(0x37, 0x06);//开始识别
      spi_writeReg(0x1c, gMic);//选择麦克风
      spi_writeReg(0x29, 0x10);//开同步中断
      spi_writeReg(0xbd, 0x00);///启动为语音识别
    }
    else{
      spi_write_buffer(readnum);
    
    }
   condition = IDLE;
   return readnum;
  
}
int buttonMode(){
  
   uint8_t Asr_Count=0;
   uint8_t  readnum = 0xFF;
   
    if((spi_readReg(0x2b) & 0x10) && spi_readReg(0xb2)==0x21 && spi_readReg(0xbf) == 0x35)//如果有语音识别中断、DSP闲、ASR正常结束
    {
      spi_writeReg(0x29,0) ;///////////关中断
      spi_writeReg(0x02,0) ;/////////////关FIFO中断
      Asr_Count = spi_readReg(0xba);//读中断辅助信息
      if(Asr_Count>=1 && Asr_Count<4 ) //////如果有识别结果
      {
          readnum=spi_readReg(0xc5);
          if( state == 1){
            spi_write_buffer(readnum);
            blinkFlag = 1;
          }
          else
          {
            spi_write_buffer(0xff);
          }
      }
      else{
          spi_write_buffer(readnum);
      
      }
      spi_writeReg(0x2b,0);//////清楚中断编号
      spi_writeReg(0x1C,0);////////貌似关麦克风啊~~为毛
      spi_writeReg(0x37, 0x06);//开始识别
      spi_writeReg(0x1c, gMic);//选择麦克风
      spi_writeReg(0x29, 0x10);//开同步中断
      spi_writeReg(0xbd, 0x00);///启动为语音识别
    }
    else{
      spi_write_buffer(readnum);
    
    }
   condition = IDLE;
   return readnum;

}





void EXIT_ASR(){

   // Delay(100);
   // if(GPIO_ReadInputPin(GPIOA , GPIO_PIN_3) == RESET)   //判断是否是PD->3,被按下，即KEY1,也可以说这个判断是PD端口区分是哪个引脚被按下的主要标志
 // {
    if(_mode == BUTTON){
      //state = 1 - state;
        if(GPIO_ReadInputPin(GPIOA , GPIO_PIN_3) == RESET)   //判断是否是PD->3,被按下，即KEY1,也可以说这个判断是PD端口区分是哪个引脚被按下的主要标志
             {
                  state = 1;
                  GPIO_WriteLow(GPIOD,GPIO_PIN_6);           //异或取反控制LED1的亮灭
             }else{
               state = 0;
                   GPIO_WriteHigh(GPIOD,GPIO_PIN_6);
    }
}
// }
}

 void main(void)
{
      //   uint8_t i;
          //SPI->DR = 12; 
        // CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV2);
     //  disableInterrupts();  //关闭系统总中断
   //__enable_interrupt();
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
        //GPIO_Init(GPIOD, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST);
      // GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_OUT_PP_LOW_FAST);
        //GPIO_WriteHigh(GPIOD,GPIO_PIN_6);
      //   uart_init(115200); 
        
       // setRGB();
	spi_init();
        i2c_init();
        RGB_init();
        //gpioInit();
	__enable_interrupt();
        while(1)
        {
           read_buffer();
           blink();
           switch (condition)             // 根据menu()的结果跳转
           {
              case      BEGIN:  LDbegin();
                         break; 
              case     ADDCOMMAND: addCommand();               // 执行动作1
                        break;           // 不执行任何其他动作
              case      START: start();
                        break;   
              case       LOOP: loopMode();       // 执行动作2
                        break;           // 不执行默认的动作
              case   PASSWORD: commandMode();       // 执行动作2
                       break;    
              case     BUTTON: buttonMode();       // 执行动作2
                        break;    
              case       IDLE:     // 执行动作2
                        break;   
              default: break; // 如果没有识别到任何命令，输出一个警告信息
           }
           
          }
      
       //read_buffer();
       //spi_writeBuffer();
          //ready = spi_readReg(0x06);
         // uart1_txdata(ready);
       //uart1_txdata(spi_readReg(0x09));
      //  uart1_txdata(spi_readReg(0x06));
           
        }

#ifdef USE_FULL_ASSERT

/**
*    @brief  Reports the name of the source file and the source line number
*      where the assert_param error has occurred.
*    @param file: pointer to the source file name
*    @param line: assert_param error line source number
*    @retval : None
*/
void assert_failed(u8* file, u32 line)
{ 
	/* User can add his own implementation to report the file name and line number,
	ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	
	/* Infinite loop */
	while (1)
	{
	}
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
