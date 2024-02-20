/****************************---STM32驱动TDC-GP22代码参考---*****************************/

/************************--TDC-GP22  SPI1模式 最高时钟频率20MHz-----***********************/

/*********************************--相关IO口配置参阅对应.h文件--****************************/

/*******--调试步骤：SPI通信成功->检查激励波形->中断能否正常触发—>读出结果—>滤波拟合---*******/

#include "spi.h"
#include "tdc.h"


#define Dummy_Byte                      0xFF
#define TDC_CS_1  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
#define TDC_CS_0  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

uint8_t    SPI1_ReadWriteByte(uint8_t byte)
{
    uint8_t d_read,d_send=byte;

    if(HAL_SPI_TransmitReceive(&hspi1,&d_send,&d_read,1,0xFFFFFF)!=HAL_OK)
        d_read=Dummy_Byte;

    return d_read;
}

/***************************************************************************
 * DotHextoDotDec function.
 *
 * @brief	Convert Dot HEX to Dot DEC of the char.
 **************************************************************************/
float dotHextoDotDec(unsigned long dotHex)
{
   float dotDec = 0;
   dotDec = (float)(((dotHex >> 28) & 0x0000000F)*4096 + ((dotHex >> 24) & 0x0000000F)*256 + ((dotHex >> 20) & 0x0000000F)*16 + ((dotHex >> 16) & 0x0000000F)*1);
   dotDec = dotDec + ((float)((dotHex >> 12) & 0x0000000F))/16 + ((float)((dotHex >> 8) & 0x0000000F))/256 + ((float)((dotHex >> 4) & 0x0000000F))/4096 + ((float)((dotHex >> 0) & 0x0000000F))/65536;
   return dotDec*0.25;
}


uint8_t SPI1_ReadByteFromReg(uint8_t regAddr)
{
  uint8_t res = 0;
  
  TDC_CS_0;
  HAL_Delay(1);
  SPI1_ReadWriteByte(regAddr);
  HAL_Delay(1);
  //读第一个字节
  res = SPI1_ReadWriteByte(Dummy_Byte);
  HAL_Delay(1);
  TDC_CS_1;//P9.4 TDC_SSN置高 
  
  return res;
}

uint16_t SPI1_Read2ByteFromReg(uint8_t regAddr)
{
  uint16_t res = 0;
  
  TDC_CS_0;
//  delay_us(3);
  SPI1_ReadWriteByte(regAddr);
  //读第一个字节
  res |= SPI1_ReadWriteByte(Dummy_Byte);
  res <<= 8;
  res |= SPI1_ReadWriteByte(Dummy_Byte);
	
//  delay_us(3);
	
  TDC_CS_1;//P9.4 TDC_SSN置高 
  
  return res;
}

uint32_t SPI1_ReadWordFromReg(uint8_t regAddr)
{
  uint32_t res = 0;
  
  TDC_CS_0;
//  HAL_Delay(1);
  SPI1_ReadWriteByte(regAddr);
  //读第一个字节
  res |= SPI1_ReadWriteByte(Dummy_Byte);
  res <<= 8;
  //读第二个字节
  res |= SPI1_ReadWriteByte(Dummy_Byte);
  res <<= 8;
  //读第三个字节
  res |= SPI1_ReadWriteByte(Dummy_Byte);
  res <<= 8;
  //读第四个字节
  res |= SPI1_ReadWriteByte(Dummy_Byte);
//  HAL_Delay(1);
  TDC_CS_1;//P9.4 TDC_SSN置高 
  
  return res;
}



//tdc寄存器配置
void TDC_Init_Reg( void )
{
	powerOnResetTDCGP22();
	HAL_Delay(1);
  configureRegisterTDCGP22( WRITE_REG0, 0x00242000 );      /// Configure TDC register. 0100 0011 1000 1001 1110 1000 0000 0000
  configureRegisterTDCGP22( WRITE_REG1, 0x19490000 );      /// Configure TDC register.
  configureRegisterTDCGP22( WRITE_REG2, 0xE0000000 );      /// Configure TDC register.
  configureRegisterTDCGP22( WRITE_REG3, 0x00000000 );      /// Configure TDC register.
  configureRegisterTDCGP22( WRITE_REG4, 0x20000000 );      /// Configure TDC register.
  configureRegisterTDCGP22( WRITE_REG5, 0x10000000 );      /// Configure TDC register.
  configureRegisterTDCGP22( WRITE_REG6, 0x00000000 );      /// Configure TDC register.
}



void powerOnResetTDCGP22(void)
{
		TDC_CS_0;//TDC_SSN置低 
		SPI1_ReadWriteByte(POWER_ON_RESET);
		TDC_CS_1;//TDC_SSN置高 
}

void initMeasureTDCGP22(void)
{
		TDC_CS_0;//TDC_SSN置低 
		SPI1_ReadWriteByte(INIT_MEASURE);
		TDC_CS_1;//TDC_SSN置高
}

void timeFlightStartTDCGP22(void)
{
		TDC_CS_0;//TDC_SSN置低
		SPI1_ReadWriteByte(START_TOF);
		TDC_CS_1;//TDC_SSN置高
}

//void timeFlightRestartTDCGP22(void)
//{
//		TDC_CS_0;//TDC_SSN置低
////	  delay_us(1);
//		SPI1_ReadWriteByte(START_TOF_RESTART);
//	  delay_us(1);
//		TDC_CS_1;//TDC_SSN置高
//}

void tempStartTDCGP22(void)
{
		TDC_CS_0;//TDC_SSN置低
		SPI1_ReadWriteByte(START_TEMP);
		TDC_CS_1;//TDC_SSN置高
}
void tempReStartTDCGP22(void)
{
		TDC_CS_0;//TDC_SSN置低
		SPI1_ReadWriteByte(START_TEMP_RESTART);
		TDC_CS_1;//TDC_SSN置高
}

void configureRegisterTDCGP22(uint8_t opcode_address,uint32_t config_reg_data)
{
  unsigned char data_byte_lo;
  unsigned char data_byte_mid1;
  unsigned char data_byte_mid2;
  unsigned char data_byte_hi;
  
  data_byte_lo = (uint8_t)(config_reg_data & 0xff);
  data_byte_mid1 = (uint8_t)(config_reg_data>>8 & 0xff);
  data_byte_mid2 = (uint8_t)(config_reg_data>>16 & 0xff);
  data_byte_hi = (uint8_t)(config_reg_data>>24 & 0xff);
  
  TDC_CS_0;//P9.4 TDC_SSN置低
  SPI1_ReadWriteByte(opcode_address);
  SPI1_ReadWriteByte(data_byte_hi);
  SPI1_ReadWriteByte(data_byte_mid2);
  SPI1_ReadWriteByte(data_byte_mid1);
  SPI1_ReadWriteByte(data_byte_lo); 
  TDC_CS_1;//P9.4 TDC_SSN置高
}


//u16 readStatusRegisterTDCGP22(void)
//{
//  unsigned int result = 0;
//  
//  TDC_CS_0;
//  delay_us(3);
//  SPI1_ReadWriteByte(READ_STAT);
//    //读第一个字节
//  result |= SPI1_ReadWriteByte(DUMMY_DATA);
//  result <<= 8;
//    //读第二个字节
//  result |= SPI1_ReadWriteByte(DUMMY_DATA);
//  delay_us(3);
//  TDC_CS_1;//P9.4 TDC_SSN置高 
//  delay_us(3);
//  return result;
//}

//float readPW1STRegisterTDCGP22(void)
//{
//  float result = 0;
//  unsigned char resultByte;
//  
//  TDC_CS_0;
//  delay_us(3);
//  SPI1_ReadWriteByte(READ_PW1ST);
//    //读第一个字节
//  resultByte = SPI1_ReadWriteByte(DUMMY_DATA);
//  delay_us(3);
//  TDC_CS_1;//P9.4 TDC_SSN置高 
//  delay_us(3);
//  if( (resultByte & 0x80) != 0x00 )
//  {
//    result += 1;
//  }
//  resultByte = resultByte & 0x7F;
//  result = result + (float)(resultByte >> 3)/16 + (float)((resultByte << 1) & 0x0F)/256; 
//  
//  return result;
//}


//unsigned char ID_Bytes[7];
////testcomunication();
//void readIDbytesTDCGP22(void)
//{ 
//  TDC_CS_0;
//  delay_us(3);
//  SPI1_ReadWriteByte(READ_IDBIT);
//  //读第一个字节
//  ID_Bytes[0] = SPI1_ReadWriteByte(0xff);
//  //读第二个字节
//  ID_Bytes[1] = SPI1_ReadWriteByte(0xff);
//  //读第三个字节
//  ID_Bytes[2] = SPI1_ReadWriteByte(0xff);
//  //读第四个字节
//  ID_Bytes[3] = SPI1_ReadWriteByte(0xff);
//  //读第五个字节
//  ID_Bytes[4] = SPI1_ReadWriteByte(0xff);
//  //读第六个字节
//  ID_Bytes[5] = SPI1_ReadWriteByte(0xff); 
//  //读第七个字节
//  ID_Bytes[6] = SPI1_ReadWriteByte(0xff);
//  delay_us(3);
//  TDC_CS_1;//P9.4 TDC_SSN置高 
//	delay_us(3);
//}

//u8 SPI_TDC_SendByte(u8 byte)
//{
//	 
//  u16 SPITimeout = 10000;

//  /* 等待发送缓冲区为空，TXE事件 */
//  while (SPI_I2S_GetFlagStatus(TDC_SPI, SPI_I2S_FLAG_TXE) == RESET)
//   {
//    if((SPITimeout--) == 0)  Error();
//   }

//  /* 写入数据寄存器，把要写入的数据写入发送缓冲区 */
//  SPI_I2S_SendData(TDC_SPI, byte);

//  SPITimeout = 10000;

//  /* 等待接收缓冲区非空，RXNE事件 */
//  while (SPI_I2S_GetFlagStatus(TDC_SPI, SPI_I2S_FLAG_RXNE) == RESET)
//   {
//    if((SPITimeout--) == 0) Error();
//   }

//  /* 读取数据寄存器，获取接收缓冲区数据 */
//  return SPI_I2S_ReceiveData(TDC_SPI);
//}
//unsigned char G_tdcInterruptFlag                 = 0;
// u16          G_tdcStatusRegister                 =0 ;
//unsigned long G_tdcTimeOutCount                  = 0;
//float         G_calibrateResult                  = 0;
//float  G_calibrateCorrectionFactor=0.000f           ;
//void calibrateResonator( void )
//{
//  unsigned long temp;

//	TDC_CS_0;//TDC_SSN置低
//	SPI1_ReadWriteByte(START_CAL_OSC);
//	TDC_CS_1;//TDC_SSN置高

//  while( INTSign == 0);
//  INTSign = 0;
//  G_tdcStatusRegister = readStatusRegisterTDCGP22();
//  
//  temp = readRegisterTDCGP22(READ_RES0+(G_tdcStatusRegister&0x0007)-1);
//  G_calibrateResult = dotHextoDotDec(temp);
//  // 16*(1/32768)/0.25*1000000=1953.125
//  G_calibrateCorrectionFactor = 1953.125/G_calibrateResult;
//}





//u32 readRegisterTDCGP22(unsigned char read_opcode_address)
//{
//  unsigned long result_read = 0;
//  
//  TDC_CS_0;
//  SPI1_ReadWriteByte(read_opcode_address);
//  //读第一个字节
//  result_read |= SPI1_ReadWriteByte(DUMMY_DATA);
//  result_read <<= 8;
//  //读第二个字节
//  result_read |= SPI1_ReadWriteByte(DUMMY_DATA);
//  result_read <<= 8;
//  //读第三个字节
//  result_read |= SPI1_ReadWriteByte(DUMMY_DATA);
//  result_read <<= 8;
//  //读第四个字节
//  result_read |= SPI1_ReadWriteByte(DUMMY_DATA);
//  TDC_CS_1;
//  delay_us(3);
//  return result_read;
//}










		
		
		
		
		
	
