/*********************************************************************
 * INCLUDES
 */

#include "hal_types.h"
#include "hal_assert.h"
#include "hal_board.h"
#include "hal_defs.h"
#include "hal_mcu.h"
#include "hal_uart.h"
#include "npi.h"

#include "_hal_uart_spi.c"

static void simpleBLE_NpiSpiCallback( uint8 port, uint8 events )
{
    (void)port; 
    if (events & HAL_UART_RX_TIMEOUT)   //串口有数据      
    {
        (void)port;
        uint8 RxCount;
       // if(OplOn==0)
        {
          RxCount = NPI_RxBufLen();           //读出串口缓冲区有多少字节
          if(RxCount>0)
          {
             //NPI_ReadTransport(RxBuffer,RxCount);    //释放串口数据 
          }
          
        }        
    } 
    //HAL_UART_TX_EMPTY
}

void NPI_InitTransportSPI( npiCBack_t npiCBack )
{
  halUARTCfg_t uartConfig;

  // configure UART
  uartConfig.callBackFunc         = (halUARTCBack_t)npiCBack;

  HalUARTOpenSPI(&uartConfig);

    U0BAUD = 0x00;   // BAUD_M 0  SPI CLK = 8MHZ 

  return;
}

//HalUARTPollSPI();
//static spiLen_t HalUARTWriteSPI(uint8 *buf, spiLen_t len)
//static spiLen_t HalUARTReadSPI(uint8 *buf, spiLen_t len)

//static spiLen_t HalUARTRxAvailSPI(void)//Number of bytes ready to be read with HalUARTReadSPI().

void SPI_FLASH_Init()
{
    HalUARTInitSPI();
    NPI_InitTransportSPI(simpleBLE_NpiSpiCallback);
    
    /*
	PERCFG |= 0x02;//USART 1 I/O location 2
	P1SEL &= ~0x10; //P1.4 as General-purpose I/O pin
	P1SEL |= 0xE0;  // P1.5 and P1.6 and P1.7 as peripheral function pin
	P1DIR |= 0x50; // P1.4 and p1.6 as output
	P1DIR &= ~0x80;// p1.7 as Input
	P1 |= 0x10;   //P0.4初始化为高电平   低电平使能
	
	P2SEL &= ~0x02;//P2.1 as gpio pin
	P2DIR |= 0x02;//Port 2 pins P2.2 as output
  	P2 &= ~0x02;   //P2.1初始化为低电平写保护，高电平无放。
	
	//*** Setup the SPI interface ***     // SPI master mode    
	U1CSR = 0x00;    // SPI mode    
	U1GCR |= 0x6F;   // BAUD_E 18  Negative clock polarity  MSB first   
	//U1GCR |= 0xF1;			 // CPOL为1 即：空闲状态保持高电平 CPHA为0 即：数据在SPCK起始边沿改变,在SPCK 下一个边沿捕获。  
	U1BAUD = 0x00;   // BAUD_M 0  SPI CLK = 8MHZ 
    */
}

#if 0

void SPI_FLASH_SendByte(uint8 byte)
{
  	WP = 1;
  	U1CSR &= ~0x02;                 // Clear TX_BYTE 
  	U1DBUF = byte;
	while (!(U1CSR & 0x02));        // Wait for TX_BYTE to be set
	/* Loop while DR register in not emplty */
}

uint8 SPI_FLASH_RecvByte()
{
	SPI_FLASH_SendByte(Dummy_Byte);
	while(U1CSR & 0x04);
	return SPI_FLASH_RX();
}

uint8 SPI_FLASH_RecvByte1()
{
	SPI_FLASH_SendByte(0xEF);
	while(U1CSR & 0x04);
	return SPI_FLASH_RX();
}

void SPI_FLASH_WaitForWriteEnd()
{
	uint8  FLASH_Status = 0;
  	/* Send "Read Status Register" instruction */
  	CS_F=CS_ENABLED;
  
  	SPI_FLASH_SendByte(RDSR1);
  	/* Loop as long as the memory is busy with a write cycle */
  	do
  	{
    	FLASH_Status = SPI_FLASH_RecvByte();
  	}
  	while (FLASH_Status & WIP_Flag); /* Write in progress */
	
  	CS_F = CS_DISABLED; 
}

void SPI_FLASH_WriteEnable()
{
	CS_F=CS_ENABLED;
  	SPI_FLASH_SendByte(WREN);
	CS_F = CS_DISABLED; 
}

void SPI_FLASH_SectorErase(uint32 SectorAddr)
{
  	/* Wait the end of Flash writing */
  	SPI_FLASH_WaitForWriteEnd();
  	/* Send write enable instruction */
  	SPI_FLASH_WriteEnable();
	
	CS_F=CS_ENABLED;
  	/* Send Sector Erase instruction */
  	SPI_FLASH_SendByte(SE);
  	/* Send SectorAddr high nibble address byte */
  	SPI_FLASH_SendByte((SectorAddr & 0xFF0000) >> 16);
  	/* Send SectorAddr medium nibble address byte */
  	SPI_FLASH_SendByte((SectorAddr & 0xFF00) >> 8);
  	/* Send SectorAddr low nibble address byte */
  	SPI_FLASH_SendByte(SectorAddr & 0xFF);
  	/* Deselect the FLASH: Chip Select high */
  	CS_F = CS_DISABLED; 
  	/* Wait the end of Flash writing */
  	//SPI_FLASH_WaitForWriteEnd();
}

void SPI_FLASH_BlockErase(uint32 BlockAddr)
{
	  /* Wait the end of Flash writing */
	  SPI_FLASH_WaitForWriteEnd();
	  
	  /* Send write enable instruction */
	  SPI_FLASH_WriteEnable();
	
	  /* Chip Select low */
	  CS_F=CS_ENABLED;
	  /* Send Sector Erase instruction */
	  SPI_FLASH_SendByte(BE);
	  /* Send SectorAddr high nibble address byte */
	  SPI_FLASH_SendByte((BlockAddr & 0xFF0000) >> 16);
	  /* Send SectorAddr medium nibble address byte */
	  SPI_FLASH_SendByte((BlockAddr & 0xFF00) >> 8);
	  /* Send SectorAddr low nibble address byte */
	  SPI_FLASH_SendByte(BlockAddr & 0xFF);
	  /* Chip Select high */
	  CS_F = CS_DISABLED; 
	  
	  /* Wait the end of Flash writing */
	  //SPI_FLASH_WaitForWriteEnd();
}

void SPI_FLASH_ChipErase()
{
	  /* Wait the end of Flash writing */
	  SPI_FLASH_WaitForWriteEnd();
	  
	  /* Send write enable instruction */
	  SPI_FLASH_WriteEnable();
	
	  /*Chip Select low */
	  CS_F=CS_ENABLED;
	  /* Send Bulk Erase instruction  */
	  SPI_FLASH_SendByte(CE);
	  /*Chip Select high */
	   CS_F = CS_DISABLED; 
	
	  /* Wait the end of Flash writing */
	  //SPI_FLASH_WaitForWriteEnd();
}

void SPI_FLASH_PageWrite(uint8* pBuffer, uint32 WriteAddr, uint16 NumByteToWrite)
{
  /* Wait the end of Flash writing */
  SPI_FLASH_WaitForWriteEnd();
  
  /* Enable the write access to the FLASH */
  SPI_FLASH_WriteEnable();

  /* Chip Select low */
   CS_F=CS_ENABLED;
  /* Send "Write to Memory " instruction */
  SPI_FLASH_SendByte(WRITE);
  /* Send WriteAddr high nibble address byte to write to */
  SPI_FLASH_SendByte((WriteAddr & 0xFF0000) >> 16);
  /* Send WriteAddr medium nibble address byte to write to */
  SPI_FLASH_SendByte((WriteAddr & 0xFF00) >> 8);
  /* Send WriteAddr low nibble address byte to write to */
  SPI_FLASH_SendByte(WriteAddr & 0xFF);

  /* while there is data to be written on the FLASH */
  while (NumByteToWrite--)
  {
    /* Send the current byte */
    SPI_FLASH_SendByte(*pBuffer);
    /* Point on the next byte to be written */
    pBuffer++;
  }
  /* Chip Select high */
  CS_F = CS_DISABLED; 

  /* Wait the end of Flash writing */
  //SPI_FLASH_WaitForWriteEnd();
}

void SPI_FLASH_BufferWrite(uint8* pBuffer, uint32 WriteAddr, uint16 NumByteToWrite)
{
  uint8 NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

  Addr = WriteAddr % SPI_FLASH_PageSize;
  count = SPI_FLASH_PageSize - Addr;
  NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
  NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;

  if (Addr == 0) /* WriteAddr is SPI_FLASH_PageSize aligned  */
  {
    if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */
    {
      SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */
    {
      while (NumOfPage--)
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize);
        WriteAddr +=  SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }

      SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle);
    }
  }
  else /* WriteAddr is not SPI_FLASH_PageSize aligned  */
  {
    if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */
    {
      if (NumOfSingle > count) /* (NumByteToWrite + WriteAddr) > SPI_FLASH_PageSize */
      {
        temp = NumOfSingle - count;

        SPI_FLASH_PageWrite(pBuffer, WriteAddr, count);
        WriteAddr +=  count;
        pBuffer += count;

        SPI_FLASH_PageWrite(pBuffer, WriteAddr, temp);
      }
      else
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
      }
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */
    {
      NumByteToWrite -= count;
      NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
      NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;
      
      /*  First Write count byte, then the addr is Page aligned  */
      SPI_FLASH_PageWrite(pBuffer, WriteAddr, count);
      WriteAddr +=  count;
      pBuffer += count;

      while (NumOfPage--)
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize);
        WriteAddr +=  SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }

      if (NumOfSingle != 0)
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle);
      }
    }
  }
}

void SPI_FLASH_BufferRead(uint8* pBuffer, uint32 ReadAddr, uint16 NumByteToRead)
{
  /* Wait the end of Flash writing */
  SPI_FLASH_WaitForWriteEnd();
  
  /*Chip Select low */
   CS_F=CS_ENABLED;

  /* Send "Read from Memory " instruction */
  SPI_FLASH_SendByte(READ);

  /* Send ReadAddr high nibble address byte to read from */
  SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
  /* Send ReadAddr medium nibble address byte to read from */
  SPI_FLASH_SendByte((ReadAddr& 0xFF00) >> 8);
  /* Send ReadAddr low nibble address byte to read from */
  SPI_FLASH_SendByte(ReadAddr & 0xFF);

  while (NumByteToRead--) /* while there is data to be read */
  {
    /* Read a byte from the FLASH */
    *pBuffer = SPI_FLASH_RecvByte();
    /* Point to the next location where the byte read will be saved */
    pBuffer++;
  }

  /* Chip Select high */
  CS_F = CS_DISABLED; 
}

uint32 SPI_FLASH_ReadID()
{
  uint8 Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;
  uint16 spi_ID1 = 0,spi_ID2 = 0,spi_ID0 = 0,spi_ID = 0;
  /*Chip Select low */
  SPI_FLASH_WaitForWriteEnd();
  CS_F=CS_ENABLED;

  /* Send "RDID " instruction */
  SPI_FLASH_SendByte(0x90);

  /* Read a byte from the FLASH */
  SPI_FLASH_SendByte((0x000000 & 0xFF0000) >> 16);
 // Temp0 = SPI_FLASH_RecvByte();

  /* Read a byte from the FLASH */
  SPI_FLASH_SendByte((0x000000 & 0xFF00) >> 8);
 // Temp1 = SPI_FLASH_RecvByte();

  /* Read a byte from the FLASH */
  SPI_FLASH_SendByte(0x00);
 // SPI_FLASH_SendByte(Dummy_Byte);
 spi_ID1 = SPI_FLASH_RecvByte1();
 spi_ID2 = SPI_FLASH_RecvByte();

  /* Chip Select high */
  CS_F=CS_ENABLED;  
	//spi_ID = (spi_ID1<<8)|(spi_ID2);
 // Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;

 // return spi_ID;
}

uint8 buf[10]="123456789";
uint8 bufrx[10];
uint8 buf1[10]="987654321";

void test_SPI_FLASH() 
{
	uint8 testbuf[10] = {0};
	uint32 Temp = 0;
	SPI_FLASH_Init();
	//while(1)
	//{
	SPI_FLASH_BlockErase(0x0);
	SPI_FLASH_BufferWrite(buf,0x0,10);
	SPI_FLASH_BufferRead(bufrx,0x0,10);
	//strcpy(testbuf,bufrx);
	Temp = SPI_FLASH_ReadID();
	//SPI_FLASH_BlockErase(0x0);
	//SPI_FLASH_BufferWrite(buf1,0x0,10);
	//SPI_FLASH_BufferRead(bufrx,0x0,10);
	//}
}

#endif
