/**************************************************************************************************
  Filename:       _hal_uart_dma.c
  Revised:        $Date: 2013-02-19 11:10:32 -0800 (Tue, 19 Feb 2013) $
  Revision:       $Revision: 33202 $

  Description: This file contains the interface to the H/W UART driver by DMA.

  A known defect is that when flow control is enabled, the function HalUARTPollTxTrigDMA() can
  prematurely invoke HAL_DMA_MAN_TRIGGER(HAL_DMA_CH_TX) and clobber that last byte of one txBuf[]
  block transfer with the first byte of the next txBuf[] block transfer.  Additionally, Tx can
  become permanently stalled during heavy use and/or simultaeous heavy radio traffic when using
  DMA for the Tx and hardware flow control.

  Copyright 2006-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include <string.h>

#include "hal_assert.h"
#include "hal_board.h"
#include "hal_defs.h"
#include "hal_dma.h"
#include "hal_mcu.h"
#include "hal_types.h"
#include "hal_uart.h"
extern uint8 NubRN8209DCommand;

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

// UxCSR - USART Control and Status Register.
#define CSR_MODE                   0x80
#define CSR_RE                     0x40
#define CSR_SLAVE                  0x20
#define CSR_FE                     0x10
#define CSR_ERR                    0x08
#define CSR_RX_BYTE                0x04
#define CSR_TX_BYTE                0x02
#define CSR_ACTIVE                 0x01

// UxUCR - USART UART Control Register.
#define UCR_FLUSH                  0x80
#define UCR_FLOW                   0x40
#define UCR_D9                     0x20
#define UCR_BIT9                   0x10
#define UCR_PARITY                 0x08
#define UCR_SPB                    0x04
#define UCR_STOP                   0x02
#define UCR_START                  0x01

#define UTX0IE                     0x04
#define UTX1IE                     0x08

#define P2DIR_PRIPO                0xC0

#define HAL_DMA_U0DBUF             0x70C1
#define HAL_DMA_U1DBUF             0x70F9

#undef  UxCSR
#undef  UxUCR
#undef  UxDBUF
#undef  UxBAUD
#undef  UxGCR
#undef UTXxIE
#undef UTXxIF
#if    (HAL_UART_DMA == 1)
#define UxCSR                      U0CSR
#define UxUCR                      U0UCR
#define UxDBUF                     U0DBUF
#define UxBAUD                     U0BAUD
#define UxGCR                      U0GCR
#define UTXxIE                     UTX0IE
#define UTXxIF                     UTX0IF
#elif  (HAL_UART_DMA == 2)
#define UxCSR                      U1CSR
#define UxUCR                      U1UCR
#define UxDBUF                     U1DBUF
#define UxBAUD                     U1BAUD
#define UxGCR                      U1GCR
#define UTXxIE                     UTX1IE
#define UTXxIF                     UTX1IF
#endif

#undef  PxSEL
#undef  HAL_UART_PERCFG_BIT
#undef  HAL_UART_PRIPO
#undef  HAL_UART_Px_CTS
#undef  HAL_UART_Px_RTS
#undef  HAL_UART_Px_SEL
#if    (HAL_UART_DMA == 1)
#define PxSEL                      P0SEL
#define HAL_UART_PERCFG_BIT        0x01         // USART0 on P0, Alt-1; so clear this bit.
#define HAL_UART_PRIPO             0x00         // USART0 priority over UART1.
#define HAL_UART_Px_CTS            0x10         // Peripheral I/O Select for CTS flow control.
#define HAL_UART_Px_RTS            0x20         // Peripheral I/O Select for RTS must be manual.
#define HAL_UART_Px_SEL            0x20         // Peripheral I/O Select for Rx/Tx.
#elif  (HAL_UART_DMA == 2)
#define PxSEL                      P1SEL
#define HAL_UART_PERCFG_BIT        0x02         // USART1 on P1, Alt-2; so set this bit.
#define HAL_UART_PRIPO             0x40         // USART1 priority over UART0.
#define HAL_UART_Px_CTS            0x10         // Peripheral I/O Select for CTS flow control.
#define HAL_UART_Px_RTS            0x20         // Peripheral I/O Select for RTS must be manual.
#define HAL_UART_Px_SEL            0xC0         // Peripheral I/O Select for Rx/Tx.
#endif

#undef  DMA_PAD
#undef  DMA_UxDBUF
#undef  DMATRIG_RX
#undef  DMATRIG_TX
#if    (HAL_UART_DMA == 1)
#define DMA_PAD                    U0BAUD
#define DMA_UxDBUF                 HAL_DMA_U0DBUF
#define DMATRIG_RX                 HAL_DMA_TRIG_URX0
#define DMATRIG_TX                 HAL_DMA_TRIG_UTX0
#elif  (HAL_UART_DMA == 2)
#define DMA_PAD                    U1BAUD
#define DMA_UxDBUF                 HAL_DMA_U1DBUF
#define DMATRIG_RX                 HAL_DMA_TRIG_URX1
#define DMATRIG_TX                 HAL_DMA_TRIG_UTX1
#endif

#undef  PxDIR
#undef  PxIEN
#undef  PxIFG
#undef  PxIF
#undef  PICTL_BIT
#undef  IENx
#undef  IEN_BIT
#if   (HAL_UART_DMA == 1)
#define PxDIR                      P0DIR
#define PxIEN                      P0IEN
#define PxIFG                      P0IFG
#define PxIF                       P0IF
#define DMA_RDYIn                  P0_4
#define DMA_RDYOut                 P0_5
#define DMA_RDYIn_BIT              BV(4)        // Same as the I/O Select for CTS flow control.
#define DMA_RDYOut_BIT             BV(5)        // Same as the I/O Select for manual RTS flow ctrl.
// Falling edge ISR on P0 pins.
#define PICTL_BIT                  BV(0)
#define IENx                       IEN1
#define IEN_BIT                    BV(5)
#elif  (HAL_UART_DMA == 2)
#define PxDIR                      P1DIR
#define PxIEN                      P1IEN
#define PxIFG                      P1IFG
#define PxIF                       P1IF
#define DMA_RDYIn                  P1_4
#define DMA_RDYOut                 P1_5
#define DMA_RDYIn_BIT              BV(4)        // Same as the I/O Select for CTS flow control.
#define DMA_RDYOut_BIT             BV(5)        // Same as the I/O Select for manual RTS flow ctrl.
// Falling edge ISR on P1.4-7 pins.
#define PICTL_BIT                  BV(2)
#define IENx                       IEN2
#define IEN_BIT                    BV(4)
#endif

#if !defined( DMA_PM )
#if defined POWER_SAVING
#define DMA_PM                     1
#else
#define DMA_PM                     0
#endif // POWER_SAVING
#endif // !DMA_PM

// For known defects described above in the moduel description, prefer to drive the Tx by ISR vice
// DMA unless H/W flow control is not used and full-throughput on Tx is absolutely essential.
#if !defined HAL_UART_TX_BY_ISR
#define HAL_UART_TX_BY_ISR         0
#endif

// TRUE or FALSE whether or not to flush the Rx queue when an Rx overrun is detected.
// Note that when HAL_UART_RX_FLUSH is set to TRUE, then uartRxBug and its hunt for where the Rx
// DMA is working is absolutely necessary because the flush can leave the dmaCfg.rxHead stranded.
#if !defined HAL_UART_RX_FLUSH
#define HAL_UART_RX_FLUSH          TRUE
#endif

// Minimum delay before allowing sleep and/or clearing DMA ready-out after a DMA ready-in ISR.
// ST-ticks for 6-msecs plus 1 tick added for when the dmaRdyDly is forced from zero to 0xFF.
// If a greater delay than 6-msec is configured, then the logic should be changed to use a uint16.
//efine DMA_PM_DLY                 198   // 32768 * 0.006 + 1 -> 198.
// This delay should be set as short as possible to work with the max expected latency in the sender
// between its asserting ready-out and its checking of the ready-in response. The RBA Master
// logic in the internal uart-to-uart bridge app checks for ready-in immediately,
// so this is just set to zero.
#define DMA_PM_DLY                 198//0

// The timeout tick is at 32-kHz, so multiply msecs by 33.
#define HAL_UART_MSECS_TO_TICKS    33

#if !defined HAL_UART_DMA_RX_MAX
#define HAL_UART_DMA_RX_MAX        128
#endif
#if !defined HAL_UART_DMA_TX_MAX
#define HAL_UART_DMA_TX_MAX        HAL_UART_DMA_RX_MAX
#endif
#if !defined HAL_UART_DMA_HIGH
#define HAL_UART_DMA_HIGH         (HAL_UART_DMA_RX_MAX - 1)
#endif
#if !defined HAL_UART_DMA_IDLE
#define HAL_UART_DMA_IDLE         (0 * HAL_UART_MSECS_TO_TICKS)
 
#endif
#if !defined HAL_UART_DMA_FULL
#define HAL_UART_DMA_FULL         (HAL_UART_DMA_RX_MAX - 16)
#endif

// ST-ticks for 1 byte @ 38.4-kB plus 1 tick added for when the txTick is forced from zero to 0xFF.
#define HAL_UART_TX_TICK_MIN       11

/* ------------------------------------------------------------------------------------------------
 *                                           TypeDefs
 * ------------------------------------------------------------------------------------------------
 */

#if HAL_UART_DMA_RX_MAX <= 256
typedef uint8 rxIdx_t;
#else
typedef uint16 rxIdx_t;
#endif

#if HAL_UART_DMA_TX_MAX <= 256
typedef uint8 txIdx_t;
#else
typedef uint16 txIdx_t;
#endif

typedef struct
{
  uint16 rxBuf[HAL_UART_DMA_RX_MAX];
  rxIdx_t rxHead;
  rxIdx_t rxTail;
#if HAL_UART_DMA_IDLE
  uint8 rxTick;
#endif

#if HAL_UART_TX_BY_ISR
  uint8 txBuf[HAL_UART_DMA_TX_MAX];
  volatile txIdx_t txHead;
  txIdx_t txTail;
  uint8 txMT;
#else
  uint8 txBuf[2][HAL_UART_DMA_TX_MAX];
  txIdx_t txIdx[2];
  uint8 txMT;    // Indication that at least one of two Tx buffers are free.
  uint8 txTick;  // ST ticks of delay to allow at least one byte-time at a given baud rate.
  uint8 txTrig;  // Flag indicating that Tx should be manually triggered after txTick expires.

  // Although all of the txVars above are modified by the Tx-done ISR, only this one should need
  // the special volatile consideration by the optimizer (critical sections protect the rest).
  volatile uint8 txSel;
#endif

  halUARTCBack_t uartCB;
} uartDMACfg_t;

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

#define HAL_UART_DMA_NEW_RX_BYTE(IDX)  ((uint8)DMA_PAD == HI_UINT16(dmaCfg.rxBuf[(IDX)]))
#define HAL_UART_DMA_GET_RX_BYTE(IDX)  (*(volatile uint8 *)(dmaCfg.rxBuf+(IDX)))
#define HAL_UART_DMA_CLR_RX_BYTE(IDX)  (dmaCfg.rxBuf[(IDX)] = BUILD_UINT16(0, (DMA_PAD ^ 0xFF)))

#define HAL_UART_DMA_CLR_RDY_OUT()     (DMA_RDYOut = 1)
#define HAL_UART_DMA_SET_RDY_OUT()     (DMA_RDYOut = 0)

#define HAL_UART_DMA_RDY_IN()          (DMA_RDYIn == 0)
#define HAL_UART_DMA_RDY_OUT()         (DMA_RDYOut == 0)

#if HAL_UART_DMA_RX_MAX == 256
#define HAL_UART_RX_IDX_T_DECR(IDX)    (IDX)--
#else
#define HAL_UART_RX_IDX_T_DECR(IDX) st (  \
  if ((IDX)-- == 0) \
  { \
    (IDX) = HAL_UART_DMA_RX_MAX-1; \
  } \
)
#endif

#if HAL_UART_DMA_RX_MAX == 256
#define HAL_UART_RX_IDX_T_INCR(IDX)    (IDX)++
#else
#define HAL_UART_RX_IDX_T_INCR(IDX) st (  \
  if (++(IDX) >= HAL_UART_DMA_RX_MAX) \
  { \
    (IDX) = 0; \
  } \
)
#endif

#define HAL_UART_DMA_TX_AVAIL() \
  (dmaCfg.txHead > dmaCfg.txTail) ? \
  (dmaCfg.txHead - dmaCfg.txTail - 1) : \
  (HAL_UART_DMA_TX_MAX - dmaCfg.txTail + dmaCfg.txHead - 1)

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

// The following two variables are only used when POWER_SAVING is defined.
static volatile uint8 dmaRdyIsr;
static uint8 dmaRdyDly;  // Minimum delay before allowing sleep after detecting RdyIn de-asserted.

static uartDMACfg_t dmaCfg;

/* Used to walk the dmaCfg.rxBuf[] one byte per polling pass as a work-around for the case when
 * full-duplex traffic using Rx & Tx DMA running simultaneously.
 * Although not captured in this UART by DMA case, the _hal_uart_spi.c was able to show that the
 * immediate buffer area around the spiRxIdx consists of "cleared" uint16 values,
 * but 10-16 indices ahead, there are valid SPI packets ready to be parsed.
 */
static rxIdx_t uartRxBug;  // Pre-emptive (not empirically determined necessary) use from SPI case.

/* ------------------------------------------------------------------------------------------------
 *                                          Global Functions
 * ------------------------------------------------------------------------------------------------
 */

void HalUART_DMAIsrDMA(void);

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

// Invoked by functions in hal_uart.c when this file is included.
static void HalUARTInitDMA(void);
static void HalUARTOpenDMA(halUARTCfg_t *config);
static uint16 HalUARTReadDMA(uint8 *buf, uint16 len);
static uint16 HalUARTWriteDMA(uint8 *buf, uint16 len);
static void HalUARTPollDMA(void);
static uint16 HalUARTRxAvailDMA(void);
static uint8 HalUARTBusyDMA(void);
#if !HAL_UART_TX_BY_ISR
static void HalUARTPollTxTrigDMA(void);
static void HalUARTArmTxDMA(void);
#endif

/******************************************************************************
 * @fn      HalUARTInitDMA
 *
 * @brief   Initialize the UART
 *
 * @param   none
 *
 * @return  none
 *****************************************************************************/
static void HalUARTInitDMA(void)
{
  halDMADesc_t *ch;  
  //uart0
//
  PERCFG |= 0x01;   // Set UART0 I/O to Alt. 2 location on P1.
  P1SEL  |=  0x20;//Enable Peripheral control of Tx on p1.5
  U0CSR = CSR_MODE;                  // Mode is UART Mode.
  U0GCR |= 7;
  U0BAUD = 59;//波特率设为4800 
  U0UCR = UCR_STOP;//停止位高电平
  U0UCR |= (UCR_PARITY | UCR_D9 | UCR_BIT9); //偶校验，9位传送
  //注：默认起始位低电平，1位停止位，流控制禁止
  
  //uart1
#if (HAL_UART_DMA == 1)
  PERCFG &= ~HAL_UART_PERCFG_BIT;    // Set UART0 I/O to Alt. 1 location on P0.
#else
  PERCFG |= HAL_UART_PERCFG_BIT;     // Set UART1 I/O to Alt. 2 location on P1.
#endif
  PxSEL  |= HAL_UART_Px_SEL;         // Enable Peripheral control of Rx/Tx on p1.7.p1.6.
  UxCSR = CSR_MODE;                  // Mode is UART Mode.
  UxUCR = UCR_FLUSH;                 // Flush it.

  P2DIR &= ~P2DIR_PRIPO;
  //P2DIR |= HAL_UART_PRIPO;

  if (DMA_PM)
  {
    // Setup GPIO for interrupts by falling edge on DMA_RDY_IN.
    PxIEN |= DMA_RDYIn_BIT;
    PICTL |= PICTL_BIT;

    HAL_UART_DMA_CLR_RDY_OUT();
    PxDIR |= DMA_RDYOut_BIT;
  }

#if !HAL_UART_TX_BY_ISR
  // Setup Tx by DMA.
  ch = HAL_DMA_GET_DESC1234( HAL_DMA_CH_TX );

  // Abort any pending DMA operations (in case of a soft reset).
  HAL_DMA_ABORT_CH( HAL_DMA_CH_TX );

  // The start address of the destination.
  HAL_DMA_SET_DEST( ch, DMA_UxDBUF );

  // Using the length field to determine how many bytes to transfer.
  HAL_DMA_SET_VLEN( ch, HAL_DMA_VLEN_USE_LEN );

  // One byte is transferred each time.
  HAL_DMA_SET_WORD_SIZE( ch, HAL_DMA_WORDSIZE_BYTE );

  // The bytes are transferred 1-by-1 on Tx Complete trigger.
  HAL_DMA_SET_TRIG_MODE( ch, HAL_DMA_TMODE_SINGLE );
  HAL_DMA_SET_TRIG_SRC( ch, DMATRIG_TX );

  // The source address is incremented by 1 byte after each transfer.
  HAL_DMA_SET_SRC_INC( ch, HAL_DMA_SRCINC_1 );

  // The destination address is constant - the Tx Data Buffer.
  HAL_DMA_SET_DST_INC( ch, HAL_DMA_DSTINC_0 );

  // The DMA Tx done is serviced by ISR in order to maintain full thruput.
  HAL_DMA_SET_IRQ( ch, HAL_DMA_IRQMASK_ENABLE );

  // Xfer all 8 bits of a byte xfer.
  HAL_DMA_SET_M8( ch, HAL_DMA_M8_USE_8_BITS );

  // DMA has highest priority for memory access.
  HAL_DMA_SET_PRIORITY( ch, HAL_DMA_PRI_HIGH);
#endif

  // Setup Rx by DMA.
  ch = HAL_DMA_GET_DESC1234( HAL_DMA_CH_RX );

  // Abort any pending DMA operations (in case of a soft reset).
  HAL_DMA_ABORT_CH( HAL_DMA_CH_RX );

  // The start address of the source.
  HAL_DMA_SET_SOURCE( ch, DMA_UxDBUF );

  // Using the length field to determine how many bytes to transfer.
  HAL_DMA_SET_VLEN( ch, HAL_DMA_VLEN_USE_LEN );

  /* The trick is to cfg DMA to xfer 2 bytes for every 1 byte of Rx.
   * The byte after the Rx Data Buffer is the Baud Cfg Register,
   * which always has a known value. So init Rx buffer to inverse of that
   * known value. DMA word xfer will flip the bytes, so every valid Rx byte
   * in the Rx buffer will be preceded by a DMA_PAD char equal to the
   * Baud Cfg Register value.
   */

  HAL_DMA_SET_WORD_SIZE( ch, HAL_DMA_WORDSIZE_WORD );

  // The bytes are transferred 1-by-1 on Rx Complete trigger.
  HAL_DMA_SET_TRIG_MODE( ch, HAL_DMA_TMODE_SINGLE_REPEATED );
  HAL_DMA_SET_TRIG_SRC( ch, DMATRIG_RX );

  // The source address is constant - the Rx Data Buffer.
  HAL_DMA_SET_SRC_INC( ch, HAL_DMA_SRCINC_0 );

  // The destination address is incremented by 1 word after each transfer.
  HAL_DMA_SET_DST_INC( ch, HAL_DMA_DSTINC_1 );
  HAL_DMA_SET_DEST( ch, dmaCfg.rxBuf );
  HAL_DMA_SET_LEN( ch, HAL_UART_DMA_RX_MAX );

  // The DMA is to be polled and shall not issue an IRQ upon completion.
  HAL_DMA_SET_IRQ( ch, HAL_DMA_IRQMASK_DISABLE );

  // Xfer all 8 bits of a byte xfer.
  HAL_DMA_SET_M8( ch, HAL_DMA_M8_USE_8_BITS );

  // DMA has highest priority for memory access.
  HAL_DMA_SET_PRIORITY( ch, HAL_DMA_PRI_HIGH);

  volatile uint8 dummy = *(volatile uint8 *)DMA_UxDBUF;  // Clear the DMA Rx trigger.
  HAL_DMA_CLEAR_IRQ(HAL_DMA_CH_RX);
  HAL_DMA_ARM_CH(HAL_DMA_CH_RX);
  (void)memset(dmaCfg.rxBuf, (DMA_PAD ^ 0xFF), HAL_UART_DMA_RX_MAX*2);
  
}

/******************************************************************************
 * @fn      HalUARTOpenDMA
 *
 * @brief   Open a port according tp the configuration specified by parameter.
 *
 * @param   config - contains configuration information
 *
 * @return  none
 *****************************************************************************/
static void HalUARTOpenDMA(halUARTCfg_t *config)
{
  dmaCfg.uartCB = config->callBackFunc;

  // Only supporting subset of baudrate for code size - other is possible.
  HAL_ASSERT((config->baudRate == HAL_UART_BR_9600) ||
                  (config->baudRate == HAL_UART_BR_19200) ||
                  (config->baudRate == HAL_UART_BR_38400) ||
                  (config->baudRate == HAL_UART_BR_57600) ||
                  (config->baudRate == HAL_UART_BR_1200) ||
                  (config->baudRate == HAL_UART_BR_2400) ||
                  (config->baudRate == HAL_UART_BR_4800) ||
                  (config->baudRate == HAL_UART_BR_115200));

  if (config->baudRate == HAL_UART_BR_57600 ||
      config->baudRate == HAL_UART_BR_115200)
  {
    UxBAUD = 216;
  }
  else
  {
    UxBAUD = 59;
  }

  switch (config->baudRate)
  {
    case HAL_UART_BR_9600:
      UxGCR = 8;
      break;
    case HAL_UART_BR_19200:
      UxGCR = 9;
      break;
    case HAL_UART_BR_38400:
    case HAL_UART_BR_57600:
      UxGCR = 10;
      break;
    case HAL_UART_BR_1200:
      UxGCR = 5;
      break;
    case HAL_UART_BR_2400:
      UxGCR = 6;
      break;
    case HAL_UART_BR_4800:
      UxGCR = 7;
      break;
    default:
      // HAL_UART_BR_115200
      UxGCR = 11;
      break;
  }

  if (DMA_PM || config->flowControl)
  {
    UxUCR = UCR_FLOW | UCR_STOP;      // 8 bits/char; no parity; 1 stop bit; stop bit hi.
    PxSEL |= HAL_UART_Px_CTS;         // Enable Peripheral control of CTS flow control on Px.
  }
  else
  {
    UxUCR = UCR_STOP;                 // 8 bits/char; no parity; 1 stop bit; stop bit hi.
  }
  
#if 1// amomcu.com 修改, 增加 奇偶校验  与 停止位
  /*
    Para 范围 0,1,2 
    0: 无校验
    1:  ODD
    2:  EVEN
    Default: 0   
  */
    if(config->parity == 1)
    {      
        UxUCR |= ( UCR_PARITY | UCR_BIT9); 
    }
    else if(config->parity == 2)
    {
        UxUCR |= (UCR_PARITY | UCR_D9 | UCR_BIT9);
    }


  /*
    Para: 0~1 
    0: 1 停止位
    1: 2 停止位
    Default: 0 
  */
    if(config->stopbit == 1)
    {      
        UxUCR |= UCR_SPB; 
    }
#endif

  UxCSR = (CSR_MODE | CSR_RE);

  if (DMA_PM)
  {
    PxIFG = 0;
    PxIF = 0;
    IENx |= IEN_BIT;
  }
  else if (UxUCR & UCR_FLOW)
  {
    // DMA Rx is always on (self-resetting). So flow must be controlled by the S/W polling the
    // circular Rx queue depth. Start by allowing flow.
    HAL_UART_DMA_SET_RDY_OUT();
    PxDIR |= HAL_UART_Px_RTS;
  }

#if HAL_UART_TX_BY_ISR
  UTXxIF = 1;  // Prime the ISR pump.
#endif
}

/*****************************************************************************
 * @fn      HalUARTReadDMA
 *
 * @brief   Read a buffer from the UART
 *
 * @param   buf  - valid data buffer at least 'len' bytes in size
 *          len  - max length number of bytes to copy to 'buf'
 *
 * @return  length of buffer that was read
 *****************************************************************************/
static uint16 HalUARTReadDMA(uint8 *buf, uint16 len)
{
  uint16 cnt;

  for (cnt = 0; cnt < len; cnt++)
  {
    if (!HAL_UART_DMA_NEW_RX_BYTE(dmaCfg.rxHead))
    {
      break;
    }
    *buf++ = HAL_UART_DMA_GET_RX_BYTE(dmaCfg.rxHead);
    HAL_UART_DMA_CLR_RX_BYTE(dmaCfg.rxHead);
    HAL_UART_RX_IDX_T_INCR(dmaCfg.rxHead);
  }

  if (!DMA_PM && (UxUCR & UCR_FLOW))
  {
    if (HalUARTRxAvailDMA() < HAL_UART_DMA_HIGH)
    {
      HAL_UART_DMA_SET_RDY_OUT();  // Re-enable the flow asap (i.e. not wait until next uart poll).
    }
  }

  return cnt;
}

/******************************************************************************
 * @fn      HalUARTWriteDMA
 *
 * @brief   Write a buffer to the UART, enforcing an all or none policy if the requested length
 *          exceeds the space available.
 *
 * @param   buf - pointer to the buffer that will be written, not freed
 *          len - length of
 *
 * @return  length of the buffer that was sent
 *****************************************************************************/
static uint16 HalUARTWriteDMA(uint8 *buf, uint16 len)
{
#if HAL_UART_TX_BY_ISR
  // Enforce all or none.
  if (HAL_UART_DMA_TX_AVAIL() < len)
  {
    return 0;
  }

  for (uint16 cnt = 0; cnt < len; cnt++)
  {
    dmaCfg.txBuf[dmaCfg.txTail] = *buf++;
    dmaCfg.txMT = 0;

    if (dmaCfg.txTail >= HAL_UART_DMA_TX_MAX-1)
    {
      dmaCfg.txTail = 0;
    }
    else
    {
      dmaCfg.txTail++;
    }

    // Keep re-enabling ISR as it might be keeping up with this loop due to other ints.
    IEN2 |= UTXxIE;
  }
#else
  txIdx_t txIdx;
  uint8 txSel;
  halIntState_t his;

  HAL_ENTER_CRITICAL_SECTION(his);
  txSel = dmaCfg.txSel;
  txIdx = dmaCfg.txIdx[txSel];
  HAL_EXIT_CRITICAL_SECTION(his);

  // Enforce all or none.
  if ((len + txIdx) > HAL_UART_DMA_TX_MAX)
  {
    return 0;
  }

  (void)memcpy(&(dmaCfg.txBuf[txSel][txIdx]), buf, len);

  HAL_ENTER_CRITICAL_SECTION(his);
  /* If an ongoing DMA Tx finished while this buffer was being *appended*, then another DMA Tx
   * will have already been started on this buffer, but it did not include the bytes just appended.
   * Therefore these bytes have to be re-copied to the start of the new working buffer.
   */
  if (txSel != dmaCfg.txSel)
  {
    HAL_EXIT_CRITICAL_SECTION(his);
    txSel ^= 1;

    (void)memcpy(&(dmaCfg.txBuf[txSel][0]), buf, len);
    HAL_ENTER_CRITICAL_SECTION(his);
    dmaCfg.txIdx[txSel] = len;
  }
  else
  {
    dmaCfg.txIdx[txSel] = txIdx + len;
  }

  // If there is no ongoing DMA Tx, then the channel must be armed here.
  if (dmaCfg.txIdx[(txSel ^ 1)] == 0)
  {
    HAL_EXIT_CRITICAL_SECTION(his);
    HalUARTArmTxDMA();
  }
  else
  {
    dmaCfg.txMT = FALSE;
    HAL_EXIT_CRITICAL_SECTION(his);
  }
#endif

  return len;
}

/******************************************************************************
 * @fn      HalUARTPollDMA
 *
 * @brief   Poll a USART module implemented by DMA, including the hybrid solution in which the Rx
 *          is driven by DMA but the Tx is driven by ISR.
 *
 * @param   none
 *
 * @return  none
 *****************************************************************************/
static void HalUARTPollDMA(void)
{
  uint8 evt = 0;
  uint16 cnt;

#if DMA_PM
  PxIEN &= ~DMA_RDYIn_BIT;  // Clear to not race with DMA_RDY_IN ISR.
  {
    if (dmaRdyIsr || HAL_UART_DMA_RDY_IN() || HalUARTBusyDMA())
    {
      // Master may have timed-out the SRDY asserted state & may need a new edge.
#if HAL_UART_TX_BY_ISR
      if (!HAL_UART_DMA_RDY_IN() && (dmaCfg.txHead != dmaCfg.txTail))
#else
      if (!HAL_UART_DMA_RDY_IN() && ((dmaCfg.txIdx[0] != 0) || (dmaCfg.txIdx[1] != 0)))
#endif
      {
        HAL_UART_DMA_CLR_RDY_OUT();
      }
      dmaRdyIsr = 0;

      if (dmaRdyDly == 0)
      {
        (void)osal_set_event(Hal_TaskID, HAL_PWRMGR_HOLD_EVENT);
      }

      if ((dmaRdyDly = ST0) == 0)  // Reserve zero to signify that the delay expired.
      {
        dmaRdyDly = 0xFF;
      }
      HAL_UART_DMA_SET_RDY_OUT();
    }
    else if ((dmaRdyDly != 0) && (!DMA_PM_DLY || ((uint8)(ST0 - dmaRdyDly) > DMA_PM_DLY)))
    {
      dmaRdyDly = 0;
      (void)osal_set_event(Hal_TaskID, HAL_PWRMGR_CONSERVE_EVENT);
    }
  }
  PxIEN |= DMA_RDYIn_BIT;
#endif

#if !HAL_UART_TX_BY_ISR
  HalUARTPollTxTrigDMA();
#endif

  if (!HAL_UART_DMA_NEW_RX_BYTE(dmaCfg.rxHead))
  {
    if (HAL_UART_DMA_NEW_RX_BYTE(uartRxBug))
    {
      do {
        HAL_UART_RX_IDX_T_INCR(dmaCfg.rxHead);
      } while (!HAL_UART_DMA_NEW_RX_BYTE(dmaCfg.rxHead));

      uartRxBug = dmaCfg.rxHead;
      dmaCfg.rxTail = dmaCfg.rxHead;
    }
    HAL_UART_RX_IDX_T_INCR(uartRxBug);
  }

  cnt = HalUARTRxAvailDMA();  // Wait to call until after the above DMA Rx bug work-around.

#if HAL_UART_DMA_IDLE
  if (dmaCfg.rxTick)
  {
    // Use the LSB of the sleep timer (ST0 must be read first anyway) to measure the Rx timeout.
    if ((ST0 - dmaCfg.rxTick) > HAL_UART_DMA_IDLE)
    {
      dmaCfg.rxTick = 0;
      evt = HAL_UART_RX_TIMEOUT;
    }
  }
  else if (cnt != 0)
  {
    if ((dmaCfg.rxTick = ST0) == 0)  // Zero signifies that the Rx timeout is not running.
    {
      dmaCfg.rxTick = 0xFF;
    }
  }
#else
//  if (cnt != 0)
//  {
//    evt = HAL_UART_RX_TIMEOUT;
//  }
  
        switch (NubRN8209DCommand)
        {
          case 0://计量状态及校验和寄存器
            
          case 1://通道A电流的有效值

          case 2://通道B电流的有效值

          case 3://电压有效值

          case 8://有功能量 b
          case 7://有功能量a，读后清零寄存器、冻结电能寄存器可选，默认为读后清零寄存器
                if (cnt>=4)
                {
                  evt = HAL_UART_RX_TIMEOUT;
                }
                break;
          case 4://读频率
                if (cnt>=3)
                {
                  evt = HAL_UART_RX_TIMEOUT;
                }
                break;
          case 9://读系统状态寄存器
                if ( cnt>=2)
                {
                  evt = HAL_UART_RX_TIMEOUT;
                }
                break;                
          case 5://有功功率A

          case 6://有功功率B
                if (cnt>=5)
                {
                  evt = HAL_UART_RX_TIMEOUT;
                }
          
                break;       
        default:
                evt = 10;
                break;
                   
        }
#endif

  if (cnt >= HAL_UART_DMA_FULL)
  {
    evt |= HAL_UART_RX_FULL;
  }
  else if (cnt >= HAL_UART_DMA_HIGH)
  {
    evt |= HAL_UART_RX_ABOUT_FULL;

    if (!DMA_PM && (UxUCR & UCR_FLOW))
    {
      HAL_UART_DMA_CLR_RDY_OUT();  // Disable Rx flow.
    }
  }

  if (dmaCfg.txMT)
  {
    dmaCfg.txMT = FALSE;
    evt |= HAL_UART_TX_EMPTY;
  }

  if ((evt != 0) && (dmaCfg.uartCB != NULL))
  {
    dmaCfg.uartCB(HAL_UART_DMA-1, evt);
  }

  if (DMA_PM && (dmaRdyDly == 0) && !HalUARTBusyDMA())
  {
    HAL_UART_DMA_CLR_RDY_OUT();
  }
}

/**************************************************************************************************
 * @fn      HalUARTRxAvailDMA()
 *
 * @brief   Calculate Rx Buffer length - the number of bytes in the buffer.
 *
 * @param   none
 *
 * @return  length of current Rx Buffer
 **************************************************************************************************/
static uint16 HalUARTRxAvailDMA(void)
{
  // First, synchronize the Rx tail marker with where the DMA Rx engine is working.
  rxIdx_t tail = dmaCfg.rxTail;

  do
  {
    if (!HAL_UART_DMA_NEW_RX_BYTE(tail))
    {
      break;
    }

    HAL_UART_RX_IDX_T_INCR(tail);
  } while (tail != dmaCfg.rxHead);

  dmaCfg.rxTail = tail;

  uint16 cnt = tail - dmaCfg.rxHead;

  // If the DMA Rx may have overrun the circular queue, investigate further.
  if ((cnt == 0) && HAL_UART_DMA_NEW_RX_BYTE(tail))
  {
    /* Ascertain whether this polling is racing with the DMA Rx which may have clocked in a byte
     * since walking the tail. The Rx queue has wrapped only if the byte before the head is new.
     */
    tail = dmaCfg.rxHead;
    HAL_UART_RX_IDX_T_DECR(tail);

    if (HAL_UART_DMA_NEW_RX_BYTE(tail))
    {
      if (HAL_UART_RX_FLUSH)
      {
        (void)memset(dmaCfg.rxBuf, (DMA_PAD ^ 0xFF), HAL_UART_DMA_RX_MAX*2);

        uartRxBug = dmaCfg.rxHead;
        dmaCfg.rxTail = dmaCfg.rxHead;
      }
      else
      {
        cnt = HAL_UART_DMA_RX_MAX;
      }
    }
    else
    {
      cnt = 1;
    }
  }
  else if (cnt > HAL_UART_DMA_RX_MAX)  // If the tail has wrapped at the end of the Rx queue.
  {
    cnt += HAL_UART_DMA_RX_MAX;
  }

  return cnt;
}

/******************************************************************************
 * @fn      HalUARTBusyDMA
 *
 * @brief   Query the UART hardware & buffers before entering PM mode 1, 2 or 3.
 *
 * @param   None
 *
 * @return  TRUE if the UART H/W is busy or buffers are not empty; FALSE otherwise.
 *****************************************************************************/
static uint8 HalUARTBusyDMA( void )
{
#if HAL_UART_TX_BY_ISR
  return !((!(UxCSR & (CSR_ACTIVE | CSR_RX_BYTE))) && (HalUARTRxAvailDMA() == 0) &&
           (dmaCfg.txHead == dmaCfg.txTail));
#else
  return !((!(UxCSR & (CSR_ACTIVE | CSR_RX_BYTE))) && (HalUARTRxAvailDMA() == 0) &&
           (dmaCfg.txIdx[0] == 0) && (dmaCfg.txIdx[1] == 0));
#endif
}

#if !HAL_UART_TX_BY_ISR
/******************************************************************************
 * @fn      HalUARTPollTxTrigDMA
 *
 * @brief   Ascertain whether a manual trigger is required for the DMA Tx channel.
 *
 * @param   None
 *
 * @return  None
 *****************************************************************************/
static void HalUARTPollTxTrigDMA(void)
{
  if ((UxCSR & CSR_TX_BYTE) == 0)  // If no TXBUF to shift register transfer, then TXBUF may be MT.
  {
    if ((dmaCfg.txTick == 0) || ((uint8)(ST0 - dmaCfg.txTick) > HAL_UART_TX_TICK_MIN))
    {
      dmaCfg.txTick = 0;

      if (dmaCfg.txTrig && HAL_DMA_CH_ARMED(HAL_DMA_CH_TX))
      {
        HAL_DMA_MAN_TRIGGER(HAL_DMA_CH_TX);
      }
      dmaCfg.txTrig = 0;
    }
  }
  else
  {
    UxCSR = (CSR_MODE | CSR_RE);  // Clear the CSR_TX_BYTE flag.
    dmaCfg.txTick = ST0;

    if (dmaCfg.txTick == 0)  // Reserve zero to signify that the minimum delay has been met.
    {
      dmaCfg.txTick = 0xFF;
    }
  }
}

/******************************************************************************
 * @fn      HalUARTArmTxDMA
 *
 * @brief   Arm the Tx DMA channel.
 *
 * @param   None
 *
 * @return  None
 *****************************************************************************/
static void HalUARTArmTxDMA(void)
{
  halDMADesc_t *ch = HAL_DMA_GET_DESC1234(HAL_DMA_CH_TX);
  HAL_DMA_SET_SOURCE(ch, dmaCfg.txBuf[dmaCfg.txSel]);
  HAL_DMA_SET_LEN(ch, dmaCfg.txIdx[dmaCfg.txSel]);

  dmaCfg.txSel ^= 1;
  dmaCfg.txTrig = 1;
  HAL_DMA_ARM_CH(HAL_DMA_CH_TX);

  HalUARTPollTxTrigDMA();

  if (DMA_PM)
  {
    HAL_UART_DMA_SET_RDY_OUT();
  }
}
#endif

/******************************************************************************
 * @fn      HalUART_DMAIsrDMA
 *
 * @brief   Handle the Tx done DMA ISR.
 *
 * @param   none
 *
 * @return  none
 *****************************************************************************/
void HalUART_DMAIsrDMA(void)
{
#if !HAL_UART_TX_BY_ISR
  if (dmaCfg.txIdx[dmaCfg.txSel])
  {
    // If there is more Tx data ready to go, re-arm the DMA immediately on it.
    HalUARTArmTxDMA();

    // Indicate that the Tx buffer just finished is now free (re-arming did a ^= toggle of txSel).
    dmaCfg.txIdx[dmaCfg.txSel] = 0;
  }
  else
  {
    dmaCfg.txIdx[(dmaCfg.txSel ^ 1)] = 0;  // Indicate that the Tx buffer just finished is now free.

    // Clear the CSR_TX_BYTE flag & start the txTick to allow the possibility of an immediate
    // manual trigger from the next Write(), if it occurs more than one character time later.
    HalUARTPollTxTrigDMA();
  }

  dmaCfg.txMT = TRUE;  // Notify CB that at least one Tx buffer is now free to use.
#endif
}

#if DMA_PM
/**************************************************************************************************
 * @fn      PortX Interrupt Handler
 *
 * @brief   This function is the PortX interrupt service routine.
 *
 * @param   None.
 *
 * @return  None.
 *************************************************************************************************/
#if (HAL_UART_DMA == 1)
HAL_ISR_FUNCTION(port0Isr, P0INT_VECTOR)
#else
HAL_ISR_FUNCTION(port1Isr, P1INT_VECTOR)
#endif
{
  HAL_ENTER_ISR();

  PxIFG = 0;
  PxIF = 0;

  dmaRdyIsr = 1;

  CLEAR_SLEEP_MODE();
  HAL_EXIT_ISR();
}
#endif

#if HAL_UART_TX_BY_ISR
/***************************************************************************************************
 * @fn      halUartTxIsr
 *
 * @brief   UART Transmit Interrupt
 *
 * @param   None
 *
 * @return  None
 ***************************************************************************************************/
#if (HAL_UART_DMA == 1)
HAL_ISR_FUNCTION( halUart0TxIsr, UTX0_VECTOR )
#else
HAL_ISR_FUNCTION( halUart1TxIsr, UTX1_VECTOR )
#endif
{
  HAL_ENTER_ISR();

  if (dmaCfg.txHead == dmaCfg.txTail)
  {
    IEN2 &= ~UTXxIE;
    dmaCfg.txMT = 1;
  }
  else
  {
    UTXxIF = 0;
    UxDBUF = dmaCfg.txBuf[dmaCfg.txHead++];

    if ((HAL_UART_DMA_TX_MAX != 256) && (dmaCfg.txHead >= HAL_UART_DMA_TX_MAX))
    {
      dmaCfg.txHead = 0;
    }
  }

  HAL_EXIT_ISR();
}
#endif

/**************************************************************************************************
*/
