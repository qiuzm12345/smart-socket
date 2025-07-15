/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_shiftout.h"
#include "npi.h"
#include "string.h"
#include "stdio.h"

void HalShiftInit( void )
{
  //P1.5        CLK
  P1SEL &= ~( BV(5) );   //Set pin function to GPIO
  P1DIR |= BV(5);        //Set pin direction to Output
  //P2.0        DAT
  P2SEL &= ~( BV(0) );   //Set pin function to GPIO
  P2DIR |= BV(0);        //Set pin direction to Output
  //P0.1        SET
  P0SEL &= ~( BV(1) | BV(0) );   //Set pin function to GPIO
  P0DIR |= BV(1);        //Set pin direction to Output
  P0DIR &= ~BV(0);        //Set pin direction to Input
    
  //P2INP |= PUSH2_BV;  /* Configure GPIO tri-state. */
}        


#define        CLR_595ShiftCLK          P1_5=0
#define        SET_595ShiftCLK          P1_5=1
#define        CLR_595SerDATA           P2_0=0
#define        SET_595SerDATA           P2_0=1
#define        CLR_595StorCLK           P0_1=0
#define        SET_595StorCLK           P0_1=1

//===========================================================================
//Function   :	Out_595_onebyte(unsigned char outdata)
//Description:  往595数据链串行移位1字节数据	
//Parameters : 	
//Return     :  
//Notes      :  
//===========================================================================
void Out_595_onebyte(unsigned char outdata)
{
	unsigned char i;//,cnt;
	CLR_595ShiftCLK;//上升沿数据移位
	for(i=0;i<8;i++)
	{		   
		CLR_595ShiftCLK;//上升沿数据移位
		if(outdata&0x80)
		{
			SET_595SerDATA;
		}
		else
		{
			CLR_595SerDATA;
		}
		//for(cnt=0;cnt<10;cnt++);
		SET_595ShiftCLK;//上升沿数据移位
		outdata <<= 1;
	}
	SET_595SerDATA;   
	//CLR_595StorCLK;//上升沿锁存移位寄存器数据
	//for(i=0;i<10;i++)_nop_();
	//SET_595StorCLK;//上升沿锁存移位寄存器数据	
}

//===========================================================================
//Function   :	SET_595_output()
//Description: 	设置 595 输出的内容
//Parameters : 
//Return     :	
//Notes      :  
//===========================================================================
void HalShiftOutput(unsigned char outdata)
{
	CLR_595StorCLK;//上升沿锁存移位寄存器数据
	Out_595_onebyte( outdata );
	SET_595StorCLK;//上升沿锁存移位寄存器数据	
}

extern uint16 TalkingSecond;
extern void DTMFdetectEN(uint8 flag);




#define	CLR_165ShiftCLK	   P1_5=0
#define	SET_165ShiftCLK	   P1_5=1

#define	Get_165SerDATA	   P0_0

#define	CLR_165StorCLK	   P2_0 = 0
#define	SET_165StorCLK	   P2_0 = 1
