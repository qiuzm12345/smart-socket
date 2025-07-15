
#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "string.h"

#include "AppUserProcess.h"
#include "hal_shiftout.h"

#define __HT1621_DRV_C__
#include "HT1621_drv.h"

extern uint16 TalkingSecond;
extern uint8 FlashSecondleft;
extern bool bFlag_KLED;


#define HT1621_WR_HIGH()	P1_4 = 1
#define HT1621_WR_LOW()		P1_4 = 0

#define HT1621_DATA_HIGH()	P2_0 = 1
#define HT1621_DATA_LOW()	P2_0 = 0

#define DELAY_C		        4	//24M 应为 4

#define HT1621_CMDDLY()

extern bool b_View_YueRiFlag,b_Telephone_online,b_FlashToggle;

static void Ht1621_write_bit(uint8 bits)
{
	if(bits == 0)
		HT1621_DATA_LOW();
	else
		HT1621_DATA_HIGH();
        ASM_NOP;
	HT1621_WR_LOW();
    for(bits=0;bits<2;bits++)
        ASM_NOP;
	HT1621_WR_HIGH();
}

static void Ht1621_Bytes_W_H(uint8 Bytes,uint8 Len)
{
	Bytes <<= 8-Len;
	while(Len--)
	{
		Ht1621_write_bit(Bytes&0x80);
		Bytes <<= 1;
	}
}

/*
static void Ht1621_Bytes_W_L(uint8 Bytes,uint8 Len)
{
	while(Len--)
	{
		Ht1621_write_bit(Bytes&0x01);
		Bytes >>= 1;
	}
}
*/

#define POER_CMD		0x04
#define POER_WRITE		0x05
#define POER_READ		0x06

static void Ht1621_Cmd_One(uint8 Cmd)
{
	HT1621_CS_LOW();
        
        //P2.0        DAT
        P2DIR |= BV(0);        //Set pin direction to Output
        
	Ht1621_Bytes_W_H(POER_CMD,3);
	Ht1621_Bytes_W_H(Cmd,8);
	Ht1621_write_bit(1);
        
	HT1621_CS_HIGH();
	HT1621_CMDDLY();
}

static void Ht1621_Cmd(uint8 *Cmd,uint8 Len)
{
	uint8 i;
	for(i=0;i<Len;i++)
		Ht1621_Cmd_One(Cmd[i]);
}

static void Ht1621_Flash(uint8 *Buf,uint8 Addr,uint8 Len)
{
	uint8 i,j,Bytes;
        register uint8 k;
        
	HT1621_CS_LOW();
        
        //P2.0        DAT
        P2DIR |= BV(0);        //Set pin direction to Output

	//Ht1621_Bytes_W_H(POER_WRITE,3);
        Bytes = ( POER_WRITE << 5 );
        Ht1621_write_bit(Bytes&0x80);
        Bytes <<= 1;
        Ht1621_write_bit(Bytes&0x80);
        Bytes <<= 1;
        Ht1621_write_bit(Bytes&0x80);
        Bytes <<= 1;
        
	//Ht1621_Bytes_W_H(Addr,6);
        Bytes = ( Addr << 2);
        j = 6;
	while(j--)
	{
            Ht1621_write_bit(Bytes&0x80);
            Bytes <<= 1;
	}
        
        for(i=0;i<Len;i++)
        {
            Bytes = Buf[i];
            for(j=0;j<8;j++)
            {
                if( (Bytes&0x01) == 0)
                    HT1621_DATA_LOW();
                else
                    HT1621_DATA_HIGH();
                ASM_NOP;
                HT1621_WR_LOW();
                for(k=0;k<2;k++)
                    ASM_NOP;
                HT1621_WR_HIGH();
                Bytes >>= 1;
            }
        }

	HT1621_CS_HIGH();
    P1_4 = bFlag_KLED;
}

//____________________________________________________应用层接口
uint8 HT1621_Buf[32];

static uint8 HT1621_init_Cmd[]=	//LCD的初始化命令列表
{	

	0x28,	//0010 0010//BIAS		偏压选择，commons
	0x01,	//0000 0001 //SYS EN	系统使能	
	0x03,	//0000 0011 //LCD ON	打开LCD偏压电路
//	0x04,	//0000 0100	//TIMER DIS	停止time输出
//	0x05,	//0000 0101	//WDT DIS	禁止WDT输出
//	0x08,	//0000 1000//TONE OFF	禁止蜂鸣器输出
//	0x1b,	//0001 1011//RC 256K	使用内部256K时钟源选择
};

const uint8  _Bits[]={0xfe,0xfd,0xfb,0xf7,0xef,0xdf,0xbf,0x7f,FW_VERSION / 100, FW_VERSION % 100};

void HT1621_Init(void)
{
        
	HT1621_CS_HIGH();
	HT1621_WR_HIGH();

	Ht1621_Cmd((uint8*)HT1621_init_Cmd,sizeof(HT1621_init_Cmd));
	
	HT1621_Clear_All();
	HT1621_Updata_All();
}

void HT1621_Init_again(void)
{
	HT1621_CS_HIGH();
	HT1621_WR_HIGH();
	HT1621_DATA_HIGH();
	Ht1621_Cmd((uint8*)HT1621_init_Cmd,sizeof(HT1621_init_Cmd));
}

void HT1621_Clear_All(void)
{
	memset(HT1621_Buf,0,sizeof(HT1621_Buf));
	//Ht1621_Write(HT1621_Buf,0,sizeof(HT1621_Buf));
}

void HT1621_Updata_All(void)
{
	//Ht1621_Write(HT1621_Buf,0,sizeof(HT1621_Buf));
	Ht1621_Flash(HT1621_Buf,0,sizeof(HT1621_Buf));
}

void HT1621_LBuf_Write(uint8 Addr,uint8 Datas)
{
	if(Addr<sizeof(HT1621_Buf))
		HT1621_Buf[Addr]=Datas;
}

uint8 HT1621_LBuf_Read(uint8 Addr)
{
	//if(Addr<sizeof(HT1621_Buf))
	Addr %= sizeof(HT1621_Buf);
	return HT1621_Buf[Addr];
}

//_________________________________________________应用层接口

void _Ht_Bit_Set(uint8 Addr,uint8 Bits,uint8 Da)
{
	if( Da )
		Da = 1;
	HT1621_Buf[Addr] &= _Bits[Bits];
	HT1621_Buf[Addr] |= Da<<Bits;
}

/* 7段数码管排序

D	e g f x d c b a	hex

0	1 0 1 0 1 1 1 1	0xaf
1	0 0 0 0 0 1 1 0	0x06
2	1 1 0 0 1 0 1 1	0xcb
3	0 1 0 0 1 1 1 1 0x4f
4	0 1 1 0 0 1 1 0 0x66
5	0 1 1 0 1 1 0 1 0x6d
6	1 1 1 0 1 1 0 1 0xed
7	0 0 0 0 0 1 1 1 0x07
8	1 1 1 0 1 1 1 1 0xef
9	0 1 1 0 1 1 1 1 0x6f
┗	1 0 0 0 1 0 0 0 0x88
┛	0 0 0 0 1 1 0 0 0x0c
-	0 1 0 0 0 0 0 0 0x40
P	1 1	1 0	0 0	1 1 0xe3

_	0 0 0 0 1 0 0 0 0x08
A	1 1 1 0 0 1 1 1 0xe7
b	1 1 1 0 1 1 0 0 0xec
c	1 1 0 0 1 0 0 0	0xc8
d	1 1 0 0 1 1 1 0 0xce
E	1 1 1 0 1 0 0 1 0xe9
F	1 1 1 0 0 0 0 1 0xe1

t	1 1 1 0 1 0 0 0 0xe8
M	1 0	1 0	0 0	1 1 0xa3
M	0 0 1 0 0 1 1 1 0x27
*/


const uint8  D_7Seg[]=
{
	0x00,	//空	
	0x06,	//1	1
	0xcb,	//2	2
	0x4f,	//3	3
	0x66,	//4	4
	0x6d,	//5	5
	0xed,	//6	6
	0x07,	//7	7
	0xef,	//8	8
	0x6f,	//9	9
	0xaf,	//a	0
	0x88,	//b	┗
	0x0c,	//c	┛
	0x40,	//d -
	0x08,	//e _
	0xff,	//全
	0x00,	
	0xe3,	//P
	0xe7,	//A		//offset = 0x12
	0xec,	//b
	0xc8,	//c
	0xce,	//d
	0xe9,	//E
	0xe1,	//F
	0xe8,	//t
	0xa3,	//M		//offset = 0x19
	0x27,	//M
};

void HT161_Disp_7Seg2(uint8 addr,uint8 Datas)	//显示7段数码管 0不显示
{
	uint8 i,temp ;
	i = addr-13;
	if ( Datas == 0 && addr != 15 && addr != 17 )
		Datas = 10;	//'0'
	
	temp = D_7Seg[Datas];
	HT1621_Buf[i*2] &= 0xf0;
	if ( addr == 15 )
		HT1621_Buf[i*2] |= ( (temp & 0xf0) >> 6 );
	else
	{
		HT1621_Buf[i*2] |= ( (temp & 0xf0) >> 5 );
		if ( addr == 17 )
		{
			if ( b_View_YueRiFlag )
				HT1621_Buf[i*2] |= 0x08;	;	//‘日’
		}
	}
	HT1621_Buf[i*2+1] &= 0xf0;
	HT1621_Buf[i*2+1] |= (temp & 0x0f) ;
}

void HT161_Disp_NumLine(uint8 * databuf)	//显示7段数码管 0不显示
{
	uint8 i,temp ;
	for (i=0;i<VIEW_MAX_NUM;i++ )
	{
		if ( databuf == NULL )
			temp = 0;
		else
			temp = D_7Seg[databuf[i]];
		
		HT1621_Buf[i*2] &= 0x0f;
		HT1621_Buf[i*2] |= (temp & 0xf0);
		HT1621_Buf[i*2+1] &= 0x0f;
		HT1621_Buf[i*2+1] |= ( (temp & 0x0f) << 4 );
	}
}

void HT161_Disp_MACcode(uint8 * addr)
{
	uint8 i,k,temp;
	for (i=0;i<6;i++)
	{
		temp = ( ( addr[i] & 0xf0 ) >> 4 );
		if ( temp == 0 )
			temp = D_7Seg[0x0a];
		else
		if ( temp < 10 )
			temp = D_7Seg[temp];
		else
		{
			temp = D_7Seg[0x12+temp-10];
		}
		k = (i*2)*2;
		HT1621_Buf[k] &= 0x0f;
		HT1621_Buf[k] |= (temp & 0xf0);
		HT1621_Buf[k+1] &= 0x0f;
		HT1621_Buf[k+1] |= ( (temp & 0x0f) << 4 );

		temp = ( addr[i] & 0x0f );
		if ( temp == 0 )
			temp = D_7Seg[0x0a];
		else
		if ( temp < 10 )
			temp = D_7Seg[temp];
		else
		{
			temp = D_7Seg[0x12+temp-10];
		}
		HT1621_Buf[k+2] &= 0x0f;
		HT1621_Buf[k+2] |= (temp & 0xf0);
		HT1621_Buf[k+3] &= 0x0f;
		HT1621_Buf[k+3] |= ( (temp & 0x0f) << 4 );
	}
}

void HT1621_Disp_SecDot(uint8 Flag)
{
	_Ht_Bit_Set(12,3,Flag);//秒点闪烁
}

void HT1621_Disp_InOutFlag(uint8 flag)
{
	_Ht_Bit_Set(16,0,flag == 1);//来电
	_Ht_Bit_Set(16,1,flag == 2);//去电
}

void HT1621_Disp_Weeks(uint8 Flag)
{
	if ( Flag > 7 )
		Flag = 7;
	
	HT1621_Buf[22] &= 0xf0;
	HT1621_Buf[23] &= 0xf0;
	if ( Flag > 0 )
	{
		if ( Flag < 5 )
			HT1621_Buf[22] |= ( 0x10 >> Flag ) ;
		else
			HT1621_Buf[23] |= ( 0x01 << (Flag-5) );
	}
}

void HT1621_Disp_Years(uint8 year)
{
	uint8 tempbuf[VIEW_MAX_NUM];

	memset(tempbuf,0,VIEW_MAX_NUM);
	
	if ( b_Telephone_online )
	{
		tempbuf[1]=0x0d;
		tempbuf[2]=0x0d;
		tempbuf[9]=0x0d;
		tempbuf[10]=0x0d;
	}
	else
	{
		tempbuf[1]=0x0;
		tempbuf[2]=0x0;
		tempbuf[9]=0x0;
		tempbuf[10]=0x0;
	}
	
	tempbuf[4]=0x02;
	tempbuf[5]=0x0a;
	if ( ( year / 10 ) == 0 )
		tempbuf[6]= 0x0a;
	else
		tempbuf[6]= year / 10;
	if ( ( year % 10 ) == 0 )
		tempbuf[7]= 0x0a;
	else
		tempbuf[7]= year % 10;
	
	HT161_Disp_NumLine(tempbuf);
}

void HT1621_Disp_SetEasyCallOut(uint8 index)
{
	uint8 tempbuf[VIEW_MAX_NUM];

	memset(tempbuf,0,VIEW_MAX_NUM);
	tempbuf[1]=0x0d;
	tempbuf[2]=0x0d;
	tempbuf[3]=0x0d;
	tempbuf[8]=0x0d;
	tempbuf[9]=0x0d;
	tempbuf[10]=0x0d;
	
	tempbuf[4]=0x19;
	tempbuf[5]=0x1a;
	
	tempbuf[6]= index / 10;
	if ( ( index % 10 ) == 0 )
		tempbuf[7]= 0x0a;
	else
		tempbuf[7]= index % 10;
	HT161_Disp_NumLine(tempbuf);
}

void HT1621_Disp_Counter(uint8 cnt)
{
	uint8 temp;

	/*
	if ( cnt >= 200 )
	{
		HT1621_Buf[18] &= 0xf0;
		HT1621_Buf[19] &= 0xf0;
		HT1621_Buf[20] &= 0xf0;
		HT1621_Buf[21] &= 0xf0;
		return;
	}
	*/
	temp = (cnt %100) / 10;
	if ( temp == 0 )
		temp = 10;	//'0'
	temp = D_7Seg[temp];
	HT1621_Buf[18] &= 0xf0;
	HT1621_Buf[18] |= ( temp >> 4 ) ;

	_Ht_Bit_Set(18,0,cnt/100);	
	
	HT1621_Buf[19] &= 0xf0;
	HT1621_Buf[19] |= ( temp & 0x0f ) ;

	temp = cnt % 10;
	if ( temp == 0 )
		temp = 10;	//'0'
	temp = D_7Seg[temp];
	HT1621_Buf[20] &= 0xf0;
	HT1621_Buf[20] |= ( temp >> 5 ) ;
	HT1621_Buf[21] &= 0xf0;
	HT1621_Buf[21] |= ( temp & 0x0f ) ;
}

void HT1621_Disp_Times(RTC_Time *Times,uint8 Flag)
{
	_Ht_Bit_Set(1,0,b_View_YueRiFlag);//月
	_Ht_Bit_Set(1,1,Times->Month/10);//月份十位	
	HT161_Disp_7Seg2(14,Times->Month%10);

	HT161_Disp_7Seg2(15,Times->Day/10);
	HT161_Disp_7Seg2(16,Times->Day%10);

	HT161_Disp_7Seg2(17,Times->Hour/10);	
	HT161_Disp_7Seg2(18,Times->Hour%10);

	HT161_Disp_7Seg2(19,Times->Minute/10);
	HT161_Disp_7Seg2(20,Times->Minute%10);
		
	if ( Flag > 0 )
	{
		if ( Flag == 1 )
		{
			if ( b_FlashToggle )
			{
				_Ht_Bit_Set(1,1,0);//月份十位
				HT161_Disp_7Seg2(14,0x10);
			}
		}
		else
		{
			if ( b_FlashToggle )
			{
				HT161_Disp_7Seg2(11+Flag*2,0x10);
				HT161_Disp_7Seg2(12+Flag*2,0x10);
			}
		}
	}
}

void HT1621_Disp_Version(uint8 Version)
{
	b_View_YueRiFlag = 0;
	HT161_Disp_7Seg2(17,0x0a);	
	HT161_Disp_7Seg2(18,Version/100);

	Version %= 100;
	HT161_Disp_7Seg2(19,Version/10);
	HT161_Disp_7Seg2(20,Version%10);
}

void HT1621_Disp_Talking(uint16 talksec)
{
   	uint8 temp;

	if ( talksec > 4199 )	//69:59
		talksec = 4199;
	
	_Ht_Bit_Set(1,0,0);//月
	_Ht_Bit_Set(1,1,0);//月份十位	
	HT161_Disp_7Seg2(14,0x10);

	HT161_Disp_7Seg2(15,0x10);
	HT161_Disp_7Seg2(16,0x10);

	temp = talksec / 60;
	HT161_Disp_7Seg2(17,temp/10);	
	HT161_Disp_7Seg2(18,temp%10);

	temp = talksec % 60;
	HT161_Disp_7Seg2(19,temp/10);
	HT161_Disp_7Seg2(20,temp%10);
}

void HT161_Disp_BTflag(uint8 Flag)
{
	uint8 temp;
	if ( Flag )
	{
		HT1621_Buf[18] &= 0xf0;
		HT1621_Buf[19] &= 0xf0;
		HT1621_Buf[20] &= 0xf0;
		HT1621_Buf[21] &= 0xf0;
	}
	else	
	{
		temp = D_7Seg[0x13];	//b
		HT1621_Buf[18] &= 0xf0;
		HT1621_Buf[18] |= ( temp >> 4 ) ;
		HT1621_Buf[19] &= 0xf0;
		HT1621_Buf[19] |= ( temp & 0x0f ) ;

		temp = D_7Seg[0x18];	//t
		HT1621_Buf[20] &= 0xf0;
		HT1621_Buf[20] |= ( temp >> 5 ) ;
		HT1621_Buf[21] &= 0xf0;
		HT1621_Buf[21] |= ( temp & 0x0f ) ;
	}
}

void HT161_Disp_HandSet_Small_flag(uint8 Flag)
{
	if ( Flag )
	{
		HT161_Disp_7Seg2(14,0x14);
	}
	else	
	{
		HT161_Disp_7Seg2(14,0x0e);
	}
}

void HT161_Disp_HandSet_flag(uint8 Flag)
{
	uint8 temp;
	if ( Flag )
	{
		temp = D_7Seg[0x14];
	}
	else	
	{
		temp = D_7Seg[0x0e];
	}
	HT1621_Buf[0] &= 0x0f;
	HT1621_Buf[0] |= (temp & 0xf0);
	HT1621_Buf[1] &= 0x0f;
	HT1621_Buf[1] |= ( (temp & 0x0f) << 4 );
}



//end file

