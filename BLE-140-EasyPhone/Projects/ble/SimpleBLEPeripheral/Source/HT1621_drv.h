

#ifndef __HT1621_DRV_H__
#define __HT1621_DRV_H__

#ifdef __HT1621_DRV_C__
#define GLOBAL
#else
#define GLOBAL extern
#endif

#ifndef BX_SUM

#define BX_SUN
#define B0 1
#define B1 2
#define B2 4
#define B3 8
#define B4 16
#define B5 32
#define B6 64
#define B7 128
#endif


// DZ S Define
#define ADDR_OFFSET	9

#define ADDR_S1		12
#define ADDR_S2		38
#define ADDR_S3		40
#define ADDR_S4		36
#define ADDR_S5		32
#define ADDR_S6		16
#define ADDR_S7		16
#define ADDR_S8		16
//#define ADDR_S9
#define ADDR_S10	17
#define ADDR_S11	17
#define ADDR_S12	17
#define ADDR_S13	18
#define ADDR_S14	24
#define ADDR_S15	28
#define ADDR_S16	40
#define ADDR_S17	29
#define ADDR_S18	28
#define ADDR_S19	40
#define ADDR_S20	40

#define BITE_S1		0
#define BITE_S2		0
#define BITE_S3		0
#define BITE_S4		0
#define BITE_S5		0
#define BITE_S6		3
#define BITE_S7		2
#define BITE_S8		4
//#define BITE_S9
#define BITE_S10	0
#define BITE_S11	1
#define BITE_S12	2
#define BITE_S13	4
#define BITE_S14	4
#define BITE_S15	3
#define BITE_S16	1
#define BITE_S17	3
#define BITE_S18	2
#define BITE_S19	2
#define BITE_S20	3

#define ADDR_T1		10
#define ADDR_T2		12
#define ADDR_T3		14
#define ADDR_T4		17
#define ADDR_T5		22
#define ADDR_T6		26
#define ADDR_T7		28
#define ADDR_T8		30
#define ADDR_T9		32
#define ADDR_T10	9
#define ADDR_T11	9
#define ADDR_T12	9
#define ADDR_T13	9
#define ADDR_T14	34
#define ADDR_T15	34
#define ADDR_T16	34
#define ADDR_T17	34
#define ADDR_T18	34
#define ADDR_T19	34
#define ADDR_T20	35
#define ADDR_T21	35
#define ADDR_T22	35
#define ADDR_T23	35
#define ADDR_T24	35
#define ADDR_T25	35
#define ADDR_T26	35
#define ADDR_T27	34

#define BITE_T1		4
#define BITE_T2		4
#define BITE_T3		4
#define BITE_T4		3
#define BITE_T5		4
#define BITE_T6		4
#define BITE_T7		4
#define BITE_T8		4
#define BITE_T9		4
#define BITE_T10	4
#define BITE_T11	5
#define BITE_T12	6
#define BITE_T13	7
#define BITE_T14	0
#define BITE_T15	1
#define BITE_T16	3
#define BITE_T17	2
#define BITE_T18	6
#define BITE_T19	5
#define BITE_T20	5
#define BITE_T21	6
#define BITE_T22	7
#define BITE_T23	3
#define BITE_T24	2
#define BITE_T25	1
#define BITE_T26	0
#define BITE_T27	7

GLOBAL void Ht1621_IO_CONFIG(void);
GLOBAL void HT1621_Init(void);
GLOBAL void HT1621_Init_again(void);
GLOBAL void HT1621_Clear_All(void);
GLOBAL void HT1621_Updata_All(void);
GLOBAL void HT1621_LBuf_Write(uint8 Addr,uint8 Datas);
//GLOBAL uint8 HT1621_LBuf_Read(uint8 Addr);
GLOBAL uint8 NumLineBuf[16];

GLOBAL void _Ht_Bit_Set(uint8 Addr,uint8 Bits,uint8 Da);
GLOBAL void HT161_Disp_7Seg(uint8 addr,uint8 Datas);
GLOBAL void HT161_Disp_7Seg2(uint8 addr,uint8 Datas);

GLOBAL void HT1621_Disp_Times(RTC_Time *Times,uint8 Flag);

/*
GLOBAL void HT1621_Disp_TelStart(void);
GLOBAL void HT1621_Disp_RecordMode(uint8 Flag);
GLOBAL void HT1621_Disp_MSum(char *Str,uint8 Len);
GLOBAL void HT1621_Disp_Vol(uint8 Vol);
GLOBAL void HT1621_Disp_SaveSum(void);
GLOBAL void HT1621_Disp_RingFlag(uint8 Flag);
GLOBAL void HT1621_Disp_LeaveTimeFlag(uint8 Flag);
GLOBAL void HT1621_Disp_RingTime(uint8 Flag);
GLOBAL void HT1621_Disp_MICFlag(uint8 Flag);
GLOBAL void HT1621_Disp_ALLOWFlag(uint8 Flag);
GLOBAL void HT1621_Disp_AlarmFlag(uint8 Flag);
GLOBAL void Ht1621_Round_Start(void);
GLOBAL void Ht1621_Round_Stop(void);
GLOBAL void HT1621_Disp_Sd(uint8 Flag);
GLOBAL void HT1621_Disp_InfoFlag(uint8 Flag);	//显示录音的标志
*/
void HT161_Disp_NumLine(uint8 * databuf);
void HT161_Disp_MACcode(uint8 * addr);
void HT1621_Disp_SecDot(uint8 Flag);
void HT1621_Disp_InOutFlag(uint8 flag);
void HT1621_Disp_Weeks(uint8 Flag);
void HT1621_Disp_Years(uint8 year);
void HT1621_Disp_SetEasyCallOut(uint8 index);
void HT1621_Disp_Counter(uint8 cnt);
void HT1621_Disp_Times(RTC_Time *Times,uint8 Flag);
void HT1621_Disp_Version(uint8 Version);
void HT1621_Disp_Talking(uint16 talksec);
void HT161_Disp_BTflag(uint8 Flag);
void HT161_Disp_HandSet_Small_flag(uint8 Flag);
void HT161_Disp_HandSet_flag(uint8 Flag);
GLOBAL void HT1621_10ms(void);

#undef GLOBAL

#endif //__HT1621_DRV_H__

