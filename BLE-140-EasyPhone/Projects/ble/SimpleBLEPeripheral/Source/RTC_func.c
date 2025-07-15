/* Includes ------------------------------------------------------------------*/
#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "OSAL_Clock.h"

#include "AppUserProcess.h"

#define SecsPerDay 86400 //(3600*24)

#define SecsPerComYear  3153600		//(365*3600*24)						//普通一年 n 秒
#define SecsPerLeapYear 31622400	//(366*3600*24)						//闰年 n 秒
#define SecsPerFourYear 126230400	//((365*3600*24)*3+(366*3600*24))	//每4年 n 秒


const uint8 MonthDaysArray[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
const uint16 Month_Days_Accu_C[13] = {0,31,59,90,120,151,181,212,243,273,304,334,365};
const uint16 Month_Days_Accu_L[13] = {0,31,60,91,121,152,182,213,244,274,305,335,366};
const uint32 Year_Secs_Accu[5]={0,				//起点
							31622400,			//闰年
							63158400,			//第2年
							94694400,			//第3年
							126230400};			//第4年
const uint32 Month_Secs_Accu_C[13] = { 0,		//起点
                            2678400,	//1月
                            5097600,	//2月  (非润月)
                            7776000,	//3月
                            10368000,	//4月
                            13046400,	//5月
                            15638400,	//6月
                            18316800,	//7月
                            20995200,	//8月
                            23587200,	//9月
                            26265600,	//10月
                            28857600,	//11月
                            31536000};	//12月
const uint32 Month_Secs_Accu_L[13] = {0,			//起点
                            2678400,	//1月
                            5184000,	//2月	(闰月)
                            7862400,  	//3月
                            10454400,	//4月
                            13132800,	//5月
                            15724800,	//6月
                            18403200,	//7月
                            21081600,	//8月
                            23673600,	//9月
                            26352000,	//10月
                            28944000,	//11月
                            31622400};	//12月

uint8 GetMaxDayInMonth(uint16 year,uint8 mon)
{
	uint8 day;
	day = MonthDaysArray[mon-1];
	if ( mon == 2 )
	{
		if ( ( year % 4 ) == 0 && ( year % 100 ) != 0 || ( year % 400 ) == 0 )
		{
			day++;
		}
	}
	return day;
}

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//XAB 禁止tamper pin(PC.13) 输出 RTC Clock/64 脉冲，作为GPIO使用
//#define RTCClockOutput_Enable  /* RTC Clock/64 is output on tamper pin(PC.13) */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8 CurrWeekOffset = WEEK_FIRST_DAY;

uint32 RTC_GetSecond(uint16 Tmp_Year,uint8 Tmp_Month,uint8 Tmp_Date,uint8 Tmp_HH,uint8 Tmp_MM,uint8 Tmp_SS)
{ 
	uint16 LeapY, ComY;
    uint32 TotSeconds, TotDays;

  	if(Tmp_Year<=START_YEAR)
      	LeapY = 0;
    else
      	LeapY = (Tmp_Year - START_YEAR -1)/4 +1;
    
  ComY = (Tmp_Year - START_YEAR)-(LeapY);
  
  if (Tmp_Year%4)
    //common year
    TotDays = (uint32)LeapY*366 + (uint32)ComY*365 + Month_Days_Accu_C[Tmp_Month-1] + (Tmp_Date-1); 
  else
    //leap year	//闰年
    TotDays = (uint32)LeapY*366 + (uint32)ComY*365 + Month_Days_Accu_L[Tmp_Month-1] + (Tmp_Date-1); 
  
  	TotSeconds = TotDays*SecsPerDay + ((uint32)Tmp_HH*3600 + Tmp_MM*60 + Tmp_SS);
	return TotSeconds;
}

void RTC_SetDateTime(uint16 Tmp_Year,uint8 Tmp_Month,uint8 Tmp_Date,uint8 Tmp_HH,uint8 Tmp_MM,uint8 Tmp_SS,uint8 Tmp_WW)
{ 
	uint8 week;
	uint32 TotSeconds;
  	TotSeconds = RTC_GetSecond(Tmp_Year,Tmp_Month,Tmp_Date,Tmp_HH,Tmp_MM,Tmp_SS);

	week = ((TotSeconds/SecsPerDay)+ CurrWeekOffset )%7+1;	//以起始日期作为参考
	if ( Tmp_WW != week )
	{
		if ( week > Tmp_WW ) 
			CurrWeekOffset -= ( week - Tmp_WW );
		else
			CurrWeekOffset += ( Tmp_WW - week );
	}
	
	osal_setClock(TotSeconds);
	//sprintf(tempbuf,"SetDateTime %04d-%02d-%02d %02d:%02d:%02d\r\n",Tmp_Year,Tmp_Month,Tmp_Date,Tmp_HH,Tmp_MM,Tmp_SS);
	//printf("%s",tempbuf);
}

uint32 RTCTimeToSeconds(RTC_Time * Time)
{
  	return RTC_GetSecond(Time->Year,Time->Month,Time->Day,Time->Hour,Time->Minute,Time->Second);
}

void SecondsToRTCTime(uint32 TimeVar,RTC_Time * Time)
{
	uint16 TY = 0, TM = 1, TD = 0;
	uint16 Num4Y,NumY;
	uint32 OffSec, Off4Y = 0;
	uint16 i;
	uint32 NumDay;
	//RTC_Time Time;
	//u16 THH = 0, TMM = 0, TSS = 0;

	Time->Week = ((TimeVar/SecsPerDay)+ CurrWeekOffset )%7+1;	//以起始日期作为参考

        Num4Y = TimeVar/SecsPerFourYear;		//偏移 n 个4年
        OffSec = TimeVar%SecsPerFourYear;		//在 4 年 内的偏移

	i=1;
	while(OffSec >= Year_Secs_Accu[i++])		//在4年中的哪一年
		Off4Y++;
	
	NumY = Num4Y*4 + Off4Y;					//计算出是哪一年
	TY = START_YEAR + NumY;							//加上 START_YEAR 年的偏移量
	OffSec = OffSec - Year_Secs_Accu[i-2];	//在一年内的偏移

	i=0;
	if(TY%4)	//平年
	{
		while(OffSec >= Month_Secs_Accu_C[i++]);
		TM = i-1;									//
		OffSec = OffSec - Month_Secs_Accu_C[i-2];
	}
	else		//闰年
	{
		while(OffSec >= Month_Secs_Accu_L[i++]);
		TM = i-1;
		OffSec = OffSec - Month_Secs_Accu_L[i-2];
	}

	NumDay = OffSec/SecsPerDay;
	OffSec = OffSec%SecsPerDay;
	TD = NumDay+1;

	Time->Year = TY;
	Time->Month = TM;
	Time->Day = TD;
	Time->Hour = OffSec/3600;
	Time->Minute = (OffSec % 3600)/60;
	Time->Second = (OffSec % 3600)% 60;
	
}


uint32 RTC_GetGloablTime(RTC_Time *pTimes)		//获取时间
{
	uint32 GloablTimeSecond;
	//PWR_BackupAccessCmd(ENABLE);
	GloablTimeSecond=osal_getClock();
	//PWR_BackupAccessCmd(DISABLE);
	SecondsToRTCTime(GloablTimeSecond,pTimes);	//初始化全局时钟	
    return GloablTimeSecond;
}

void RTC_Set_RTC_Time(RTC_Time *pTimes)		//更改时间
{
	RTC_SetDateTime(pTimes->Year,pTimes->Month,pTimes->Day,pTimes->Hour,pTimes->Minute,pTimes->Second,pTimes->Week);
}

uint8 SetSystemDateTimePara(RTC_Time * pSdt,uint16 year,uint16 mon,uint16 day,uint16 hour,uint16 min,uint16 sec)
{
    uint16 days;
    if ( year <= 1980 || year >= 2100 )
            return FALSE;
    pSdt->Year = year;
    if ( mon < 1 || mon > 12 )
            return FALSE;
    pSdt->Month = mon;
    days = MonthDaysArray[mon-1];
    if ( mon == 2 )
    {
            if ( ( year % 4 ) == 0 && ( year % 100 ) != 0 || ( year % 400 ) == 0 )
            {
                    days++;
            }
    }
    if ( day < 1 || day > days )
            return FALSE;
    pSdt->Day = day;
    if ( hour >= 24 )
		return FALSE;
    pSdt->Hour = hour;
    if ( min >= 60 )
		return FALSE;
    pSdt->Minute = min;
    if ( sec >= 60 )
		return FALSE;
    pSdt->Second = sec;
    return TRUE;
}

uint8 CheckTimePara(RTC_Time * pSdt)
{
    uint16 days;
    if ( pSdt->Year < START_YEAR || pSdt->Year >= 2090 )
            return FALSE;

    if ( pSdt->Month < 1 || pSdt->Month > 12 )
            return FALSE;

    days = MonthDaysArray[pSdt->Month-1];
    if ( pSdt->Month == 2 )
    {
            if ( ( pSdt->Year % 4 ) == 0 && ( pSdt->Year % 100 ) != 0 || ( pSdt->Year % 400 ) == 0 )
            {
                    days++;
            }
    }
    if ( pSdt->Day < 1 || pSdt->Day > days )
            return FALSE;

    if ( pSdt->Week < 1 || pSdt->Week > 7 )
            return FALSE;

    if ( pSdt->Hour >= 24 )
		return FALSE;
     
    if ( pSdt->Minute >= 60 )
		return FALSE;

    if ( pSdt->Second >= 60 )
		return FALSE;
     
    return TRUE;
}

uint16 ConvTimeStr2SysTime(char * pData,RTC_Time * pTime)
{
	uint16 year,mon,day,hour,min,sec;
	year = (pData[0]-0x30)*1000+(pData[1]-0x30)*100+(pData[2]-0x30)*10+(pData[3]-0x30);
	mon = (pData[4]-0x30)*10+(pData[5]-0x30);
	day = (pData[6]-0x30)*10+(pData[7]-0x30);
	hour = (pData[8]-0x30)*10+(pData[9]-0x30);
	min = (pData[10]-0x30)*10+(pData[11]-0x30);
	sec = (pData[12]-0x30)*10+(pData[13]-0x30);
	if ( SetSystemDateTimePara(pTime,year,mon,day,hour,min,sec) )
		return TRUE;
	return FALSE;
}

void ConvSecondLenth2SysTime(uint32 seclen,RTC_Time * pTime)
{
	uint32 temp;
	temp = seclen / 3600;
	pTime->Day = temp / 24;
	pTime->Hour = temp % 24;
	temp = seclen % 3600;
	pTime->Minute = temp / 60 ;
	pTime->Second = temp % 60 ;
}

static const unsigned short crc_tab[]=
{ /* CRCóàê?±í */
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
};

unsigned char CheckCRC( unsigned char  * Buffer, unsigned short lenth )
{
    unsigned short crc=0,rt=0;
	unsigned char da,*ptr;

	crc=0;
    ptr = Buffer;
	while(lenth--!=0)
	{
		da=(unsigned char) (crc/256); /* ò?8???t????êyμ?D?ê??Y′?CRCμ???8?? */
		crc<<=8; /* ×óò?8??￡??àμ±óúCRCμ?μí8??3?ò? */
		crc^=crc_tab[da^*ptr]; /* ??8??oíμ±?°×??ú?à?óoó?ù2é±í?óCRC ￡??ù?óé?ò??°μ?CRC */
		ptr++;
	}

    if ( *ptr != ( crc & 0x00ff ) || *(ptr+1) != ( ( crc & 0xff00 ) >> 8 ) )
        rt = 1; //err
	*ptr = ( crc & 0x00ff );
	*(ptr+1) = ( ( crc & 0xff00 ) >> 8 );
	return rt;
}

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
