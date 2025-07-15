/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#include "stdio.h"
   
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OSAL_snv.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hci.h"
#include "peripheral.h"
#include "npi.h"
#include "att.h"
#include "gatt.h"

#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "string.h"
#include "hal_shiftout.h"
#include "RN8209D.h"   
#include "simpleBLEPeripheral.h"
#include "AppUserProcess.h"

#include "HT1621_drv.h"
#include "hal_timer.h"
#include "hal_dma.h"
#include "OSAL_Clock.h"
//#include "simpleBLE.h"
#include "ioCC2540.h"

#define   RedLed        P1_2
#define   GreenLed      P1_1
#define   BlueLed       P1_0
#define   LineN         P2_2
#define   LineL         P2_1

#define   Light100      1200//光亮度
#define   Light50       800//
#define   Light25       500//

#define    Light100Per 14 //光亮度上升百分比 

#define    Light50Per  3   

#define    Light25Per  5   

#define   Light10Per  8  

uint8 AppUserProcess_TaskID;   // Task ID for internal task/event processing
static void AppUser_ProcessOSALMsg( osal_event_hdr_t *pMsg );

SYS_PARM	Curr_Sys_Parm;
SYS_PARM1	Curr_Sys_Parm1;
SYS_PARM2	Curr_Sys_Parm2;

uint8 ConnectUser = 0;

bool b_Get_BT1_Commond=0,b_GetNewData=0,b_DateTimeUpdataed=0;

uint8 DATA ChaKongPwOn=0;

uint8 ChaKongHave=0,ChaKongLockOff=0;
//bool b_Bt_Restart;
bool b_Refresh80;
bool b_Refresh81;
bool b_Refresh82;

bool Send_State_Flag = 0;


uint8 HostCom_Loop_Buf[LOOP_BUF_DATA_SIZE];
uint8 HostCom_ptr_out,HostCom_ptr_in;
uint8 HostComBuf[HOST_COM_DATA_SIZE*2+8];

uint8 HostCom_Buf1[19];
uint8 HostCom_Buf2[19];
uint8 HostCom_Buf_Temp[19];

uint8 Bt_ConnectingTimer = 0;
uint8 LedTimer = 0;

bool AutoSendFlag = 0;
bool SendStateFlag = 0; 
bool Open_Led_Flag = 1 ;
bool debug=0;//1;//
uint8   PowerOverAlarm = 0;

uint8 SendBackBuffer[SEND_LOOP_BUF_SIZE];

unsigned char CurrentID_ptr,Current_CallID_cnt,LockTimer,InMenuStep,Current_NewEvent_cnt;

RTC_Time Event_CreateTime;
RTC_Time Curr_RTC_time;

uint16 LtChnl[4],LtChnl5;
uint32 LtChnlAdd[4],LtChnl5Add;
uint32 LtChnlMax[4];
uint32 LtChnlMin[4];
//void simpleBLE_NpiSerialCallback( uint8 port, uint8 events );
//void CheckHostCmd(unsigned char BTn,unsigned char * cmdbuf);
void CheckHostCmd();
void Check_HostCom_BT1_Buf(void);
uint8 Check_HostCom_Buf(void);

extern void SendState1( void);//向App发送开关 插孔有无用电器 电压 定时周期等
extern void SendState2( void );//向App发送能量功率
extern void SendingReturnBuf(void);
uint8 converASC2HEX(uint8 * input);

uint8 checkBCC(uint8* data_buffer,uint8 Nb_bytes );

uint32 RTCTimeToSeconds(RTC_Time * Time);

uint8 GetMaxDayInMonth(uint16 year,uint8 mon);

uint8 CheckTimePara(RTC_Time * pSdt);

void RTC_Set_RTC_Time(RTC_Time *pTimes);

uint32 RTC_GetGloablTime(RTC_Time *pTimes);

void SecondsToRTCTime(uint32 TimeVar,RTC_Time * Time);

uint8  ConverPackedTelNumToStr(unsigned char * buf,unsigned char maxlen,unsigned char * pData);

void  UpdateTimeAdd(void);


bool b_BT1_InitOK = 0,b_InitDeviceOK = 0,b_APP_User1_OK = 0;

uint8 BT1_LocalDevAddr[6];

extern uint16 RefreshSysParaDelay,gapConnHandle,LedFlashCnt;

extern uint8 RetryCount;

uint8 LineDetectTimer = 0;
uint8 P2_0_LowTimer = 0;
uint16 SendStateTimer = 0;
uint8 SendStateTimerSave = 0;
uint8 ClearTimer = 0;
uint8 Timer25ms = 0;
uint8 Timer300ms = 0;
bool LineDetectFlag = 0;

bool b_SendingState = 0;
uint8 AutoControlFlag;
bool b_Clear_Plan_time ;


extern uint8 ReturnBuf[22];
extern gaprole_States_t gapProfileState;   // 从机连接状态

extern uint8 AbsTime_cout ;
extern uint8 CycleTime_cout ;

extern uint16  AbsTimeAdd ;   
extern uint16  CycleTimeAdd ; 

uint8 SendStateOrder;
uint8 LtChnlSaveTimer = 0;
uint8 LtChnlCnt = 0;

uint8 LedFlashSeq = 0;
uint8 Plan_time_cout=0;
uint8 DATA PowerOffcheckcnt=0;
uint8 P2_0_HighCnt;
//P0_5_LowCnt,P0_6_LowCnt;
bool P2_0_LowFlag=0,P2_0_LongFlag=0,CHK0_LOW_flag=0,CHK1_LOW_flag=0,CHK_PowerOff_flag=0,Led_Changed_flag=0,Led_Flash_flag=0,Led_Flash_updown=0;

//uint8 DATA Ext_bits _at_ 0x21;
//BIT CHK1_LOW_checked = 0x21^1;
bool CHK0_LOW_checked = 0,CHK1_LOW_checked = 0, DelaySwitchFlag = 0, RetryCheckLN_Flag = 0;

uint8 GuangUpIsOkFlag = 0;
uint8 GuangDownIsOkFlag = 0;
uint16 P2_0_LowCnt = 0;
extern void CheckHostCmd(void);
extern void FindCycleAbsCout(void);
extern void simpleBLE_NPI_init(void);
void PWM_RGB(uint8 red, uint8 green, uint8 blue);
uint16 GetGongLvAverage(uint8 index,uint8 count);
void LedSetVault(uint8 led,uint8 red);
//uint16  GetDaylyPower(uint8 index,uint8 day);

uint8 LedColorSet[3][3];
uint8 IntoIdleDelay = 250;

bool simpleBLE_IfConnected()
{
    {
        return (gapProfileState == GAPROLE_CONNECTED);
    }
}
void Init_Watchdog(void) 
{ 
    WDCTL = 0x00;       //打开IDLE才能设置看门狗
    WDCTL |= 0x08;      //定时器间隔选择,间隔一秒
}

void FeetDog(void) 
{ 
    WDCTL = 0xa0;       //清除定时器。当0xA跟随0x5写到这些位，定时器被清除
    WDCTL = 0x50; 
}

void Common_Init( void )
{
     P2INP = 0xe0;

     P2DIR|=BV(1);//DBG_DD    (CTRL5)
     LineL=0;
         
     P2DIR|=BV(2);//DBG_DC    (CTRL4)
     LineN=0;

     P1SEL&=~BV(3);
     P1DIR|=BV(3); //CTRL3
     P1_3 = 0;
     
     P1SEL&=~BV(4);
     P1DIR|=BV(4);//CTRL2
     P1_4 = 0;

     P2SEL&=~BV(1);
     P2DIR|=BV(3);//CTRL1    
     P2_3=0;
         
     P2SEL&=~BV(2);
     P2DIR|=BV(4);//CTRL0   
     P2_4=0;
     
     P0SEL |= BV(0);//AD0
     P0DIR &= ~BV(0);
     P0INP |= BV(0);  /* Configure GPIO tri-state. */  
     
     P0SEL |= BV(1);//AD1
     P0DIR &= ~BV(1);
     P0INP |= BV(1);  /* Configure GPIO tri-state. */  
     
     P0SEL |= BV(2);//AD2
     P0DIR &= ~BV(2);
     P0INP |= BV(2);  /* Configure GPIO tri-state. */  
     
     
     P0SEL |= BV(3);//AD3
     P0DIR &= ~BV(3);
     P0INP |= BV(3);  /* Configure GPIO tri-state. */  
     
     P0SEL |= BV(4);//AD4
     P0DIR &= ~BV(4);
     P0INP |= BV(4);  /* Configure GPIO tri-state. */  
     
     P0SEL &= ~BV(5);//CHK0
     P0DIR &= ~BV(5);
     P0INP |= BV(5);  /* Configure GPIO tri-state. */ 
     
     P0SEL &= ~BV(6);//CHK1
     P0DIR &= ~BV(6);
     P0INP |= BV(6);  /* Configure GPIO tri-state. */ 
     
     P0SEL &= ~BV(7);//CHK2
     P0DIR &= ~BV(7);
     P0INP |= BV(7);  /* Configure GPIO tri-state. */ 
     
     
     P1SEL &= ~BV(0);
     P1DIR |= BV(0);
     BlueLed = 1;

     P1SEL &= ~BV(1);
     P1DIR |= BV(1);
     GreenLed = 1;
     
     P1SEL &= ~BV(2);
     P1DIR |= BV(2);
     RedLed = 1;
     
     //P1INP &=~0x02;
     P1SEL |= 0x07;
     PERCFG |= 0x40;        //PWM
     
     P1SEL|=BV(5);
     P1DIR|=~BV(5);//TX1     
          
     P1SEL|= BV(7);
     P1DIR &= ~BV(7);//RX2
     P1INP |= BV(7);  /* Configure GPIO tri-state. */ 
     
     P1SEL|=BV(6);
     P1DIR |=~BV(6);//TX2

     P2SEL&=~BV(0);//KEY
     P2DIR&=~BV(0);
   
}

void RN8209_init()
{
     LineDetectFlag = 1;
     b_Clear_Plan_time = 1;
     AutoControlFlag = 0;
     fnScomPk_Init(SCOM_PORT_RN8209); 


}
/****************************************************************************
* 名    称: DelayMS()
* 功    能: 以毫秒为单位延时 16M时约为535,系统时钟不修改默认为16M
* 入口参数: msec 延时参数，值越大，延时越久
* 出口参数: 无
****************************************************************************/
void DelayMS(uint16 msec)
{ 
    uint16 i,j;
    
    for (i=0; i<msec; i++)
        for (j=0; j<1070; j++);
}


void halAssertHandler(int line,const char * func)
{
    //char assertbuf[64];
    //sprintf(assertbuf,"Assert line = %d in Func = %s File = %s\r\n",line,func,file);
    //Debug_output(assertbuf);
//    Debug_output("Assert line = %d in Func = %s \r\n",line,func);
}


unsigned char timercnt = 0;
extern  char tempStringBuf[32]; 
uint16 CheckVolume;

void Serial_Delay(int times)
{
  while(times--)
  {
      int i=0;
      for(i=6000;i>0;i--)
      {
    	  asm("nop");
      }
  }
}

void Serial_Delayus(int times)
{
  while(times--)
  {
      int i=0;
      for(i=6;i>0;i--)
      {
    	  asm("nop");
      }
  }
}


uint32 str2Num(uint8* numStr, uint8 iLength)
{
    uint8 i = 0;
    int32 rtnInt = 0;
 
    /* 
          为代码简单，在确定输入的字符串都是数字的
          情况下，此处未做检查，否则要检查
          numStr[i] - '0'是否在[0, 9]这个区间内
    */
    for(; i < iLength && numStr[i] != '\0'; ++i)
        rtnInt = rtnInt * 10 + (numStr[i] - '0');    
 
    return rtnInt;
}

/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
#define B_ADDR_STR_LEN                        15

  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}

void SetDefaultParm(uint8 flag)	//è???è??μ	
{
        SYS_PARM *pSys_parm = &Curr_Sys_Parm;
        SYS_PARM1 *pSys_parm1 = &Curr_Sys_Parm1;
        SYS_PARM2 *pSys_parm2 = &Curr_Sys_Parm2;
        if( flag == 2)
        {
          memset((uint8*)pSys_parm2,0,sizeof(SYS_PARM2));
        }
        else
        if( flag == 1)
        {
          memset((uint8*)pSys_parm1,0,sizeof(SYS_PARM1));
        }
        else
        {  
          memset((uint8*)pSys_parm,0,sizeof(SYS_PARM));

#if defined HAL_IMAGE_A
          pSys_parm->VerCode = 0x01;
          sprintf((char*)pSys_parm->devName,"nm_outlet_A");
          
#else
          pSys_parm->VerCode = 0x02;
          sprintf((char*)pSys_parm->devName,"nm_outlet_B");
          
#endif
          
          pSys_parm->ParmLenth = sizeof(SYS_PARM) - 6;	//×?′ó510×?
          pSys_parm->ForceSwitch_Flag = 1;//默认
          
          sprintf((char*)pSys_parm->EncryptBuf,"纽曼蓝牙电话");
          
          
          pSys_parm->PwUpMul = 8;
          pSys_parm->PwDnMul = 3;
          pSys_parm->KUrms5 = 8474;//8552;
          
          pSys_parm->PwOffSet[0] = 500;
          pSys_parm->PwOffSet[1] = 500;
          pSys_parm->PwOffSet[2] = 500;
          pSys_parm->PwOffSet[3] = 500;
          pSys_parm->KPrms[3] = 23000;//23500;
          pSys_parm->KPrms[2] = 24500;//25500;
          pSys_parm->KPrms[1] = 24500;//24500;
          pSys_parm->KPrms[0] = 24000;//23500;
          pSys_parm->ChaKongCtrl = 0x0f;

//          CheckCRC( (uint8 *)&pSys_parm->EncryptBuf, pSys_parm->ParmLenth );
        }
}



void AppUser_Init( uint8 task_id )
{

    Dl645Front.PriData.Flag = RN8209RST;
    
    AppUserProcess_TaskID = task_id;

    osal_set_event( AppUserProcess_TaskID, AUP_START_DEVICE_EVT );
    
    
}

void ChaKongTimerCrl(uint8 Plan)
{
    uint8 temp;
    if( ( Plan & 0x70 )== 0x30 )
    {
        AutoControlFlag |= Plan & 0x0f;
        Curr_Sys_Parm.AutoControlEable |= Plan & 0x0f;  //定时开该功能
        RefreshSysParaDelay = REFRESHTIME ;// 250ms
        b_Refresh80 = 1;
    }
    else
    if( ( Plan & 0x70 )== 0x40 )
    {
        temp = Plan | 0xf0;
        AutoControlFlag &= ~temp;
        Curr_Sys_Parm.AutoControlEable &= ~temp;        //定时关闭该功能
        RefreshSysParaDelay = REFRESHTIME ;// 250ms
        b_Refresh80 = 1;
    }
    else
    if( ( Plan & 0x70 )== 0x10 )
    {
        Curr_Sys_Parm.ChaKongCtrl |= Plan & 0x0f;
        AutoSendFlag=1;
    }
    else
    if( ( Plan & 0x70 )== 0x20 )
    {
        temp = Plan | 0xf0;
        Curr_Sys_Parm.ChaKongCtrl &= ~temp;
        ChaKongHave &= ~temp;      //关闭后强制标定为无负载
        AutoSendFlag=1;
    }
}


void ChaKongTimerCmp( void ) 
{
  uint8 i,offset;
  uint16 temp16;
  bool b_AbsTime_Dec;
  
  uint32 Plan_time_Temp;
  
  Plan_time_Temp = RTC_GetGloablTime(&Curr_RTC_time);		//获取时间
  Curr_Sys_Parm.SecondOfRtc = Plan_time_Temp;
  Plan_time_Temp++;//保持前一天日期
  if( ( Plan_time_Temp % 3600) == 0 )// (3600)  
  {
      if( ( Plan_time_Temp % 86400) == 0 )//86400 (3600*24)
      {
        for( i = 0; i < 8; i++)
        {
           if( i != ConnectUser ) 
           {
               if(  Curr_Sys_Parm.UserPriority[i] > 2)
               {
                 Curr_Sys_Parm.UserPriority[i] -=2;
               }
           }
        }
        if ( b_DateTimeUpdataed )
        {
            offset = Curr_RTC_time.Day - 1;
            for (i=0;i<4;i++)
            {
                temp16 = (Curr_Sys_Parm.StartNengLiang[i] + 50)/100;  //四舍五入
                temp16 &= 0x03FF;
                Curr_Sys_Parm2.DaylySaveNL[offset][i] = Curr_RTC_time.Month;
                Curr_Sys_Parm2.DaylySaveNL[offset][i] <<= 12;
                Curr_Sys_Parm2.DaylySaveNL[offset][i] += temp16;
                
                Curr_Sys_Parm.StartNengLiang[i] = 0;
            }
            RefreshSysParaDelay = REFRESHTIME ;// 250ms
            b_Refresh82 = 1;
        }
      }
      if( CHK0_LOW_flag == 0 &&  CHK1_LOW_flag == 0 
         || CHK0_LOW_flag == 1 &&  CHK1_LOW_flag == 1  )	//缺地线,按小时保存
      {
          RefreshSysParaDelay = REFRESHTIME ;// 250ms
          b_Refresh80 = 1;
      }
  }
  
  
  for( i = 0; i <  MAX_SCH_COUNT; i++ )
  {
      if ( Curr_Sys_Parm1.Plan_time[i].Plan & 0x80 )
      {//开启有效
        if ( Curr_Sys_Parm1.Plan_time[i].Data.byteTime[3] & 0x80 )
        {//周期定时
            Plan_time_Temp = (uint32)Curr_RTC_time.Hour*3600;
            Plan_time_Temp += (uint16)Curr_RTC_time.Minute*60;
            Plan_time_Temp += Curr_RTC_time.Second;
            if( ( Curr_Sys_Parm1.Plan_time[i].Data.Time & 0x01FFFF ) == Plan_time_Temp )
            {
                if ( Curr_Sys_Parm1.Plan_time[i].Data.byteTime[2] & (0x02 << (Curr_RTC_time.Week - 1) ) )
                    ChaKongTimerCrl(Curr_Sys_Parm1.Plan_time[i].Plan);
            }
        }
        else
        {
            if( Curr_Sys_Parm1.Plan_time[i].Data.Time > 0 )
            {
                if ( Curr_Sys_Parm1.Plan_time[i].Data.Time <= Curr_Sys_Parm.SecondOfRtc )
                {
                    ChaKongTimerCrl(Curr_Sys_Parm1.Plan_time[i].Plan);  
                    Curr_Sys_Parm1.Plan_time[i].Plan &= 0x7F;
                    b_AbsTime_Dec = 1;
                }
            }
        }
      }
  }
  if(b_AbsTime_Dec == 1)
  {
      UpdateTimeAdd();
      b_AbsTime_Dec = 0;
      RefreshSysParaDelay = REFRESHTIME ;// 2500ms
      b_Refresh81 = 1;
  }  
}

void ChaKongIsHave( void )
{
    uint8 i,maskbit;
    for (i=0;i<4;i++)
    {
        maskbit = ( 0x01 << i );
        if ( ( DianUpIsOkFlag & maskbit ) && ( GuangDownIsOkFlag & maskbit ) )
        {  
          ChaKongHave |= maskbit;                         
          DianUpIsOkFlag &= ~maskbit;
          GuangDownIsOkFlag &=~maskbit;
          AutoSendFlag = 1;  
        }
        if( ( DianDownIsOkFlag & maskbit ) && ( GuangUpIsOkFlag & maskbit ) )
        {  
          ChaKongLockOff &= ~maskbit;   //清除由于超负荷而关闭的插座不能再次自动开启的标志
          ChaKongHave &= ~maskbit;      //电和光条件同时满足清除有负载标志
          DianDownIsOkFlag &= ~maskbit;
          GuangUpIsOkFlag &= ~maskbit;
          AutoSendFlag = 1;        
        }
    }
}

void UpdatePwOnIO(void)
{
    uint8 i;
    for (i=0;i<4;i++)
    {
        switch(i)
        {
            case 0: ChaKong1PwOn = (ChaKongPwOn & 0x01) ? 1:0;break;
            case 1: ChaKong2PwOn = (ChaKongPwOn & 0x02) ? 1:0;break;
            case 2: ChaKong3PwOn = (ChaKongPwOn & 0x04) ? 1:0;break;
            case 3: ChaKong4PwOn = (ChaKongPwOn & 0x08) ? 1:0;break;
        }
    }
}

void  ChaKongParmInit(uint8 i)
{
    uint8 j;
    
    GongLvListIndex[i] = 0;
    for (j=0;j<GONGLV_LIST_SIZE;j++)
        GongLvList[i][j] = 0;
    
    GongLvSave[i] = 0;
}

void RuntimeParaInit()        
{
    uint8 i;    
    for (i=0;i<4;i++)
    {
      LtChnl[i]=0;
      LtChnlAdd[i]=0;
      ChaKongParmInit(i);
    }
    
    LtChnl5 = 0;
    LtChnl5Add = 0;
}

void ChaKongPwOnCtrl( void )
{
    uint8 i,maskbit,temp;
    temp = ChaKongPwOn;
    for (i=0;i<4;i++)
    {
        maskbit = ( 0x01 << i );
        if( ( ChaKongHave & maskbit )//插孔有用电器插入
          && ( Curr_Sys_Parm.ChaKongCtrl & maskbit ) )//开启命令
        {
            if ( ( ChaKongPwOn & maskbit ) == 0 && ( ( ChaKongLockOff & maskbit ) == 0 ) )    //由于超负荷而关闭的插座不能再次自动开启
            {
                 ChaKongPwOn |= maskbit;      //开启（通电）
                 Led_Changed_flag = 0;
            }
        }
        else
        {
          if( ChaKongPwOn & maskbit )//插孔已经通电
          {
              ChaKongPwOn &= ~maskbit;      //关闭（断电）
              ChaKongParmInit(i);
              AutoControlFlag &= ~maskbit;      //被关闭的通道主控检测中则关闭检测
              PowerOverAlarm &= ~maskbit;              
              /*
              ChaKongHave &= ~maskbit;      //关闭后强制标定为无负载？
              DianUpIsOkFlag &= ~maskbit;
              GuangDownIsOkFlag &= ~maskbit;
              DianDownIsOkFlag &= ~maskbit;
              GuangUpIsOkFlag &= ~maskbit;
              */
              Led_Changed_flag = 0;
          }
        }
    }
    if ( temp != ChaKongPwOn )
    {
        //UpdatePwOnIO();
        DelaySwitchFlag = 1;
        AutoSendFlag = 1;//向APP传状态
    }
}

void AutoPowerDownOpen( void )
{
    uint8 i,maskbit;
    uint16 gonglv;
    for( i = 0;i < 4;i++ )//Curr_Sys_Parm.Auto_PwrDn_cout
    {      
      maskbit = ( 0x01 << i );
      if  ( Curr_Sys_Parm.AutoControlEable & maskbit ) 
      {
        if( Curr_Sys_Parm.Auto_PowerDown[i].Control > 0 )
        {   //有控制需求
            gonglv = GetGongLvAverage(i,3);
            if ( AutoControlFlag & maskbit )
            {
                if( ( (uint32)gonglv*100 ) < ((uint32)GongLvSave[i] * Curr_Sys_Parm.Auto_PowerDown[i].Percent) )
                {
                    Curr_Sys_Parm.ChaKongCtrl &= ~Curr_Sys_Parm.Auto_PowerDown[i].Control;  //准备关闭
                    ChaKongHave &= ~maskbit;      //关闭后强制标定为无负载
                    AutoControlFlag &= ~maskbit;    
                    AutoSendFlag=1;
                }
            }
            else
            if ( ( ChaKongPwOn & maskbit ) &&  gonglv > 5 )//插孔已经通电，且功率大于5W
            {
                AutoControlFlag |= maskbit;
            }
        }
      }
    }
}

uint8 ledtest=0;
void KeyDetect( void )      //25ms
{
  
  if(P2_0==0)//按键按下
  {
      P2_0_LowCnt++;
      if(P2_0_LowCnt > 60 && P2_0_LongFlag == 0)    //去抖
      {     //长按1.5 S
          P2_0_LongFlag = 1; 
          P2_0_LowFlag = 0;
          P2_0_LowCnt = 0;
          Led_Flash_flag = 0;          
          if(Curr_Sys_Parm.ForceSwitch_Flag == 1)
          {
              ChaKong1PwOn = 0;
              ChaKong2PwOn = 0;
              ChaKong3PwOn = 0;
              ChaKong4PwOn = 0;     
              ChaKongPwOn = 0;
              ChaKongLockOff = 0;
              RuntimeParaInit();
              Curr_Sys_Parm.ChaKongCtrl=0;
              Curr_Sys_Parm.ForceSwitch_Flag = 0;
              Led_Changed_flag = 0;
          }
          else
          {
              Curr_Sys_Parm.ForceSwitch_Flag = 1;
              Curr_Sys_Parm.ChaKongCtrl= 0x0f;
          }
          RefreshSysParaDelay = REFRESHTIME ;// 2500ms
          b_Refresh80=1;
          AutoSendFlag = 1;             
          Led_Changed_flag = 0;
      }
      P2_0_HighCnt=0;
  }
  else
  {
      b_Clear_Plan_time = 0;
      P2_0_HighCnt++;
      if( P2_0_HighCnt >= 3 )       //100ms去抖
      {
        P2_0_HighCnt=0;
        if( P2_0_LowCnt >= 3 )      //100ms去抖
        {
           P2_0_LowCnt=0;
           CHK_PowerOff_flag = 0;
           Led_Changed_flag = 0;
           Led_Flash_flag = 0;
           
           if( P2_0_LongFlag == 1)
           {
              P2_0_LongFlag = 0;
              //放弃当次抬起
           }
           else          
           {
              if( P2_0_LowFlag == 0) 
              {
                P2_0_LowFlag = 1;   //第一次点击
                if( Open_Led_Flag == 0 )
                {
                    Open_Led_Flag = 1;
                    Curr_Sys_Parm.Auto_Led_Flag = 0;
                }
                if( Bt_ConnectingTimer > 0 )
                {
                    Bt_ConnectingTimer = 0;
                    P2_0_LowFlag = 0;
                }
              }
              else
              {
                 P2_0_LowFlag = 0;
                 P2_0_LowTimer = 0;
                 if( b_APP_User1_OK == 0) //未正确连接
                 {
                     Bt_ConnectingTimer = 30;   //允许连接
                     PWM_RGB(0, 0, 0);
                 }
              }
           }
        }
      }
  }
}

void LineDetect( void )     //5ms
{
  if(P0_5==0)//CHK0 LOW
  {
      //P0_5_LowCnt++;
      //if(P0_5_LowCnt>3)//去抖
      {//	正常状态
        CHK0_LOW_flag=1;
        //P0_5_LowCnt=0;
        CHK_PowerOff_flag = 0;
        PowerOffcheckcnt = 10;  //50ms        
      }
  }
  if(P0_6==0)//CHK1 LOW
  {
      //P0_6_LowCnt++;
      //if(P0_6_LowCnt>3)//去抖
      {//火线零线反接绿灯闪
        CHK1_LOW_flag=1;        
        //P0_6_LowCnt=0;
        CHK_PowerOff_flag = 0;
        PowerOffcheckcnt = 10;  //50ms        
      }
  }
  
  if ( PowerOffcheckcnt > 0 )
  {
      PowerOffcheckcnt--;
      if ( PowerOffcheckcnt == 0 )
      {
          CHK0_LOW_flag = 0;
          CHK1_LOW_flag = 0;
          if ( osal_snv_write(0x80, sizeof(SYS_PARM), &Curr_Sys_Parm) == SUCCESS )
          {
              if ( b_Refresh81 )//RefreshSysParaDelay > 0 )
              {
                  osal_snv_write(0x81, sizeof(SYS_PARM1), &Curr_Sys_Parm1);  
                  RefreshSysParaDelay = 0;
                  b_Refresh81 = 0;
              }
              if ( b_Refresh82 )
              {
                  osal_snv_write(0x82, sizeof(SYS_PARM2), &Curr_Sys_Parm2);  
                  RefreshSysParaDelay = 0;
                  b_Refresh82 = 0;
              }
              b_Refresh80 = 0;
              CHK_PowerOff_flag = 1;
              Led_Changed_flag = 0;
          }
      }
  }
}

void LedShow( void )
{
    if ( CHK_PowerOff_flag )
    {
        //if ( Led_Changed_flag == 0 )
        {
            PWM_RGB(32, 0, 0);     //突然断电 红灯常亮
            //Led_Changed_flag = 1;
        }
    }
    else
    if ( Open_Led_Flag == 0 )
    {
        if ( Led_Changed_flag == 0 )
        {
            Led_Flash_flag = 0;
            PWM_RGB(0,0,0);     //关闭指示灯
        }
        Led_Changed_flag = 1;
    }
    else      
    if(Curr_Sys_Parm.ForceSwitch_Flag == 0)
    {
        if ( Led_Changed_flag == 0 )
        {
            //PWM_RGB(30,30,20);
            Led_Flash_flag = 0;
            PWM_RGB(LedColorSet[0][0],LedColorSet[0][1],LedColorSet[0][2]);
        }
        Led_Changed_flag = 1;
    }
    else
    if( ChaKongPwOn == 0 )
    {//没有通电（常态）
        if ( Led_Changed_flag == 0 )
        {
            Led_Flash_flag = 0;
            if( simpleBLE_IfConnected() )//b_APP_User1_OK )//连上手机
            {
                if ( IntoIdleDelay > 0 )
                {
                    PWM_RGB(0, 0, 128);
                }
                else
                {
                    Led_Flash_flag = 1;
                    LedFlashCnt = 48*3;
                    Led_Flash_updown = 1;
                    PWM_RGB(0, 0, 128);
                }
            }
            else
            if( CHK0_LOW_flag == 1 &&  CHK1_LOW_flag == 0)  //	正常状态  绿灯常亮
            {
                PWM_RGB(0, 128, 0);
            }
            else
            if( CHK0_LOW_flag == 0 &&  CHK1_LOW_flag == 1)//火线零线反接  橙色常亮
            {
                LineN = 1;
                LineL = 1;
                //PWM_RGB(128, 128, 0);
                PWM_RGB(LedColorSet[1][0],LedColorSet[1][1],LedColorSet[1][2]);
            }
            else
            //if( CHK0_LOW_flag == 0 &&  CHK1_LOW_flag == 0 )	//缺地线 红灯常亮															
            {
                PWM_RGB(128, 0, 0);
            }
            Led_Changed_flag = 1;
        }
    }
    else
    if ( PowerOverAlarm == 0 )
    {
        LedTimer ++ ;
        if(LedTimer > CYCTIME)
        {
            LedTimer = 0;
            LedFlashSeq++;
            if( LedFlashSeq == 1 ||  LedFlashSeq > OFFTIME + SHORTTIME + OFFTIME + LONGTIME )
            {
                Led_Flash_flag = 0;
                PWM_RGB(0, 0, 0);
                LedFlashSeq = 1;
            }
            else
            if( LedFlashSeq == OFFTIME )
            {
                //PWM_RGB(128, 0, 128);
                PWM_RGB(LedColorSet[2][0],LedColorSet[2][1],LedColorSet[2][2]);
            }
            else
            if( LedFlashSeq == OFFTIME + SHORTTIME )
            {
                PWM_RGB(0, 0, 0);
            }
            else
            if( LedFlashSeq == OFFTIME + SHORTTIME + OFFTIME )
            {
                if( simpleBLE_IfConnected() )//b_APP_User1_OK ) //连上手机
                    PWM_RGB(0, 0, 128);
                else
                    PWM_RGB(0, 128, 0);
            }
            else
            if( LedFlashSeq == OFFTIME + SHORTTIME + OFFTIME + LONGTIME )
            {
                LedFlashSeq = 0;
                Led_Changed_flag = 0;
            }
        }
    }
    else
    {
        LedTimer ++ ;
        if(LedTimer > CYCTIME)
        {
            LedTimer = 0;
            LedFlashSeq++;
            if( LedFlashSeq == 1 || LedFlashSeq > OFFTIME + SHORTTIME + OFFTIME + LONGTIME + OFFTIME + SHORTTIME + OFFTIME + LONGTIME )
            {
                Led_Flash_flag = 0;
                PWM_RGB(0, 0, 0);
                LedFlashSeq = 1;
            }
            else
            if( LedFlashSeq == OFFTIME + SHORTTIME + OFFTIME )
            {
                if( simpleBLE_IfConnected() )//b_APP_User1_OK )//连上手机
                    PWM_RGB(0, 0, 128);
                else
                    PWM_RGB(128, 0, 0);
            }
            else
            if( LedFlashSeq == OFFTIME + SHORTTIME + OFFTIME + LONGTIME )
            {
                PWM_RGB(0, 0, 0);
            }
            else
            if( LedFlashSeq == OFFTIME + SHORTTIME + OFFTIME + LONGTIME + OFFTIME + SHORTTIME + OFFTIME)
            {
                PWM_RGB(128, 0, 0);
            }
            else
            if( LedFlashSeq == OFFTIME + SHORTTIME + OFFTIME + LONGTIME + OFFTIME + SHORTTIME + OFFTIME + LONGTIME)
            {
                LedFlashSeq = 0;
                Led_Changed_flag = 0;
            }
        }
    }
}

void LedStartTime( void ) //定时开关灯
{
    uint32 Plan_time_Temp;
    
    Plan_time_Temp = Curr_RTC_time.Hour;
    Plan_time_Temp *= 60;
    Plan_time_Temp += Curr_RTC_time.Minute;
    Plan_time_Temp *= 30;
    
    //Plan_time_Temp = Curr_Sys_Parm.SecondOfRtc % 86400 / 2;
    if( Open_Led_Flag == 1)
    {
      if( Plan_time_Temp == Curr_Sys_Parm.LedOnTime)
      {
        Open_Led_Flag = 0;
      }    
    }
    else
    {
      if( Plan_time_Temp == Curr_Sys_Parm.LedOffTime)
      {
        Open_Led_Flag = 1;
      }
    }
}

void GetLightValue(void)    //25ms
{
    uint8 i,maskbit;

  LtChnlAdd[0] += HalAdcRead( HAL_ADC_CHN_AIN4, HAL_ADC_RESOLUTION_12);//转换时间65.5us
 
  
  LtChnlAdd[1]  += HalAdcRead( HAL_ADC_CHN_AIN3, HAL_ADC_RESOLUTION_12);
  

  LtChnlAdd[2]  += HalAdcRead( HAL_ADC_CHN_AIN2, HAL_ADC_RESOLUTION_12);

    
  LtChnlAdd[3]  += HalAdcRead( HAL_ADC_CHN_AIN1, HAL_ADC_RESOLUTION_12);

  
  LtChnl5Add  += HalAdcRead( HAL_ADC_CHN_AIN0, HAL_ADC_RESOLUTION_12);
  
  LtChnlCnt++;
  if( LtChnlCnt >= LTAGVCONST)      //10*25ms
  {   
    LtChnlCnt = 0;
    for (i=0;i<4;i++)
    {
      LtChnl[i]=LtChnlAdd[i] / LTAGVCONST;
      LtChnlAdd[i]=0;
    }

    LtChnl5=LtChnl5Add / LTAGVCONST;
    LtChnl5Add=0;

    if(debug > 0 )
    {   
      DianYa =  LtChnl5/10;
      for (i=0;i<4;i++)
      {
        GongLv[i] =  LtChnl[i];     
        Curr_Sys_Parm.NengLiang[i] = PwChnl[i];     
      }
    }   
    
    LtChnlSaveTimer++;       // SAVELTCONST 10 * 250ms = 2.5
    if(LtChnlSaveTimer<2)
    {   //第一个周期先清除标志
      for (i=0;i<4;i++)
      {
        LtChnlMax[i]=LtChnl[i];
        LtChnlMin[i]=LtChnl[i];
      }
      GuangUpIsOkFlag = 0;
      GuangDownIsOkFlag = 0;
    }
    else
    {
      for (i=0;i<4;i++)
      {
          maskbit = 0x01 << i;
          if(LtChnl[i]>LtChnlMax[i])
          {
            LtChnlMax[i]=LtChnl[i];
          }
          if(LtChnl[i]<LtChnlMin[i])  
          {
            LtChnlMin[i]=LtChnl[i];
          }
          if ( ( Curr_Sys_Parm.ChaKongCtrl & maskbit ) == 0 ) //如果已经关闭不用再进行光量处理
              continue;
                
          if(LtChnl5>=Light100)//参考光亮度100
          {
             if( ChaKongPwOn & maskbit )//插孔已经通电
             {
               if(LtChnl5<(LtChnlMin[i] * Light100Per/15) || LtChnl[i]>(LtChnlMin[i] * Light100Per/10)) 
               {
                  GuangUpIsOkFlag |= maskbit;       //拔出插头光亮上升（准备关闭）
                  //GuangDownIsOkFlag &= ~maskbit;
                  LtChnlMax[i]=LtChnl[i];
                  LtChnlMin[i]=LtChnl[i];                         
               }
             }
             else
             {
               if(LtChnl5>(LtChnlMin[i] * Light100Per/10) || LtChnlMax[i] >(LtChnl[i] * Light100Per/10))   
               {
                  GuangDownIsOkFlag |= maskbit;     //插上插头光亮下降(准备开启)
                  //GuangUpIsOkFlag &= ~maskbit;
                  LtChnlMax[i]=LtChnl[i];
                  LtChnlMin[i]=LtChnl[i]; 
               }
             }
          }
          else
          if(LtChnl5>=Light50)
          {
               if( ChaKongPwOn & maskbit )//插孔已经通电
               {
                  if( LtChnl5<(LtChnlMin[i] * Light50Per) || LtChnl[i] > (LtChnlMin[i] * Light50Per)) 
                  {
                    GuangUpIsOkFlag |= maskbit;
                    LtChnlMax[i]=LtChnl[i];
                    LtChnlMin[i]=LtChnl[i];                         
                  }
               }
               else
               {
                 if(LtChnl5>(LtChnlMin[i] * Light50Per) || LtChnlMax[i] > (LtChnl[i] * Light50Per))   
                 {
                    GuangDownIsOkFlag |= maskbit;
                    LtChnlMax[i]=LtChnl[i];
                    LtChnlMin[i]=LtChnl[i]; 
                 }
               }
          }
          else
          if(LtChnl5>=Light25)
          {
               if( ChaKongPwOn & maskbit )//插孔已经通电
               {
                 if(LtChnl5<(LtChnlMin[i] * Light25Per) || LtChnl[i] >(LtChnlMin[i] * Light25Per)) 
                 {
                    GuangUpIsOkFlag |= maskbit;
                    LtChnlMax[i]=LtChnl[i];
                    LtChnlMin[i]=LtChnl[i];                        
                 }
               }
               else
               {
                 if(LtChnl5>(LtChnlMin[i] * Light25Per) || LtChnlMax[i] >(LtChnl[i] * Light25Per))   
                  {
                    GuangDownIsOkFlag |= maskbit;
                    LtChnlMax[i]=LtChnl[i];
                    LtChnlMin[i]=LtChnl[i]; 
                 }
               }
          }
          else
          { //LtChnl5 < Light25
               if( ChaKongPwOn & maskbit )//插孔已经通电
               {
                 if(LtChnl5<(LtChnlMin[i] * Light10Per) || LtChnl[i]>(LtChnlMin[i] * Light10Per)) 
                 {
                    GuangUpIsOkFlag |= maskbit;
                    LtChnlMax[i]=LtChnl[i];
                    LtChnlMin[i]=LtChnl[i];                                                 
                 }
               }
               else
               {
                 if(LtChnl5>(LtChnlMin[i] * Light10Per) || LtChnlMax[i]>(LtChnl[i] * Light10Per))   
                 {
                    GuangDownIsOkFlag |= maskbit;
                    LtChnlMax[i]=LtChnl[i];
                    LtChnlMin[i]=LtChnl[i];                        
                 }
               }
          }
      }
    }
    if(LtChnlSaveTimer > SAVELTCONST)
    {
      LtChnlSaveTimer=0;
    }
  }
}

void SendAppState(void)
{
 
          if ( Send_State_Flag == 1 ) 
          {
              SendStateTimer++;
              if( SendStateTimer >= ( SendStateTimerSave*4))
              {
                SendStateTimer = 0;
                SendStateFlag = 1;
                SendStateOrder = 0;
              }
          }          
          if( AutoSendFlag == 1 && SendStateFlag == 0 )
          {
            memset(ReturnBuf,0,19);
            ReturnBuf[0] = 0xfc;
            ReturnBuf[1] = 0x00;
            ReturnBuf[2] = 0x00;
            
            b_SendingState = 1;
            SendState1();//向App发送开关 插孔有无用电器 电压 定时周期等

            SendingReturnBuf();
            SendStateTimer = 0;
            AutoSendFlag = 0;
          }
          else
          if( SendStateFlag == 1 && SendStateTimer % 2 == 0)
          {
              
              SendStateOrder++;
              if(SendStateOrder == 1 )
              {
                ReturnBuf[0] = 0xfc;
                ReturnBuf[1] = 0x00;
                ReturnBuf[2] = 0x00;
                
                b_SendingState = 1;
                SendState1( );//向App发送开关 插孔有无用电器 电压 定时周期等

                SendingReturnBuf();              
                AutoSendFlag = 0;       //zw
              }
              else
              if( SendStateOrder == 2)
              {                
                ReturnBuf[0] = 0xfc;
                ReturnBuf[1] = 0x10;
                ReturnBuf[2] = 0x00;
                HostCom_Buf_Temp[7] = 0x01; //借用传递参数
                HostCom_Buf_Temp[8] = 0x01;
                HostCom_Buf_Temp[9] = 0x02;

                b_SendingState = 1;
                SendState2( );//向App发送开关 插孔有无用电器 电压 定时周期等

                SendingReturnBuf();                         
              }
              else
              if( SendStateOrder == 3 )
              {
                SendStateFlag = 0;
                SendStateOrder =0;
                
                HostCom_Buf_Temp[7] = 0x01;
                HostCom_Buf_Temp[8] = 0x03;
                HostCom_Buf_Temp[9] = 0x04;
                
                ReturnBuf[0] = 0xfc;
                ReturnBuf[1] = 0x10;
                ReturnBuf[2] = 0x00;
                
                b_SendingState = 1;
                SendState2( );//向App发送开关 插孔有无用电器 电压 定时周期等

                SendingReturnBuf();           
              }
          }
}

uint16 LedFlashCnt;
const uint8 LedTable[113] =
{
0xFE,0xFE,0xFE,0xFE,0xFE,0xFD,0xFD,0xFC,0xFC,0xFB,0xFA,0xFA,0xF9,0xF8,0xF7,0xF6
,0xF4,0xF3,0xF2,0xF0,0xEF,0xED,0xEC,0xEA,0xE8,0xE7,0xE5,0xE3,0xE1,0xDF,0xDD,0xDB
,0xD8,0xD6,0xD4,0xD1,0xCF,0xCC,0xCA,0xC7,0xC5,0xC2,0xBF,0xBD,0xBA,0xB7,0xB4,0xB1
,0xAF,0xAC,0xA9,0xA6,0xA3,0xA0,0x9D,0x9A,0x96,0x93,0x90,0x8D,0x8A,0x87,0x84,0x81
,0x7D,0x7A,0x77,0x74,0x71,0x6E,0x6B,0x68,0x64,0x61,0x5E,0x5B,0x58,0x55,0x52,0x4F
,0x4D,0x4A,0x47,0x44,0x41,0x3F,0x3C,0x39,0x37,0x34,0x32,0x2F,0x2D,0x2A,0x28,0x26
,0x23,0x21,0x1F,0x1D,0x1B,0x19,0x17,0x16,0x14,0x12,0x11,0x0F,0x0E,0x0C,0x0B,0x0A
,0x0A
//,0x08,0x07,0x06,0x05,0x04,0x04,0x03,0x02,0x02,0x01,0x01,0x00,0x00,0x00,0x00,0x00
};

void LedFlashCtrl()
{
    static uint8 keepdelay ;
    if ( Led_Flash_updown )
    {
        if ( LedFlashCnt > 0 )
            LedFlashCnt --;
        else
        {
            Led_Flash_updown = 0;
            keepdelay = 250;
        }
    }
    else
    {
        if ( keepdelay > 0 )
        {
            keepdelay--;
        }
        else
        if ( LedFlashCnt < 112*3 )
            LedFlashCnt ++;
        else
            Led_Flash_updown = 1;
    }
    if ( LedFlashCnt % 3 ==  0 )
        LedSetVault(2,LedTable[112 - (LedFlashCnt / 3)]);
}

uint16 AppUser_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function     
  uint8 ucTemp8;

    if (events & AUP_POWEROFF_DET_EVT )
    {
          CHK0_LOW_flag = 0;
          CHK1_LOW_flag = 0;
          if ( osal_snv_write(0x80, sizeof(SYS_PARM), &Curr_Sys_Parm) == SUCCESS )
          {
              if ( b_Refresh81 )
              {
                  osal_snv_write(0x81, sizeof(SYS_PARM1), &Curr_Sys_Parm1);  
                  RefreshSysParaDelay = 0;
                  b_Refresh81 = 0;
              }
              if ( b_Refresh82 )
              {
                  osal_snv_write(0x82, sizeof(SYS_PARM2), &Curr_Sys_Parm2);  
                  RefreshSysParaDelay = 0;
                  b_Refresh82 = 0;
              }
              b_Refresh80 = 0;
              CHK_PowerOff_flag = 1;
              Led_Changed_flag = 0;
          }
          return (events ^ AUP_POWEROFF_DET_EVT);
    }
    
    if ( events & AUP_PERIODIC_EVT )
    {
        // Restart timer
        osal_start_timerEx( AppUserProcess_TaskID, AUP_PERIODIC_EVT, 5 );    //5ms
        
        //LineDetect();//相线检测
        if ( Led_Flash_flag )
            LedFlashCtrl();
                
        Timer25ms++;
        if ( Timer25ms < 5 )
        {
            return (events ^ AUP_PERIODIC_EVT);
        }
        Timer25ms = 0;
        //以下为25ms定时处理        

        GetLightValue();
     
        ChaKongIsHave();//插孔有用电器插入检测
        
        if( Curr_Sys_Parm.ForceSwitch_Flag == 1)
        {
            ChaKongPwOnCtrl();//插孔通电检测
        }
                 
        KeyDetect();//按键检测     
        
        if ( b_Clear_Plan_time == 0 && Bt_ConnectingTimer == 0 )
        {
            LedShow();//指示灯显示
        }
        else
        {
            Timer300ms++;
            if ( Timer300ms > 16 )
                Timer300ms=0;
            if ( Timer300ms == 0 )
            {
                if ( Bt_ConnectingTimer > 0 )
                    LedSetVault(2,128);
                else
                    PWM_RGB(128, 128, 128);     //PWM
            }
            else
            if ( Timer300ms == 8 )              
            {
                if ( Bt_ConnectingTimer > 0 )
                    LedSetVault(2,0);
                else
                    PWM_RGB(0, 0, 0);     //PWM
            }
        }
        
        if ( Curr_Sys_Parm.AutoControlEable > 0 )
        {
            AutoPowerDownOpen();//自动断电控制打开
        }
       
        if( b_APP_User1_OK )
        {
            SendAppState();
        }
        
        if(RetryCount>0)          
        {
            RetryCount--;
            if(RetryCount==0)
            {
                Dl645Front.PriData.Flag = RN8209RST;
            }
        }
        if( Dl645Front.PriData.Flag == RN8209RST )//RN8209复位
        {
            RetryCount=0;
            ResetTimer ++ ; 
            if( ResetTimer  >= 4 )//复位前需等待100ms
            {
              ResetTimer = 0;       
              HalUARTInit();
              simpleBLE_NPI_init();  
              Dl645Front.PriData.Flag = 0;// 
              fnScomPk_Init(SCOM_PORT_RN8209); 
              fnEMU_Init();
            }  
        }
        else
        {
          if( b_Param_Ok ==1)
          {
            fnDl645Front_Exec();
          }         
        }

        if( P2_0_LowFlag == 1 )//按键短按1秒内需按两次进入蓝牙加密连接时间
        {
          P2_0_LowTimer++;
          if( P2_0_LowTimer > 40 )
          {
            P2_0_LowFlag = 0;
            P2_0_LowTimer = 0;
          }              
        }
        // Perform periodic application task
        return (events ^ AUP_PERIODIC_EVT);
    }
        
    if (events & AUP_GET_UART1_RX_EVT )
    {
      
        RN8209D_Recv_Analyse();
      
        return (events ^ AUP_GET_UART1_RX_EVT);
    }
    
    if ( events & SYS_EVENT_MSG )
    {
        uint8 *pMsg;
        if ( (pMsg = osal_msg_receive( AppUserProcess_TaskID )) != NULL )
        {
            AppUser_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }
        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & AUP_START_DEVICE_EVT )
    {    
        // Set timer for first periodic event
        osal_start_timerEx( AppUserProcess_TaskID, AUP_PERIODIC_EVT, 2 );    //2ms

        RTC_GetGloablTime(&Curr_RTC_time);
        
        FindCycleAbsCout();
        
        RuntimeParaInit();

        PWM_RGB(0, 0, 0);     //PWM
        //PWM_RGB(30,30,20);
        //LedColorSet[0][0] = 32;
        //LedColorSet[0][1] = 20;
        //LedColorSet[0][2] = 14;
        LedColorSet[0][0] = 0;
        LedColorSet[0][1] = 15;
        LedColorSet[0][2] = 12;
        
        //PWM_RGB(128, 16, 0);
        LedColorSet[1][0] = 255;
        LedColorSet[1][1] = 20;
        LedColorSet[1][2] = 0;

        //PWM_RGB(128, 0, 128);
        LedColorSet[2][0] = 128;
        LedColorSet[2][1] = 0;
        LedColorSet[2][2] = 64;

        //LineDetect();//相线检测
        return ( events ^ AUP_START_DEVICE_EVT );
    }


    if ( events & AUP_GETNEWDATA_EVT )
    {
      if(b_SendingState == 0)
      {
        if( b_GetNewData == 0)
        {
           memcpy( HostCom_Buf_Temp ,HostCom_Buf2, 19 ) ;        
        }
        else
        {
           memcpy( HostCom_Buf_Temp ,HostCom_Buf1, 19 ) ;        
               
        }
        if( Check_HostCom_Buf() )
        {
          CheckHostCmd();
        }
        return (events ^ AUP_GETNEWDATA_EVT);
      }
    }
 
    if ( events & AUP_TIMEROUT_EVT )
    {
      
      if(ComPack[SCOM_PORT_RN8209].TxLen > 0)
      {
          ComPack[SCOM_PORT_RN8209].TxLen --;
          fnRN8209_Read( ComPack[SCOM_PORT_RN8209].TxBuf[ComPack[SCOM_PORT_RN8209].TxLen] );                 
          osal_start_timerEx( AppUserProcess_TaskID, AUP_TIMEROUT_EVT, AUP_TIMEROUT );
      }
      else        
      {
        if( WriteCnt < 16)
            WriteCnt++;
        
        switch( WriteCnt )
        {
            case 1:
              
                    ucTemp8 = WriteOpen;//打开写使能 
                    fnRN8209_Write( WriteCmd ,(uint8 *)&ucTemp8  , 1 ) ;
              
                    break;
              
            case 2:
              
                    NubRN8209DCommand=9;
                    RN8209DCommand=ADSysStatus;//读取写使能是否成功
                    fnRN8209_Read( RN8209DCommand ) ;              
                  
                    break;   
            case 3:
                
                    ucTemp8 = Reset8209;//复位 
//                    ucTemp8 = PwSeltB;
                    fnRN8209_Write( WriteCmd ,(uint8 *)&ucTemp8  , 1 ) ; 

                
                    break;  
                
            case 4:
              
                     NubRN8209DCommand=9;
                     RN8209DCommand=ADSysStatus;//读取复位是否成功
                     fnRN8209_Read( RN8209DCommand ) ;                       

                     break; 
                      
            case 5:
              
                     ucTemp8 = WriteOpen;//打开写使能 
                     fnRN8209_Write( WriteCmd ,(uint8 *)&ucTemp8  , 1 ) ;
                
                     break;
              
            case 6:
              
                    NubRN8209DCommand=9;
                    RN8209DCommand=ADSysStatus;//读取写使能是否成功
                    fnRN8209_Read( RN8209DCommand ) ;              
                  
                    break; 
                    
            case 7:
              
                    Dl645FirmPara.SYSCON = 0x51;//0x40;//0x7f;//0x40;//开电流通道B，电压通道增益为1，电流通道2,
                    fnRN8209_Write( ADSYSCON ,(uint8 *)&Dl645FirmPara.SYSCON , 2 );   //写系统控制寄存器                   
                    
                    break;   
              
            case 8:
              
                    Dl645FirmPara.EMUCON =0x03;// 使能QF脉冲输出和自定义电能寄存器累加；使能PF脉冲输出和有功电能寄存器累
                    fnRN8209_Write( ADEMUCON ,(uint8 *)&Dl645FirmPara.EMUCON , 2 );   //  
                    
                    break;
              
            case 9:
              
                    Dl645FirmPara.HFConst =0x40a6;//0x2503;//0x5099;//电表常数1000
                    fnRN8209_Write( ADHFConst ,(uint8 *)&Dl645FirmPara.HFConst , 2 );   //       
                    
                    break;   
              
            case 10:
                
                    if(b_Uart0orUart1)
                      Dl645FirmPara.PStart = Curr_Sys_Parm.PwOffSet[0];//1通道A有功起动功率设置
                    else
                      Dl645FirmPara.PStart = Curr_Sys_Parm.PwOffSet[3];//4通道A有功起动功率设置

                    fnRN8209_Write( ADPStart  ,(uint8 *)&Dl645FirmPara.PStart , 2 );   //                 
                  
                    break;
                  
            case 11:
                    if(b_Uart0orUart1)
                      Dl645FirmPara.QStart = Curr_Sys_Parm.PwOffSet[1]; //2通道B有功起动功率设置
                    else
                      Dl645FirmPara.QStart = Curr_Sys_Parm.PwOffSet[2]; //3通道B有功起动功率设置                   
                    
                    fnRN8209_Write( ADDStart ,(uint8 *)&Dl645FirmPara.QStart , 2 );   //                  
                  
                    break;   
                    
            case 12:
              
                    if(b_Uart0orUart1)
                      Dl645FirmPara.APOSA =0;//1通道A功率有效值偏置校正
                    else
                      Dl645FirmPara.APOSA =0;//4
                    
                    fnRN8209_Write( ADAPOSA  ,(uint8 *)&Dl645FirmPara.APOSA , 2 );   //                 
              
                    break;
              
            case 13:
              
                    if(b_Uart0orUart1)
                      
                      Dl645FirmPara.APOSB =0;//2通道A功率有效值偏置校正
                    
                    else
                      
                      Dl645FirmPara.APOSB =0;//3
                    
                    fnRN8209_Write( ADAPOSB  ,(uint8 *)&Dl645FirmPara.APOSB , 2 );   //                 
              
                    break;
              
            case 14:

                    Dl645FirmPara.EMUCON2 =0x30;//自定义电能输入选择为通道B有功功
                    fnRN8209_Write( ADEMUCON2 ,(uint8 *)&Dl645FirmPara.EMUCON2 , 2 );   //                  
              
                    break; 
              
              
            case 15:
              
                    ucTemp8 = WriteClose;//写禁止
                    fnRN8209_Write( WriteCmd ,(uint8 *)&ucTemp8  , 1 ) ;                
                    
                    break;  

            case 16:
              
                    NubRN8209DCommand=9;
                    RN8209DCommand=ADSysStatus;
                    fnRN8209_Read( RN8209DCommand );          
 
                    break; 
        }
      }
        return (events ^ AUP_TIMEROUT_EVT);
    }

    if (events & AUP_SECOND_PROC_EVT )
    {   //以下为秒定时处理
        if ( IntoIdleDelay > 0 )
        {
            IntoIdleDelay--;
            if ( IntoIdleDelay == 0 )
              Led_Changed_flag = 0;
        }

        if( b_Clear_Plan_time == 1 ) //清除所有定时计划
        {
          ClearTimer++;
          if( ClearTimer > 3 ) //3s
          {
            ClearTimer = 0;
            b_Clear_Plan_time = 0;
            P2_0_LongFlag = 1;
            PWM_RGB(0, 0, 0);     //PWM

            memset(&Curr_Sys_Parm1,0,sizeof(SYS_PARM1));
            AbsTimeAdd = 0;   
            CycleTimeAdd = 0; 
            RefreshSysParaDelay = REFRESHTIME ;// 2500ms      
            b_Refresh81 = 1;
            
            for ( ucTemp8 = 0; ucTemp8< 4; ucTemp8++ )
              Curr_Sys_Parm.NengLiang[ucTemp8] = 0;
            b_Refresh80 = 1;
          }
        }

        ChaKongTimerCmp();//插孔定时到期检测
        if( Curr_Sys_Parm.Auto_Led_Flag == 1 )
        {
          LedStartTime();
        }
        
        if(LineDetectFlag == 1) //相线检测显示有效时间
        {
          LineDetectTimer++;
          if( LineDetectTimer > 3)
          {
            LineDetectFlag = 0;
            LineDetectTimer = 0;
          }
        }          
     
        if( Bt_ConnectingTimer > 0 )//蓝牙加密连接 30秒内 有效
        {
            Bt_ConnectingTimer--;
        }
        
        if ( RefreshSysParaDelay > 0 )
        {//延时保存参数
            RefreshSysParaDelay--;
            if ( RefreshSysParaDelay == 0 )
            {
                if ( b_Refresh80  == 1 )
                {//更新参数设置
                  osal_snv_write(0x80, sizeof(SYS_PARM), &Curr_Sys_Parm);  
                  b_Refresh80 = 0;
                  if( b_Refresh81 == 1 || b_Refresh82 == 1 )//|| b_Bt_Restart==1)
                  {
                    RefreshSysParaDelay = REFRESHTIME;
                  }
                }
                else
                if ( b_Refresh81 == 1 )
                {
                  osal_snv_write(0x81, sizeof(SYS_PARM1), &Curr_Sys_Parm1);
                  b_Refresh81 = 0;
                  if( b_Refresh80 == 1 || b_Refresh82 == 1 )//|| b_Bt_Restart==1)
                  {
                    RefreshSysParaDelay = REFRESHTIME;
                  }
                }
                else
                if ( b_Refresh82 == 1 )
                {
                  osal_snv_write(0x82, sizeof(SYS_PARM2), &Curr_Sys_Parm2);
                  b_Refresh81 = 0;
                  if( b_Refresh80 == 1 || b_Refresh81 == 1 )//|| b_Bt_Restart==1)
                  {
                    RefreshSysParaDelay = REFRESHTIME;
                  }
                }
                /*
                else
                if(b_Bt_Restart==1)
                {
                   b_Bt_Restart=0;               
                   HAL_SYSTEM_RESET(); 
                }
                */
            }

          }   
        return (events ^ AUP_SECOND_PROC_EVT);
    }
        
    return 0;
}

static void AppUser_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {


    default:
        // do nothing
        break;
  }
}



void ConvertU32toByte4(uint8 * output,uint32 temp32)
{
	uint8 i;
	for (i=0;i<4;i++)
	{
		output[i] = (temp32 & 0xff);
		temp32 >>= 8;
	}	
}

uint32 ConvertByte4toU32(uint8 * input)
{
	uint32 temp32 = 0;
	uint8 i;
	for (i=0;i<4;i++)
	{
		temp32 <<= 8;
		temp32 |= input[3-i]; 
	}
	return temp32;
}





unsigned short ConvertStringToU16( const char *buffer )
{
	const char * pstr;
	unsigned short result=0;
	if ( buffer == NULL )
		return 0;
	pstr = buffer;
	while(*pstr != 0 )
	{
		result *= 10;
		if ( *pstr >='0' && *pstr <='9' )
		{
			result += ( *pstr - '0' );
			pstr++;
		}
		else
			return 0;
	}
	return result;
}

void GetCurrBTphoneName(uint8 BTn,char * tempbuf)
{
	uint8 len;
  	
        memcpy(tempbuf,Curr_Sys_Parm.devName,sizeof(Curr_Sys_Parm.devName));
//	sprintf(tempbuf,Curr_Sys_Parm.devName);
        len = strlen(tempbuf);
	tempbuf[len+1] = '\0';
}

void ConvertBT21str2addr(uint8 * str,uint8 * data)
{
	uint8 i;
	for (i=0;i<6;i++)
	{
		data[i] = converASC2HEX(&str[i*3]);
	}
}

uint16 gRed = 0;
uint16 gGreen = 0;
uint16 gBlue = 0;
//void HalTimer1Init (halTimerCBack_t cBack);
//halTimer1SetChannelDuty (uint8 channel, uint16 promill);

void PWM_Init()
{
  // Initialize Timer 1
  T1CTL = 0x01;               // Div = 128, CLR, MODE = Suspended          
  T1CCTL0 = 0;//x1C;             // IM = 0, CMP = Clear output on compare; Mode = Compare
  T1CCTL1 = 0;//x1C;             // IM = 0; CMP = Clear output on compare; Mode = Compare
  T1CCTL2 = 0;//x1C;             // IM = 0; CMP = Clear output on compare; Mode = Compare
  T1CNTL = 0;                 // Reset timer to 0;

  //必须设置，否则定时器不工作
  //T1CCTL0 = 0x4C;            // IM = 1, CMP = Clear output on compare; Mode = Compare
  
#define VALUE_H     0x10
#define VALUE_L     0x00
  T1CC0H = VALUE_H;    
  T1CC0L = VALUE_L;    
  T1CC1H = VALUE_H;    
  T1CC1L = VALUE_L;
  T1CC2H = VALUE_H;    
  T1CC2L = VALUE_L;

  //EA=1;
  //IEN1 |= 0x02;               // Enable T1 cpu interrupt
}

void LedSetVault(uint8 led,uint8 val)
{
  switch(led)
  {
      case 2:
        //蓝色
        T1CC2L = 0;
        T1CC2H = val;
        if(val!=0){
          T1CCTL2 = 0x1C;
        }else{
          T1CCTL2 = 0x00;
        }
        break;
     case 1:
        //绿色
        T1CC1L = 0;
        T1CC1H = val;
        if(val!=0){
          T1CCTL1 = 0x1C;
        }else{
          T1CCTL1 = 0x00;
        }
        break;
     case 0:
        //红色
        T1CC0L = 0;
        T1CC0H = val;
        if(val!=0){
          T1CCTL0 = 0x1C;
        }else{
          T1CCTL0 = 0x00;
        }          
  }        
}

//red， green， blue 的值必须是 1~255, 其他值无效
void PWM_Pulse(uint16 red, uint16 green, uint16 blue)
{
  uint16 r,g,b;

  // stop,注意，不能加这句，加了周期偏差十几倍，具体原因未查明
  //T1CTL &= BV(0)|BV(1); 
  b=red;
  r=green;
  g=blue;

  // Set up the timer registers
  
  //红色
  T1CC0L = (uint8)b;
  T1CC0H = (uint8)(b >> 8);
  if(b!=0){
    T1CCTL0 = 0x1C;
  }else{
    T1CCTL0 = 0x00;
  }

  //绿色
  T1CC1L = (uint8)r;
  T1CC1H = (uint8)(r >> 8);
  if(r!=0){
    T1CCTL1 = 0x1C;
  }else{
    T1CCTL1 = 0x00;
  }

  //蓝色
  T1CC2L = (uint8)g;
  T1CC2H = (uint8)(g >> 8);
  if(g!=0){
    T1CCTL2 = 0x1C;
  }else{
    T1CCTL2 = 0x00;
  }


  // Reset timer
  T1CNTL = 0;  

  // Start timer in modulo mode.
  //T1CTL |= 0x02;   
}

//red， green， blue 的值必须是 1~375, 其他值无效
void PWM_RGB(uint8 red, uint8 green, uint8 blue)
{    
  gRed = red ;
  gRed <<= 8;
  gGreen= green;
  gGreen <<= 8;
  gBlue= blue;
  gBlue <<= 8;
  PWM_Pulse(gRed,gGreen,gBlue);
}

/*
//#pragma register_bank=2
HAL_ISR_FUNCTION(pwmISR, T1_VECTOR)
{
    uint8 flags = T1STAT;
    // T1 ch 0
    if (flags & 0x01)
    {          
      // Stop Timer 1
      //T1CTL |= 0x02;
      //red， green， blue 的值必须是 1~375, 其他值无效
      //PWM_Pulse(gRed,gGreen,gBlue);
    }
    T1STAT = ~ flags;
}
*/

//定时器初始化
void InitT3()
{     
  T3CTL |= 0x08 ;         //开溢出中断     
  T3IE = 1;               //开总中断和T3中断
  //IEN1 |= 0x08;               // Enable T3 cpu interrupt
  T3CC0 = 49;//249;
  
  T3CTL|= 0xE0;            //128分频
  T3CTL &= ~0x03;         //自动重装
  T3CTL |=0x02;           //周期250，=1ms
  T3CCTL0 = 0x44;
  T3CTL |=0x10;           //启动
  //EA = 1;
}

extern uint32  OSAL_timeSeconds;
uint16 DATA count = 0;
uint8 DATA DelaySwitchCnt = 0;
//#pragma vector = T3_VECTOR //定时器T3
 //__interrupt void T3_ISR(void) 
HAL_ISR_FUNCTION(T3_ISR, T3_VECTOR) //0.8ms
{ 
  if(P0_5==0)//CHK0 LOW
  {
      if( CHK0_LOW_checked == 0 )
      {//	正常状态
          PowerOffcheckcnt = 60;  //48ms        
          CHK0_LOW_flag=1;
          CHK0_LOW_checked = 1;
          CHK_PowerOff_flag = 0;
          if  ( DelaySwitchFlag )
          {
              DelaySwitchCnt = 7;       //delay 7x0.8 = 6ms
              DelaySwitchFlag = 0;
          }
          if ( RetryCheckLN_Flag )
          {
              CHK1_LOW_checked = 0;
              CHK1_LOW_flag = 0;
              RetryCheckLN_Flag = 0;
          }
      }
  }
  else
  {
      CHK0_LOW_checked = 0;
  }
  
  if(P0_6==0)//CHK1 LOW
  {
      if( CHK1_LOW_checked == 0 )
      {//火线零线反接绿灯闪
        PowerOffcheckcnt = 60;  //48ms        
        CHK1_LOW_flag = 1;        
        CHK1_LOW_checked = 1;
        CHK_PowerOff_flag = 0;
        if  ( DelaySwitchFlag )
        {
            DelaySwitchCnt = 7;
            DelaySwitchFlag = 0;
        }
        if ( RetryCheckLN_Flag )
        {
            CHK0_LOW_checked = 0;
            CHK0_LOW_flag = 0;
            RetryCheckLN_Flag = 0;
        }
      }
  }
  else
  {
      CHK1_LOW_checked = 0;
  }
  
  if ( PowerOffcheckcnt > 0 )
  {
      PowerOffcheckcnt--;
      if ( PowerOffcheckcnt == 0 )
      {
          HAL_ENTER_ISR();
          //PWM_RGB(32, 0, 0);     //突然断电 红灯常亮
          osal_set_event( AppUserProcess_TaskID, AUP_POWEROFF_DET_EVT );
          HAL_EXIT_ISR();
      }
  }

  if ( DelaySwitchCnt > 0 )
  {
      DelaySwitchCnt--;
      if ( DelaySwitchCnt == 0 )
          UpdatePwOnIO();       //刷新
  }

  //IRCON = 0x00;             //清中断标志, 也可由硬件自动完成 
  if(++count >= 1250 )
  {          
      HAL_ENTER_ISR();
      count = 0;               // 计数清零 
      OSAL_timeSeconds++;
      osal_set_event( AppUserProcess_TaskID, AUP_SECOND_PROC_EVT );
      HAL_EXIT_ISR();
      RetryCheckLN_Flag = 1;
  } 
  //LineDetect();//相线检测
  
}
