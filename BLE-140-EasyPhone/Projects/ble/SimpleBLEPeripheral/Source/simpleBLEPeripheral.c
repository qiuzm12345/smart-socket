/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

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

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include "stdio.h"
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"

#include "gatt.h"

#include "hci.h"
//#include "usb_framework.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#if defined( CC2540_MINIDK )
  #include "simplekeys.h"
#endif

#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif

#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"
#include "AppUserProcess.h"
   
#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

//#include "simpleble.h"
#include "npi.h"
#include "hal_shiftout.h"
#include "OSAL_Clock.h"
#include "OSAL_snv.h"
#include "RN8209D.h"       
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define	BT_CMD_MAX_SIZE		51

void GetCurrBTphoneName(uint8 BTn,char * tempbuf);
void SetOneDayPower(uint8 * buf,uint8 index,uint16 input);

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8//80   连接间隔与数据发送量有关， 连接间隔越短， 单位时间内就能发送越多的数据

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     10//800   连接间隔与数据发送量有关， 连接间隔越短， 单位时间内就能发送越多的数据

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          600//1000  -各种原因断开连接后，超时并重新广播的时间:  100 = 1s

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         3

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern void Serial_Delayus(int times);

/*********************************************************************
 * LOCAL VARIABLES
 */

uint8 SSS;
uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

gaprole_States_t gapProfileState = GAPROLE_INIT;

uint32 LastOprationTick;

extern bool b_APP_User1_OK,Led_Changed_flag,b_DateTimeUpdataed,Led_Flash_flag,DelaySwitchFlag;

//uint8 ownAddress[B_ADDR_LEN];
uint8 masterAddress[B_ADDR_LEN];

//unsigned char RevLen,result;
//unsigned char RecvCommBuff[BT_CMD_MAX_SIZE],RecvComType,tempbuf[BT_CMD_MAX_SIZE]; 

uint16 RefreshSysParaDelay = 0;

uint8 SlaveSendBuf[16];
uint8 ReturnBuf[22];
uint8 ReturnLen;


uint8 AbsTime_cout = 0;
uint8 CycleTime_cout = 0;

uint16  AbsTimeAdd = 0;   
uint16  CycleTimeAdd = 0; 
extern bool b_GetNewData,b_Get_BT1_Commond;
extern uint8 SendBackBuffer[];

extern uint8 HostCom_Loop_Buf[];
extern uint8 HostCom_ptr_out,HostCom_ptr_in;
extern uint8 HostComBuf[];
extern uint16 SendStateTimer;
extern uint8 SendStateTimerSave;
extern uint8 HostCom_Buf1[19];
extern uint8 HostCom_Buf2[19];
extern uint8 HostCom_Buf_Temp[19];
extern uint8 BT1_LocalDevAddr[];
extern uint16 TalkingSecond;
extern uint8 FlashTimeCnt,WaitKeepTMR;
extern uint8   Plan_time_cout;
extern uint8 LedColorSet[3][3];
extern uint8 DATA ChaKongPwOn;

uint16 GetGongLvAverage(uint8 index,uint8 count);
uint8 CheckTimePara(RTC_Time * pSdt);
uint32 ConvertByte4toU32(uint8 * input);

void RTC_Set_RTC_Time(RTC_Time *pTimes);
void RTC_GetGloablTime(RTC_Time *pTimes);
void SecondsToRTCTime(uint32 TimeVar,RTC_Time * Time);

extern RTC_Time Curr_RTC_time;
extern bool CHK0_LOW_flag,CHK1_LOW_flag;

extern bool Open_Led_Flag;
//extern bool b_Bt_Restart; 
extern bool b_Refresh80,b_Refresh81,b_Refresh82;

extern bool b_SendingState;
extern bool Send_State_Flag;
extern uint8 AutoControlFlag,Bt_ConnectingTimer,IntoIdleDelay,ConnectUser,ChaKongLockOff;
extern bool AutoSendFlag;
extern bool debug;

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   //0x11,// length of this data
  GAP_ADTYPE_16BIT_MORE,      // GAP_ADTYPE_128BIT_MORE,//some of the UUID's, but not all
  LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
  HI_UINT16( SIMPLEPROFILE_SERV_UUID ),
  //0xF0,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,0xF9,0xFA,0xFB,0xFC,0xC2,0xA6,0xC5  
};


uint16 gapConnHandle = 0;

//static int8 gMP = 0xCD;
// GAP GATT Attributes

// GAP GATT Attributes
//static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Peripheral";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void peripheralRssiReadCB( int8 rssi );
static void simpleProfileChangeCB( uint8 paramID );
//extern uint8 osal_snv_write( uint16 id, uint16 len, void *pBuf );
//#if defined( BLE_BOND_PAIR )
typedef enum
{
  BOND_PAIR_STATUS_PAIRING,  //未配对
  BOND_PAIR_STATUS_PAIRED,  //已配对
}BOND_PAIR_STATUS;
// 用来管理当前的状态，如果密码不正确，立即取消连接，并重启
//static BOND_PAIR_STATUS gPairStatus = BOND_PAIR_STATUS_PAIRING;

void ProcessPasscodeCB(uint8 *deviceAddr,uint16 connectionHandle,uint8 uiInputs,uint8 uiOutputs );
//static void ProcessPairStateCB( uint16 connHandle, uint8 state, uint8 status );
//#endif


/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  peripheralRssiReadCB,               // When a valid RSSI is read from controller (not used by application)
};


// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};


uint8 checkBCC(uint8* data_buffer,uint8 Nb_bytes )
{
    uint8 i,bcc=0;
    for(i=0;i< Nb_bytes;i++)
    {
        bcc += data_buffer[i];
    }
    return bcc;
}

unsigned char onebyteswap ( unsigned char byte )
{
	unsigned char temp;
	temp = ( ( byte & 0x0f ) << 4 );
	temp |= ( ( byte & 0xf0 ) >> 4 );
	return temp;
}

uint8 converASC2HEX(uint8 * input)
{
	uint8 result=0;
	if ( input[0] >= '0' && input[0] <= '9' || input[0] >= 'A' && input[0] <= 'F' )
	{
		if ( input[1] >= '0' && input[1] <= '9' || input[1] >= 'A' && input[1] <= 'F' )
		{
			if ( input[0] >= '0' && input[0] <= '9' )
				result = input[0] - '0';
			else
				result = input[0] - 'A' + 10;
			result <<= 4;
			if ( input[1] >= '0' && input[1] <= '9' )
				result += input[1] - '0';
			else
				result += input[1] - 'A' + 10;
		}
	}
	return result;
}

void Read_System_ID(uint8 * pIDstr)
{ 
    pIDstr[0] =  BT1_LocalDevAddr[0];
    pIDstr[1] =  ~BT1_LocalDevAddr[0];  
    pIDstr[2] =  BT1_LocalDevAddr[1];
    pIDstr[3] =  ~BT1_LocalDevAddr[1];
    pIDstr[4] =  BT1_LocalDevAddr[2];
    pIDstr[5] =  ~BT1_LocalDevAddr[2];  
    pIDstr[6] =  BT1_LocalDevAddr[3];
    pIDstr[7] =  ~BT1_LocalDevAddr[3];
    pIDstr[8] =  BT1_LocalDevAddr[4];
    pIDstr[9] =  ~BT1_LocalDevAddr[4];  
    pIDstr[10] =  BT1_LocalDevAddr[5];
    pIDstr[11] =  ~BT1_LocalDevAddr[5];

}

uint8 Check_HostCom_Buf(void)
{
  uint8 i;
  uint8 Addtemp=0;  
  uint8 length;
  length =  HostCom_Buf_Temp[1]&0x0f;
  length += 5;
  if(HostCom_Buf_Temp[0] != 0xfe)
      return 0;

  for( i =1; i< length+2; i++)
      Addtemp += HostCom_Buf_Temp[i];
  if(Addtemp == HostCom_Buf_Temp[i] || HostCom_Buf_Temp[i] == 0xbc)//累加和校验
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void FindCycleAbsCout( void )
{
  uint8 i=0;
  CycleTime_cout = 0;
  AbsTime_cout = 0;
  for( i = 0; i< MAX_SCH_COUNT;i++ )
  {
      if (Curr_Sys_Parm1.Plan_time[i].Data.byteTime[3] & 0x80)
      {
        CycleTime_cout++;
      }
      else
      if (Curr_Sys_Parm1.Plan_time[i].Data.Time > 0)
      {
        
        AbsTime_cout++;
      }    
  }
}
void SendState1( void)//向App发送开关 插孔有无用电器 电压 定时周期等
{
          memset(&ReturnBuf[3],0,10);           

          if( Curr_Sys_Parm.ChaKongCtrl & 0x01 )//1号插座通电
            ReturnBuf[3] |= 0x01;
          
          if( ChaKongHave & 0x01 )//1号插座有用电器插入
            ReturnBuf[3] |= 0x02;    
          
          if( Curr_Sys_Parm.ChaKongCtrl & 0x02 )
            ReturnBuf[3] |= 0x10;
          
          if( ChaKongHave & 0x02 )
            ReturnBuf[3] |= 0x20;  
          
          if( Curr_Sys_Parm.ChaKongCtrl & 0x04 )
            ReturnBuf[4] |= 0x01;
          
          if( ChaKongHave & 0x04 )
            ReturnBuf[4] |= 0x02;   
          
          if( Curr_Sys_Parm.ChaKongCtrl & 0x08 )
            ReturnBuf[4] |= 0x10;
          
          if( ChaKongHave & 0x08 )
            ReturnBuf[4] |= 0x20;     
          
          if( CHK0_LOW_flag == 0 &&  CHK1_LOW_flag == 0 )																	
          {
             ReturnBuf[5] |= 0x01;//缺地线
          }
          else
          if( CHK0_LOW_flag == 0 &&  CHK1_LOW_flag == 1)
          {
            ReturnBuf[5]  |= 0x02;//火线零线反接
          }         
          
          if( Open_Led_Flag == 1)
            ReturnBuf[5] |= 0x40;
              
          if( Curr_Sys_Parm.ForceSwitch_Flag == 1)
            ReturnBuf[5] |= 0x80;
          
          //FindCycleAbsCout();    //zw
          ReturnBuf[7] = AbsTime_cout ;//绝对时间条数
          ReturnBuf[6] = CycleTime_cout ;//周期时间条数       
          
          ReturnBuf[8] = CycleTimeAdd & 0xff ;//周期定时表项的累加和
          ReturnBuf[9] = ( CycleTimeAdd >> 8 ) & 0xff ;//周期定时表项的累加和
          ReturnBuf[10] =  AbsTimeAdd & 0xff;//绝对定时表项的累加和    
          ReturnBuf[11] = ( AbsTimeAdd >> 8 ) & 0xff ;//绝对定时表项的累加和  
          
          ReturnBuf[12] = DianYa;
          
          ReturnLen = 10;
}

void SendState2( void )//向App发送能量功率
{
    uint8 i;
    uint16 gonglv;
    
    ReturnBuf[3] = HostCom_Buf_Temp[7] ;
    ReturnBuf[4] = HostCom_Buf_Temp[8];//插座编号
    
    i = HostCom_Buf_Temp[8] - 1;
    if ( i > 3 )
    {
      ReturnLen = 1;
      return;
    }
    
    if ( debug > 0 )
      gonglv = GongLv[i];
    else
    if ( ChaKongPwOn & (0x01<<i) )
      gonglv = GetGongLvAverage(i,3);
    else
      gonglv = 0;
    
    ReturnBuf[5] = gonglv % 256;
    ReturnBuf[6] = gonglv / 256;               
    ReturnBuf[7] = Curr_Sys_Parm.NengLiang[i] & 0xff;
    ReturnBuf[8] = ( Curr_Sys_Parm.NengLiang[i] >> 8 ) & 0xff;
    ReturnBuf[9] = ( Curr_Sys_Parm.NengLiang[i] >> 16 ) & 0xff;
    ReturnLen = 13;
    if ( HostCom_Buf_Temp[9] > 0 && HostCom_Buf_Temp[9] < 5 )
    {
        i = HostCom_Buf_Temp[9] - 1;
        if ( debug > 0 )
          gonglv = GongLv[i];
        else
        if ( ChaKongPwOn & (0x01<<i) )
          gonglv = GetGongLvAverage(i,3);
        else
          gonglv = 0;
        
        ReturnBuf[10] =  HostCom_Buf_Temp[9];
        ReturnBuf[11] =  gonglv % 256;
        ReturnBuf[12] =  gonglv / 256;     
        ReturnBuf[13] =  Curr_Sys_Parm.NengLiang[i] & 0xff;
        ReturnBuf[14] = ( Curr_Sys_Parm.NengLiang[i] >> 8 ) & 0xff;
        ReturnBuf[15] = ( Curr_Sys_Parm.NengLiang[i] >> 16 ) & 0xff;
    }
    else
        ReturnLen = 7;
}

void SendingReturnBuf(void)
{
    uint8 i;
    attHandleValueNoti_t pReport;
    uint8 ReturnAdd;
    ReturnAdd = 0;

    ReturnBuf[1] |= ReturnLen;
  
    for( i = 1; i <= ReturnLen+2 ;i++ )
    {      
      ReturnAdd +=  ReturnBuf[i];//累加和
    }
    ReturnBuf[i] = ReturnAdd;
    
    pReport.len = i + 1;
    osal_memcpy(pReport.value, ReturnBuf, pReport.len);
    pReport.handle = 0x0025;//0x0035; 
    GATT_Notification( gapConnHandle, &pReport, FALSE );  
    if( b_SendingState == 1)
      b_SendingState = 0;
}

void  UpdateTimeAdd(void)
{
    uint8 i;
    CycleTimeAdd = 0;
    AbsTimeAdd = 0;
    for ( i = 0; i < MAX_SCH_COUNT ; i++ )
    {
      if ( Curr_Sys_Parm1.Plan_time[i].Data.byteTime[3] & 0x80 ) 
      {
        CycleTimeAdd +=  Curr_Sys_Parm1.Plan_time[i].Data.byteTime[0];
        CycleTimeAdd +=  Curr_Sys_Parm1.Plan_time[i].Data.byteTime[1];
        CycleTimeAdd +=  Curr_Sys_Parm1.Plan_time[i].Data.byteTime[2];
        CycleTimeAdd +=  Curr_Sys_Parm1.Plan_time[i].Data.byteTime[3];
        CycleTimeAdd +=  Curr_Sys_Parm1.Plan_time[i].Plan ;
      }  
      else
      if (Curr_Sys_Parm1.Plan_time[i].Data.Time > 0)
      {
        AbsTimeAdd +=  Curr_Sys_Parm1.Plan_time[i].Data.byteTime[0];
        AbsTimeAdd +=  Curr_Sys_Parm1.Plan_time[i].Data.byteTime[1];
        AbsTimeAdd +=  Curr_Sys_Parm1.Plan_time[i].Data.byteTime[2];
        AbsTimeAdd +=  Curr_Sys_Parm1.Plan_time[i].Data.byteTime[3];
        AbsTimeAdd +=  Curr_Sys_Parm1.Plan_time[i].Plan;
      }
    }
}

const uint8 BitsMask[8]={0x00,0x01,0x03,0x07,0x0f,0x1f,0x3f,0x7f};

void SetOneDayPower(uint8 * buf,uint8 index,uint16 input)
{
    uint8 j,offset,bits;
    uint16 temp16;
    temp16 = index*10;
    bits = temp16 % 8;
    offset = temp16/8;

    buf[offset] &= BitsMask[bits];
    buf[offset] |= ( input << bits );
    j = 10 - ( 8 - bits );
    if ( j < 8 )
    {
        buf[offset+1] &= ~BitsMask[j];
        buf[offset+1] |= ( input >> ( 8 - bits ) );
    }
    else
    {
        buf[offset+1] = 0;
        buf[offset+1] = ( input >> 2 );
    }
}


void CheckHostCmd(void)
{
  uint8 i,j;
  uint8 cmd;
  uint8 len;
  uint16 Temp;
  bool FindOneFlag;
  bool FindTwoFlag;
  uint32 TimeTemp;
  ReturnLen = 0;

  memset(ReturnBuf,0,19);
  //SendReadyFlag = 0;
  cmd = HostCom_Buf_Temp[1]&0xf0;
  len = HostCom_Buf_Temp[1]&0x0f;
  
  ReturnBuf[0] = 0xfc;
  ReturnBuf[1] = cmd;
  ReturnBuf[2] = HostCom_Buf_Temp[2];
  
  if( HostCom_Buf_Temp[3] == 0 && HostCom_Buf_Temp[4] == 0 && HostCom_Buf_Temp[5] == 0 && HostCom_Buf_Temp[6] == 0 )
  {
      b_APP_User1_OK = 0;
      Led_Changed_flag = 0;
  }
  else
  for ( i = 0; i< Curr_Sys_Parm.UserCout; i++ )//当前操作使用的APP设备ID是否注册过
  {
      for( j = 0; j< 4; j++)
      {
          if( Curr_Sys_Parm.User[i][j] == HostCom_Buf_Temp[3+j] )
          {
                if( j >= 3)
                {
                    b_APP_User1_OK = 1;
                    Led_Changed_flag = 0;
                    if ( Curr_Sys_Parm.UserPriority[i] < 65535 )
                        Curr_Sys_Parm.UserPriority[i] += 1;
                    ConnectUser = i;
                    IntoIdleDelay = 10;
                }
          }
      }
  }
  
  
  cmd = cmd >> 4;
  if(b_APP_User1_OK == 1 || cmd == 10 || cmd == 13 || cmd == 14 )
  {
  
    switch (cmd)
    {
    
      case 0://查询设备状态（无后续数据）
            SendState1( );
            break;
        
      case 1://查询 功率 能量
            if ( HostCom_Buf_Temp[7] == 0x01 )        
                SendState2( );
            break;    
        
      case 2://插座开关控制
            if ( HostCom_Buf_Temp[7] == 0 )
            {
                if( HostCom_Buf_Temp[8] == 1 )
                {
                    Curr_Sys_Parm.ChaKongCtrl= 0x0f;
                }
                else
                {
                    if ( ChaKongPwOn > 0 )
                    {
                        ChaKongPwOn = 0;
                        DelaySwitchFlag = 1;
                    }
                    ChaKongLockOff = 0;
                    RuntimeParaInit();
                    Curr_Sys_Parm.ChaKongCtrl=0;
                    Curr_Sys_Parm.ForceSwitch_Flag = 0;
                    Led_Changed_flag = 0;
                }
            }
            else
            if ( HostCom_Buf_Temp[7] < 5 )
            {
                i = 0x01 << ( HostCom_Buf_Temp[7] - 1 );
                if( HostCom_Buf_Temp[8] == 1 )
                {
                    Curr_Sys_Parm.ChaKongCtrl |= i;
                    ChaKongLockOff &= ~i;   //清除由于超负荷而关闭的插座不能再次自动开启的标志
                }
                else
                {
                    Curr_Sys_Parm.ChaKongCtrl &= ~i;
                    ChaKongHave &= ~i;      //关闭后强制标定为无负载
                }
            }
            if( Curr_Sys_Parm.ChaKongCtrl > 0 )
            {
                Curr_Sys_Parm.ForceSwitch_Flag = 1;
            }
            AutoSendFlag=1;
            RefreshSysParaDelay = REFRESHTIME ;// 250ms
            b_Refresh80 = 1;
            ReturnLen = 0;            
            break;
        
      case 3://定时参数设定                       
            TimeTemp = HostCom_Buf_Temp[10];
            TimeTemp <<= 8;
            TimeTemp |= HostCom_Buf_Temp[9];
            TimeTemp <<= 8;
            TimeTemp |= HostCom_Buf_Temp[8];
            TimeTemp <<= 8;
            TimeTemp |= HostCom_Buf_Temp[7];            
              
            for( i = 0;i < MAX_SCH_COUNT ; i++)
            {
                if( Curr_Sys_Parm1.Plan_time[i].Data.Time == TimeTemp )
                {
                    if ( Curr_Sys_Parm1.Plan_time[i].Plan == HostCom_Buf_Temp[11] ) 
                        break;
                }
            }
            if( i >= MAX_SCH_COUNT )
            {
                if( AbsTime_cout + CycleTime_cout >=  MAX_SCH_COUNT ) //定时计划超过50个
                {
                    ReturnBuf[3] = 0xff;//指令错误
                    ReturnLen = 1;            
                    break;
                }
                for( i = 0; i< MAX_SCH_COUNT;i++ )
                {
                  if( Curr_Sys_Parm1.Plan_time[i].Data.Time == 0 )
                    break;
                }
                if ( i>= MAX_SCH_COUNT )
                {
                    ReturnBuf[3] = 0xff;//指令错误
                    ReturnLen = 1;            
                    break;
                }
               
                Curr_Sys_Parm1.Plan_time[i].Plan = HostCom_Buf_Temp[11];
                Curr_Sys_Parm1.Plan_time[i].Data.Time = TimeTemp;
                
                if (  TimeTemp < 86400*2 )//SecsPerDay )
                {
                      if ( TimeTemp > 86400 )
                          debug = ( TimeTemp - 86400 ) / 3600;
                      else
                          debug = TimeTemp / 3600;
                }
                else
                if (  TimeTemp < 86400*6 )
                {
                    j = TimeTemp / 86400;
                    j -= 2;
                    if ( (  TimeTemp % 86400 ) == 0 )
                    {
                       for (i=0;i<31;i++)
                       {
                          Temp = (uint16)i*10+5;
                          if ( i < Curr_RTC_time.Day )
                              Temp |= (uint16)Curr_RTC_time.Month << 12;
                          else
                              Temp |= (uint16)(Curr_RTC_time.Month-1) << 12;
                          Curr_Sys_Parm2.DaylySaveNL[i][j] = Temp;
                       }
                    }
                    else
                    {
                        TimeTemp %= 86400;
                        Temp = TimeTemp%3600;
                        Temp /= 10;
                        Temp |= (uint16)Curr_RTC_time.Month << 12;
                        Curr_Sys_Parm2.DaylySaveNL[TimeTemp/3600][j] = Temp;
                    }
                    RefreshSysParaDelay = REFRESHTIME ;// 250ms
                    b_Refresh82 = 1;
                }
                FindCycleAbsCout();  //加入新的定时更新校验值
                UpdateTimeAdd();                

                RefreshSysParaDelay = REFRESHTIME ;// 2500ms
                b_Refresh81 = 1;
            }
            ReturnLen = 0;
            
            break;      
        
      case 4://设置自动断电
            if( HostCom_Buf_Temp[7] > 0 && HostCom_Buf_Temp[7] <= 4 )
            {
              i = HostCom_Buf_Temp[7] - 1;
              Curr_Sys_Parm.Auto_PowerDown[i].Control = HostCom_Buf_Temp[8];
              Curr_Sys_Parm.Auto_PowerDown[i].Percent = HostCom_Buf_Temp[9];              
              if ( HostCom_Buf_Temp[8] > 0 && HostCom_Buf_Temp[10] > 0 )
              {
                  Curr_Sys_Parm.AutoControlEable |= ( 0x01 << i );
                  AutoControlFlag |= ( 0x01 << i );
              }
              else
              {
                  Curr_Sys_Parm.AutoControlEable &= ~( 0x01 << i );
                  AutoControlFlag &= ~( 0x01 << i );
              }              
              ReturnLen = 0;
              RefreshSysParaDelay = REFRESHTIME ;// 250ms
              b_Refresh80 = 1;
            }
            else
            if( HostCom_Buf_Temp[7] == 0x80 )
            {
                ReturnBuf[3] = 0x80;//HostCom_Buf_Temp[7];
                ReturnBuf[4] = Curr_Sys_Parm.Auto_PowerDown[0].Control;
                ReturnBuf[5] = Curr_Sys_Parm.Auto_PowerDown[0].Percent;
                ReturnBuf[6] = Curr_Sys_Parm.Auto_PowerDown[1].Control;
                ReturnBuf[7] = Curr_Sys_Parm.Auto_PowerDown[1].Percent;                
                ReturnBuf[8] = Curr_Sys_Parm.Auto_PowerDown[2].Control;
                ReturnBuf[9] = Curr_Sys_Parm.Auto_PowerDown[2].Percent;                
                ReturnBuf[10] = Curr_Sys_Parm.Auto_PowerDown[3].Control;
                ReturnBuf[11] = Curr_Sys_Parm.Auto_PowerDown[3].Percent; 
                ReturnBuf[12] = ( (AutoControlFlag & 0x0f) << 4 ) + ( Curr_Sys_Parm.AutoControlEable & 0x0f );
                ReturnLen = 10;
            }
            break;
        
  //    case 5:
  //      
  //      break;    
  //      
  //    case 6:
  //      
  //      break;
  //      
  //    case 7:
  //      
  //      break;      
      case 8://读取已有定时参数
              cmd = 0;//CycleTime_cout=0;
              len = 0;//AbsTime_cout=0;
              FindOneFlag = 0;
              FindTwoFlag = 0;
              memset(&ReturnBuf[3],0,6);
            
              for( i = 0; i< MAX_SCH_COUNT;i++ )
              {
                if (Curr_Sys_Parm1.Plan_time[i].Data.byteTime[3] & 0x80) 
                {
                      cmd++;
                }
                else
                if (Curr_Sys_Parm1.Plan_time[i].Data.Time > 0)
                {
                      len++;
                }
                if( FindOneFlag == 0 )
                {
                    if(((HostCom_Buf_Temp[7] < 100) && (HostCom_Buf_Temp[7] == len))
                       || ( (HostCom_Buf_Temp[7] > 100) && (cmd  == (HostCom_Buf_Temp[7] - 100) ) ))
                    {
                        FindOneFlag = 1;
                        ReturnBuf[3] = HostCom_Buf_Temp[7];
                        ReturnBuf[4] = Curr_Sys_Parm1.Plan_time[i].Data.byteTime[0];
                        ReturnBuf[5] = Curr_Sys_Parm1.Plan_time[i].Data.byteTime[1];
                        ReturnBuf[6] = Curr_Sys_Parm1.Plan_time[i].Data.byteTime[2];
                        ReturnBuf[7] = Curr_Sys_Parm1.Plan_time[i].Data.byteTime[3];
                        ReturnBuf[8] = Curr_Sys_Parm1.Plan_time[i].Plan;
                        ReturnLen = 6;
                    }
                }
                if( HostCom_Buf_Temp[8] > 0 )
                {
                    if( FindTwoFlag == 0 )
                    {
                        if(((HostCom_Buf_Temp[8] < 100) && (HostCom_Buf_Temp[8] == len)) 
                           ||( HostCom_Buf_Temp[8] > 100 && (cmd  == (HostCom_Buf_Temp[8] - 100) ) ))
                        {
                          FindTwoFlag = 1;
                          ReturnBuf[9] = HostCom_Buf_Temp[8];
                          ReturnBuf[10] = Curr_Sys_Parm1.Plan_time[i].Data.byteTime[0];
                          ReturnBuf[11] = Curr_Sys_Parm1.Plan_time[i].Data.byteTime[1];
                          ReturnBuf[12] = Curr_Sys_Parm1.Plan_time[i].Data.byteTime[2];
                          ReturnBuf[13] = Curr_Sys_Parm1.Plan_time[i].Data.byteTime[3];
                          ReturnBuf[14] = Curr_Sys_Parm1.Plan_time[i].Plan;
                          ReturnLen = 12;
                        }
                    }
                }            
              }
              break;
        
      case 9:
            
            ReturnBuf[3] = HostCom_Buf_Temp[7];
            
            if( HostCom_Buf_Temp[7] == 0x01 )
            {
              TimeTemp = ConvertByte4toU32(&HostCom_Buf_Temp[8]);
              if ( TimeTemp - Curr_Sys_Parm.SecondOfRtc > 86400 )
              {//更新时间差超过一天
                  for(i=0;i<4;i++)
                      Curr_Sys_Parm.StartNengLiang[i] = 0;
              }
              Curr_Sys_Parm.SecondOfRtc = TimeTemp;
              SecondsToRTCTime( TimeTemp ,&Curr_RTC_time);              
              RTC_Set_RTC_Time(&Curr_RTC_time);    
              b_DateTimeUpdataed = 1;
              ReturnLen = 1;          
            
            }
            else
            if( HostCom_Buf_Temp[7] == 0x02 )
            {
              ReturnBuf[4] = HostCom_Buf_Temp[8];
              ReturnLen = 2;    
              if( HostCom_Buf_Temp[8] == 0x00) // 关闭自动上报（默认状态）
              {
                Send_State_Flag = 0;
              }
              else
              if ( HostCom_Buf_Temp[8] >= 5 )
              {
                SendStateTimerSave = HostCom_Buf_Temp[8];
                Send_State_Flag = 1;
                SendStateTimer = SendStateTimerSave*4 - 3 ;  //尽快发出第一次
              } 
            }
             else
            if( HostCom_Buf_Temp[7] == 0x03 )
            {
              ReturnBuf[4] = HostCom_Buf_Temp[8];
              ReturnLen = 2;    
              if( HostCom_Buf_Temp[8] == 0x00 ) // 设置指示灯状态
              {
                Open_Led_Flag = 0;
                Curr_Sys_Parm.Auto_Led_Flag = 0;
                Led_Changed_flag = 0;
              }
              else
              if( HostCom_Buf_Temp[8] == 0x01 )
              {
                  Open_Led_Flag = 1;
                  Curr_Sys_Parm.Auto_Led_Flag = 0;
                  Led_Changed_flag = 0;
              }
              else
              if( HostCom_Buf_Temp[8] == 0x02 )//周期定时开启时间指示灯
              {
                if( HostCom_Buf_Temp[13] == 0)
                {                
                   Curr_Sys_Parm.Auto_Led_Flag = 0;
                }
                else
                {                
                   Curr_Sys_Parm.Auto_Led_Flag = 1;                   
                }
                 Temp = HostCom_Buf_Temp[10];
                 Temp <<=  8;
                 Temp += HostCom_Buf_Temp[9];
                 Temp *= 30;
                 Curr_Sys_Parm.LedOnTime = Temp;
                 Temp = HostCom_Buf_Temp[12];
                 Temp <<=  8;
                 Temp += HostCom_Buf_Temp[11];
                 Temp *= 30;
                 Curr_Sys_Parm.LedOffTime = Temp;
                 if ( Curr_Sys_Parm.LedOnTime == Curr_Sys_Parm.LedOffTime )
                      Curr_Sys_Parm.Auto_Led_Flag = 0;
                 
                RefreshSysParaDelay = REFRESHTIME ;// 2500ms
                b_Refresh80 = 1;                
              }
              else
              if( HostCom_Buf_Temp[8] == 0x03 )//周期定时开启时间指示灯读取
              {

                    Temp = Curr_Sys_Parm.LedOnTime / 30;
                    ReturnBuf[5] = Temp & 0xff;
                    ReturnBuf[6] = ( Temp >> 8 ) ;
                    Temp = Curr_Sys_Parm.LedOffTime / 30;
                    ReturnBuf[7] = Temp & 0xff;
                    ReturnBuf[8] = ( Temp >> 8 ) ;
                    ReturnBuf[9] = Curr_Sys_Parm.Auto_Led_Flag ;
                    ReturnLen = 7;    
              } 
            }
            else
            if( HostCom_Buf_Temp[7] == 0x04 )//删除指定定时参数
            {
              ReturnLen = 1;   
              TimeTemp = HostCom_Buf_Temp[11];
              TimeTemp <<= 8;
              TimeTemp |= HostCom_Buf_Temp[10];
              TimeTemp <<= 8;
              TimeTemp |= HostCom_Buf_Temp[9];
              TimeTemp <<= 8;
              TimeTemp |= HostCom_Buf_Temp[8];            
                
              for( i = 0; i <  MAX_SCH_COUNT; i++ )
              { 
                  if (Curr_Sys_Parm1.Plan_time[i].Plan == HostCom_Buf_Temp[12] )
                  {
                      if( Curr_Sys_Parm1.Plan_time[i].Data.Time == TimeTemp )
                      {
                          if ( HostCom_Buf_Temp[11] & 0x80)
                          {
                              if( CycleTime_cout > 0 )
                              {
                                  CycleTime_cout--; 
                              }
                          }
                          else
                          {
                              if( AbsTime_cout > 0 )
                              {
                                  AbsTime_cout--; 
                              }
                          }
                          Curr_Sys_Parm1.Plan_time[i].Data.Time = 0;
                          Curr_Sys_Parm1.Plan_time[i].Plan = 0;
                      }
                  }
              }

              UpdateTimeAdd();
              RefreshSysParaDelay = REFRESHTIME ;// 2500ms      
              b_Refresh81 = 1;
              
            }           
            else
            if( HostCom_Buf_Temp[7] == 0x05 )//删除指定目标ID（仅用于内部测试）
            {
              ReturnBuf[4] = HostCom_Buf_Temp[8];
              ReturnLen = 2;   
               if( HostCom_Buf_Temp[8] == 0xff ) // 删除所有
               {
                 for ( i = 0; i< Curr_Sys_Parm.UserCout; i++ )//当前操作使用的APP设备ID是否注册过
                 {
                   for( j = 0; j< 4; j++)
                   {
                      Curr_Sys_Parm.User[i][j] = 0;                         
                   }
                 }
                  Curr_Sys_Parm.UserCout = 0;
                  RefreshSysParaDelay = REFRESHTIME ;// 2500ms      
                  b_Refresh80 = 1;
               }
               else
               if( HostCom_Buf_Temp[8] == 0 ) 
               {
                  for ( i = 0; i< Curr_Sys_Parm.UserCout; i++ )//当前操作使用的APP设备ID是否注册过
                  {
                      for( j = 0; j< 4; j++)
                      {
                        if( Curr_Sys_Parm.User[i][j] != HostCom_Buf_Temp[9+j] )
                          break;
                      }
                      if( j >= 4 )
                      {
                        for( j = 0; j< 4; j++)
                        {
                          Curr_Sys_Parm.User[i][j] = 0;
                        }                        
                        Curr_Sys_Parm.UserPriority[i] = 0;
                        //Curr_Sys_Parm.UserCout不变
                        RefreshSysParaDelay = REFRESHTIME ;// 2500ms      
                        b_Refresh80 = 1;
                      }
                  }               
               }
            }
            else
            if( HostCom_Buf_Temp[7] == 0x06 )   //清除电量累计
            {
                ReturnBuf[4] = HostCom_Buf_Temp[8];
                ReturnLen = 2;   
                if( HostCom_Buf_Temp[8] == 0xff ) // 删除所有
                {
                   for ( i = 0; i< 4; i++ )
                   {
                      Curr_Sys_Parm.NengLiang[i] = 0;
                   }
                    RefreshSysParaDelay = REFRESHTIME ;// 2500ms      
                    b_Refresh80 = 1;
               }
               else
               if(  HostCom_Buf_Temp[8] > 0 && HostCom_Buf_Temp[8] <= 4 ) 
               {
                  Curr_Sys_Parm.NengLiang[HostCom_Buf_Temp[8]-1] = 0;
                  RefreshSysParaDelay = REFRESHTIME ;// 2500ms      
                  b_Refresh80 = 1;
               }
            }
            else
            if ( HostCom_Buf_Temp[7] == 0x07 )   //读取电量累计
            {
                if ( HostCom_Buf_Temp[8] <= 31 )
                {
                    ReturnBuf[4] = HostCom_Buf_Temp[8];
                    if ( HostCom_Buf_Temp[8] == 0x00 )
                        ReturnBuf[5] = 0;
                    else
                        ReturnBuf[5] = Curr_Sys_Parm2.DaylySaveNL[HostCom_Buf_Temp[8]-1][0] >> 12;
                    
                    for(i=0;i<4;i++)    //最多支持8个插孔
                    {
                        if ( HostCom_Buf_Temp[8] == 0x00 )
                              Temp = (Curr_Sys_Parm.StartNengLiang[i]+50)/100;  //四舍五入
                        else
                              Temp = Curr_Sys_Parm2.DaylySaveNL[HostCom_Buf_Temp[8]-1][i];
                        SetOneDayPower(&ReturnBuf[6],i,Temp);
                    }
                    ReturnLen = 8;
                }
            }
            else
            if( HostCom_Buf_Temp[7] == 0xFC )
            {
                if ( HostCom_Buf_Temp[8] < 3 )
                {
                    LedColorSet[HostCom_Buf_Temp[8]][0] = HostCom_Buf_Temp[9];
                    LedColorSet[HostCom_Buf_Temp[8]][1] = HostCom_Buf_Temp[10];
                    LedColorSet[HostCom_Buf_Temp[8]][2] = HostCom_Buf_Temp[11];
                    ReturnBuf[4] = HostCom_Buf_Temp[8];
                    Led_Changed_flag = 0;
                    Led_Flash_flag = 0;
                    ReturnLen = 2;   
                }
            }
            else
            if( HostCom_Buf_Temp[7] == 0xFD )
            {
                debug = HostCom_Buf_Temp[8];
                ReturnBuf[4] = HostCom_Buf_Temp[8];
                ReturnLen = 2;   
            }
            break;    
        
      case 10:
        
            ReturnBuf[3] = FW_VERSION / 100;
            
            len = FW_VERSION % 100;
            ReturnBuf[4]  = len % 10;
            ReturnBuf[4]  += ( ( len /10 ) << 4 );
            
            LastOprationTick = osal_getClock();
            
            for ( i = 0; i< 4; i++ )           
              ReturnBuf[i+5] = *( (uint8 *)&LastOprationTick + i );     
            
            memcpy(&ReturnBuf[9],BT1_LocalDevAddr,6);
            
            ReturnLen = 12;
            
            break;
        
  //    case 11:
  //      
  //      
  //      break;      
  //      
      case 12://读写当前软件自定义蓝牙设备名称
            ReturnBuf[3] = HostCom_Buf_Temp[7];
            ReturnBuf[4] =  HostCom_Buf_Temp[8] ;
            if( HostCom_Buf_Temp[7] == 0x00 )//读取
            {
             
              if( HostCom_Buf_Temp[8] == 0x01 )
              {
                  for( i = 0; i <  12; i++ )
                  {
                    ReturnBuf[5+i] = Curr_Sys_Parm.devName[i];
                    if( Curr_Sys_Parm.devName[i] == 0)               
                      break;
                    
                    
                  }
                  if(i>=12)
                  {
                    ReturnLen = i+2;
                  }
                  else
                  {
                    ReturnLen = i+2+1;
                  }
              
              }
              else
              if( HostCom_Buf_Temp[8] == 0x02 )
              {
                  for( i = 0; i <  12; i++ )
                  {
                    ReturnBuf[5+i] = Curr_Sys_Parm.devName[i+12];
                    if( Curr_Sys_Parm.devName[i+12] == 0)               
                      break;                    
                    
                  }  
                  if(i>=12)
                  {
                    ReturnLen = i+2;
                  }
                  else
                  {
                    ReturnLen = i+2+1;
                  }
              }
              

            }
            else
            if( HostCom_Buf_Temp[7] == 0x01 )//设置
            {
              
              if( HostCom_Buf_Temp[8] == 0x01 )
              {
                for( i = 0; i < 8; i++ )
                {
                  if( HostCom_Buf_Temp[9+i] != 0)
                      Curr_Sys_Parm.devName[i] = HostCom_Buf_Temp[9+i];
                  else
                  {
                      Curr_Sys_Parm.devName[i] = 0;
                      RefreshSysParaDelay = 2 ;// 2500ms
                      b_Refresh80 = 1;
                      //b_Bt_Restart = 1;                  
                      break;
                  }
                }
                
              }  
              else
              if( HostCom_Buf_Temp[8] == 0x02 )
              {
                for( i = 0; i < 8; i++ )
                {
                  if( HostCom_Buf_Temp[9+i] != 0)
                    Curr_Sys_Parm.devName[8+i] = HostCom_Buf_Temp[9+i];
                  else
                  {
                      Curr_Sys_Parm.devName[8+i] = 0;
                      RefreshSysParaDelay = 2 ;// 2500ms
                      b_Refresh80 = 1;
                      //b_Bt_Restart = 1;                  
                      break;
                  }
                }
              }
              else
              if( HostCom_Buf_Temp[8] == 0x03 )
              {
                for( i = 0; i < 8; i++ )
                {
                  if( HostCom_Buf_Temp[9+i] != 0)
                    Curr_Sys_Parm.devName[16+i] = HostCom_Buf_Temp[9+i];
                  else
                  {
                        Curr_Sys_Parm.devName[16+i]= 0; 
                        RefreshSysParaDelay = 2 ;// 2500ms
                        b_Refresh80 = 1;
                        //b_Bt_Restart = 1;                  
                        break;
                  }
                }
              }            
              ReturnLen = 2;
            }
            break;
        
      case 13://读取话机MCU唯一ID（不受目标ID限制）
            Read_System_ID(&ReturnBuf[3]);
            ReturnLen = 12;
            break;    
        
      case 14://加密鉴权信息操作（不受目标ID限制
            Temp = 65535;
            ReturnBuf[3] = HostCom_Buf_Temp[7] ;
            ReturnBuf[4] = HostCom_Buf_Temp[8] ;
            if( Bt_ConnectingTimer > 0)
            {
              if( HostCom_Buf_Temp[7] == 0x51 )            
              {//读取话机内部预留数据（固定读取20个字节，分两次读取）
                
                if( HostCom_Buf_Temp[8] == 1 )
                {
                  
                  //memcpy(Curr_Sys_Parm.EncryptBuf,&HostCom_Buf_Temp[9],8);
                  memcpy( SlaveSendBuf,&HostCom_Buf_Temp[9],8 );
                  
                  LastOprationTick = osal_getClock();
                  //LastOprationTick = 0x1D5E9340;
                  
                  for ( i = 0; i< 4; i++ )           
                    ReturnBuf[i+5] = *( (uint8 *)&LastOprationTick + i );        
                  
                  ReturnLen = i+2;
                }
                else
                  if( HostCom_Buf_Temp[8] == 2 )
                  {
                    
                    // memcpy(&Curr_Sys_Parm.EncryptBuf[8],&HostCom_Buf_Temp[9],8);
                    memcpy( &SlaveSendBuf[8],&HostCom_Buf_Temp[9],8 );
                    
                    for ( i = 0;i < 16;i++ )
                      SlaveSendBuf[i] += Curr_Sys_Parm.EncryptBuf[i];
                    
                    for (i = 10;i < 16;i++ )
                      SlaveSendBuf[i] += BT1_LocalDevAddr[i-10];
                    
                    Read_System_ID(&ReturnBuf[5]);
                    
                    for ( i = 0;i < 12;i++ )
                      SlaveSendBuf[i] += ReturnBuf[5+i];
                    
                    
                    for ( i = 0;i < 16;i++ )
                      SlaveSendBuf[i] += *( (uint8 *)&LastOprationTick + ( i%4 ) );
                    
                    memcpy(&ReturnBuf[5],SlaveSendBuf,8);
                    ReturnLen = 10;
                  }
                  else
                    if( HostCom_Buf_Temp[8] == 3 )
                    {
                      
                      memcpy(&ReturnBuf[5],&SlaveSendBuf[8],8);
                      ReturnLen = 10;
                    }  
                
              }
              //else
              if( HostCom_Buf_Temp[7] == 0xc5) 
              {//鉴权认证操作（启动操作授权）
                
                if( HostCom_Buf_Temp[8] == 1 )
                {
                  memcpy(SlaveSendBuf,&HostCom_Buf_Temp[9],8);
                  ReturnLen = 2;
                }
                //else
                if( HostCom_Buf_Temp[8] == 2 )
                {
                  
                  memcpy(&SlaveSendBuf[8],&HostCom_Buf_Temp[9],8);
                  
                  
                  Read_System_ID(&ReturnBuf[5]);
                  
                  
                  ReturnBuf[5+12]=0x00;
                  ReturnBuf[5+13]=0x00;
                  ReturnBuf[5+14]=0x00;
                  ReturnBuf[5+15]=0x00;
                  
                  //check_buf(Curr_Sys_Parm.EncryptBuf,&currbuf[2],(u8 *)&LastOprationTick,&cmdbuf[3]);
                  
                  for (i=0;i<16;i++)
                    ReturnBuf[i+5] += ~Curr_Sys_Parm.EncryptBuf[i];
                  for (i=0;i<16;i++)
                    ReturnBuf[i+5] += *( (uint8 *)&LastOprationTick + ( i%4 ) );
                  
                  for (i=0;i<16;i++)
                  {
                    if ( SlaveSendBuf[i] != ReturnBuf[i+5] )
                      break;
                  }
                  if ( i >= 16 )
                  { 
                      if( HostCom_Buf_Temp[3] != 0 || HostCom_Buf_Temp[4] != 0 || HostCom_Buf_Temp[5] != 0 || HostCom_Buf_Temp[6] != 0 )
                      {
                          if( Curr_Sys_Parm.UserCout >= 8 )
                          {
                              cmd = 0;
                              for( i = 0; i < Curr_Sys_Parm.UserCout; i++)
                              {
                                if( Curr_Sys_Parm.UserPriority[i] < Temp)
                                {
                                  Temp = Curr_Sys_Parm.UserPriority[i];
                                  cmd = i;
                                }
                              }
                              Curr_Sys_Parm.User[cmd][0] = HostCom_Buf_Temp[3];
                              Curr_Sys_Parm.User[cmd][1] = HostCom_Buf_Temp[4];
                              Curr_Sys_Parm.User[cmd][2] = HostCom_Buf_Temp[5];  
                              Curr_Sys_Parm.User[cmd][3] = HostCom_Buf_Temp[6];
                          }
                          else
                          {
                            Curr_Sys_Parm.User[Curr_Sys_Parm.UserCout][0] = HostCom_Buf_Temp[3];
                            Curr_Sys_Parm.User[Curr_Sys_Parm.UserCout][1] = HostCom_Buf_Temp[4];
                            Curr_Sys_Parm.User[Curr_Sys_Parm.UserCout][2] = HostCom_Buf_Temp[5];  
                            Curr_Sys_Parm.User[Curr_Sys_Parm.UserCout][3] = HostCom_Buf_Temp[6];
                            Curr_Sys_Parm.UserCout++;
                          }

                          RefreshSysParaDelay = REFRESHTIME ;// 2500ms        
                          b_Refresh80 = 1;
                      }
                      b_APP_User1_OK = 1;
                      Bt_ConnectingTimer = 0;
                      Led_Changed_flag = 0;
                      ReturnBuf[5] = 0x00;
                      ReturnLen = 3;
                  }
                  else
                  {
                    
                    ReturnBuf[5] = 0xff;//鉴权失败
                    ReturnLen = 3;
                    //ReturnBuf[3] = 0xff;//鉴权失败
                    b_APP_User1_OK = 0;
                    Led_Changed_flag = 0;
                  }
                  
                  
                  
                  
                }          
                
              }          
            }
            else
            {
            
               ReturnBuf[3] = 0xfb;//未进入鉴权状态
               ReturnLen = 1;
            }
            
            break;
    default:
            
               ReturnBuf[3] = 0xff;//指令错误
               ReturnLen = 1;          
               break;
      
  //    case 15:
  //      
  //      break;         
    }
    
  }
  else
  {
    ReturnBuf[3] = 0xfd;//目标ID不在已有注册表中
    ReturnLen = 1;   
  
  }   
  SendingReturnBuf();  
}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLEPeripheral_Init( uint8 task_id )
{
  simpleBLEPeripheral_TaskID = task_id;

  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  
    #if defined( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      uint8 initial_advertising_enable = FALSE;
    #else
      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = TRUE;
    #endif

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    //GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
 

    
    char devName[26];
    GetCurrBTphoneName(0,devName);
    uint8 AttDeviceNameLen = osal_strlen( devName);
    uint8 pSscanRspDataLen = ( 2 + AttDeviceNameLen);
    uint8 *pSscanRspData = osal_mem_alloc(32);//pSscanRspDataLen);
    if(pSscanRspData)
    {
      //  uint8 index = 0;
        
        pSscanRspData[0] = AttDeviceNameLen + 1;
        pSscanRspData[1] = GAP_ADTYPE_LOCAL_NAME_COMPLETE;
        osal_memcpy(&pSscanRspData[2], devName, AttDeviceNameLen);
        

        GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, pSscanRspDataLen, pSscanRspData );

        osal_mem_free(pSscanRspData);
    }


   // GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, devName );    
    
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
    

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  

  // Set the GAP Characteristics
  //GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, devName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;
 
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }



  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  
  DevInfo_AddService();                           // Device Information Service
  
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the SimpleProfile Characteristic Values
  {
#if 0    
    uint8 charValue1 = 1;
    uint8 charValue2 = 2;
    uint8 charValue3 = 3;
    uint8 charValue4 = 4;
    uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
#endif    
    uint8 charValue6[SIMPLEPROFILE_CHAR6_LEN] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I' };
#if 0
    uint8 charValue7[SIMPLEPROFILE_CHAR7_LEN] = { 1, 2, 3, 4, 5 };
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint8 ), &charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &charValue3 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &charValue4 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5 );
#endif
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR6, SIMPLEPROFILE_CHAR6_LEN, charValue6 );
//    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR7, SIMPLEPROFILE_CHAR7_LEN, charValue7 );
  }


  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  // 需要关闭的CLK自动分频，在初始化中加入HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_DISABLE_CLK_DIVIDE_ON_HALT )?  // 如果开启，会导致频率自动切换，DMA工作受到影响，小范围丢数。  
  // 这里把他关闭， 如果想降低功耗， 这个应该要开启的， 这里矛盾了  
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_DISABLE_CLK_DIVIDE_ON_HALT );
  //HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

  // 信号发射强度
  HCI_EXT_SetTxPowerCmd(LL_EXT_TX_POWER_4_DBM);
  
  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );

}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function
 
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
    {
     simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SBP_START_DEVICE_EVT )
  {    
    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

    // Start Bond Manager
   // VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

    // Set timer for first periodic event
    //osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );

    //CheckKeyForSetAllParaDefault(); //按键按下3秒， 回复出厂设置
    
    return ( events ^ SBP_START_DEVICE_EVT );
  }

  if ( events & SBP_PERIODIC_EVT )
  {
    // Restart timer
    //if ( SBP_PERIODIC_EVT_PERIOD )
    {
      //osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    }
    // Perform periodic application task
    //performPeriodicTask();
    return (events ^ SBP_PERIODIC_EVT);
  }

#if defined ( PLUS_BROADCASTER )
  if ( events & SBP_ADV_IN_CONNECTION_EVT )
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );

    return (events ^ SBP_ADV_IN_CONNECTION_EVT);
  }
#endif // PLUS_BROADCASTER

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {

  default:
    // do nothing
    break;
  }
}


/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        SSS = 1;
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR,systemId);    //可以后期独立读取
        
        BT1_LocalDevAddr[0] = systemId[5];
        BT1_LocalDevAddr[1] = systemId[4];
        BT1_LocalDevAddr[2] = systemId[3];
        BT1_LocalDevAddr[3] = systemId[2];
        BT1_LocalDevAddr[4] = systemId[1];
        BT1_LocalDevAddr[5] = systemId[0];
        
        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = BT1_LocalDevAddr[5];
        systemId[1] = BT1_LocalDevAddr[4];
        systemId[2] = BT1_LocalDevAddr[3];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = BT1_LocalDevAddr[0];
        systemId[6] = BT1_LocalDevAddr[1];
        systemId[5] = BT1_LocalDevAddr[2];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
        
        char buf[B_MAX_ADV_LEN];
        buf[0] = 14;
        buf[1] = GAP_ADTYPE_MANUFACTURER_SPECIFIC;//GAP_ADTYPE_LOCAL_NAME_COMPLETE;
        GAPRole_GetParameter( GAPROLE_BD_ADDR,&buf[2]);    //可以后期独立读取
        buf[8] = ~buf[2];
        buf[9] = ~buf[3];
        buf[10] = ~buf[4];
        buf[11] = ~buf[5];
        buf[12] = ~buf[6];
        buf[13] = ~buf[7];
        buf[14] = 0x01;
        GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, buf[0]+1, buf );
        
        
    //GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, devName );
       
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          // Display device address
          //HalLcdWriteString( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_2 );
          //HalLcdWriteString( "Initialized",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

        //mac 地址   bdAddr2Str( pEvent->initDone.devAddr ) 的输出格式是， 例如     0xD03972A5F3 
        //osal_memcpy(sys_config.mac_addr, bdAddr2Str( ownAddress )+2, 12);
        //sys_config.mac_addr[12] = 0
        //PrintAllPara();
      }
      break;

    case GAPROLE_ADVERTISING:
      {
        SSS = 2;
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          //HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_CONNECTED:
      {
        SSS = 3;
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          //HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
        //LedSetState(HAL_LED_MODE_OFF);
        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, masterAddress);
        GAPRole_GetParameter( GAPROLE_CONNHANDLE, &gapConnHandle );//获取Connection Handle连接的Handle, 设备连上以后会产生一个Handle. 
        //NPI_WriteTransport("Connected\r\n", 11);
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      {
        SSS = 4;
      }
      break;      

    case GAPROLE_WAITING:
      {
        SSS = 5;
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          //HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
         SSS = 6;
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          //HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_ERROR:
      {
        SSS = 7;
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          //HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    default:
      {
        SSS = 8;
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          //HalLcdWriteString( "",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

  }

  gapProfileState = newState;
  Led_Changed_flag = 0;
  if ( gapProfileState != GAPROLE_CONNECTED )
  {
      b_APP_User1_OK = 0;
      Send_State_Flag = 0;
  }
  
#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif

    //LCD_WRITE_STRING( "?", HAL_LCD_LINE_1 );

}

static void peripheralRssiReadCB( int8 rssi )
{
    //LCD_WRITE_STRING_VALUE( "RSSI -dB:", (uint8) (-rssi), 10, HAL_LCD_LINE_1 );
}

static void BlueCommondRev(uint8 newChar6Value[SIMPLEPROFILE_CHAR6_LEN],uint8 len)
{
  
  if( b_GetNewData == 1 )
  {
  
      b_GetNewData = 0;
      memcpy( HostCom_Buf2 ,newChar6Value, len ) ;
      
  }
  else
  {
      b_GetNewData = 1;
      memcpy( HostCom_Buf1 ,newChar6Value, len ) ;
  }
      
}

static void simpleProfileChangeCB( uint8 paramID )
{
  uint8 newChar6Value[SIMPLEPROFILE_CHAR6_LEN];
  uint8 rtbyte;//newValue,
  switch( paramID )
  {
#if 0
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &newValue, &rtbyte );

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        //HalLcdWriteStringValue( "Char 1:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &newValue, &rtbyte );

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        //HalLcdWriteStringValue( "Char 3:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;
#endif    
    case SIMPLEPROFILE_CHAR6:
            SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR6, newChar6Value, &rtbyte );
            BlueCommondRev(newChar6Value,rtbyte);
            osal_set_event( AppUserProcess_TaskID, AUP_GETNEWDATA_EVT );
       /*
      if(returnBytes > 0)
      {
        NPI_WriteTransport(newChar6Value,returnBytes);
        
          uint8   i,j;
          
          if(returnBytes > 12)returnBytes=12;
            returnBytes1=(returnBytes)*2;
            for ( i = 0,j=0; i <returnBytes1 ; j++ )
            {
                newChar6Value1[i++]=j+'0';
                newChar6Value1[i++]=newChar6Value[j];
            }
          
            static attHandleValueNoti_t pReport;
            pReport.len = returnBytes1;
            pReport.handle = 0x0035;
            osal_memcpy(pReport.value, newChar6Value1, returnBytes1);
            GATT_Notification( 0, &pReport, FALSE );    
      }
      */
      break;
      
    default:
      // should not reach here!
      break;
  }
}

/*********************************************************************
*********************************************************************/
