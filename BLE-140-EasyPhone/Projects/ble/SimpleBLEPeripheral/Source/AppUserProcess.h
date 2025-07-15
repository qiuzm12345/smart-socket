#ifndef APPUSERPROCESS_H
#define APPUSERPROCESS_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * MACROS
 */
//#define debug_test
   
#define         FW_VERSION              (100)

#define	    START_YEAR	    2016
#define	    WEEK_FIRST_DAY	5       //星期五  2016.1.1

#define     MAX_SCH_COUNT	    50

#define     RETURN_DATA_SIZE  32    //最大回应数据长度（非ASC）
#define     HOST_COM_DATA_SIZE 32
#define     LOOP_BUF_DATA_SIZE 48   //最小42个字节
  
#define     SEND_LOOP_BUF_SIZE 128   //(RETURN_DATA_SIZE*2+16)*2
  
#define		BT_CMD_MAX_SIZE		51
 
#define         HALFTIME                20 //单灯两灭时间 周期20ms
   
#define         CYCTIME                 4 // 双灯或三灯闪烁 时隙
   
#define         OFFTIME                 2   //熄灭时间  
   
#define         LONGTIME                15  // 长亮时间 
 
#define         SHORTTIME               2   // 闪亮时间 

#define         REFRESHTIME             10     //2.5S
  
#define         AUP_TIMEROUT              25
   
#define         AUP_TIMEROUTL              25
   
#define         AUP_START_DEVICE_EVT                              0x0001
#define         AUP_PERIODIC_EVT                                  0x0002
#define         AUP_TIMEROUT_EVT                                  0x0004
#define         AUP_STOPCHECK_EVT                                 0x0008
#define         AUP_GETNEWDATA_EVT                                0x0010
#define         AUP_SENDDATA_EVT                                  0x0020

  /*
#define         AUP_GET_FSK_END_EVT                             0x0040
#define         AUP_GET_DTMF_END_EVT                            0x0080
   
#define         AUP_STACHANGED_EVT                              0x0100   
#define         AUP_KEYDOWN_EVT                                 0x0200   
#define         AUP_PWMBUFCHK_EVT                               0x0400   
#define         AUP_DTMF_DET_EVT                                0x0800
*/
#define         AUP_SECOND_PROC_EVT                               0x1000
#define         AUP_GET_UART1_RX_EVT                              0x2000
#define         AUP_POWEROFF_DET_EVT                              0x4000

   
#define AT_InitRcv_OK	0x01
#define AT_GverRcv_OK	0x02
#define AT_GlbdRcv_OK	0x03
#define AT_SldnRcv_OK	0x04
#define AT_SecuRcv_OK	0x05
#define AT_ScanRcv_OK	0x06 
#define AT_ClasRcv_OK	0x07
#define AT_PairRcv_OK	0x08
#define AT_SpinRcv_OK	0x09
#define AT_RespRcv_OK	0x0A
#define AT_DialRcv_OK	0x0B
#define AT_SminRcv_OK	0x0C
#define AT_SmsdRcv_OK	0x0D
#define AT_SdpbRcv_OK	0x0E
#define AT_DiscRcv_OK	0x0F
#define AT_VolcRcv_OK	0x10
#define AT_HookRcv_OK	0x11
#define	AT_DtmfRcv_OK	0x12
#define AT_GrdnRcv_OK	0x13
#define AT_GprdRcv_OK	0x14
#define AT_ConnRcv_OK	0x15
#define AT_DprdRcv_OK	0x16
#define AT_InquRcv_OK	0x17
#define AT_RdsnRcv_OK	0x18
#define AT_CtrsRcv_OK	0x19
#define AT_PbinRcv_OK	0x1A
#define AT_PbdaRcv_OK	0x1B
#define AT_PblsRcv_OK	0x1C
#define AT_DataRcv_OK	0x1D


#define	AT_GetNameRcv_OK	0x1E
#define	AT_GetPinRcv_OK		0x1F
#define	AT_GetModeRcv_OK	0x20

#define AT_TestRcv_OK	0x51

#define AT_INDIRcv_OK	0x80
#define AT_PSSIRcv_OK	0x81


typedef struct _RTC_Time
{
	uint16	Year;
	uint8	Month;	//1-12
	uint8	Day;	//1-31
	uint8	Hour;
	uint8	Minute;
	uint8	Second;
	uint8	Week;
}RTC_Time;
typedef struct _Plan_Time
{
  uint8 Plan;
  union 
  { 
      uint32 Time;
      uint8 byteTime[4];
  }Data;
}PLAN_TIME;

typedef struct _Auto_PowerDown
{
  //uint8 Channel;
  uint8 Control;
  uint8 Percent;
}AUTOPOWERDOWN;

typedef struct _SYS_PARM
{
	uint8	VerCode;
	uint8	standby;
	uint16	ParmLenth;	
	uint8	EncryptBuf[16]; //加密字节
        uint8   devName[24];//设备名字
        uint8   User[8][4];
        uint16  UserPriority[8];
        //uint8   ConnectUser;
        uint8   UserCout;
        uint16  LedOnTime;
        uint16  LedOffTime;
        //uint8   Auto_PwrDn_cout;
        uint32  SecondOfRtc;
        uint16  PwUpMul;//上升功率倍数
        uint16  PwDnMul;//下降功率倍数
        uint32	PwOffSet[4];// 插孔零点校正
        uint32	KUrms5;	// 电压系数
        uint32	KPrms[4];	// 插孔功率系数
        
        bool   Auto_Led_Flag ;
        
        uint8   ChaKongCtrl;
        
        bool   ForceSwitch_Flag;
        
        uint8   AutoControlEable;
        AUTOPOWERDOWN   Auto_PowerDown[4];
        
        uint32  NengLiang[4];
        uint32  StartNengLiang[4];
        
	uint16	CRC16;	//BitByte及后续数据CRC16校验

}SYS_PARM;


typedef struct _SYS_PARM1
{

   PLAN_TIME       Plan_time[MAX_SCH_COUNT];
    
}SYS_PARM1;

typedef struct _SYS_PARM2
{
    uint16       DaylySaveNL[31][4];
    
}SYS_PARM2;

/*********************************************************************

 * FUNCTIONS
 */
extern  unsigned char SysWorkMode;
extern  uint16 PowerVolume,BatteryVolume,CheckVolume;

extern void AppUser_Init( uint8 task_id );
extern uint16 AppUser_ProcessEvent( uint8 task_id, uint16 events );

extern uint8 AppUserProcess_TaskID;
extern SYS_PARM	Curr_Sys_Parm;
extern SYS_PARM1	Curr_Sys_Parm1;
extern SYS_PARM2	Curr_Sys_Parm2;

extern unsigned char CheckCRC( unsigned char  * Buffer, unsigned short lenth );
extern uint8 ChaKongHave;
extern uint8 DATA ChaKongPwOn;

//SFRBIT( EXT_BITS  ,  0x21, EXT_BITS7, EXT_BITS6, EXT_BITS5, EXT_BITS4, EXT_BITS3, EXT_BITS2, EXT_BITS1, EXT_BITS0 )
//SFRBIT(EXT_BITS  ,  0x21);

/*********************************************************************
*********************************************************************/
#include "hal_board.h"
  
/**************************************************************************************************
 * MACROS
 **************************************************************************************************/



void InitSleepTimer(void);





#ifdef __cplusplus
}
#endif

#endif /* APPUSERPROCESS_H */
