
#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "string.h"

#include "bcomdef.h"
#include "OSAL.h"

#include "stdio.h"
#include "stdlib.h"
#include "AppUserProcess.h"
#include "hal_shiftout.h"
#include "npi.h"
#define __RN8209D_C__
#include "RN8209D.h"
#define RETRYMAX  5
#define   DianPer1     4
#define   DianPer2     4
#define   DianPer3     4
#define   DianPer4     4
#define   DianPer11     15
#define   DianPer21     15
#define   DianPer31     15
#define   DianPer41     15

#define   DianPer     4
#define   DianPerH     15

uint32 PwChnlLow[4] = {PWLOWCONST,PWLOWCONST,PWLOWCONST,PWLOWCONST};

extern bool AutoSendFlag,Led_Changed_flag,DelaySwitchFlag;
extern bool debug;
extern uint8 ChaKongLockOff;
void UpdatePwOnIO(void);
extern uint8   PowerOverAlarm;

uint8 SaveCnt[4];

uint8 RetryCount;
uint32	fnHexToBcd_u32(uint32 Dat)
{	
	uint32 result = 0;
	
	Dat = Dat % 100000000;
	result += (Dat / 10000000) * 0x10000000;
	Dat = Dat % 10000000;	
	result += (Dat / 1000000) * 0x1000000;
	Dat = Dat % 1000000;
	result += (Dat / 100000) * 0x100000;
	Dat = Dat % 100000;
	result += (Dat / 10000) * 0x10000;
	Dat = Dat % 10000;	
	result += (Dat / 1000) * 0x1000;
	Dat = Dat % 1000;
	result += (Dat / 100) * 0x100;
	Dat = Dat % 100;
	result += (Dat / 10) * 0x10;
	Dat = Dat % 10;	
	result += Dat;
	
	return(result);
}
sDF09	fnDFConver_Hex32ToDF09(uint32 Dat)
{	
	sDF09	Result;	
	osal_memset(&Result , 0 , sizeof(sDF09) );
	//if(Dat < 0) Result.S = 1;
	//else Result.S = 0;	
	
	//Dat = abs(Dat) % 800000;
	Dat = fnHexToBcd_u32(Dat);
	
	Result.Dat0 = Dat;
	Result.Dat1 = Dat >> 8;
	Result.Dat2 = Dat >> 16;
	return(Result);	
}

void fnScomPk_Init(uint8 ComPort)  //串口Buf初始化
{
	ComPack[ComPort].EFlag = 0;
 	ComPack[ComPort].RxLen = 0;
 	ComPack[ComPort].TxLen = 0;
  	ComPack[ComPort].TimeOutStamp = 0;	
 	osal_memset(&ComPack[ComPort].RxBuf[0] , 0 , MAX_COMPACK_SIZE);
	osal_memset(&ComPack[ComPort].TxBuf[0] , 0 , MAX_COMPACK_SIZE);
	

}
/*****************************************************************************
** Function name:Rn8209Delay(uint16 t)
**
** Description:延时函数
**
** Parameters: t ：延时时间
**
** Returned value:	NONE
**
******************************************************************************/


uint16 fnHexToBcd_u16(uint16 Dat)
{
	uint16 Result = 0;

	Dat = Dat % 10000;
	Result += (Dat / 1000) * 0x1000;
	Dat = Dat % 1000;	
	Result += (Dat / 100) * 0x100;
	Dat = Dat % 100;	
	Result += (Dat / 10) * 0x10;
	Dat = Dat % 10;	
	Result += Dat;
		
	return(Result);	
}

uint16 fnDFConver_Bcd16To16(int16 Dat)
{
	uint16 Result;
	Result = abs(Dat) % 8000;			
	Result = fnHexToBcd_u16(Result);
        if(Dat < 0 ) Result |= 0x8000;
	else Result &= 0x7fff;	
	return(Result);
}
uint32 fnDFConver_Bcd32To32(int32 Dat)
{
	uint32 Result;
	Result = labs(Dat) % 80000000;			
	Result = fnHexToBcd_u32(Result);
        if(Dat < 0 ) Result |= 0x80000000;
	else Result &= 0x7fffffff;	
	return(Result);
}

/*****************************************************************************
** Function name:fnRN8209_Write(uint8 wReg,uint8 *pBuf,uint8 ucLen)
**
** Description:写RN8209寄存器
**
** Parameters:wReg 寄存器地址，*pBuf待写入值的存放地址，ucLen：待写入值的长度
**
** Returned value:	操作标识-成功或失败
**
******************************************************************************/

ErrorStatus fnRN8209_Write(uint8 wReg,uint8 *pBuf,uint8 ucLen)
{
  uint8 i,temp,chksum;
  uint8 Buf[4];
  ErrorStatus	err;	
// 	ComPack[SCOM_PORT_RN8209].TxLen = 0;
//	osal_memset(&ComPack[SCOM_PORT_RN8209].TxBuf[0] , 0 , MAX_COMPACK_SIZE);
	
  if( (ucLen == 0) || (ucLen > 4) ) return(ERROR);
  err = RIGHT;	
  //写数据前，先发送命令字节，命令字节的最高位bit[7]=0:读操作；1：写操作；bit[6:0]为待操作寄存的地址		
  temp =wReg|0x80;//待操作寄存器地址最高位或1，使命令字节为写命令

  ComPack[SCOM_PORT_RN8209].TxBuf[0] = temp;
  chksum = temp;		
  for( i = 0; i < ucLen ;i++ )
  {		

    ComPack[SCOM_PORT_RN8209].TxBuf[i+1] =  pBuf[ucLen-i-1];
    chksum += pBuf[i];
  }
  chksum = ~chksum;
  ComPack[SCOM_PORT_RN8209].TxBuf[i+1] =  chksum;
  ComPack[SCOM_PORT_RN8209].TxLen = ucLen+2;
  for( i = 0 ;i < ComPack[SCOM_PORT_RN8209].TxLen ; i++ )
  {
    Buf[i] = ComPack[SCOM_PORT_RN8209].TxBuf[ComPack[SCOM_PORT_RN8209].TxLen-i-1];
    
  }
  memcpy(ComPack[SCOM_PORT_RN8209].TxBuf,Buf,ComPack[SCOM_PORT_RN8209].TxLen);
  osal_start_timerEx( AppUserProcess_TaskID, AUP_TIMEROUT_EVT, AUP_TIMEROUT );

  return(err);
}
/*****************************************************************************  
** Function name:fnRN8209_Read(uint8 wReg)                      
**                                                                              
** Description:读RN8209寄存器                                                   
**                                                                              
** Parameters:wReg 寄存器地址
**                                                                              
** Returned value:	no                                        
**                                                                              
******************************************************************************/ 

void fnRN8209_Read(uint8 wReg)
{ 
 if(b_Uart0orUart1==1)	
    U0DBUF=wReg;
 else
    U1DBUF=wReg;  

 
}
/*****************************************************************************
** Function name:	fnEMU_Init(void)
**
** Description:		计量EMU初始化函数，初始化RN8209寄存器
**
** Parameters:		NONE
**
** Returned value:	NONE
**
******************************************************************************/
void fnEMU_Init(void)
{

    b_Param_Ok = 0;
    WriteCnt=0;
    b_Uart0orUart1 = 0;
    osal_start_timerEx( AppUserProcess_TaskID, AUP_TIMEROUT_EVT, AUP_TIMEROUT );
   

}

/*****************************************************************************
** Function name:	fnDl645Front_Exec(void)
**
** Description:		从计量芯片读取数据（电压、电流、功率、脉冲数），及电压、电流、频率、功率因素计算
**
** Parameters:		NONE
**
** Returned value:	NONE
**
******************************************************************************/	
	
void fnDl645Front_Exec(void)
{ 
//	uint8 i;
//	uint32 TempI,TempIn;//TempU,
//	uint32 TempStatus;
//	//uint16 TempAngle;	
//	uint8	PFlag;		
//	Dl645FrontTmp.ChkSum1=0XD879;

        
        if(RetryCount==0)
        {
                
             switch (NubRN8209DCommand)
            {
              case 0://计量状态及校验和寄存器
                    
                    RN8209DCommand=ADEMUStatus;
                    fnRN8209_Read( RN8209DCommand) ;
                    RetryCount=RETRYMAX;
                    break;
                    
//              case 1://通道A电流的有效值
//                    RN8209DCommand=ADIARMS;
//                    fnRN8209_Read( RN8209DCommand) ;
//                    break;
//              case 2://通道B电流的有效值
//                    RN8209DCommand=ADIBRMS;
//                    fnRN8209_Read( RN8209DCommand) ;            
//                    break;
              case 3://电压有效值
                
                    if( RN8209DCommand != ADURMS )
                    {
                        RN8209DCommand=ADURMS;
                        fnRN8209_Read( RN8209DCommand) ;  
                    }
                    RetryCount=RETRYMAX;
                    break;
                    
    //          case 4://读频率
    //                RN8209DCommand=ADUFreq;
    //                fnRN8209_Read( RN8209DCommand) ;  
    //                break;
              case 5://有功功率A

                    if( RN8209DCommand != ADPowerPA )
                    {
                      
                        RN8209DCommand=ADPowerPA;
                        fnRN8209_Read( RN8209DCommand) ; 
                    
                    }       
                    RetryCount=RETRYMAX;
                    break;
              case 6://有功功率B
                
                    if( RN8209DCommand != ADPowerPB )
                    {
                        RN8209DCommand=ADPowerPB;
                        fnRN8209_Read( RN8209DCommand) ; 
                    
                    }                
                    RetryCount=RETRYMAX;
                    break;
              case 7://有功能量，读后清零寄存器、冻结电能寄存器可选，默认为读后清零寄存器
                
                    if( RN8209DCommand != ADEnergyP2 )
                    {
                      
                      RN8209DCommand=ADEnergyP2;
                      fnRN8209_Read( RN8209DCommand) ;  
                      
                    }
                    RetryCount=RETRYMAX;
                    break;
                    
              case 8://有功能量，读后清零寄存器、冻结电能寄存器可选，默认为读后清零寄存器
                
                    if( RN8209DCommand != ADEnergyD2 )
                    {
                      
                      RN8209DCommand=ADEnergyD2;
                      fnRN8209_Read( RN8209DCommand) ;  
                      
                    }
                    RetryCount=RETRYMAX;
                    break;                  
                    
              default:					
                        break;                    
                   
              
            }       
        
        }
}

uint16 GetGongLvAverage(uint8 index,uint8 count)
{
  uint8 offset = GongLvListIndex[index];
  uint8 cnt = count;
  uint32 total=0;
  while(cnt>0)
  {
      cnt--;
      total += GongLvList[index][offset];
      if ( offset > 0 )
          offset --;
      else
          offset = GONGLV_LIST_SIZE - 1;
  }
  return total / count;
}

/*
void ClearGongLvList(uint8 index,uint8 cnt)
{
  uint8 offset = GongLvListIndex[index];
  while(cnt>0)
  {
      cnt--;
      GongLvList[index][offset] = 0;
      if ( offset > 0 )
          offset --;
      else
          offset = GONGLV_LIST_SIZE - 1;
  }
}
*/

void CheckGonglvTotal()
{
    uint8 i,maskbit,maxindex;
    uint16 GonglvTotal = 0,maxgonglv = 0;
    for (i=0;i<4;i++)
    {
        GonglvTotal += GongLvSave[i];
        if ( GongLvSave[i] > maxgonglv )
        {
            maxgonglv = GongLvSave[i];
            maxindex = i;
        }
    }
    if ( GonglvTotal > PWONMAX )
    {
        maskbit = (0x01<<maxindex);
        ChaKongPwOn &= ~maskbit;    //紧急关闭
        
        //Curr_Sys_Parm.ChaKongCtrl &= ~maskbit;
        ChaKongLockOff |= maskbit;
        PowerOverAlarm &= ~maskbit;
        Led_Changed_flag = 0;
        AutoSendFlag=1;
    }
    else
    if ( GonglvTotal > PWONALARM )
    {
        PowerOverAlarm |= 0x80;
    }
    else
        PowerOverAlarm &= ~0x80;
}

void ConvPwChnl(uint8 index)                        
{
    uint8 maskbit = (0x01<<index);
    if( PwChnl[index] > Curr_Sys_Parm.PwOffSet[index]) //插孔1零点校正
    {
      if(debug ==0)
        GongLv[index] = ( PwChnl[index] - Curr_Sys_Parm.PwOffSet[index]) / Curr_Sys_Parm.KPrms[index];//除于功率系数

      GongLvListIndex[index]++;
      GongLvListIndex[index] %= GONGLV_LIST_SIZE;
      GongLvList[index][GongLvListIndex[index]] = GongLv[index];
      
      SaveCnt[index]++;
      if(SaveCnt[index]< 2)
      {
          PwChnlHigh[index] = PwChnl[index];                               
          PwChnlLow[index] = PwChnl[index];
          DianDownIsOkFlag &= ~maskbit;
          DianUpIsOkFlag &= ~maskbit;
      }
      else
      {      
        if(PwChnl[index]>PwChnlHigh[index])
        {        
          PwChnlHigh[index]=PwChnl[index];
        }
        
        if(PwChnl[index]<PwChnlLow[index])  
        {
          PwChnlLow[index]=PwChnl[index];          
        }     
        if( ChaKongPwOn & maskbit )//插孔已经通电
        {
          if(PwChnlLow[index]<(Curr_Sys_Parm.PwOffSet[index]*DianPerH))
          {
            if ( ChaKongHave & maskbit )
            {
                DianDownIsOkFlag |= maskbit;
                DianUpIsOkFlag &= ~maskbit;
                //ClearGongLvList(index,3);
            }
            SaveCnt[index]=0;
          }                               
        }
        else
        {//已经断电
          if(PwChnlHigh[index]>(PwChnlLow[index] * DianPer) || PwChnlHigh[index]>(Curr_Sys_Parm.PwOffSet[index]*DianPer))
          {
            if ( ( ChaKongHave & maskbit ) == 0 )
            {
                DianUpIsOkFlag |= maskbit;      //发现插孔有负载插入
                DianDownIsOkFlag &= ~maskbit;
            }
            SaveCnt[index]=0;
          }                                
        }
      }
      if(SaveCnt[index] > SAVEPWCONST)
      {
        SaveCnt[index]=0;
        if( ChaKongPwOn & maskbit )//插孔已经通电
        {
            GongLvSave[index] = GetGongLvAverage(index,SAVEPWCONST);
            if ( index == 2 )   //一个周期结束
            {   
                CheckGonglvTotal();
            }
        }
      }           
      
       if(debug ==0)
       {
          if( GongLv[index] > PWONMAX )//功率超过最大值
          {
              ChaKongPwOn &= ~maskbit; //紧急关闭
                     
              ChaKongLockOff |= maskbit;
              //Curr_Sys_Parm.ChaKongCtrl &= ~maskbit;
              PowerOverAlarm &= ~maskbit;
              Led_Changed_flag = 0;
              AutoSendFlag=1;
          }
          else
          if ( GongLvSave[index] > PWONALARM )
          {              
              PowerOverAlarm |= maskbit;
          }
          else
              PowerOverAlarm &= ~maskbit;
       }      
    }
    else
    {
      if(debug ==0)
        GongLv[index] = 0;
      if ( ChaKongHave & maskbit )
      {
          DianDownIsOkFlag |= maskbit;
          DianUpIsOkFlag &= ~maskbit;
      }
    }
}

void RN8209D_Recv_Analyse(void)
{
	//uint32 TempI,TempIn;//TempU,
	//uint32 TempStatus;
	//uint16 TempAngle;	
	//uint8	PFlag;
        uint8	temp,SysStatus;
        uint32  ChkSum;
        
  	switch(RN8209DCommand)
        {
          case ADSysStatus:	//
          
                           SysStatus = ComPack[SCOM_PORT_RN8209].RxBuf[0];
                           
                           if( WriteCnt == 4 ) //读取复位状态
                           {
                              SysStatus &= 0x03;
                              
                              if( SysStatus == 0)//复位不成功
                              {
                                Reset_Err_Cnt++;
                                if( Reset_Err_Cnt >= 3 )
                                {
                                  Dl645Front.PriData.Flag = RN8209RST;
                                  Reset_Err_Cnt = 0;
                                  
                                }
                                else
                                {
                                  
                                  WriteCnt = 3 ;//再次读取
                                  
                                }
                                   
                                
                              }
                               
                             
                           }
                           else
                           if( WriteCnt == 2 ) //读取写使能状态
                           {
                              SysStatus &= 0x10;
                              
                              if( SysStatus == 0)//写使能不成功
                              {
                                Dl645Front.PriData.Flag = WRITE_EN_ERR;
                                
                                Write_En_Err_Cnt++;
                                if( Write_En_Err_Cnt >= 3 ) //写使能错误次数大于等于3次时，进入复位处理
                                {
                                  Write_En_Err_Cnt = 0;
                                  Dl645Front.PriData.Flag = RN8209RST;
                                  
                                }
                                else
                                {
                                  WriteCnt = 1 ;//再次读取
                                }
                              }                              
                           
                           }    
                           else
                           if( WriteCnt == 6 ) //读取写使能状态
                           {
                              SysStatus &= 0x10;
                              
                              if( SysStatus == 0)//写使能不成功
                              {
                                Dl645Front.PriData.Flag = WRITE_EN_ERR;
                                
                                Write_En_Err_Cnt1++;
                                if( Write_En_Err_Cnt1 >= 3 ) //写使能错误次数大于等于3次时，进入复位处理
                                {
                                  Write_En_Err_Cnt1 = 0;
                                  Dl645Front.PriData.Flag = RN8209RST;
                                 
                                }
                                
                                else
                                {
                                  WriteCnt = 5 ;//再次读取
                                }
                              }                              
                           
                           }                               
                           else
                           if( WriteCnt == 16)  //读取写禁止状态
                           {
                           
                              SysStatus &= 0x10;
                              
                              if( SysStatus == 1)//写禁止不成功
                              {
                                Dl645Front.PriData.Flag = WRITE_DIS_ERR;
                                
                                Write_Dis_Err_Cnt++;
                                if( Write_Dis_Err_Cnt >= 3 ) //写禁止错误次数大于等于3次时，进入复位处理
                                {
                                  Write_Dis_Err_Cnt = 0;
                                  Dl645Front.PriData.Flag = RN8209RST;
                                  
                                }
                                else
                                {
                                  WriteCnt = 15 ;//再次读取
                                }                                
                              }     
                              else
                              {
                                if( b_Uart0orUart1 == 0 )     //串口变换
                                {
                                  b_Uart0orUart1 = 1;
                                  
                                  Dl645FrontTmp.ChkSum = 0 ;
                                  Dl645FrontTmp.ChkSum += Dl645FirmPara.SYSCON ;
                                  Dl645FrontTmp.ChkSum += Dl645FirmPara.EMUCON ;
                                  Dl645FrontTmp.ChkSum += Dl645FirmPara.HFConst ;
                                  Dl645FrontTmp.ChkSum += Dl645FirmPara.PStart ;
                                  Dl645FrontTmp.ChkSum += Dl645FirmPara.QStart ;
                                  Dl645FrontTmp.ChkSum += Dl645FirmPara.APOSA ;
                                  Dl645FrontTmp.ChkSum += Dl645FirmPara.APOSB ;
                                  Dl645FrontTmp.ChkSum += Dl645FirmPara.EMUCON2 ; 
                                  Dl645FrontTmp.ChkSum += 0x1600 ;//加上波特率uartbr[6:0]参与校验和计算，在通信口选择为uart时会影响到校验和计算结果。RN8209C固定为4800波特率。 
                                  Dl645FrontTmp.ChkSum = 0xffff - Dl645FrontTmp.ChkSum;//校验需要取反
                                  
                                }
                                else
                                {
                                  b_Uart0orUart1 = 0;
                                  NubRN8209DCommand = 0;
                                  b_Param_Ok = 1; //参数配置完成
                                  
                                  Dl645FrontTmp.ChkSum1 = 0 ;
                                  Dl645FrontTmp.ChkSum1 += Dl645FirmPara.SYSCON ;
                                  Dl645FrontTmp.ChkSum1 += Dl645FirmPara.EMUCON ;
                                  Dl645FrontTmp.ChkSum1 += Dl645FirmPara.HFConst ;
                                  Dl645FrontTmp.ChkSum1 += Dl645FirmPara.PStart ;
                                  Dl645FrontTmp.ChkSum1 += Dl645FirmPara.QStart ;
                                  Dl645FrontTmp.ChkSum1 += Dl645FirmPara.APOSA ;
                                  Dl645FrontTmp.ChkSum1 += Dl645FirmPara.APOSB ;
                                  Dl645FrontTmp.ChkSum1 += Dl645FirmPara.EMUCON2 ;  
                                  Dl645FrontTmp.ChkSum1 += 0x1600 ;//加上波特率uartbr[6:0]参与校验和计算，在通信口选择为uart时会影响到校验和计算结果。RN8209C固定为4800波特率。 
                                  Dl645FrontTmp.ChkSum1 = 0xffff - Dl645FrontTmp.ChkSum1;//校验需要取反                                  
                                  
                                }  
                                WriteCnt = 0 ;//从头开始配置
                              
                              }

                           }  
                            
                            if(b_Param_Ok == 0 )//参数配置完成
                            {
                               if( Dl645Front.PriData.Flag !=  RN8209RST ) //复位时 终止正常流程处理
                              {
                                osal_start_timerEx( AppUserProcess_TaskID, AUP_TIMEROUT_EVT, AUP_TIMEROUT );
                              }
                                                         
                            }

                            
                          
                          
                          break;
	
	case ADEMUStatus://
                        if(!(ComPack[SCOM_PORT_RN8209].RxBuf[0]& 0x01))
                        {//校表数据校验和计算已完成，校验值可用
                          ChkSum = ((uint32)ComPack[SCOM_PORT_RN8209].RxBuf[1]<<8) + (uint32)ComPack[SCOM_PORT_RN8209].RxBuf[2];
                          if(b_Uart0orUart1 == 1)
                          {

                            if(Dl645FrontTmp.ChkSum1 == ChkSum)
                            {
                              Dl645FrontTmp.ChkErrCnt1 = 0 ;
                            }
                            else
                            {
                                Dl645FrontTmp.ChkErrCnt1++; 
                                if(Dl645FrontTmp.ChkErrCnt1 > 3) 
                                {//校验错误次数大于3次后，复位RN8209
                                  Dl645Front.PriData.Flag = RN8209RST;
                                  Dl645FrontTmp.ChkErrCnt1 = 0 ;
                                }
                            }
                          }
                          else
                          {
                            if(Dl645FrontTmp.ChkSum == ChkSum)
                            {
                              Dl645FrontTmp.ChkErrCnt = 0 ;
                            }
                            else
                            {
                                Dl645FrontTmp.ChkErrCnt++; 
                                if(Dl645FrontTmp.ChkErrCnt > 3) 
                                {//校验错误次数大于3次后，复位RN8209
                                  Dl645Front.PriData.Flag = RN8209RST;
                                  Dl645FrontTmp.ChkErrCnt = 0 ;
                                }
                            }
                          
                          }

                        }
                        RetryCount=0;
                        NubRN8209DCommand = 3;
			break;

//	case ADIARMS://通道A电流的有效值
//                       // Dl645FrontTmp.UI[0] = 0;//清通道A电流的有效值
//                        if((ComPack[SCOM_PORT_RN8209].RxBuf[0]& 0x80)==0)
//                        {  
//                          Dl645FrontTmp.UI[0] = ((uint32)ComPack[SCOM_PORT_RN8209].RxBuf[0]<<16) +((uint32)ComPack[SCOM_PORT_RN8209].RxBuf[1]<<8)+ ComPack[SCOM_PORT_RN8209].RxBuf[2] ;
//                        }
//                        else
//                        {
//                          //;Dl645FrontTmp.UI[0] = ~(((uint32)ComPack[SCOM_PORT_RN8209].RxBuf[0]<<16) +((uint32)ComPack[SCOM_PORT_RN8209].RxBuf[1]<<8)+ ComPack[SCOM_PORT_RN8209].RxBuf[2]) + 1 ;
//                        }
//                        if(b_Uart0orUart1)
//                          Chnl1=Dl645FrontTmp.UI[0];
//                        else
//                          Chnl4=Dl645FrontTmp.UI[0];
//                        NubRN8209DCommand=2;   
//			break;
//	case ADIBRMS://通道B电流的有效值
//                        //Dl645FrontTmp.UI[1] = 0;
//                        if((ComPack[SCOM_PORT_RN8209].RxBuf[0]& 0x80)==0)
//                        {  
//                          Dl645FrontTmp.UI[1] = ((uint32)ComPack[SCOM_PORT_RN8209].RxBuf[0]<<16) +((uint32)ComPack[SCOM_PORT_RN8209].RxBuf[1]<<8)+ ComPack[SCOM_PORT_RN8209].RxBuf[2] ;
//                        }
//                        if(b_Uart0orUart1)
//                          Chnl2=Dl645FrontTmp.UI[1];
//                        else
//                          Chnl3=Dl645FrontTmp.UI[1];
//                        NubRN8209DCommand=3;      
			break;
	case ADURMS://电压有效值
                        Dl645FrontTmp.UI[2] = 0;//清电压有效值
                        if((ComPack[SCOM_PORT_RN8209].RxBuf[0]& 0x80)==0)
                        {  
                          Dl645FrontTmp.UI[2] = ((uint32)ComPack[SCOM_PORT_RN8209].RxBuf[0]<<16) +((uint32)ComPack[SCOM_PORT_RN8209].RxBuf[1]<<8)+ ComPack[SCOM_PORT_RN8209].RxBuf[2]; 
                        }
                        if(b_Uart0orUart1)
                        {
                            PwChnl5=Dl645FrontTmp.UI[2];
                            if(debug ==0)
                              DianYa = PwChnl5/Curr_Sys_Parm.KUrms5;
                        }                       
                        
                        //NubRN8209DCommand=4;    
                        NubRN8209DCommand=5; 
                        RetryCount=0;
			break;
//	
//	case ADUFreq://读频率
//                        Dl645FrontTmp.Frequency= ((uint16)ComPack[SCOM_PORT_RN8209].RxBuf[0]<<8) + ComPack[SCOM_PORT_RN8209].RxBuf[1];
//                        NubRN8209DCommand=5;   
//			break;
        case ADPowerPA://有功功率A
                        Dl645FrontTmp.Pw[0] =((uint32)ComPack[SCOM_PORT_RN8209].RxBuf[0]<<24) +((uint32)ComPack[SCOM_PORT_RN8209].RxBuf[1]<<16)+ ((uint32)ComPack[SCOM_PORT_RN8209].RxBuf[2]<<8)+ ComPack[SCOM_PORT_RN8209].RxBuf[3]; 
                        if(Dl645FrontTmp.Pw[0]&0x80000000) 
                        {
                          //Dl645FrontTmp.Pw[0]=(~Dl645FrontTmp.Pw[0])+1;
                          Dl645FrontTmp.Pw[0] = 0;
                        }
                        
                        temp = ChaKongPwOn;
                        if(b_Uart0orUart1==1)
                        {
                            PwChnl[0] = Dl645FrontTmp.Pw[0] ;
                            ConvPwChnl(0);
                        }
                        else
                        {
                            PwChnl[3] = Dl645FrontTmp.Pw[0] ;
                            ConvPwChnl(3);
                        }    
                        if ( temp != ChaKongPwOn )
                        {
                            //UpdatePwOnIO();
                            DelaySwitchFlag = 1;
                        }

                        NubRN8209DCommand=6;  
                        RetryCount=0;
                        break;
                        
        case ADPowerPB://有功功率B
                        Dl645FrontTmp.Pw[1] =((uint32)ComPack[SCOM_PORT_RN8209].RxBuf[0]<<24) +((uint32)ComPack[SCOM_PORT_RN8209].RxBuf[1]<<16)+ ((uint32)ComPack[SCOM_PORT_RN8209].RxBuf[2]<<8)+ ComPack[SCOM_PORT_RN8209].RxBuf[3];                         
                        if(Dl645FrontTmp.Pw[1]&0x80000000) 
                        {
//                          Dl645FrontTmp.Pw[1]=(~Dl645FrontTmp.Pw[1])+1;
                           Dl645FrontTmp.Pw[1] =  0 ;

                        }
                        temp = ChaKongPwOn;
                        if(b_Uart0orUart1)
                        {
                            PwChnl[1] = Dl645FrontTmp.Pw[1] ;
                            ConvPwChnl(1);
                        }
                        else
                        {
                            PwChnl[2] = Dl645FrontTmp.Pw[1] ;
                            ConvPwChnl(2);
                        }
                        if ( temp != ChaKongPwOn )
                        {
                            //UpdatePwOnIO();
                            DelaySwitchFlag = 1;
                        }
                        FirstStart++;
                        if( FirstStart > 10 )
                            FirstStart = 12;

                        NubRN8209DCommand=7; 
                        RetryCount=0;
                        break;

//                        
        case ADEnergyP2://有功能量，读后清零寄存器、冻结电能寄存器可选，默认为读后清零寄存器

                        Dl645FrontTmp.Pulse =  ((uint32)ComPack[SCOM_PORT_RN8209].RxBuf[0]<<16) +((uint32)ComPack[SCOM_PORT_RN8209].RxBuf[1]<<8)+ ComPack[SCOM_PORT_RN8209].RxBuf[2] ;
                        if(Dl645FrontTmp.Pulse > 100) Dl645FrontTmp.Pulse = 0;	//容错，脉冲个数过大，清除
                        Dl645FrontTmp.Pulse_Eg+=Dl645FrontTmp.Pulse;
                        
#if  DL645SOFT_DEBUG
                        Dl645FrontTmp.Pulse_Eg+=1;
#endif 
                        NubRN8209DCommand=8; 
                        RetryCount=0;
                        if(debug > 0)
                            break;
                        
                        if(b_Uart0orUart1==0)
                        {
                          //b_Uart0orUart1=1;
                            Curr_Sys_Parm.NengLiang[3] += Dl645FrontTmp.Pulse;
                            Curr_Sys_Parm.StartNengLiang[3] += Dl645FrontTmp.Pulse;
                            Curr_Sys_Parm.NengLiang[3] &= 0x00FFFFFF;
                        }
                        else
                        {
                          //b_Uart0orUart1=0;
                            Curr_Sys_Parm.NengLiang[0] += Dl645FrontTmp.Pulse;
                            Curr_Sys_Parm.StartNengLiang[0] += Dl645FrontTmp.Pulse;
                            Curr_Sys_Parm.NengLiang[0] &= 0x00FFFFFF;
                        }
                        
//                        Dl645Front.PriData.Flag |= FRONT_FLAG_RN8209RST;
                    
                        break;
                        
        case ADEnergyD2://有功能量，读后清零寄存器、冻结电能寄存器可选，默认为读后清零寄存器

                        Dl645FrontTmp.Pulse =  ((uint32)ComPack[SCOM_PORT_RN8209].RxBuf[0]<<16) +((uint32)ComPack[SCOM_PORT_RN8209].RxBuf[1]<<8)+ ComPack[SCOM_PORT_RN8209].RxBuf[2] ;
                        if(Dl645FrontTmp.Pulse > 100) Dl645FrontTmp.Pulse = 0;	//容错，脉冲个数过大，清除
                        Dl645FrontTmp.Pulse_Eg+=Dl645FrontTmp.Pulse;
                        
#if  DL645SOFT_DEBUG
                        Dl645FrontTmp.Pulse_Eg+=1;
#endif 
                         
                        NubRN8209DCommand = 0;
                        
//                        Dl645Front.PriData.Flag |= FRONT_FLAG_RN8209RST;
                        RetryCount=0;
                        if(debug > 0)
                        {
                            if(b_Uart0orUart1==0)
                                b_Uart0orUart1=1;
                            else
                                b_Uart0orUart1=0;
                            break;
                        }
                        if(b_Uart0orUart1==0)
                        {
                            b_Uart0orUart1=1;
                            Curr_Sys_Parm.NengLiang[2] += Dl645FrontTmp.Pulse;
                            Curr_Sys_Parm.StartNengLiang[2] += Dl645FrontTmp.Pulse;
                            Curr_Sys_Parm.NengLiang[2] &= 0x00FFFFFF;
                        }
                        else
                        {
                            b_Uart0orUart1=0;
                            Curr_Sys_Parm.NengLiang[1] += Dl645FrontTmp.Pulse;
                            Curr_Sys_Parm.StartNengLiang[1] += Dl645FrontTmp.Pulse;
                            Curr_Sys_Parm.NengLiang[1] &= 0x00FFFFFF;
                        }
                        break;                        
                        
	default:					
                        break;
	}
        fnScomPk_Init(SCOM_PORT_RN8209);   
}
//end file

