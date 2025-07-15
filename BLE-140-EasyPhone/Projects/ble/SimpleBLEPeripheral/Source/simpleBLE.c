#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"
#include "npi.h"
#include "osal_snv.h"
#include "simpleBLE.h"
#include "stdio.h"
#include "RN8209D.h"
#include "AppUserProcess.h"
// AT 命令处理

//flag: PARA_ALL_FACTORY:  全部恢复出厂设置
//flag: PARA_PARI_FACTORY: 清除配对信息
uint8 RxBuffer[64];
void SetAllParaDefault(PARA_SET_FACTORY flag)    
{
    if(flag == PARA_ALL_FACTORY)
    {
        sys_config.mode = BLE_MODE_SERIAL;         //工作模式 0:透传 ， 1: 直驱 , 2: iBeacon
        sys_config.baudrate = HAL_UART_BR_4800;
       // sys_config.baudrate = HAL_UART_BR_9600;  
        //sys_config.baudrate = HAL_UART_BR_115200;  
        sys_config.parity = 2;  
        sys_config.stopbit = 0;  

        sprintf((char*)sys_config.name, "NM_BTPH_Test");     //设备名称

        //sys_config.role = BLE_ROLE_PERIPHERAL;         //主从模式, 默认从机
        //sys_config.role = BLE_ROLE_CENTRAL;

        //sprintf((char*)sys_config.pass, "000000");      //密码
        //sys_config.type = 0;                     //鉴权模式
        //sys_config.mac_addr[16];               //本机mac地址
        
        sys_config.connl_status = 0;             //连接最后一次的状态
        sys_config.connect_mac_status = 0;       //连接指定地址的返回状态
        //sys_config.ever_connect_mac_status[MAX_PERIPHERAL_MAC_ADDR][13];       //曾经成功连接过的从机地址
        
        //osal_memset(sys_config.ever_connect_mac_status, 0, MAX_PERIPHERAL_MAC_ADDR*13);
        sprintf((char*)sys_config.verion, "%s", VERSION);       //版本信息 v1.0 ~ v9.9

        sys_config.try_connect_time_ms = 0;       // 

        sys_config.rssi = 0;       //  RSSI 信号值

        sys_config.rxGain = HCI_EXT_RX_GAIN_HIGH;       //  接收增益强度
        sys_config.txPower = 3;       //  发射信号强度

        sys_config.ibeacon_adver_time_ms = 500;

        sys_config.workMode = 0;      //  模块工作类型  0: 立即工作， 1: 等待AT+CON 或 AT+CONNL 命令
    }
    else if(flag == PARA_PARI_FACTORY)
    {
        //osal_memset(sys_config.ever_connect_mac_status, 0, MAX_PERIPHERAL_MAC_ADDR*13);
        sprintf((char*)sys_config.verion, "%s", VERSION);       //版本信息 v1.0 ~ v9.9
        //sprintf((char*)verion, "%s", VERSION);       //版本信息 v1.0 ~ v9.9
    }
}
#if 1
// 串口回调函数， 下面把该回调函数里实现的功能讲解一下
/*
1, 思路:  当串口收到数据后，就会马上调用以下回调函数，在实际测试中发现，此回调
函数调用频繁， 如果你不执行NPI_ReadTransport函数进行读取， 那么这个回调函数就会
频繁地被执行，但是，你通过串口发送一段数据， 你本意是想处理这一完整一段的数据，所以，
我们在下面引入了时间的处理方法， 也即接收的数据够多或者超时，就读取一次数据， 
然后根据当前的状态决定执行，如果没有连接上，就把所有数据当做AT命令处理， 如果连接
上了，就把数据送到对端。
*/

//uart 回调函数
static void simpleBLE_NpiSerialCallback( uint8 port, uint8 events )
{
    (void)port; 
     uint8 i; 
     uint8 RxCount;
     uint8 chksum=RN8209DCommand;
     uint8 RxBuffer[10];
     osal_memset(RxBuffer , 0x00 , 10);
    if (events & HAL_UART_RX_TIMEOUT)   //串口有数据      
    {
        (void)port;
   
        
             RxCount = NPI_RxBufLen();           //读出串口缓冲区有多少字节      
             
             NPI_ReadTransport(RxBuffer,RxCount);    //释放串口数据 
             
             for(i=0;i<RxCount;i++)
             {
               
               ComPack[SCOM_PORT_RN8209].RxBuf[ComPack[SCOM_PORT_RN8209].RxLen++] = RxBuffer[i];
               chksum += RxBuffer[i];//计算接收数据的校验和
               
             }         

 
             if(chksum==0xFF)
             {
                osal_set_event( AppUserProcess_TaskID, AUP_GET_UART1_RX_EVT );//收到有效信息  	
                
             }   
             else
             {     
                Dl645Front.PriData.Flag = RN8209RST;                
                
             }
                     
      
    } 

    URX1IF = 0; // 清中断标志
 
}


#endif

void simpleBLE_NPI_init(void)
{
  
    uint8               baudrate=HAL_UART_BR_4800;
//---------------------------------add by amomcu.com
  /*
    Para 范围 0,1,2 
    0: 无校验
    1:  ODD
    2:  EVEN
    Default: 0   
  */
  uint8               parity=2;     

  /*
    Para: 0~1 
    0: 1 停止位
    1: 2 停止位
    Default: 0 
  */
  uint8               stopbit=0;
#if 1    
    NPI_InitTransportEx(simpleBLE_NpiSerialCallback, baudrate, 
        parity, stopbit );
#else
    NPI_InitTransport(simpleBLE_NpiSerialCallback);
#endif

}


