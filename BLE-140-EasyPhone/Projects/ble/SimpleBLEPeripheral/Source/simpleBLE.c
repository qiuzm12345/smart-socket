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
// AT �����

//flag: PARA_ALL_FACTORY:  ȫ���ָ���������
//flag: PARA_PARI_FACTORY: ��������Ϣ
uint8 RxBuffer[64];
void SetAllParaDefault(PARA_SET_FACTORY flag)    
{
    if(flag == PARA_ALL_FACTORY)
    {
        sys_config.mode = BLE_MODE_SERIAL;         //����ģʽ 0:͸�� �� 1: ֱ�� , 2: iBeacon
        sys_config.baudrate = HAL_UART_BR_4800;
       // sys_config.baudrate = HAL_UART_BR_9600;  
        //sys_config.baudrate = HAL_UART_BR_115200;  
        sys_config.parity = 2;  
        sys_config.stopbit = 0;  

        sprintf((char*)sys_config.name, "NM_BTPH_Test");     //�豸����

        //sys_config.role = BLE_ROLE_PERIPHERAL;         //����ģʽ, Ĭ�ϴӻ�
        //sys_config.role = BLE_ROLE_CENTRAL;

        //sprintf((char*)sys_config.pass, "000000");      //����
        //sys_config.type = 0;                     //��Ȩģʽ
        //sys_config.mac_addr[16];               //����mac��ַ
        
        sys_config.connl_status = 0;             //�������һ�ε�״̬
        sys_config.connect_mac_status = 0;       //����ָ����ַ�ķ���״̬
        //sys_config.ever_connect_mac_status[MAX_PERIPHERAL_MAC_ADDR][13];       //�����ɹ����ӹ��Ĵӻ���ַ
        
        //osal_memset(sys_config.ever_connect_mac_status, 0, MAX_PERIPHERAL_MAC_ADDR*13);
        sprintf((char*)sys_config.verion, "%s", VERSION);       //�汾��Ϣ v1.0 ~ v9.9

        sys_config.try_connect_time_ms = 0;       // 

        sys_config.rssi = 0;       //  RSSI �ź�ֵ

        sys_config.rxGain = HCI_EXT_RX_GAIN_HIGH;       //  ��������ǿ��
        sys_config.txPower = 3;       //  �����ź�ǿ��

        sys_config.ibeacon_adver_time_ms = 500;

        sys_config.workMode = 0;      //  ģ�鹤������  0: ���������� 1: �ȴ�AT+CON �� AT+CONNL ����
    }
    else if(flag == PARA_PARI_FACTORY)
    {
        //osal_memset(sys_config.ever_connect_mac_status, 0, MAX_PERIPHERAL_MAC_ADDR*13);
        sprintf((char*)sys_config.verion, "%s", VERSION);       //�汾��Ϣ v1.0 ~ v9.9
        //sprintf((char*)verion, "%s", VERSION);       //�汾��Ϣ v1.0 ~ v9.9
    }
}
#if 1
// ���ڻص������� ����Ѹûص�������ʵ�ֵĹ��ܽ���һ��
/*
1, ˼·:  �������յ����ݺ󣬾ͻ����ϵ������»ص���������ʵ�ʲ����з��֣��˻ص�
��������Ƶ���� ����㲻ִ��NPI_ReadTransport�������ж�ȡ�� ��ô����ص������ͻ�
Ƶ���ر�ִ�У����ǣ���ͨ�����ڷ���һ�����ݣ� �㱾�����봦����һ����һ�ε����ݣ����ԣ�
����������������ʱ��Ĵ������� Ҳ�����յ����ݹ�����߳�ʱ���Ͷ�ȡһ�����ݣ� 
Ȼ����ݵ�ǰ��״̬����ִ�У����û�������ϣ��Ͱ��������ݵ���AT����� �������
���ˣ��Ͱ������͵��Զˡ�
*/

//uart �ص�����
static void simpleBLE_NpiSerialCallback( uint8 port, uint8 events )
{
    (void)port; 
     uint8 i; 
     uint8 RxCount;
     uint8 chksum=RN8209DCommand;
     uint8 RxBuffer[10];
     osal_memset(RxBuffer , 0x00 , 10);
    if (events & HAL_UART_RX_TIMEOUT)   //����������      
    {
        (void)port;
   
        
             RxCount = NPI_RxBufLen();           //�������ڻ������ж����ֽ�      
             
             NPI_ReadTransport(RxBuffer,RxCount);    //�ͷŴ������� 
             
             for(i=0;i<RxCount;i++)
             {
               
               ComPack[SCOM_PORT_RN8209].RxBuf[ComPack[SCOM_PORT_RN8209].RxLen++] = RxBuffer[i];
               chksum += RxBuffer[i];//����������ݵ�У���
               
             }         

 
             if(chksum==0xFF)
             {
                osal_set_event( AppUserProcess_TaskID, AUP_GET_UART1_RX_EVT );//�յ���Ч��Ϣ  	
                
             }   
             else
             {     
                Dl645Front.PriData.Flag = RN8209RST;                
                
             }
                     
      
    } 

    URX1IF = 0; // ���жϱ�־
 
}


#endif

void simpleBLE_NPI_init(void)
{
  
    uint8               baudrate=HAL_UART_BR_4800;
//---------------------------------add by amomcu.com
  /*
    Para ��Χ 0,1,2 
    0: ��У��
    1:  ODD
    2:  EVEN
    Default: 0   
  */
  uint8               parity=2;     

  /*
    Para: 0~1 
    0: 1 ֹͣλ
    1: 2 ֹͣλ
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


