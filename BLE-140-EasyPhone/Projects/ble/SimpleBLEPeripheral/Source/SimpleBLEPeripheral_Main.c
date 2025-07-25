/**************************************************************************************************
  Filename:       SimpleBLEPeripheral_Main.c
  Revised:        $Date: 2010-07-06 15:39:18 -0700 (Tue, 06 Jul 2010) $
  Revision:       $Revision: 22902 $

  Description:    This file contains the main and callback functions for
                  the Simple BLE Peripheral sample application.

  Copyright 2010 - 2011 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED 揂S IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/**************************************************************************************************
 *                                           Includes
 **************************************************************************************************/
/* Hal Drivers */
#include "hal_types.h"
//#include "hal_key.h"
#include "hal_timer.h"
#include "hal_drivers.h"
//#include "hal_led.h"
#include "hal_shiftout.h"

/* OSAL */
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "OSAL_PwrMgr.h"
#include "osal_snv.h"
#include "OnBoard.h"

//#include "simpleble.h"
#include "AppUserProcess.h"

void Common_Init( void );
void Init_Watchdog( void );
void RN8209_init();

void SetDefaultParm(uint8 flag);
extern void simpleBLE_NPI_init(void);
extern RTC_Time Curr_RTC_time;
 
extern void SecondsToRTCTime(uint32 TimeVar,RTC_Time * Time);
extern void RTC_Set_RTC_Time(RTC_Time *pTimes);
/**************************************************************************************************
 * FUNCTIONS
 **************************************************************************************************/

/**************************************************************************************************
 * @fn          main
 *
 * @brief       Start of application.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
int main(void)
{
    /* Initialize hardware */
    HAL_BOARD_INIT();

    // Initialize board I/O
    InitBoard( OB_COLD );

    Common_Init();

    /* Initialze the HAL driver */
    HalDriverInit();

    //Init_Watchdog();
    InitT3();
    PWM_Init();     //PWM
    RN8209_init();

    /* Initialize NV system */
    osal_snv_init();
    

#if 1//这一段代码和说明是 amomcu 增加的  
    // 从设置中读出以保存的数据， 以便决定现在应该是跑主机还是从机
    // 注意， 这里用到了 osal_snv_xxx ， 数据是存在flash里边的， 大家可以找找相关代码和说明
    // 需要注意的是 osal_snv_read 和 osal_snv_write ， 第一个 参数 osalSnvId_t id
    // 这个id， 我们编程可用的是从 0x80 至 0xff, 其中目前程序中可用的空间是 2048 字节
    // 这个大小定义于 osal_snv.c 中， 即以下宏定义， 跟踪代码进去就能看到
    /*
    // NV page configuration
    #define OSAL_NV_PAGE_SIZE       HAL_FLASH_PAGE_SIZE
    #define OSAL_NV_PAGES_USED      HAL_NV_PAGE_CNT
    #define OSAL_NV_PAGE_BEG        HAL_NV_PAGE_BEG
    #define OSAL_NV_PAGE_END       (OSAL_NV_PAGE_BEG + OSAL_NV_PAGES_USED - 1)
    */
    
    {
        int8 ret8 = osal_snv_read(0x80, sizeof(SYS_PARM), &Curr_Sys_Parm);
        // 如果该段内存未曾写入过数据， 直接读，会返回 NV_OPER_FAILED ,
        // 我们利用这个特点作为第一次烧录后的运行， 从而设置参数的出厂设置
        if(NV_OPER_FAILED == ret8)
        {
            SetDefaultParm(0);
            osal_snv_write(0x80, sizeof(SYS_PARM), &Curr_Sys_Parm);    // 写所有参数
        } 

        ret8 = osal_snv_read(0x81, sizeof(SYS_PARM1), &Curr_Sys_Parm1);
        if(NV_OPER_FAILED == ret8)
        {
            SetDefaultParm(1);
            osal_snv_write(0x81, sizeof(SYS_PARM1), &Curr_Sys_Parm1);    // 写所有参数
        }  

        ret8 = osal_snv_read(0x82, sizeof(SYS_PARM2), &Curr_Sys_Parm2);
        if(NV_OPER_FAILED == ret8)
        {
            SetDefaultParm(2);
            osal_snv_write(0x82, sizeof(SYS_PARM2), &Curr_Sys_Parm2);    // 写所有参数
        }  
        
        SecondsToRTCTime( Curr_Sys_Parm.SecondOfRtc,&Curr_RTC_time);        
        RTC_Set_RTC_Time(&Curr_RTC_time); 
        // 执行  串口初始化
        simpleBLE_NPI_init();   
        
    }
#endif//这一段代码和说明是 amomcu 增加的  [


    /* Initialize LL */

    /* Initialize the operating system */
    osal_init_system();

    /* Enable interrupts */
    HAL_ENABLE_INTERRUPTS();

    // Final board initialization
    InitBoard( OB_READY );

    #if defined ( POWER_SAVING )
      osal_pwrmgr_device( PWRMGR_BATTERY );
    #endif

    /* Start OSAL */
    osal_start_system(); // No Return from here

    return 0;
}

/**************************************************************************************************
                                           CALL-BACKS
**************************************************************************************************/


/*************************************************************************************************
**************************************************************************************************/
