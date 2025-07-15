

#ifndef __RN8209D_H__
#define __RN8209D_H__

#ifdef __RN8209D_C__
#define GLOBAL
#else
#define GLOBAL extern
#endif

//----------------------start RN8209  ��ַ����---------------------------------------------------//
#define		ADSYSCON 0x00 
#define        	ADEMUCON 0x01
#define        	ADHFConst     	0x02 
#define        	ADPStart      	0x03 
#define        	ADDStart      	0x04 
#define		ADGPQA        	0x05 
#define        	ADGPQB        	0x06 
#define        	ADPhsA        	0x07 
#define        	ADPhsB        	0x08
#define		ADQPHSCAL	0x09    
#define		ADAPOSA 	0x0a
#define        	ADAPOSB 	0x0b
#define        	ADRPOSA 	0x0c
#define        	ADRPOSB 	0x0d
#define        	ADIARMSOS     	0x0e
#define        	ADIBRMSOS     	0x0f
#define        	ADIBGain      	0x10
#define		ADD2FPL       	0x11
#define        	ADD2FPH       	0x12
#define        	ADDCIAH       	0x13
#define        	ADDCIBH       	0x14
#define         ADDCUH		0x15   
#define         ADDCL   	0x16 
#define         ADEMUCON2	0x17
#define		ADPFCnt    	0x20
#define        	ADDFcnt    	0x21
#define        	ADIARMS       	0x22
#define        	ADIBRMS       	0x23
#define        	ADURMS        	0x24
#define		ADUFreq       	0x25
#define        	ADPowerPA     	0x26
#define        	ADPowerPB     	0x27
#define         ADEnergyP  	0x29
#define         ADEnergyP2 	0x2a
#define         ADEnergyD  	0x2b
#define         ADEnergyD2    	0x2c
#define         ADEMUStatus   	0x2d
#define         ADSPL_IA      	0x30
#define         ADSPL_IB      	0x31
#define         ADSPL_U       	0x32
#define         ADIE  		0x40
#define         ADIF  		0x41
#define         ADRIF    	0x42
#define         ADSysStatus  	0x43
#define         ADRData      	0x44
#define         ADWData      	0x45
#define         ADDeviceID   	0x7f
#define         WriteCmd   	0xea
#define         WriteOpen   	0xe5
#define         WriteClose   	0xdc
#define         Reset8209       0xfa
#define         PwSeltA         0x5a
#define         PwSeltB         0xa5
//----------------------end RN8209  ��ַ����-----------------------------------------------//
  
#define         SCOM_PORT_RN8209        0
#define         MAX_COMPACK_SIZE        4//256
#define         GONGLV_LIST_SIZE        16

#define         RN8209RST    1 //��λ
#define         WRITE_DIS_ERR        2  //д��ֹ���ɹ�
#define         WRITE_EN_ERR         3  //дʹ�ܲ��ɹ�           
#define         INVERSION               1
#define         POSITIVE                0  
#define         ChaKong1PwOn            P1_3
#define         ChaKong2PwOn            P1_4
#define         ChaKong3PwOn            P2_3
#define         ChaKong4PwOn            P2_4

#define         PWCHNL1CONSTHIGH        300000
#define         PWCHNL1CONSTLOW        260000

#define         PWCHNL2CONST            20000
#define         PWCHNL3CONST            90000
#define         PWCHNL4CONST            50000

#define         SAVEPWCONST              5 
#define         LTAGVCONST              10      //15
#define         SAVELTCONST             10
#define         PWLOWCONST              0x4000
#define         DIANDELAY               40
#define         PWOFFMAX                65535

#define         PWONALARM               2000
#define         PWONMAX                 2500


  typedef enum
{
	USART_EVEN_PARITY 	= 0x08,	//D��?��??
	USART_ODD_PARITY  	= 0x0C,
	USART_NO_PARITY  	= 0x00, 
	
	USART_1STOP_SBIT   	= 0x00,	//����?1??
	USART_2STOP_SBIT   	= 0x10,
	
	USART_5BIT_DAT  	= 0x00,	//��y?Y??
	USART_6BIT_DAT		= 0x01,
	USART_7BIT_DAT  	= 0x02,
	USART_8BIT_DAT		= 0x03,
	
	USART_BPS_300 		= 0x00,	//2����??��
	USART_BPS_600 		= 0x20,
	USART_BPS_1200 		= 0x40,
	USART_BPS_2400 		= 0x60,
	USART_BPS_4800 		= 0x80,
	USART_BPS_7200 		= 0xa0,
	USART_BPS_9600 		= 0xc0,	
	USART_BPS_19200 	= 0xe0				
} eUsartPara_TypeDef;

typedef enum 
{
    ERROR = 0 ,
    RIGHT = 1
} ErrorStatus;
typedef enum {USART_IT_RX= 0, USART_IT_TX = !USART_IT_RX} FunctionalMODE;
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
  
//---start У������ļ�-------���ɸ��ݼ���оƬ���ģ�
typedef struct 
{		
	uint16			SYSCON;
	uint16			EMUCON;
	uint16			HFConst;
	uint16			PStart;	
	uint16			QStart;  			//10
	uint16			GPQA;    	
	uint16			GPQB;    	
	uint16			IAGain;  	
	uint16			UGain;   	
	uint16			IBGain;  	
	uint16			PhsA;	   	
	uint16			PhsB;    	
	uint16			QPhsCal; 			//22
	uint16			APOSA;   	
	uint16			APOSB;	 	
	uint16			RPOSA;   	
	uint16			RPOSB;   	
	uint16			IARMSOS; 			//32
	uint16			IBRMSOS;			//34
	uint16			EMUCON2; 	
//	float		        KUrms;								// ��ѹϵ��
//	float		        KIArms;								// Aͨ������ϵ��
//	float		        KIBrms;								// Bͨ������ϵ��
//	float		        KPrms1;								// ����ϵ��
//        float		        KPrms2;								// ����ϵ��
//        float		        KPrms3;								// ����ϵ��
//        float		        KPrms4;								// ����ϵ��
	uint16			RealUI[2];						// ������ʾֵ�����ʴ��ڴ�ֵʱ��ʾ 0.2%
	uint32			RealPw;								// ������ʾֵ���������ڴ�ֵʱ��ʾ 0.2%
          	
	uint16			RTCDota0;							// RTCУ���Ĵ���
	uint8			TemperAdj[2];					// �ߵ��²���ֵ
	uint8			RTCAdj[4];						// RTC�ߵ���ʱ����ֵ
	uint8			CurAdj;								// ����Ӱ�첹��ֵ
	uint8 			OfsetAdjAcVolt[2]; 		//���ݵ�ѹ����OFFSET��ֵ
	uint16			CorrectionTemper;  		//У��ʱ�̱�Ƶ��¶�
}sDl645FirmParaFile_TypeDef;		//58 Byte


//---end У������ļ�-------���ɸ��ݼ���оƬ���ģ� 

//---start ����ֵ˲ʱֵ��ת�ļ�-------
typedef struct
{
  uint8		ChkErrCnt;
  uint8		ChkErrCnt1;
  uint32 	Pw[2];   		    //pa,pb   
  uint32 	UI[3];          // Ia=UI[0] Inal U         
  uint16 	Frequency;   		//����Ƶ�ʣ���λ��                            	
  uint32		Pulse;		    	//ǰ̨����
  uint16     Pstart;
  //---��������---	
  uint32		Pulse_Eg;	    	//�������
  uint32 	PDirect;				//���ʷ���
  uint32 	ChkSum1;				//����EMU��У�����У��
  uint32	ChkSum;   
  // У��ʹ�ò���
  uint16		RatintU;				// ���ѹ
	uint16		RatingI;				// �����
  uint32		TempU;					// ��ǰУ�����ѹ
  uint32		TempI;					// ��ǰУ�������
  uint32		TempPw;					// ��ǰУ���㹦��
} sDl645FrontTmp_TypeDef;

//---end ����ֵ˲ʱֵ��ת�ļ�-------
typedef struct {
//	u32 Dat : 23;
//	u32 S : 1;
	uint8 Dat0;
	uint8 Dat1;
	uint8 Dat2 : 7;
	uint8 S : 1;			
} sDF09;
//---start ����ֵ˲ʱֵ�ļ�-------
typedef struct
{	
	struct sDl645FrontPubData_TypeDef  
	{
		uint16			U;			    		//---��ѹ---NNN.N
		uint32	    Ia;			    		//---����NNNN.NNNN(����ֵҪ��3��3С������ֵҪ��2��4С�����λ��ʾ����)---		
		uint32	    In;         		//---���ߵ���
		sDF09		Pw;			    		//---˲ʱ�й�p
		uint16			Pf;			    		//---��������N.NNN---	���λ��ʾ����{Pf Pfa Pfb Pfc}	
		uint16			Angle;		  		//---���NNN.N---		
		uint16			Frequency;			//---Ƶ��NN.NN
		uint32			PPwave;					//---һ����ƽ������NN.NNNN
		uint8      Chnsel;     		
		uint16			Temperature;		//---NNN.N  �¶�
		uint16			ClockBat;				//---NN.NN  ��ص�ѹ
		uint32			tWorkBat;				//---NNNN  ʱ�ӹ���ʱ�䣨���ӣ�
		uint8			PDirect;				//---ԭ���ʷ���
		                    		
	  uint16    	CfIn; 					//���������ƽ�ж�
	  uint8    	CfTime;					//
	  uint8    	Step;       		
	  uint16   	FrontStamp; 		
	  uint16			tMaxI;					// ����������ʱ�䣬0.5sΪ��λ
	  uint8			SafeCurFlag;		// �������޲�Ϊ���־
	} PubData;
	
	struct sDl645FrontPriData_TypeDef  
	{		
		uint8			Flag;						//---�����쳣��־---
	} PriData;	
	
	struct sDl645FrontPriPara_TypeDef  
	{		
		uint32	 PConstE;						//�й�����
		uint16	 Crc;
	} PriPara;		
} sDl645Front_TypeDef;	


//---end ����ֵ˲ʱֵ�ļ�-------

//---start ����ͨѶ�����ļ�-------
typedef struct
{
	uint16		EFlag;							//ͨѶ״̬
	
  uint16 	RxLen;							//�������ݳ���
  uint16  	TxLen;
  uint32		TimeOutStamp;				//�������ݳ���
  uint8 		*pTx;
  
  uint8		fBps;								//�����ʱ����־
  uint8		NewBps;							//�²�����
  uint32 	NewBpsStamp;				//������ʱ��
//uint8 		TxAddr;
  
  uint8 		RxBuf[MAX_COMPACK_SIZE];//���ջ���
  uint8		TxBuf[MAX_COMPACK_SIZE];//���ͻ���
}sComPack_TypeDef;

GLOBAL sDl645FirmParaFile_TypeDef Dl645FirmPara;
GLOBAL sDl645FrontTmp_TypeDef	Dl645FrontTmp;
GLOBAL sDl645Front_TypeDef		Dl645Front;
GLOBAL sComPack_TypeDef 		ComPack[1];  
GLOBAL uint8 RN8209DCommand;
GLOBAL uint8 NubRN8209DCommand;
GLOBAL bool b_Uart0orUart1;
GLOBAL bool b_Param_Ok;
GLOBAL uint8 DianUpIsOkFlag;
GLOBAL uint8 DianDownIsOkFlag;
GLOBAL uint8 ResetTimer;
GLOBAL uint8 FirstStart;
GLOBAL uint16 WriteCnt,WriteCnt1;

//GLOBAL uint32 Chnl1,Chnl2,Chnl3,Chnl4;
GLOBAL uint32 PwChnl[4],PwChnlHigh[4],PwChnlCnt[4];

GLOBAL uint32 PwChnl5;
GLOBAL uint32 PwChnl5Save;

GLOBAL uint32 DianYa;
GLOBAL uint16 GongLvList[4][GONGLV_LIST_SIZE],GongLv[4],GongLvSave[4];
GLOBAL uint8  GongLvListIndex[4];

GLOBAL uint8  LND_State;//������״̬
GLOBAL uint8  OnOff_State;//��������״̬
GLOBAL uint8  Write_En_Err_Cnt,Write_En_Err_Cnt1;
GLOBAL uint8  Write_Dis_Err_Cnt;
GLOBAL uint8  Reset_Err_Cnt;


//---end ͨѶ�����ļ�-------  
GLOBAL ErrorStatus fnRN8209_Write(uint8 wReg,uint8 *pBuf,uint8 ucLen);
GLOBAL void fnRN8209_Read(uint8 wReg);
GLOBAL void fnEMU_Init(void);
GLOBAL void fnDl645Front_Exec(void);
GLOBAL void fnScomPk_Init(uint8 ComPort);

GLOBAL void RN8209D_Recv_Analyse(void);  


#undef GLOBAL

#endif //__RN8209D_H__

