#include "cmsis_os.h"
#include "Lib_Delay.h"
#include "TH_SENSOR_BSP.h"
#include "string.h"
#include "calc.h"
#include "global.h"

void AM_BUS_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure ;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
	
	 //Configure BUS pins: SDA_00 
		GPIO_InitStructure.GPIO_Pin =  II_AM_SDA_00_Pin;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT ;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP ;
//    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
		GPIO_InitStructure.GPIO_Speed =GPIO_Speed_10MHz;
		GPIO_Init(II_AM_SDA_00_GPIO, &GPIO_InitStructure);
		
		GPIO_SetBits(II_AM_SDA_00_GPIO,II_AM_SDA_00_Pin); 

		GPIO_InitStructure.GPIO_Pin =  II_AM_SDA_01_Pin;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT ;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP ;
//    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
		GPIO_InitStructure.GPIO_Speed =GPIO_Speed_10MHz;
		GPIO_Init(II_AM_SDA_01_GPIO, &GPIO_InitStructure);
		
		GPIO_SetBits(II_AM_SDA_01_GPIO,II_AM_SDA_01_Pin); 
}

void AM_Init(void)
{
		{
			AM_BUS_Config();
			AHT20_Init();   //��ʼ��AHT20
		}
		return ;
}

#define NUM_1 6       //�˲�����
#define T_MAX 2       //�˲�����
unsigned short AVGfilter_1(int8_t i8Type,int16_t i16Value)
{
		static int8_t i8Num[T_MAX]={0};
		static int16_t i16Value_buf[T_MAX][NUM_1];

		int16_t i16CvtValue;		
		if(i8Num[i8Type]<NUM_1)
		{
			i8Num[i8Type]++;
		}
		else
		{
			i8Num[i8Type]=0;		
		}		
		i16Value_buf[i8Type][i8Num[i8Type]] = i16Value;	
		i16CvtValue=MedianFilter((uint16_t *)i16Value_buf[i8Type],NUM_1);	
		
    return i16CvtValue;
}

void SDA_Pin_Output_High(void)   //��PB15����Ϊ��� �� ������Ϊ�ߵ�ƽ�� PB15��ΪI2C��SDA
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = II_AM_SDA_Pin;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT ;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP ;
//	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_10MHz;
	GPIO_Init(II_AM_SDA_GPIO,& GPIO_InitStructure);
	GPIO_SetBits(II_AM_SDA_GPIO,II_AM_SDA_Pin);
}

void SDA_Pin_Output_Low(void)  //��P15����Ϊ���  ������Ϊ�͵�ƽ
{

	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = II_AM_SDA_Pin;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT ;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP ;
//	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_10MHz;
	GPIO_Init(II_AM_SDA_GPIO,& GPIO_InitStructure);
	GPIO_ResetBits(II_AM_SDA_GPIO,II_AM_SDA_Pin);
}

void SDA_Pin_IN_FLOATING(void)  //SDA����Ϊ��������
{

	GPIO_InitTypeDef  GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = II_AM_SDA_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; //10M
	GPIO_Init( II_AM_SDA_GPIO,&GPIO_InitStructure);
}

void SCL_Pin_Output_High(void) //SCL����ߵ�ƽ��P14��ΪI2C��SCL
{
	GPIO_SetBits(II_AM_SCL_GPIO,II_AM_SCL_Pin);
}

void SCL_Pin_Output_Low(void) //SCL����͵�ƽ
{
	GPIO_ResetBits(II_AM_SCL_GPIO,II_AM_SCL_Pin);
}

void I2C_Start(void)		 //I2C��������START�ź�
{
	SDA_Pin_Output_High();
	Delay_us(8);
	SCL_Pin_Output_High();
	Delay_us(8);
	SDA_Pin_Output_Low();
	Delay_us(8);
	SCL_Pin_Output_Low();
	Delay_us(8);   
}


void AHT20_WR_Byte(uint8_t Byte) //��AHT20дһ���ֽ�
{
	uint8_t Data,N,i;	
	Data=Byte;
	i = 0x80;
	for(N=0;N<8;N++)
	{
		SCL_Pin_Output_Low(); 
		Delay_us(4);	
		if(i&Data)
		{
			SDA_Pin_Output_High();
		}
		else
		{
			SDA_Pin_Output_Low();
		}	
			
    SCL_Pin_Output_High();
		Delay_us(4);
		Data <<= 1;
		 
	}
	SCL_Pin_Output_Low();
	Delay_us(8);   
	SDA_Pin_IN_FLOATING();
	Delay_us(8);	
}	


uint8_t AHT20_RD_Byte(void)//��AHT20��ȡһ���ֽ�
{
	uint8_t Byte,i,a;
	Byte = 0;
	SCL_Pin_Output_Low();
	SDA_Pin_IN_FLOATING();
	Delay_us(8);	
	for(i=0;i<8;i++)
	{
    SCL_Pin_Output_High();		
		Delay_us(5);
		a=0;
		if(IIC_SDA_READ()) 
		{
			a=1;
		}
		Byte = (Byte<<1)|a;
		SCL_Pin_Output_Low();
		Delay_us(5);
	}
  SDA_Pin_IN_FLOATING();
	Delay_us(8);	
	return Byte;
}


uint8_t Receive_ACK(void)   //��AHT20�Ƿ��лظ�ACK
{
	uint16_t CNT;
	CNT = 0;
	SCL_Pin_Output_Low();	
	SDA_Pin_IN_FLOATING();
	Delay_us(8);	
	SCL_Pin_Output_High();	
	Delay_us(8);	
	while((IIC_SDA_READ())  && CNT < 100) 
	CNT++;
	if(CNT == 100)
	{
		return 0;
	}
 	SCL_Pin_Output_Low();	
	Delay_us(8);	
	return 1;
}

void Send_ACK(void)		  //�����ظ�ACK�ź�
{
	SCL_Pin_Output_Low();	
	Delay_us(8);	
	SDA_Pin_Output_Low();
	Delay_us(8);	
	SCL_Pin_Output_High();	
	Delay_us(8);
	SCL_Pin_Output_Low();	
	Delay_us(8);
	SDA_Pin_IN_FLOATING();
	Delay_us(8);
}

void Send_NOT_ACK(void)	//�������ظ�ACK
{
	SCL_Pin_Output_Low();	
	Delay_us(8);
	SDA_Pin_Output_High();
	Delay_us(8);
	SCL_Pin_Output_High();	
	Delay_us(8);		
	SCL_Pin_Output_Low();	
	Delay_us(8);
    SDA_Pin_Output_Low();
	Delay_us(8);
}

void Stop_I2C(void)	  //һ��Э�����
{
	SDA_Pin_Output_Low();
	Delay_us(8);
	SCL_Pin_Output_High();	
	Delay_us(8);
	SDA_Pin_Output_High();
	Delay_us(8);
}

uint8_t AHT20_Read_Status(void)//��ȡAHT20��״̬�Ĵ���
{
	uint8_t Byte_first;	
	I2C_Start();
	AHT20_WR_Byte(0x71);
	Receive_ACK();
	Byte_first = AHT20_RD_Byte();
	Send_NOT_ACK();
	Stop_I2C();
	return Byte_first;
}

uint8_t AHT20_Read_Cal_Enable(void)  //��ѯcal enableλ��û��ʹ��
{
	uint8_t val = 0;//ret = 0,
  val = AHT20_Read_Status();
	 if((val & 0x68)==0x08)
		 return 1;
   else  return 0;
 }

void AHT20_SendAC(void) //��AHT20����AC����
{
	I2C_Start();
	AHT20_WR_Byte(0x70);
	Receive_ACK();
	AHT20_WR_Byte(0xac);//0xAC�ɼ�����
	Receive_ACK();
	AHT20_WR_Byte(0x33);
	Receive_ACK();
	AHT20_WR_Byte(0x00);
	Receive_ACK();
	Stop_I2C();

}

//CRCУ�����ͣ�CRC8/MAXIM
//����ʽ��X8+X5+X4+1
//Poly��0011 0001  0x31
//��λ�ŵ�����ͱ�� 1000 1100 0x8c
//C��ʵ���룺
uint8_t Calc_CRC8(uint8_t *message,uint8_t Num)
{
        uint8_t i;
        uint8_t byte;
        uint8_t crc=0xFF;
  for(byte=0; byte<Num; byte++)
  {
    crc^=(message[byte]);
    for(i=8;i>0;--i)
    {
      if(crc&0x80) crc=(crc<<1)^0x31;
      else crc=(crc<<1);
    }
  }
        return crc;
}

uint8_t AHT20_Read_CTdata(uint32_t *ct) //û��CRCУ�飬ֱ�Ӷ�ȡAHT20���¶Ⱥ�ʪ������
{
	volatile uint8_t  Byte_1th=0;
	volatile uint8_t  Byte_2th=0;
	volatile uint8_t  Byte_3th=0;
	volatile uint8_t  Byte_4th=0;
	volatile uint8_t  Byte_5th=0;
	volatile uint8_t  Byte_6th=0;
	 uint32_t RetuData = 0;
	uint16_t cnt = 0;
	AHT20_SendAC();//��AHT10����AC����
	Delay_us(80000);//��ʱ80ms����	
    cnt = 0;
	while(((AHT20_Read_Status()&0x80)==0x80))//ֱ��״̬bit[7]Ϊ0����ʾΪ����״̬����Ϊ1����ʾæ״̬
	{
		Delay_us(1508);
		if(cnt++>=100)
		{
//			return FALSE;
		 break;
		 }
	}
	I2C_Start();
	AHT20_WR_Byte(0x71);
	Receive_ACK();
	Byte_1th = AHT20_RD_Byte();//״̬�֣���ѯ��״̬Ϊ0x98,��ʾΪæ״̬��bit[7]Ϊ1��״̬Ϊ0x1C������0x0C������0x08��ʾΪ����״̬��bit[7]Ϊ0
	Send_ACK();
	Byte_2th = AHT20_RD_Byte();//ʪ��
	Send_ACK();
	Byte_3th = AHT20_RD_Byte();//ʪ��
	Send_ACK();
	Byte_4th = AHT20_RD_Byte();//ʪ��/�¶�
	Send_ACK();
	Byte_5th = AHT20_RD_Byte();//�¶�
	Send_ACK();
	Byte_6th = AHT20_RD_Byte();//�¶�
	Send_NOT_ACK();
	Stop_I2C();

	RetuData = (RetuData|Byte_2th)<<8;
	RetuData = (RetuData|Byte_3th)<<8;
	RetuData = (RetuData|Byte_4th);
	RetuData =RetuData >>4;
	ct[0] = RetuData;//ʪ��
	RetuData = 0;
	RetuData = (RetuData|Byte_4th)<<8;
	RetuData = (RetuData|Byte_5th)<<8;
	RetuData = (RetuData|Byte_6th);
	RetuData = RetuData&0xfffff;
	ct[1] =RetuData; //�¶�
	return TRUE;
}


uint8_t AHT20_Read_CTdata_crc(uint32_t *ct) //CRCУ��󣬶�ȡAHT20���¶Ⱥ�ʪ������
{
	volatile uint8_t  Byte_1th=0;
	volatile uint8_t  Byte_2th=0;
	volatile uint8_t  Byte_3th=0;
	volatile uint8_t  Byte_4th=0;
	volatile uint8_t  Byte_5th=0;
	volatile uint8_t  Byte_6th=0;
	volatile uint8_t  Byte_7th=0;
	 uint32_t RetuData = 0;
	 uint16_t cnt = 0;
	// uint8_t  CRCDATA=0;
	 uint8_t  CTDATA[6]={0};//����CRC��������
	uint8_t Sensor_AnswerFlag;  //�յ���ʼ��־λ
	 
//	ENTER_CRITICAL_SECTION(); //��ȫ���ж�		
	AHT20_SendAC();//��AHT10����AC����
	Delay_us(80000);//��ʱ80ms����	
    cnt = 0;
	while(((AHT20_Read_Status()&0x80)==0x80))//ֱ��״̬bit[7]Ϊ0����ʾΪ����״̬����Ϊ1����ʾæ״̬
	{
		Delay_us(1508);
		if(cnt++>=100)
		{
		 break;
		}
	}
	
	I2C_Start();

	AHT20_WR_Byte(0x71);
	Receive_ACK();
	CTDATA[0]=Byte_1th = AHT20_RD_Byte();//״̬�֣���ѯ��״̬Ϊ0x98,��ʾΪæ״̬��bit[7]Ϊ1��״̬Ϊ0x1C������0x0C������0x08��ʾΪ����״̬��bit[7]Ϊ0
	Send_ACK();
	CTDATA[1]=Byte_2th = AHT20_RD_Byte();//ʪ��
	Send_ACK();
	CTDATA[2]=Byte_3th = AHT20_RD_Byte();//ʪ��
	Send_ACK();
	CTDATA[3]=Byte_4th = AHT20_RD_Byte();//ʪ��/�¶�
	Send_ACK();
	CTDATA[4]=Byte_5th = AHT20_RD_Byte();//�¶�
	Send_ACK();
	CTDATA[5]=Byte_6th = AHT20_RD_Byte();//�¶�
	Send_ACK();
	Byte_7th = AHT20_RD_Byte();//CRC����
	Send_NOT_ACK();                           //ע��: ����Ƿ���NAK
	Stop_I2C();

	if(Calc_CRC8(CTDATA,6)==Byte_7th)
	{
	RetuData = (RetuData|Byte_2th)<<8;
	RetuData = (RetuData|Byte_3th)<<8;
	RetuData = (RetuData|Byte_4th);
	RetuData =RetuData >>4;
	ct[0] = RetuData;//ʪ��
	RetuData = 0;
	RetuData = (RetuData|Byte_4th)<<8;
	RetuData = (RetuData|Byte_5th)<<8;
	RetuData = (RetuData|Byte_6th);
	RetuData = RetuData&0xfffff;
	ct[1] =RetuData; //�¶�
	Sensor_AnswerFlag= TRUE;		
	}
	else
	{
		ct[0]=0x00;
		ct[1]=0x00;//У����󷵻�ֵ���ͻ����Ը����Լ���Ҫ����
	Sensor_AnswerFlag= FALSE;
	}//CRC����

//	EXIT_CRITICAL_SECTION(); //��ȫ���ж�	
	return Sensor_AnswerFlag;
}


void AHT20_Init(void)   //��ʼ��AHT20
{	
//	Init_I2C_Sensor_Port();
	
	I2C_Start();
	AHT20_WR_Byte(0x70);
	Receive_ACK();
	AHT20_WR_Byte(0xa8);//0xA8����NOR����ģʽ
	Receive_ACK();
	AHT20_WR_Byte(0x00);
	Receive_ACK();
	AHT20_WR_Byte(0x00);
	Receive_ACK();
	Stop_I2C();

	Delay_us(10000);//��ʱ10ms����

	I2C_Start();
	AHT20_WR_Byte(0x70);
	Receive_ACK();
	AHT20_WR_Byte(0xbe);//0xBE��ʼ�����AHT20�ĳ�ʼ��������0xBE,   AHT10�ĳ�ʼ��������0xE1
	Receive_ACK();
	AHT20_WR_Byte(0x08);//��ؼĴ���bit[3]��1��ΪУ׼���
	Receive_ACK();
	AHT20_WR_Byte(0x00);
	Receive_ACK();
	Stop_I2C();
	Delay_us(10000);//��ʱ10ms����
}
void JH_Reset_REG(uint8_t addr)
{
	
	uint8_t Byte_first,Byte_second,Byte_third;
	I2C_Start();
	AHT20_WR_Byte(0x70);//ԭ����0x70
	Receive_ACK();
	AHT20_WR_Byte(addr);
	Receive_ACK();
	AHT20_WR_Byte(0x00);
	Receive_ACK();
	AHT20_WR_Byte(0x00);
	Receive_ACK();
	Stop_I2C();

	Delay_us(5000);//��ʱ5ms����
	I2C_Start();
	AHT20_WR_Byte(0x71);//
	Receive_ACK();
	Byte_first = AHT20_RD_Byte();
	Send_ACK();
	Byte_second = AHT20_RD_Byte();
	Send_ACK();
	Byte_third = AHT20_RD_Byte();
	Send_NOT_ACK();
	Stop_I2C();
	
    Delay_us(10000);//��ʱ10ms����
	I2C_Start();
	AHT20_WR_Byte(0x70);///
	Receive_ACK();
	AHT20_WR_Byte(0xB0|addr);////�Ĵ�������
	Receive_ACK();
	AHT20_WR_Byte(Byte_second);
	Receive_ACK();
	AHT20_WR_Byte(Byte_third);
	Receive_ACK();
	Stop_I2C();
	
	Byte_second=0x00;
	Byte_third =0x00;
}

void AHT20_Start_Init(void)
{
	JH_Reset_REG(0x1b);
	JH_Reset_REG(0x1c);
	JH_Reset_REG(0x1e);
}

#define AM_SENSOR_NUM  1
#define TH_AVE_NUM  5
//AM2301B ��ʪ��
uint8_t AM2301B_update(void)
{
		static 	uint8_t  u8CNT=0;
		static 	uint8_t  u8Err_CNT[AM_SENSOR_NUM]={0};
		
		uint8_t i=0,j=0;  //�յ���ʼ��־λ
		uint8_t u8SenFlag[AM_SENSOR_NUM]={0};  //�յ���ʼ��־λ
		Com_tnh_st u16TH_Buff={0};
		
		uint32_t u32TH[2]={0};
		static 	uint8_t  u8THInit=FALSE;
		
		if(u8THInit==FALSE)
		{
			u8THInit=TRUE;
			AM_Init();
			
			Delay_us(10000);//10ms			
			if((AHT20_Read_Status()&0x18)!=0x18)
			{
				AHT20_Start_Init(); //���³�ʼ���Ĵ���
				Delay_us(10000);
			}   
		}
		u8CNT++;
		if(u8CNT>=0xFF)
		{
			u8CNT=0x00;	
		}
		i=u8CNT%(2);
//		//���ζ�ȡ�������2S
		if(i!=0)
		{
				return 0;			
		}
    u8SenFlag[i]=AHT20_Read_CTdata_crc(u32TH);  //crcУ��󣬶�ȡAHT20���¶Ⱥ�ʪ������ 
		if(u8SenFlag[i])
		{
				if((u32TH[0]==0)&&(u32TH[1]==0))
				{
					u8Err_CNT[i]++;						
				}
				else
				{
					u8Err_CNT[i]=0;
					u16TH_Buff.Hum = u32TH[0]*100*10/1024/1024;  //����õ�ʪ��ֵc1���Ŵ���10����
					u16TH_Buff.Temp = u32TH[1]*200*10/1024/1024-500;//����õ��¶�ֵt1���Ŵ���10����
					//����״̬MBM_COM_STS_REG_NO		
          g_sVariable.status.u16Status_remap[COM_STS_REG_NO] |= (0x0001 << 0);		
					u16TH_Buff.Temp+=(int16_t)(g_sVariable.gPara.TH.u16TH_Cali[0]);
					u16TH_Buff.Hum+=(int16_t)(g_sVariable.gPara.TH.u16TH_Cali[1]);					
				}					
		}
		else
		{
				u8Err_CNT[i]++;			
		}
		
		if(u8Err_CNT[i]>ERROR_CNT_MAX)
		{
//			u8Err_CNT[i]=0;
			g_sVariable.status.u16TH[0].Temp = 285;
			g_sVariable.status.u16TH[0].Hum = 567;		
			//����״̬MBM_COM_STS_REG_NO
			g_sVariable.status.u16Status_remap[COM_STS_REG_NO] &= ~(0x0001<<i);			
				AM_Init();				//AM Sensor init	
				i=0;			
		}
		else if(u8Err_CNT[i]==0)
		{		
			g_sVariable.status.u16TH[0].Temp=u16TH_Buff.Temp;
			g_sVariable.status.u16TH[0].Hum=u16TH_Buff.Hum;	
			
			if(g_sVariable.gPara.u16Manual_Test_En==TEST_MODE_ENABLE)//����ģʽ
			{
				return u8SenFlag[i];
			}
			//ȡƽ��ֵ
			for(j=0;j<T_MAX;j++)
			{
					g_sVariable.status.u16TH[0].Temp=AVGfilter_1(j,g_sVariable.status.u16TH[0].Temp);
			}
		}	
		//ȥ��С��λ
		g_sVariable.status.u16TH[1].Temp=g_sVariable.status.u16TH[0].Temp/10;
		if(g_sVariable.status.u16TH[0].Temp%10>=5)//��������
		{
			g_sVariable.status.u16TH[1].Temp += 1;
		}
		//
		g_sVariable.status.u16TH[1].Hum=g_sVariable.status.u16TH[0].Hum/10;
		if(g_sVariable.status.u16TH[0].Hum%10>=5)//��������
		{
			g_sVariable.status.u16TH[1].Hum += 1;
		}		
		return u8SenFlag[i];
}
/********************************************\
|* ���ܣ� ��ʪ�ȸ���             	        *|
\********************************************/
void AM_Sensor_update(void)
{
		AM2301B_update();

//	SDA_Pin_Output_High();	
//	Delay_us(10000);//10ms	
//	SDA_Pin_Output_Low();		
//	Delay_us(10000);//10ms	
//	SDA_Pin_Output_High();	
//	Delay_us(10000);//10ms	
//	SDA_Pin_Output_Low();	
//		g_sys.status.ComSta.u16TH[0].Temp=285;
//		g_sys.status.ComSta.u16TH[0].Temp=567;
//		rt_kprintf("u8CNT=%x,i=%x,u8SenFlag[0]= %x,u16TH_Sensor[0]= %x,[1] = %x,u8Err_CNT[0]=%x,Temp=%d,Hum=%d\n",u8CNT,i,u8SenFlag[0],u16TH_Sensor[0],u16TH_Sensor[1],u8Err_CNT[0],gds_ptr->status.mbm.tnh[0].temp,gds_ptr->status.mbm.tnh[0].hum);			
}




