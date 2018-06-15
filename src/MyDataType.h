#ifndef MY_DATA_TYPE_H
#define MY_DATA_TYPE_H
#include "DSP28x_Project.h"

/*16_bit_Struct(for flag)*/
typedef struct _bit16_def
{
	Uint16 bit0:1;
	Uint16 bit1:1;
	Uint16 bit2:1;
	Uint16 bit3:1;
	Uint16 bit4:1;
	Uint16 bit5:1;
	Uint16 bit6:1;
	Uint16 bit7:1;
	Uint16 bit8:1;
	Uint16 bit9:1;
	Uint16 bit10:1;
	Uint16 bit11:1;
	Uint16 bit12:1;
	Uint16 bit13:1;
	Uint16 bit14:1;
	Uint16 bit15:1;
}BIT16_DEF;

/*Union for TimeFlag*/
typedef union _time_flag
{
	Uint16 word;
	BIT16_DEF bit;
}TIME_FLAG;

/*PID controller parameter(PI)*/
typedef struct _pid
{
    int16 setPoint; ///<expectation_value
    int16 input;    ///<back_value
    int16 err;      ///<error
    int32 kp;       ///<proportion
    int32 ki;       ///<integral
    int32 pOut;     ///<proportion_out
    int32 iOut;     ///<integral_out
    int32 sumOut;   ///<output_value
    int32 outMax;   ///<amplitude_limiting
    int32 outMin;   ///<amplitude_limiting
}PID;

/*upper computer command*/
typedef struct _command
{
    int16 motionCmd; ///<����
    int16 speedMode; ///<�ٶ�ģʽ(����,��������)
}COMMAND;

/*sensor message
  faultCode:
  B7(default zero) B6(1:manual status)
  B5(1:lower over) B4(1:upper over  )
  B3(1:load over ) B2(1:hall error  )
  B1(1:coil error) B0(1:over current)*/
typedef struct _back_Data
{
    int16 current;       ///<��ͨ�����
    int16 currentU;      ///<U�����
    int16 currentV;      ///<V�����
    int16 currentW;      ///<W�����
    Uint16 speedCapture; ///<�ٶȻ�ȡ
    Uint16 hallPos;      ///<����ʱ�̶�ȡ�Ļ���ֵ
    Uint16 hallPos1;     ///<����ʱ�̶�ȡ�Ļ���ֵ
    Uint16 upperOver;    ///<����λ��־
    Uint16 lowerOver;    ///<����λ��־
    Uint16 faultCode;    ///<���ϴ��� ���ڴ����д���ͷ�����λ��
    Uint16 status;       ///<����״̬ ������Ӧ���̺ͷ�����λ��
    Uint16 motorDir;     ///<�������˳��
    Uint16 posFlag;      ///<�Ƿ�������λ�� ���Э��ϵͳ���ͱ�־
    Uint16 posCntUp;     ///<���������ż�����ʾλ��
    Uint16 posCntDown;   ///<�½������ż�����ʾλ��
}BACK_DATA;

#endif
