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
    int16 motionCmd; ///<动作
    int16 speedMode; ///<速度模式(多余,不做解析)
}COMMAND;

/*sensor message
  faultCode:
  B7(default zero) B6(1:manual status)
  B5(1:lower over) B4(1:upper over  )
  B3(1:load over ) B2(1:hall error  )
  B1(1:coil error) B0(1:over current)*/
typedef struct _back_Data
{
    int16 current;       ///<导通相电流
    int16 currentU;      ///<U相电流
    int16 currentV;      ///<V相电流
    int16 currentW;      ///<W相电流
    Uint16 speedCapture; ///<速度获取
    Uint16 hallPos;      ///<捕获时刻读取的霍尔值
    Uint16 hallPos1;     ///<发送时刻读取的霍尔值
    Uint16 upperOver;    ///<上限位标志
    Uint16 lowerOver;    ///<下限位标志
    Uint16 faultCode;    ///<故障代码 用于错误集中处理和反馈上位机
    Uint16 status;       ///<工作状态 控制响应流程和反馈上位机
    Uint16 motorDir;     ///<电机换相顺序
    Uint16 posFlag;      ///<是否是正常位置 最好协调系统发送标志
    Uint16 posCntUp;     ///<上升、开门计数表示位置
    Uint16 posCntDown;   ///<下降、关门计数表示位置
}BACK_DATA;

#endif
