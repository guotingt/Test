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
    int16 input; ///<back_value
    int16 err; ///<error
    int32 kp; ///<proportion
    int32 ki; ///<integral
    int32 pOut; ///<proportion_out
    int32 iOut; ///<integral_out
    int32 sumOut; ///<output_value
    int32 outMax; ///<amplitude_limiting
    int32 outMin; ///<amplitude_limiting
    int16 mode; ///<manual_or_automatic
}PID;

/*upper computer command*/
typedef struct _command
{
    int16 motionCmd;
    int16 speedMode;
}COMMAND;

/*sensor message
  faultCode:
  B7(default zero) B6(default zero  ) 
  B5(1:lower over) B4(1:upper over  )
  B3(1:load over ) B2(1:hall error  )
  B1(1:coil error) B0(1:over current)*/
typedef struct _back_Data
{
    int16 current;
    int16 currentU;
    int16 currentV;
    int16 currentW;
    Uint16 speedCapture;
    Uint16 hallPos;
    Uint16 hallPos1;
    Uint16 upperOver;
    Uint16 lowerOver;
    Uint16 faultCode;
    Uint16 status;
    Uint16 motorDir;
    Uint16 motorRuning;
    Uint16 posFlag;
    int32  posCnt;
    Uint16 loadType;
    Uint16 posCntUp;
    Uint16 posCntDown;
}BACK_DATA;

#endif
