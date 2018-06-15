#ifndef GLOBAL_H
#define GLOBAL_H
#include "MyDataType.h"

/*motor 方向*/
#define FOREWARD 0 ///<正方向
#define BACKWARD 1 ///<反方向

/*Time Interrupt*/
extern TIME_FLAG  flagDot1ms;
#define flagDot1msW flagDot1ms.word

extern TIME_FLAG  flag1ms;
#define flag1msW flag1ms.word
#define currentLoopSample flag1ms.bit.bit0 ///<电流环
#define speedLoopSample flag1ms.bit.bit1   ///<速度环

extern TIME_FLAG  flag10ms;
#define flag10msW flag10ms.word
#define testSendSample flag10ms.bit.bit0  ///<定时发送调试信息

extern TIME_FLAG  flag100ms;
#define flag100msW flag100ms.word

extern TIME_FLAG  flag500ms;
#define flag500msW flag500ms.word

extern TIME_FLAG  flag1000ms;
#define flag1000msW flag1000ms.word

/*PID*/
extern PID speedPID;       ///<speed_controller
extern PID currentPID;     ///<current_controller

/*Sensor*/
extern BACK_DATA backData; ///<sensor_message

/*Port*/
extern COMMAND upperCommand; ///<上位机指令
extern Uint16 reciveBuf[];   ///<接收缓冲
extern Uint16 reciveFlag;    ///<接收flag
extern Uint16 moveCnt;       ///<运动计时
extern Uint16 duty;          ///<占空比
#endif

