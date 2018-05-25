#ifndef GLOBAL_H
#define GLOBAL_H
#include "MyDataType.h"

/*Time Interrupt*/
extern TIME_FLAG  flagDot1ms;
#define flagDot1msW flagDot1ms.word
#define currentLoopSample flagDot1ms.bit.bit0
#define sensorReadSample flagDot1ms.bit.bit1

extern TIME_FLAG  flag1ms;
#define flag1msW flag1ms.word

extern TIME_FLAG  flag10ms;
#define flag10msW flag10ms.word
#define speedLoopSample flag10ms.bit.bit0

extern TIME_FLAG  flag100ms;
#define flag100msW flag100ms.word
#define testSendSample flag100ms.bit.bit0

extern TIME_FLAG  flag500ms;
#define flag500msW flag500ms.word

extern TIME_FLAG  flag1000ms;
#define flag1000msW flag1000ms.word

/*PID*/
extern PID speedPID;///<speed_controller
extern PID currentPID;///<current_controller

/*Sensor*/
extern BACK_DATA backData;///<sensor_message

/*Port*/
extern COMMAND upperCommand;
extern Uint16 reciveBuf[];
extern Uint16 sendBuf[];
extern Uint16 reciveFlag;
extern Uint16 moveCnt;
extern Uint16 duty;
#endif

