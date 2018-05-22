#ifndef GLOBAL_H
#define GLOBAL_H
#include "MyDataType.h"

/*Time Interrupt*/
extern Uint32 interruptCnt;
extern TIME_FLAG  flagDot1ms;
#define flagDot1msW flagDot1ms.word
#define currentLoopSample flagDot1ms.bit.bit0
#define sensorReadSample flagDot1ms.bit.bit1
//#define pwmUpdateSample flagDot1ms.bit.bit2

extern TIME_FLAG  flag1ms;
#define flag1msW flag1ms.word
//#define speedLoopSample flag1ms.bit.bit0

extern TIME_FLAG  flag10ms;
#define flag10msW flag10ms.word
#define speedLoopSample flag10ms.bit.bit0

extern TIME_FLAG  flag100ms;
#define flag100msW flag100ms.word
#define testSendSample flag100ms.bit.bit0

extern TIME_FLAG  flag500ms;
#define flag500msW flag500ms.word
//#define testSendSample flag500ms.bit.bit0

extern TIME_FLAG  flag1000ms;
#define flag1000msW flag1000ms.word

/*PID*/
extern PID speedPID;///<speed_controller
extern PID currentPID;///<current_controller

/*Sensor*/
extern BACK_DATA backData;///<sensor_message
extern QUE_def uiQueue;///<Ui
extern QUE_def viQueue;///<Vi
extern QUE_def wiQueue;///<Wi
extern Uint16 LastHallGpio;
extern Uint16 NewHallGpio;
extern Uint32 VirtualTimer;
extern Uint32 SpeedNewTimer;
extern Uint32 SpeedLastTimer;
extern Uint16 modcnt;
/*Port*/
extern COMMAND upperCommand;
extern Uint16 motorRuning;
extern Uint16 motorDir;
extern Uint16 reciveBuf[];
extern Uint16 sendBuf[];
extern Uint16 reciveFlag;
extern Uint16 moveCnt;
extern Uint16 timeFlag;
extern Uint16 duty;
#endif
